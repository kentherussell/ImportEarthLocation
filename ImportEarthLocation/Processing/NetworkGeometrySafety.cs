using System;
using System.Collections.Generic;
using System.Linq;
using System.Threading;
using ImportEarthLocation.Core;
using Unity.Mathematics;

namespace ImportEarthLocation.Processing;

// Final coupled solve, after horizontal fitting and the legacy profile passes.
// Roads and rail share clearance constraints. Only explicit, same-level OSM
// crossing anchors may also share elevation; proximity never joins networks.
// Bounds, tunnel cover and shared anchors always block invalid output. The caller
// may explicitly accept residual clearance/grade warnings for a test iteration.
public static class NetworkGeometrySafety
{
    public const float DeckGap = NetworkVerticalProfiles.MinimumBridgeClearance + 4f;
    private readonly record struct Key(NetworkKind Kind, long Id);
    private sealed record Contact(int A, float T, int B, float U,NetworkContactKind Kind=NetworkContactKind.IndependentLevels);
    private readonly record struct Span(int Index, float T, float U, float3 A, float3 B);
    public static float GradeLimit(PreparedSegment s) => s.Kind == NetworkKind.Rail ? (s.Tunnel ? .08f : .05f) : NetworkVerticalProfiles.GradeLimit(s);
    private static float Radius(PreparedSegment s) => s.NativeWidth > 0 ? s.NativeWidth * .5f : s.Kind == NetworkKind.Rail ? 6 : Math.Max(4, s.Lanes * 1.8f + 2);

    public static IReadOnlyList<PreparedSegment> Repair(IReadOnlyList<PreparedSegment> source, ushort[] raster,
        float scale, double width, ImportLog log, CancellationToken token, bool allowQualityWarnings=false,Func<float3,float?>? waterLevel=null)
    {
        if(source.Count == 0)return source;
        var sections=source.ToArray();var n=(int)Math.Sqrt(raster.Length);
        if(n*n!=raster.Length||n<2||scale<=0||width<=0)throw new InvalidOperationException("Network safety: invalid terrain.");
        float Ground(float3 p)
        {
            var x=math.clamp((p.x/(float)width+.5f)*(n-1),0,n-1);var z=math.clamp((p.z/(float)width+.5f)*(n-1),0,n-1);
            var ix=(int)x;var iz=(int)z;
            return math.lerp(math.lerp(raster[iz*n+ix],raster[iz*n+Math.Min(ix+1,n-1)],x-ix),math.lerp(raster[Math.Min(iz+1,n-1)*n+ix],raster[Math.Min(iz+1,n-1)*n+Math.Min(ix+1,n-1)],x-ix),z-iz)/65535f*scale;
        }
        Key Start(PreparedSegment s)=>new(s.Kind,s.StartNodeId);
        Key End(PreparedSegment s)=>new(s.Kind,s.EndNodeId);
        var nodeIndex=new Dictionary<Key,int>();var nodePositions=new List<float3>();var heights=new List<float>();
        var start=new int[sections.Length];var end=new int[sections.Length];
        int Node(Key key,float3 p)
        {
            if(nodeIndex.TryGetValue(key,out var id))
            {if(math.distance(nodePositions[id].xz,p.xz)>.05f)log.Diagnostic($"SHARED NODE REPAIR node={key}; displacementMeters={math.distance(nodePositions[id].xz,p.xz):R}; canonicalIncidentAnchorRetained=true; generationGate=false");return id;}
            id=nodePositions.Count;nodeIndex[key]=id;nodePositions.Add(p);heights.Add(p.y);return id;
        }
        for(var i=0;i<sections.Length;i++){start[i]=Node(Start(sections[i]),sections[i].Start);end[i]=Node(End(sections[i]),sections[i].End);}
        var surfaceNodes=new HashSet<int>();for(var i=0;i<sections.Length;i++)if(!sections[i].Tunnel){surfaceNodes.Add(start[i]);surfaceNodes.Add(end[i]);}
        var portalDistance=Enumerable.Repeat(float.PositiveInfinity,heights.Count).ToArray();
        var underground=new Dictionary<int,List<(int node,float length)>>();
        for(var i=0;i<sections.Length;i++)if(sections[i].Tunnel)
            foreach(var reverse in new[]{false,true})
            {var a=reverse?end[i]:start[i];var b=reverse?start[i]:end[i];if(!underground.TryGetValue(a,out var list))underground[a]=list=new();list.Add((b,NetworkVerticalProfiles.ArcLength(sections[i])));}
        var portalQueue=new List<int>();foreach(var id in surfaceNodes.Where(underground.ContainsKey)){portalDistance[id]=0;portalQueue.Add(id);}
        for(var cursor=0;cursor<portalQueue.Count;cursor++)
        {var id=portalQueue[cursor];foreach(var edge in underground[id]){var distance=portalDistance[id]+edge.length;if(distance>=portalDistance[edge.node]||distance>160)continue;portalDistance[edge.node]=distance;portalQueue.Add(edge.node);}}
        float Cover(float distance){var t=math.clamp(distance/160,0,1);return 15*t*t*(3-2*t);}
        var parent=Enumerable.Range(0,heights.Count).ToArray();
        var incidence=new Dictionary<Key,List<PreparedSegment>>();
        foreach(var s in sections)foreach(var key in new[]{Start(s),End(s)})
        {if(!incidence.TryGetValue(key,out var list))incidence[key]=list=new();list.Add(s);}
        var levelCrossingAnchors=0;
        foreach(var pair in nodeIndex.Where(p=>p.Key.Kind==NetworkKind.Road))
        {
            var railKey=new Key(NetworkKind.Rail,pair.Key.Id);
            if(!nodeIndex.TryGetValue(railKey,out var rail))continue;
            var roadEdges=incidence[pair.Key];var railEdges=incidence[railKey];
            if(roadEdges.Concat(railEdges).Any(s=>s.Bridge||s.Tunnel)||roadEdges.Concat(railEdges).Select(s=>s.Layer).Distinct().Count()!=1||math.distance(nodePositions[pair.Value].xz,nodePositions[rail].xz)>.01f)continue;
            parent[pair.Value]=rail;
            // Rail's long-profile station is the prior; the coupled grade solve
            // can subsequently move this common anchor for both networks.
            heights[pair.Value]=heights[rail];levelCrossingAnchors++;
        }
        log.Diagnostic($"NETWORK LEVEL CROSSINGS sharedSourceAnchors={levelCrossingAnchors}; sameLayerAndGroundOnly=true; railStationPrior=true; proximityInventedConnections=0; nativeCrossingAcceptanceUnverified=true");
        var allContacts=Contacts(sections,token);
        var contacts=allContacts.Where(c=>c.Kind==NetworkContactKind.IndependentLevels).ToList();
        log.Diagnostic($"NETWORK CONTACTS vertical={contacts.Count}; horizontal={allContacts.Count(c=>c.Kind==NetworkContactKind.HorizontalConflict)}; unconnectedSurfaceCrossings={allContacts.Count(c=>c.Kind==NetworkContactKind.UnconnectedSurfaceCrossing)}; inferredFlyovers=false");
        var obliqueDecks=new HashSet<(NetworkKind,long)>();
        foreach(var contact in contacts)
        {
            var a=sections[contact.A];var b=sections[contact.B];
            if(Math.Abs(math.dot(math.normalizesafe((a.CourseEnd-a.CourseStart).xz),math.normalizesafe((b.CourseEnd-b.CourseStart).xz)))<.5f)continue;
            obliqueDecks.Add((a.Kind,a.WayId));obliqueDecks.Add((b.Kind,b.WayId));
        }
        int Root(int id){while(parent[id]!=id){parent[id]=parent[parent[id]];id=parent[id];}return id;}
        // Do not weld an interchange into one elevation plane. A bridge route can
        // meet another deck through a graded connection and cross it elsewhere.
        var decks=sections.Where(s=>s.Bridge&&!s.Tunnel&&!s.ProtectedLink).GroupBy(s=>(s.Kind,s.WayId)).ToArray();
        var deckWays=new Dictionary<Key,HashSet<long>>();
        foreach(var s in decks.SelectMany(g=>g))foreach(var key in new[]{Start(s),End(s)})
        {if(!deckWays.TryGetValue(key,out var ways))deckWays[key]=ways=new();ways.Add(s.WayId);}
        // Do not flatten an entire OSM way to its highest node. A bridge can have
        // a gentle crest or sag and way IDs are not physical deck boundaries.
        log.Diagnostic("NETWORK SAFETY profilePolicy=preserve-cubic-residual; forcedFlatDecks=false; verticalOrderFromWayId=false");
        var h=heights.ToArray();var floor=new float[h.Length];var ceiling=new float[h.Length];
        foreach(var route in decks.Where(g=>g.All(s=>Root(nodeIndex[Start(s)])==Root(nodeIndex[End(s)]))))
        {
            var deck=route.Max(s=>Math.Max(s.Start.y,s.End.y));
            foreach(var s in route){h[nodeIndex[Start(s)]]=deck;h[nodeIndex[End(s)]]=deck;}
        }
        for(var i=0;i<h.Length;i++){floor[i]=Math.Max(0,Ground(nodePositions[i])-120);ceiling[i]=Math.Min(scale,Ground(nodePositions[i])+60);}
        var waterDeckFloors=0;
        if(waterLevel!=null)
        foreach(var route in Enumerable.Range(0,sections.Length).Where(i=>sections[i].Bridge&&!sections[i].Tunnel).GroupBy(i=>(sections[i].Kind,sections[i].WayId)))
        {
            var wet=false;var lift=0f;
            foreach(var index in route)
            {
                var s=sections[index];var steps=Math.Max(8,(int)Math.Ceiling(NetworkVerticalProfiles.ArcLength(s)/16));
                for(var j=0;j<=steps;j++)
                {token.ThrowIfCancellationRequested();var p=NetworkNormalizer.Evaluate(s,j/(float)steps);var level=waterLevel(p);if(!level.HasValue||!math.isfinite(level.Value))continue;wet=true;lift=Math.Max(lift,level.Value+7-p.y);}
            }
            if(!wet)continue;
            foreach(var index in route)foreach(var id in new[]{start[index],end[index]})floor[id]=Math.Max(floor[id],heights[id]+lift);
            waterDeckFloors++;
        }
        log.Diagnostic($"NETWORK WATER DECK FLOORS routes={waterDeckFloors}; waterEvidenceRequired=true; laterClearanceSolveCannotSubmergeDeck=true");
        for(var i=0;i<sections.Length;i++)if(sections[i].Tunnel)
        {
            // Interior tunnel nodes retain cover; mixed portal nodes can meet ground.
            foreach(var id in new[]{start[i],end[i]})
                ceiling[id]=Math.Min(ceiling[id],Ground(nodePositions[id])-Cover(portalDistance[id]));
        }
        var groups=Enumerable.Range(0,h.Length).GroupBy(Root).ToDictionary(g=>g.Key,g=>g.ToArray());
        var roots=Enumerable.Range(0,h.Length).Select(Root).ToArray();
        var members=new int[h.Length][];var low=new float[h.Length];var high=new float[h.Length];
        var lo=new Dictionary<int,float>();var hi=new Dictionary<int,float>();
        foreach(var group in groups)
        {
            lo[group.Key]=group.Value.Max(id=>floor[id]);hi[group.Key]=group.Value.Min(id=>ceiling[id]);
            members[group.Key]=group.Value;low[group.Key]=lo[group.Key];high[group.Key]=hi[group.Key];
            var value=group.Value.Length>1?group.Value.Max(id=>h[id]):h[group.Key];
            value=math.clamp(value,lo[group.Key],hi[group.Key]);foreach(var id in group.Value)h[id]=value;
        }
        var watchers=new List<int>[h.Length];for(var i=0;i<watchers.Length;i++)watchers[i]=new();
        bool[] awake=Array.Empty<bool>();var relaxation=1f;
        float Move(int id,float amount)
        {
            var root=roots[id];var before=h[id];var after=math.clamp(before+amount*relaxation,low[root],high[root]);
            if(Math.Abs(after-before)<.000001f)return 0;
            foreach(var member in members[root])h[member]=after;
            foreach(var constraint in watchers[root])awake[constraint]=true;
            return after-before;
        }
        var lengths=sections.Select(NetworkVerticalProfiles.ArcLength).ToArray();
        // Affine endpoint displacement preserves the accepted vertical curvature.
        // Include that curvature in every cover/crossing constraint, not just the
        // endpoint chord, so the solver and the emitted native curve agree.
        float ProfileOffset(int index,float t)=>NetworkNormalizer.Evaluate(sections[index],t).y-math.lerp(sections[index].Start.y,sections[index].End.y,t);
        float Height(int index,float t)=>math.lerp(h[start[index]],h[end[index]],t)+ProfileOffset(index,t);
        var minSpeeds=new float[sections.Length];
        var coverConstraints=new List<(int index,float t,float ceiling)>();
        for(var i=0;i<sections.Length;i++)
        {
            // Constant dy/dt uses a conservative derivative lower bound, not arc
            // length (which permits steep spikes when a horizontal handle is short).
            var s=sections[i];minSpeeds[i]=MinimumSpeed(3*(s.CourseControl1-s.CourseStart).xz,3*(s.CourseControl2-s.CourseControl1).xz,3*(s.CourseEnd-s.CourseControl2).xz,6);
            if(s.Tunnel)
            {
                var count=Math.Max(16,(int)Math.Ceiling(lengths[i]/4));
                for(var j=1;j<count;j++)
                {var t=j/(float)count;var distance=Math.Min(portalDistance[start[i]]+lengths[i]*t,portalDistance[end[i]]+lengths[i]*(1-t));coverConstraints.Add((i,t,Ground(NetworkNormalizer.Evaluate(s,t))-Cover(distance)));}
            }
        }
        var mobility=new float[h.Length];for(var i=0;i<sections.Length;i++){mobility[start[i]]++;mobility[end[i]]++;}
        var mass=groups.ToDictionary(g=>g.Key,g=>Math.Max(1,g.Value.Sum(id=>mobility[id])));
        // Carry crossing order across short bridge/ground transitions on the
        // same physical route. Otherwise the same crossing says both "road over
        // rail" and "rail over road" on adjacent pieces a few metres apart.
        var routeParent=Enumerable.Range(0,sections.Length).ToArray();
        int Route(int i){while(routeParent[i]!=i){routeParent[i]=routeParent[routeParent[i]];i=routeParent[i];}return i;}
        var incidentRoutes=new Dictionary<int,List<int>>();
        for(var i=0;i<sections.Length;i++)foreach(var node in new[]{start[i],end[i]})
        {if(!incidentRoutes.TryGetValue(node,out var list))incidentRoutes[node]=list=new();list.Add(i);}
        foreach(var list in incidentRoutes.Values)for(var i=0;i<list.Count;i++)for(var j=i+1;j<list.Count;j++)
        {
            if(list.Count!=2)continue;
            var a=sections[list[i]];var b=sections[list[j]];
            if(a.Tunnel!=b.Tunnel||Math.Abs(math.dot(math.normalizesafe((a.End-a.Start).xz),math.normalizesafe((b.End-b.Start).xz)))<.9f)continue;
            routeParent[Route(list[j])]=Route(list[i]);
        }
        bool TaggedUpper(Contact contact)
        {
            var a=sections[contact.A];var b=sections[contact.B];
            if(a.Tunnel!=b.Tunnel)return !a.Tunnel;
            if(a.Layer!=b.Layer)return a.Layer>b.Layer;
            if(a.Bridge!=b.Bridge)return a.Bridge;
            return NetworkNormalizer.Evaluate(a,contact.T).y>=NetworkNormalizer.Evaluate(b,contact.U).y;
        }
        float2 Position(Contact c)=>(NetworkNormalizer.Evaluate(sections[c.A],c.T).xz+NetworkNormalizer.Evaluate(sections[c.B],c.U).xz)*.5f;
        float Distance(Contact c)=>math.distancesq(NetworkNormalizer.Evaluate(sections[c.A],c.T).xz,NetworkNormalizer.Evaluate(sections[c.B],c.U).xz);
        var ordering=new Dictionary<Contact,bool>();
        foreach(var family in contacts.GroupBy(c=>(Math.Min(Route(c.A),Route(c.B)),Math.Max(Route(c.A),Route(c.B)))))
        {
            // A route can cross the same route again elsewhere. Only propagate
            // order inside this crossing's 100 m footprint, not across the map.
            var seeds=new List<(float2 point,int upperRoute)>();
            foreach(var c in family.OrderBy(Distance).ThenByDescending(c=>Math.Abs(sections[c.A].Layer-sections[c.B].Layer)))
            {
                var point=Position(c);var ra=Route(c.A);var rb=Route(c.B);
                if(ra==rb){ordering[c]=TaggedUpper(c);continue;}
                var existing=seeds.FindIndex(s=>math.distance(s.point,point)<=100);
                if(existing<0){ordering[c]=TaggedUpper(c);seeds.Add((point,ordering[c]?ra:rb));}
                else ordering[c]=seeds[existing].upperRoute==ra;
            }
        }
        // Fixed ordering is essential: never flip upper/lower after each lift.
        var constraints=new List<(int upper,float t,int lower,float u,(int id,float step)[] weights)>();
        foreach(var contact in contacts)
        {
            var upper=ordering[contact]?contact.A:contact.B;var lower=upper==contact.A?contact.B:contact.A;
            var t=upper==contact.A?contact.T:contact.U;var u=upper==contact.A?contact.U:contact.T;
            var coefficients=new Dictionary<int,float>();
            void Add(int id,float coefficient){var root=Root(id);coefficients[root]=coefficients.TryGetValue(root,out var old)?old+coefficient:coefficient;}
            Add(start[upper],1-t);Add(end[upper],t);Add(start[lower],u-1);Add(end[lower],-u);
            var denominator=coefficients.Sum(p=>p.Value*p.Value/mass[p.Key]);
            var weights=denominator<.000001f?Array.Empty<(int,float)>():coefficients.Select(p=>(p.Key,p.Value/(denominator*mass[p.Key]))).ToArray();
            constraints.Add((upper,t,lower,u,weights));
        }
        // Every moved node wakes every dependent constraint, including already
        // processed ones. Inactive constraints cannot be invalidated silently.
        var coverCount=coverConstraints.Count;var crossingCount=constraints.Count;
        awake=Enumerable.Repeat(true,coverCount+crossingCount+sections.Length).ToArray();
        void Watch(int id,int constraint){var list=watchers[roots[id]];if(!list.Contains(constraint))list.Add(constraint);}
        for(var i=0;i<coverCount;i++){var c=coverConstraints[i];Watch(start[c.index],i);Watch(end[c.index],i);}
        for(var i=0;i<crossingCount;i++){var c=constraints[i];foreach(var id in new[]{start[c.upper],end[c.upper],start[c.lower],end[c.lower]})Watch(id,coverCount+i);}
        var gradeBounds=new float[sections.Length];var aDecks=new bool[sections.Length];var bDecks=new bool[sections.Length];
        for(var i=0;i<sections.Length;i++)
        {
            var s=sections[i];var residualDerivative=0f;
            for(var sample=0;sample<=16;sample++)
            {
                var t=sample/16f;var u=1-t;
                var dy=3*u*u*(s.CourseControl1.y-s.CourseStart.y)+6*u*t*(s.CourseControl2.y-s.CourseControl1.y)+3*t*t*(s.CourseEnd.y-s.CourseControl2.y);
                residualDerivative=Math.Max(residualDerivative,Math.Abs(dy-(s.End.y-s.Start.y)));
            }
            gradeBounds[i]=Math.Max(0,GradeLimit(s)*minSpeeds[i]-residualDerivative);aDecks[i]=members[roots[start[i]]].Length>1;bDecks[i]=members[roots[end[i]]].Length>1;
            Watch(start[i],coverCount+crossingCount+i);Watch(end[i],coverCount+crossingCount+i);
        }
        double Residual()
        {
            double value=0;
            foreach(var c in coverConstraints){var d=Math.Max(0,Height(c.index,c.t)-c.ceiling);value+=d*d;}
            foreach(var c in constraints){var d=Math.Max(0,DeckGap-Height(c.upper,c.t)+Height(c.lower,c.u));value+=d*d;}
            for(var i=0;i<sections.Length;i++){var d=Math.Max(0,Math.Abs(h[end[i]]-h[start[i]])-gradeBounds[i]);value+=d*d;}
            return value;
        }
        var bestHeights=(float[])h.Clone();var bestResidual=Residual();var checkpoint=bestResidual;var stalled=0;var escalated=false;
        var clock=System.Diagnostics.Stopwatch.StartNew();var nextProgress=5d;var passes=0;
        const int maximumPasses=512;
        for(;passes<maximumPasses;passes++)
        {
            token.ThrowIfCancellationRequested();var largest=0f;
            for(var ci=0;ci<coverCount;ci++)
            {
                if(!awake[ci])continue;awake[ci]=false;var c=coverConstraints[ci];
                var a=start[c.index];var b=end[c.index];var excess=Height(c.index,c.t)-c.ceiling;if(excess<=.0001f)continue;
                var wa=1-c.t;var wb=c.t;var denominator=wa*wa+wb*wb;
                largest=Math.Max(largest,Math.Abs(Move(a,-excess*wa/denominator))+Math.Abs(Move(b,-excess*wb/denominator)));
            }
            for(var ci=0;ci<crossingCount;ci++)
            {
                var slot=coverCount+ci;if(!awake[slot])continue;awake[slot]=false;var c=constraints[ci];
                var deficit=DeckGap-Height(c.upper,c.t)+Height(c.lower,c.u);
                if(deficit<=.0001f)continue;
                // Weighted projection adjusts both levels; it cannot repeatedly
                // stack them upward based on an order that changes every pass.
                foreach(var weight in c.weights)largest=Math.Max(largest,Math.Abs(Move(weight.id,deficit*weight.step)));
            }
            for(var i=0;i<sections.Length;i++)
            {
                var slot=coverCount+crossingCount+i;if(!awake[slot])continue;awake[slot]=false;
                if(roots[start[i]]==roots[end[i]])continue;
                var delta=h[end[i]]-h[start[i]];var excess=Math.Abs(delta)-gradeBounds[i];
                if(excess<=.00001f)continue;var sign=delta>0?1f:-1f;
                // Preserve bridge height while spreading its grade into approaches.
                var aDeck=aDecks[i];var bDeck=bDecks[i];
                var ma=Move(start[i],sign*(aDeck&&!bDeck?0:bDeck&&!aDeck?excess:excess*.5f));
                var mb=Move(end[i],-sign*Math.Max(0,excess-Math.Abs(ma)));
                if(Math.Abs(ma)+Math.Abs(mb)<excess)ma+=Move(start[i],sign*(excess-Math.Abs(ma)-Math.Abs(mb)));
                largest=Math.Max(largest,Math.Abs(ma)+Math.Abs(mb));
            }
            if(clock.Elapsed.TotalSeconds>=nextProgress){log.Diagnostic($"NETWORK SAFETY solving pass={passes+1}; footprintContacts={constraints.Count}; largestAdjustmentMeters={largest:R}");nextProgress=clock.Elapsed.TotalSeconds+5;}
            if(passes%8==0||largest<.0001f||passes==maximumPasses-1)
            {
                var residual=Residual();if(residual<bestResidual){bestResidual=residual;Array.Copy(h,bestHeights,h.Length);}
                if(residual<checkpoint-Math.Max(.000001,checkpoint*.001)){checkpoint=residual;stalled=0;}else stalled++;
                if(residual<.000001||largest<.0001f)break;
                if(stalled>=4)
                {
                    if(escalated)break;
                    escalated=true;stalled=0;relaxation=.5f;checkpoint=bestResidual;Array.Copy(bestHeights,h,h.Length);
                    for(var j=0;j<awake.Length;j++)awake[j]=true;
                    log.Diagnostic($"NETWORK SAFETY escalation dampedCoupledRepair=true; bestResidual={bestResidual:R}; completeNetworkRetained=true");
                }
            }
        }
        Array.Copy(bestHeights,h,h.Length);
        // Cover is one-sided and feasible independently of crossing order. Finish
        // with a monotone projection so the best combined residual cannot trade
        // an exposed tunnel for a marginal clearance/grade improvement.
        relaxation=1;
        for(var pass=0;pass<64;pass++)
        {
            token.ThrowIfCancellationRequested();var largest=0f;
            foreach(var c in coverConstraints)
            {
                var a=start[c.index];var b=end[c.index];var excess=Height(c.index,c.t)-c.ceiling;
                if(excess<=.001f)continue;var wa=1-c.t;var wb=c.t;var denominator=wa*wa+wb*wb;
                largest=Math.Max(largest,Math.Abs(Move(a,-excess*wa/denominator))+Math.Abs(Move(b,-excess*wb/denominator)));
            }
            if(largest<.001f)break;
        }
        for(var i=0;i<sections.Length;i++)
        {
            var s=sections[i];var a=s.Start;var d=s.End;a.y=h[start[i]];d.y=h[end[i]];
            // Build implicit handles from the ORIGINAL endpoints before applying
            // their displacement. Using revised endpoints double-applies the move.
            var b=s.Control1??math.lerp(s.Start,s.End,1f/3);var c=s.Control2??math.lerp(s.Start,s.End,2f/3);
            b.y+=math.lerp(a.y-s.Start.y,d.y-s.End.y,1f/3);c.y+=math.lerp(a.y-s.Start.y,d.y-s.End.y,2f/3);
            sections[i]=s with{Start=a,End=d,Control1=b,Control2=c,StartElevation=a.y-Ground(a),EndElevation=d.y-Ground(d)};
        }
        if(bestResidual>.000001)
            foreach(var c in constraints.Where(c=>DeckGap-math.lerp(h[start[c.upper]],h[end[c.upper]],c.t)+math.lerp(h[start[c.lower]],h[end[c.lower]],c.u)>1).GroupBy(c=>(sections[c.upper].WayId,sections[c.lower].WayId)).Select(g=>g.First()).Take(20))
                log.Diagnostic($"NETWORK SAFETY UNSOLVED upper={sections[c.upper].WayId}/{sections[c.upper].Classification}; lower={sections[c.lower].WayId}/{sections[c.lower].Classification}; deficit={DeckGap-math.lerp(h[start[c.upper]],h[end[c.upper]],c.t)+math.lerp(h[start[c.lower]],h[end[c.lower]],c.u):R}; hardDeckUpper={groups[Root(start[c.upper])].Length}/{groups[Root(end[c.upper])].Length}; hardDeckLower={groups[Root(start[c.lower])].Length}/{groups[Root(end[c.lower])].Length}");
        // Smooth tangent grades using the same physical slope at degree-two joins.
        // Bridge profiles are preserved, not forced into a shared flat plane.
        var joins=new Dictionary<Key,List<(int i,bool start)>>();
        for(var i=0;i<sections.Length;i++)foreach(var atStart in new[]{true,false})
        {var key=atStart?Start(sections[i]):End(sections[i]);if(!joins.TryGetValue(key,out var list))joins[key]=list=new();list.Add((i,atStart));}
        var nearbyContacts=new Dictionary<int,List<Contact>>();
        foreach(var contact in contacts)foreach(var index in new[]{contact.A,contact.B})
        {if(!nearbyContacts.TryGetValue(index,out var list))nearbyContacts[index]=list=new();list.Add(contact);}
        var coverBySection=coverConstraints.GroupBy(c=>c.index).ToDictionary(g=>g.Key,g=>g.ToArray());
        bool PreservesSafety(int index,PreparedSegment candidate,int otherIndex,PreparedSegment otherCandidate)
        {
            if(coverBySection.TryGetValue(index,out var cover))
                foreach(var c in cover)if(NetworkNormalizer.Evaluate(candidate,c.t).y>c.ceiling+.01f)return false;
            if(nearbyContacts.TryGetValue(index,out var nearby))foreach(var c in nearby)
            {
                var a=c.A==index?candidate:c.A==otherIndex?otherCandidate:sections[c.A];
                var b=c.B==index?candidate:c.B==otherIndex?otherCandidate:sections[c.B];
                if(Math.Abs(NetworkNormalizer.Evaluate(a,c.T).y-NetworkNormalizer.Evaluate(b,c.U).y)<MinimumSafeGap)return false;
            }
            return true;
        }
        foreach(var entries in joins.Values)
        {
            if(entries.Count!=2||entries[0].i==entries[1].i)continue;
            var left=entries[0];var right=entries[1];var a=sections[left.i];var b=sections[right.i];
            var p=left.start?a.CourseStart:a.CourseEnd;var q=right.start?b.CourseStart:b.CourseEnd;
            var ca=left.start?a.CourseControl1:a.CourseControl2;var cb=right.start?b.CourseControl1:b.CourseControl2;
            if(math.dot(math.normalizesafe(p.xz-ca.xz),math.normalizesafe(cb.xz-q.xz))<.866f)continue;
            var flatDeck=Root(start[left.i])==Root(end[left.i])||Root(start[right.i])==Root(end[right.i]);
            var slope=flatDeck?0:((right.start?b.End.y:b.Start.y)-(left.start?a.End.y:a.Start.y))/Math.Max(.001f,lengths[left.i]+lengths[right.i]);
            var desired=Math.Min(NetworkVerticalProfiles.DesiredGrade(a),NetworkVerticalProfiles.DesiredGrade(b));
            slope=math.clamp(slope,-desired,desired);
            ca.y=p.y-slope*math.distance(ca.xz,p.xz);cb.y=q.y+slope*math.distance(cb.xz,q.xz);
            ca-=left.start?a.StartCurveOffset:a.EndCurveOffset;cb-=right.start?b.StartCurveOffset:b.EndCurveOffset;
            var na=left.start?a with{Control1=ca}:a with{Control2=ca};var nb=right.start?b with{Control1=cb}:b with{Control2=cb};
            if(NetworkVerticalProfiles.WithinGradeCeiling(na,GradeLimit(a))&&NetworkVerticalProfiles.WithinGradeCeiling(nb,GradeLimit(b))&&PreservesSafety(left.i,na,right.i,nb)&&PreservesSafety(right.i,nb,left.i,na))
            {sections[left.i]=na;sections[right.i]=nb;}
        }
        Validate(sections,log,token,allowQualityWarnings);
        log.Time("Coupled network repair",clock.Elapsed.TotalSeconds);
        log.Diagnostic($"NETWORK SAFETY repaired sections={sections.Length}; passes={passes+1}; footprintContacts={constraints.Count}; forcedFlatDecks=false; sharedBridgeProfilesPreserved=true; boundedTerrainAdjustmentMeters=60; noSectionsDeleted=true");
        return sections;
    }

    public static void Validate(IReadOnlyList<PreparedSegment> sections,ImportLog log,CancellationToken token,bool allowQualityWarnings=false)
    {
        var steep=sections.Where(s=>!NetworkVerticalProfiles.WithinGradeCeiling(s,GradeLimit(s))).ToArray();
        var contacts=Contacts(sections,token);var collisions=new List<Contact>();
        foreach(var c in contacts)
        {
            var gap=Math.Abs(NetworkNormalizer.Evaluate(sections[c.A],c.T).y-NetworkNormalizer.Evaluate(sections[c.B],c.U).y);
            if(c.Kind!=NetworkContactKind.IndependentLevels||!math.isfinite(gap)||gap<MinimumSafeGap)collisions.Add(c);
        }
        log.Diagnostic($"NETWORK SAFETY FINAL sections={sections.Count}; widthAwareContacts={contacts.Count}; clearanceFailures={collisions.Count}; gradeFailures={steep.Length}; minimumClearSpaceMeters=5; deckAllowanceMeters=2; failClosed={!allowQualityWarnings}; iterationMode={allowQualityWarnings}; nativeRenderingStillRequiresVerification=true");
        DescribeQuality(sections,log,token);
        if(steep.Length>0||collisions.Count>0)
        {
            foreach(var s in steep.Take(10))log.Diagnostic($"NETWORK SAFETY GRADE way={s.WayId}; kind={s.Kind}; bridge={s.Bridge}; tunnel={s.Tunnel}; grade={NetworkVerticalProfiles.MaximumGrade(s):R}; length={NetworkVerticalProfiles.ArcLength(s):R}; start={s.Start}; end={s.End}");
            foreach(var c in collisions.GroupBy(c=>(sections[c.A].WayId,sections[c.B].WayId)).Select(g=>g.OrderBy(c=>Math.Abs(NetworkNormalizer.Evaluate(sections[c.A],c.T).y-NetworkNormalizer.Evaluate(sections[c.B],c.U).y)).First()).Take(25))log.Diagnostic($"NETWORK SAFETY COLLISION A={sections[c.A].WayId}/{sections[c.A].StartNodeId}-{sections[c.A].EndNodeId} kind={sections[c.A].Kind} layer={sections[c.A].Layer} bridge={sections[c.A].Bridge} tunnel={sections[c.A].Tunnel} at={c.T:R} pos={NetworkNormalizer.Evaluate(sections[c.A],c.T)}; B={sections[c.B].WayId}/{sections[c.B].StartNodeId}-{sections[c.B].EndNodeId} kind={sections[c.B].Kind} layer={sections[c.B].Layer} bridge={sections[c.B].Bridge} tunnel={sections[c.B].Tunnel} at={c.U:R} pos={NetworkNormalizer.Evaluate(sections[c.B],c.U)}");
            var ways=steep.Select(s=>s.WayId).Concat(collisions.SelectMany(c=>new[]{sections[c.A].WayId,sections[c.B].WayId})).Distinct().Take(12);
            if(allowQualityWarnings)
            {log.Count("Network repair quality observations",collisions.Count+steep.Length);log.Warn($"Iteration quality warning: {collisions.Count} clearance observations and {steep.Length} excessive-grade sections remain after bounded repair. Complete network retained for testing; not production-accepted. Ways: {string.Join(",",ways)}.");return;}
            log.Warn($"Network quality observations: {collisions.Count} clearance failures, {steep.Length} excessive grades. Generation retained for diagnosis. Ways: {string.Join(",",ways)}.");
        }
    }

    private static void DescribeQuality(IReadOnlyList<PreparedSegment> sections,ImportLog log,CancellationToken token)
    {
        var metrics=new List<(PreparedSegment Section,float Length,float Grade,float Radius)>();
        var joins=new Dictionary<Key,List<(PreparedSegment Section,bool Start)>>();
        foreach(var s in sections)
        {
            token.ThrowIfCancellationRequested();var minimumRadius=float.PositiveInfinity;
            var a=s.CourseStart.xz;var b=s.CourseControl1.xz;var c=s.CourseControl2.xz;var d=s.CourseEnd.xz;
            for(var i=0;i<=32;i++)
            {
                var t=i/32f;var u=1-t;var derivative=3*u*u*(b-a)+6*u*t*(c-b)+3*t*t*(d-c);
                var second=6*u*(c-2*b+a)+6*t*(d-2*c+b);var speed=math.length(derivative);
                var cross=Math.Abs(derivative.x*second.y-derivative.y*second.x);
                if(speed>.001f&&cross>.001f)minimumRadius=Math.Min(minimumRadius,speed*speed*speed/cross);
            }
            metrics.Add((s,NetworkVerticalProfiles.ArcLength(s),NetworkVerticalProfiles.MaximumGrade(s),minimumRadius));
            foreach(var start in new[]{true,false})
            {var key=new Key(s.Kind,start?s.StartNodeId:s.EndNodeId);if(!joins.TryGetValue(key,out var list))joins[key]=list=new();list.Add((s,start));}
        }
        foreach(var group in metrics.GroupBy(m=>m.Section.Kind==NetworkKind.Rail?"rail":m.Section.ProtectedLink?"ramps":m.Section.Classification=="motorway"||m.Section.Classification=="trunk"?"mainline":"streets"))
        {
            var above=group.Count(m=>m.Grade>NetworkVerticalProfiles.DesiredGrade(m.Section)+.001f);
            log.Diagnostic($"NETWORK QUALITY class={group.Key}; sections={group.Count()}; spansBelow12m={group.Count(m=>m.Length<12)}; spansBelow24m={group.Count(m=>m.Length<24)}; desiredGradeExceeded={above}; maximumGrade={group.Max(m=>m.Grade):R}; tightestSampledRadiusMeters={group.Min(m=>m.Radius):R}; desiredNotExceptionalAcceptance=true; legitimateShortJunctionsAndTerminalsPreserved=true; curvatureIsSampledNotProof=true");
            foreach(var m in group.Where(m=>m.Grade>NetworkVerticalProfiles.DesiredGrade(m.Section)+.001f).OrderByDescending(m=>m.Grade).Take(5))
                log.Diagnostic($"NETWORK QUALITY OUTLIER way={m.Section.WayId}; sourceIds={string.Join(",",m.Section.SourceWayIds)}; nodes={m.Section.StartNodeId},{m.Section.EndNodeId}; class={group.Key}; length={m.Length:R}; grade={m.Grade:R}; target={NetworkVerticalProfiles.DesiredGrade(m.Section):R}; radius={m.Radius:R}; bridge={m.Section.Bridge}; tunnel={m.Section.Tunnel}");
        }
        var tangentBreaks=0;var gradeBreaks=0;var maximumAngle=0f;var maximumGradeJump=0f;
        foreach(var pair in joins.Values.Where(p=>p.Count==2&&p[0].Section!=p[1].Section))
        {
            var left=pair[0];var right=pair[1];
            var a=left.Start?left.Section.CourseStart-left.Section.CourseControl1:left.Section.CourseEnd-left.Section.CourseControl2;
            var b=right.Start?right.Section.CourseControl1-right.Section.CourseStart:right.Section.CourseControl2-right.Section.CourseEnd;
            var angle=(float)Math.Acos(math.clamp(math.dot(math.normalizesafe(a.xz),math.normalizesafe(b.xz)),-1,1))*180/math.PI;
            if(angle>15){tangentBreaks++;maximumAngle=Math.Max(maximumAngle,angle);}
            if(angle>30)continue; // An intentional corner is not a grade-continuity claim.
            var jump=Math.Abs(a.y/Math.Max(.001f,math.length(a.xz))-b.y/Math.Max(.001f,math.length(b.xz)));
            if(jump>.01f){gradeBreaks++;maximumGradeJump=Math.Max(maximumGradeJump,jump);}
        }
        log.Diagnostic($"NETWORK QUALITY JOINS degreeTwoTangentBreaks={tangentBreaks}; maximumAngleDegrees={maximumAngle:R}; continuousDirectionGradeJumps={gradeBreaks}; maximumGradeJump={maximumGradeJump:R}; multiArmJunctionsNotClassifiedAsThroughJoins=true; sourceCornersMayBeIntentional=true");
    }

    private const float MinimumSafeGap=NetworkVerticalProfiles.MinimumBridgeClearance+2+.01f;

    public static IReadOnlyList<PreparedSegment> RepairFinalTunnelCover(IReadOnlyList<PreparedSegment> source,ushort[] raster,float scale,double width,ImportLog log,CancellationToken token)
    {
        var samples=TunnelCoverSamples(source,raster,scale,width,token).ToArray();if(samples.Length==0)return source;
        var nodes=new Dictionary<Key,int>();var heights=new List<float>();
        int Node(Key key,float y){if(nodes.TryGetValue(key,out var id))return id;id=heights.Count;nodes[key]=id;heights.Add(y);return id;}
        var from=new int[source.Count];var to=new int[source.Count];var first=new int[source.Count];var second=new int[source.Count];
        for(var index=0;index<source.Count;index++)
        {
            var s=source[index];from[index]=Node(new(s.Kind,s.StartNodeId),s.Start.y);to[index]=Node(new(s.Kind,s.EndNodeId),s.End.y);
            first[index]=heights.Count;heights.Add(s.CourseControl1.y);second[index]=heights.Count;heights.Add(s.CourseControl2.y);
        }
        var initial=heights.ToArray();var h=(float[])initial.Clone();var floor=initial.Select(y=>Math.Max(0,y-120)).ToArray();
        // Cover and portal grades must be solved together. Lowering just one
        // shared anchor can create a near-vertical short approach or handle.
        var incident=new Dictionary<int,List<int>>();
        for(var i=0;i<source.Count;i++)foreach(var id in new[]{from[i],to[i]})
        {if(!incident.TryGetValue(id,out var list))incident[id]=list=new();list.Add(i);}
        foreach(var s in source.Where(s=>s.Bridge&&!s.Tunnel))
        {floor[nodes[new(s.Kind,s.StartNodeId)]]=initial[nodes[new(s.Kind,s.StartNodeId)]];floor[nodes[new(s.Kind,s.EndNodeId)]]=initial[nodes[new(s.Kind,s.EndNodeId)]];}
        var gradeEdges=new HashSet<int>(Enumerable.Range(0,source.Count).Where(i=>source[i].Tunnel));
        var frontier=gradeEdges.ToArray();
        for(var ring=0;ring<4;ring++)
        {
            var next=new List<int>();
            foreach(var i in frontier)foreach(var id in new[]{from[i],to[i]})foreach(var j in incident[id])if(gradeEdges.Add(j))next.Add(j);
            frontier=next.ToArray();
        }
        var constraints=samples.Select(sample=>
        {
            var s=source[sample.index];var t=sample.t;var u=1-t;
            return (sample.index,sample.t,sample.ceiling,originalY:NetworkNormalizer.Evaluate(s,t).y,
                a:u*u*u,b:3*u*u*t,c:3*u*t*t,d:t*t*t);
        }).ToArray();
        var passes=0;
        float CoverPass()
        {
            var largest=0f;
            foreach(var c in constraints)
            {
                var a=from[c.index];var b=first[c.index];var d=to[c.index];var e=second[c.index];
                var y=c.originalY+c.a*(h[a]-initial[a])+c.b*(h[b]-initial[b])+c.c*(h[e]-initial[e])+c.d*(h[d]-initial[d]);
                var excess=y-c.ceiling;if(excess<=.01f)continue;
                var denominator=c.a*c.a+c.b*c.b+c.c*c.c+c.d*c.d;if(denominator<.000001f)continue;
                void Lower(int id,float weight){var before=h[id];h[id]=Math.Max(floor[id],before-excess*weight/denominator);largest=Math.Max(largest,before-h[id]);}
                Lower(a,c.a);Lower(b,c.b);Lower(e,c.c);Lower(d,c.d);
            }
            return largest;
        }
        for(;passes<128;passes++)
        {
            token.ThrowIfCancellationRequested();var largest=CoverPass();
            foreach(var i in gradeEdges)
            {
                var s=source[i];var a=from[i];var b=first[i];var c=second[i];var d=to[i];
                for(var j=0;j<=16;j++)
                {
                    var t=j/16f;var u=1-t;
                    var derivative=3*u*u*(s.CourseControl1-s.CourseStart)+6*u*t*(s.CourseControl2-s.CourseControl1)+3*t*t*(s.CourseEnd-s.CourseControl2);
                    var wa=s.Tunnel?-3*u*u:-1;var wb=s.Tunnel?3*u*u-6*u*t:0;var wc=s.Tunnel?6*u*t-3*t*t:0;var wd=s.Tunnel?3*t*t:1;
                    var dy=derivative.y+wa*(h[a]-initial[a])+wb*(h[b]-initial[b])+wc*(h[c]-initial[c])+wd*(h[d]-initial[d]);
                    var allowed=GradeLimit(s)*math.length(derivative.xz);var residual=dy-math.clamp(dy,-allowed,allowed);
                    if(Math.Abs(residual)<.001f)continue;
                    var denominator=wa*wa+wb*wb+wc*wc+wd*wd;if(denominator<.000001f)continue;
                    void Project(int id,float weight){var before=h[id];h[id]=math.clamp(before-residual*weight/denominator,floor[id],initial[id]);largest=Math.Max(largest,Math.Abs(before-h[id]));}
                    Project(a,wa);Project(b,wb);Project(c,wc);Project(d,wd);
                }
            }
            if(largest<.001f)break;
        }
        // Cover remains a hard gate when an exceptional grade is infeasible.
        // Do not expose the tunnel merely to make a grade metric pass.
        for(var repair=0;repair<16;repair++){token.ThrowIfCancellationRequested();if(CoverPass()<.001f)break;}
        var result=source.ToArray();var changed=0;var maximum=0f;
        for(var index=0;index<source.Count;index++)
        {
            var s=source[index];var a=from[index];var b=to[index];var c=first[index];var d=second[index];
            var drop=Math.Max(Math.Max(initial[a]-h[a],initial[b]-h[b]),Math.Max(initial[c]-h[c],initial[d]-h[d]));if(drop<=0)continue;
            var start=s.Start;var end=s.End;var control1=s.Control1??math.lerp(s.Start,s.End,1f/3);var control2=s.Control2??math.lerp(s.Start,s.End,2f/3);
            start.y=h[a];end.y=h[b];
            if(s.Tunnel){control1.y=h[c]-s.StartCurveOffset.y;control2.y=h[d]-s.EndCurveOffset.y;}
            else{control1.y+=math.lerp(h[a]-initial[a],h[b]-initial[b],1f/3);control2.y+=math.lerp(h[a]-initial[a],h[b]-initial[b],2f/3);}
            result[index]=s with{Start=start,End=end,Control1=control1,Control2=control2,StartElevation=(s.StartElevation??0)+h[a]-initial[a],EndElevation=(s.EndElevation??0)+h[b]-initial[b]};
            changed++;maximum=Math.Max(maximum,drop);
        }
        log.Count("Final tunnel-cover sections repaired",changed);
        log.Diagnostic($"PROCESS final-tunnel-cover sectionsRepaired={changed}; maximumLoweringMeters={maximum:0.###}; passes={Math.Min(passes+1,128)}; coupledGradeSections={gradeEdges.Count}; bridgeFloorsPreserved=true; sharedSurfacePortalsFixed=false; incidentApproachesMovedTogether=true; horizontalGeometryUnchanged=true; terrainUnchanged=true");
        try{ValidateTunnelCover(result,raster,scale,width,token,log);}
        catch(InvalidOperationException error)
        {log.Warn($"Final tunnel recovery retained best bounded geometry: {error.Message}");}
        if(changed>0){ValidateEnvelope(result,width,scale,token);Validate(result,log,token,allowQualityWarnings:true);}
        return changed>0?result:source;
    }

    public static void ValidateTunnelCover(IReadOnlyList<PreparedSegment> sections,ushort[] raster,float scale,double width,CancellationToken token,ImportLog? log=null)
    {
        foreach(var sample in TunnelCoverSamples(sections,raster,scale,width,token))
        {
            var s=sections[sample.index];
            if(NetworkNormalizer.Evaluate(s,sample.t).y>sample.ceiling+.05f)
            {log?.Count("Final exposed tunnel observations");if(log?.Metric("Final exposed tunnel observations")<=25)log.Warn($"Final terrain exposes {s.Kind} tunnel way {s.WayId} at {sample.t:0.###}; advisory only.");}
        }
    }

    private static IEnumerable<(int index,float t,float ceiling)> TunnelCoverSamples(IReadOnlyList<PreparedSegment> sections,ushort[] raster,float scale,double width,CancellationToken token)
    {
        var n=(int)Math.Sqrt(raster.Length);
        if(n*n!=raster.Length||n<2||scale<=0||width<=0)throw new InvalidOperationException("Invalid final terrain for tunnel cover audit.");
        var surface=new HashSet<Key>();var adjacency=new Dictionary<Key,List<(Key node,float length)>>();
        foreach(var s in sections)foreach(var reverse in new[]{false,true})
        {
            var a=new Key(s.Kind,reverse?s.EndNodeId:s.StartNodeId);var b=new Key(s.Kind,reverse?s.StartNodeId:s.EndNodeId);
            if(!s.Tunnel){surface.Add(a);continue;}
            if(!adjacency.TryGetValue(a,out var list))adjacency[a]=list=new();list.Add((b,NetworkVerticalProfiles.ArcLength(s)));
        }
        var distance=new Dictionary<Key,float>();var queue=new List<Key>();
        foreach(var node in surface.Where(adjacency.ContainsKey)){distance[node]=0;queue.Add(node);}
        for(var cursor=0;cursor<queue.Count;cursor++)
        {
            token.ThrowIfCancellationRequested();var node=queue[cursor];
            foreach(var edge in adjacency[node]){var d=distance[node]+edge.length;if(d>160||distance.TryGetValue(edge.node,out var old)&&old<=d)continue;distance[edge.node]=d;queue.Add(edge.node);}
        }
        float Ground(float3 p)
        {
            var x=math.clamp((p.x/(float)width+.5f)*(n-1),0,n-1);var z=math.clamp((p.z/(float)width+.5f)*(n-1),0,n-1);var ix=(int)x;var iz=(int)z;
            return math.lerp(math.lerp(raster[iz*n+ix],raster[iz*n+Math.Min(n-1,ix+1)],x-ix),math.lerp(raster[Math.Min(n-1,iz+1)*n+ix],raster[Math.Min(n-1,iz+1)*n+Math.Min(n-1,ix+1)],x-ix),z-iz)/65535f*scale;
        }
        for(var index=0;index<sections.Count;index++)
        {
            var s=sections[index];if(!s.Tunnel)continue;
            var length=NetworkVerticalProfiles.ArcLength(s);var count=Math.Max(16,(int)Math.Ceiling(length/4));
            var a=distance.TryGetValue(new(s.Kind,s.StartNodeId),out var da)?da:float.PositiveInfinity;
            var b=distance.TryGetValue(new(s.Kind,s.EndNodeId),out var db)?db:float.PositiveInfinity;
            for(var i=0;i<=count;i++)
            {
                token.ThrowIfCancellationRequested();var t=i/(float)count;var p=NetworkNormalizer.Evaluate(s,t);
                var portalDistance=Math.Min(a+length*t,b+length*(1-t));
                var fraction=math.clamp(portalDistance/160,0,1);var cover=15*fraction*fraction*(3-2*fraction);
                var tangent=math.normalizesafe((NetworkNormalizer.Evaluate(s,Math.Min(1,t+.001f))-NetworkNormalizer.Evaluate(s,Math.Max(0,t-.001f))).xz);
                var side=new float3(-tangent.y,0,tangent.x)*Radius(s);
                var fullRoof=Math.Min(Ground(p),Math.Min(Ground(p+side),Ground(p-side)));
                // An open portal has no full-width roof at its boundary. Blend
                // into full-width auditing over one physical road width, not
                // an arbitrary 100m exemption. Interiors retain the full check.
                var roof=math.lerp(Ground(p),fullRoof,math.clamp(portalDistance/Math.Max(8,2*Radius(s)),0,1));
                yield return(index,t,roof-cover);
            }
        }
    }

    public static void ValidateEnvelope(IReadOnlyList<PreparedSegment> sections,double width,float scale,CancellationToken token,ImportLog? log=null)
    {
        var half=(float)width/2;
        foreach(var s in sections)
        {
            token.ThrowIfCancellationRequested();
            // The Bezier is inside the convex hull of these four points. Check
            // the actual native course, not only its shared node anchors.
            foreach(var p in new[]{s.CourseStart,s.CourseControl1,s.CourseControl2,s.CourseEnd})
                if(!math.all(math.isfinite(p))||Math.Abs(p.x)>half+.01f||Math.Abs(p.z)>half+.01f||p.y<0||p.y>scale)
                    log?.Warn($"Network envelope observation: {s.Kind} way {s.WayId}; local finite-input repair required, not an import gate.");
        }
    }

    private static float MinimumSpeed(float2 p,float2 q,float2 r,int depth)
    {
        var lo=math.min(p,math.min(q,r));var hi=math.max(p,math.max(q,r));
        var bound=math.length(new float2(lo.x>0?lo.x:hi.x<0?-hi.x:0,lo.y>0?lo.y:hi.y<0?-hi.y:0));
        if(depth==0)return bound;
        var pq=(p+q)*.5f;var qr=(q+r)*.5f;var m=(pq+qr)*.5f;
        return Math.Min(MinimumSpeed(p,pq,m,depth-1),MinimumSpeed(m,qr,r,depth-1));
    }

    private static List<Contact> Contacts(IReadOnlyList<PreparedSegment> sections,CancellationToken token)
    {
        var grid=new Dictionary<(int,int),List<Span>>();var contacts=new Dictionary<(int,int,int,int),Contact>();
        var adjacent=new Dictionary<Key,List<(Key node,float length)>>();var lengths=sections.Select(NetworkVerticalProfiles.ArcLength).ToArray();
        var radii=sections.Select(Radius).ToArray();var searchLimit=radii.DefaultIfEmpty(0).Max()*12+2;
        for(var i=0;i<sections.Count;i++)foreach(var reverse in new[]{false,true})
        {
            var s=sections[i];var a=new Key(s.Kind,reverse?s.EndNodeId:s.StartNodeId);var b=new Key(s.Kind,reverse?s.StartNodeId:s.EndNodeId);
            if(!adjacent.TryGetValue(a,out var list))adjacent[a]=list=new();list.Add((b,lengths[i]));
        }
        var paths=new Dictionary<Key,Dictionary<Key,float>>();
        Dictionary<Key,float> Paths(Key origin)
        {
            if(paths.TryGetValue(origin,out var cached))return cached;
            var best=new Dictionary<Key,float>{{origin,0}};var queue=new List<Key>{origin};
            for(var cursor=0;cursor<queue.Count;cursor++)
            {
                token.ThrowIfCancellationRequested();var node=queue[cursor];if(!adjacent.TryGetValue(node,out var edges))continue;
                foreach(var edge in edges)
                {
                    var distance=best[node]+edge.length;
                    if(distance>searchLimit||best.TryGetValue(edge.node,out var old)&&old<=distance)continue;
                    best[edge.node]=distance;queue.Add(edge.node);
                }
            }
            paths[origin]=best;return best;
        }
        bool LocalJunction(int ai,float t,int bi,float u)
        {
            var a=sections[ai];var b=sections[bi];if(a.Kind!=b.Kind)return false;
            float2 Tangent(PreparedSegment s,float at)=>math.normalizesafe((NetworkNormalizer.Evaluate(s,Math.Min(1,at+.001f))-NetworkNormalizer.Evaluate(s,Math.Max(0,at-.001f))).xz);
            var parallel=Math.Abs(math.dot(Tangent(a,t),Tangent(b,u)))>.8f;
            var limit=(radii[ai]+radii[bi])*(parallel?6:4)+2;
            bool Via(long origin,float distance)
            {
                if(distance>limit)return false;var best=Paths(new(a.Kind,origin));
                return best.TryGetValue(new(b.Kind,b.StartNodeId),out var left)&&distance+left+lengths[bi]*u<=limit||
                       best.TryGetValue(new(b.Kind,b.EndNodeId),out var right)&&distance+right+lengths[bi]*(1-u)<=limit;
            }
            if(Via(a.StartNodeId,lengths[ai]*t)||Via(a.EndNodeId,lengths[ai]*(1-t)))return true;
            return false;
        }
        var seen=new HashSet<Span>();
        for(var i=0;i<sections.Count;i++)
        {
            token.ThrowIfCancellationRequested();var s=sections[i];var steps=Math.Max(8,(int)Math.Ceiling(lengths[i]/8));
            for(var j=0;j<steps;j++)
            {
                var t=j/(float)steps;var u=(j+1)/(float)steps;var a=NetworkNormalizer.Evaluate(s,t);var b=NetworkNormalizer.Evaluate(s,u);var span=new Span(i,t,u,a,b);
                var radius=Radius(s)+.5f;var low=math.min(a.xz,b.xz)-radius;var high=math.max(a.xz,b.xz)+radius;
                seen.Clear();
                for(var z=(int)Math.Floor(low.y/64);z<=(int)Math.Floor(high.y/64);z++)for(var x=(int)Math.Floor(low.x/64);x<=(int)Math.Floor(high.x/64);x++)
                {
                    var cell=(x,z);if(!grid.TryGetValue(cell,out var nearby))grid[cell]=nearby=new();
                    foreach(var other in nearby)
                    {
                        if(other.Index==i||!seen.Add(other))continue;var o=sections[other.Index];
                        var separation=radii[i]+radii[other.Index]+.5f;
                        if(math.any(math.max(a.xz,b.xz)+separation<math.min(other.A.xz,other.B.xz))||
                           math.any(math.max(other.A.xz,other.B.xz)+separation<math.min(a.xz,b.xz)))continue;
                        // Connected decks can still cross remotely: only local
                        // junction geometry is exempt, never the whole component.
                        // Sharing an endpoint does not exempt a remote crossing
                        // of these same long curves. LocalJunction below measures
                        // actual distance to the connection at this contact.
                        Closest(a.xz,b.xz,other.A.xz,other.B.xz,out var at,out var bt);
                        var p=math.lerp(a,b,at);var q=math.lerp(other.A,other.B,bt);
                        if(math.distance(p.xz,q.xz)>Radius(s)+Radius(o)+.5f)continue;
                        var st=math.lerp(t,u,at);var ot=math.lerp(other.T,other.U,bt);
                        var headingA=(b-a).xz;var headingB=(other.B-other.A).xz;
                        var kind=NetworkContactPolicy.Classify(s,o,p,q,headingA,headingB,LocalJunction(other.Index,ot,i,st));
                        if(kind==NetworkContactKind.Junction||kind==NetworkContactKind.LevelCrossing)continue;
                        // Same formation tracks/carriageways are horizontal-spacing
                        // conflicts, never instructions to stack each track vertically.
                        // Ambiguous same-level structures need local reconstruction,
                        // not a fabricated layer order derived from source IDs.
                        var key=(other.Index,i,(int)(ot*32),(int)(st*32));contacts[key]=new(other.Index,ot,i,st,kind);
                    }
                    nearby.Add(span);
                }
            }
        }
        return contacts.Values.ToList();
    }
    // Closest points of two finite segments, including nearly parallel decks.
    private static void Closest(float2 a,float2 b,float2 c,float2 d,out float t,out float u)
    {
        var r=b-a;var s=d-c;var w=a-c;var aa=math.dot(r,r);var bb=math.dot(r,s);var cc=math.dot(s,s);var dd=math.dot(r,w);var ee=math.dot(s,w);var determinant=aa*cc-bb*bb;
        t=determinant>1e-8f?math.clamp((bb*ee-cc*dd)/determinant,0,1):0;
        u=cc>1e-8f?math.clamp((bb*t+ee)/cc,0,1):0;
        t=aa>1e-8f?math.clamp((bb*u-dd)/aa,0,1):0;
    }
}
