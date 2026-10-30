using System;
using System.Collections.Generic;
using System.Linq;
using System.Threading;
using ImportEarthLocation.Core;
using Unity.Mathematics;

namespace ImportEarthLocation.Processing;

// Visual network compiler, horizontal stage (research report 64 §5.1–5.2, §5.5).
// Builds each perceived road once: short junction connectors are contracted,
// node-to-node sections are chained into strokes by good continuation, each
// stroke is simplified (Douglas–Peucker) and rebuilt from straights and circular
// fillets, then emitted as merged cubics. Junction nodes and topology ports keep
// their source identities; absorbed nodes become aliases. Coordinates are local
// map metres (XZ). Emission waits for VisualCompilerProfile to solve heights for
// every stroke together.
public static class VisualNetworkCompiler
{
    private const double MaximumSpan=400;
    // How far along a stroke end the fork clearance acts (metres).
    private const double ForkReach=300;
    private const long GeneratedNodeBase=long.MinValue+8000000;
    // Shortest section the compiler keeps between two distinct nodes (metres).
    private const double MicroLength=1.5;
    // Longest section contracted when it doubles back at a through node (metres).
    private const double FoldLength=15;
    // Angle (degrees) at which a one-way road without lanes of its own joins or leaves another.
    private const double MergeAngle=30;
    // A two-way street branch needs room for curve fitting while retaining
    // at least the junction's 25-degree opening after its approach is rounded.
    private const double StreetBranchAngle=35;
    // Angle for a branch of two or more lanes (3 lanes → 2 + 2, as the A55
    // splits into its 2-lane link before its tunnel): at 30° the two branches
    // still read as one widened road with an artifact at the split. Applied
    // to single-lane ramps too, 40° added 390 tight radii in DC.
    private const double SplitAngle=40;
    private static bool Splits(PreparedSegment s)=>s.Lanes>=2;
    // Largest radius (metres) of a ramp's turn back to parallel at its
    // clearance. At a motorway's 250 m floor a branch at SplitAngle needed
    // 75 m of offset to straighten, veering out and back in an S before the
    // A55's tunnel. The turn is fitted inside the clearance: a corner set
    // wider (to keep a 20 m straight start) was a hump once the ramp came
    // back in to its clearance.
    private const double TurnBackRadius=50;
    // How far a merging ramp is held at its clearance beside the through
    // road while its source stays inside it (metres), and the length over
    // which a hold cut short by a fixed point fades out.
    private const double HoldReach=600;
    private const double HoldFade=60;
    // Longest rail section on either side of a plain node straightened as a jog (metres).
    private const double JogLength=30;
    // Widest map-data break between two dead ends that is joined (metres).
    private const double MappingGap=3;
    // Height above the original ground (metres) beyond which a bridge approach
    // is built elevated rather than as a ground piece.
    private const float ElevateHeight=4;
    private readonly record struct Arm(int Section,bool AtStart);
    private readonly record struct Piece(double2 P0,double2 P1,double2 P2,double2 P3);
    private readonly record struct Cut(int Piece,long Node,int StrokeNode);

    // water: the water planner's surface sampler; sea: effective sea level (NaN
    // when unknown). Both keep bridge decks clear of the water they cross.
    public static IReadOnlyList<PreparedSegment> Prepare(IReadOnlyList<PreparedSegment> source,ushort[] terrain,float scale,double width,
        ImportLog log,CancellationToken token,Func<float3,float?>? water=null,float sea=float.NaN)
    {
        // Same rail formation and turnout-footprint owners as the legacy design;
        // they resolve terminal aliases before strokes are built.
        source=NetworkRailTerminalFormations.Prepare(source,log,token);
        source=RailCorridorConsolidator.NativeJunctionFootprints(source,log,token);
        var structureSource=source;var structurePorts=SourceTopologyObligations.Capture(source).Ports;
        source=BridgeGaps(source,log,token);
        // Topology ports and every earlier alias target must survive as nodes.
        var ports=new HashSet<SourceNodeKey>(SourceTopologyObligations.Capture(source).Ports);
        foreach(var s in source)foreach(var alias in s.SourceNodeAliases)ports.Add(new(s.Kind,alias.CanonicalNodeId));
        var retiredStructurePorts=new HashSet<SourceNodeKey>(structurePorts.Where(p=>p.Kind==NetworkKind.Road&&!ports.Contains(p)));
        var structureAliases=new Dictionary<long,long>();
        SourceNodeKey A(PreparedSegment s)=>new(s.Kind,s.StartNodeId);
        SourceNodeKey B(PreparedSegment s)=>new(s.Kind,s.EndNodeId);
        var position=new Dictionary<SourceNodeKey,double2>();var degree=new Dictionary<SourceNodeKey,int>();var fixedNodes=new HashSet<SourceNodeKey>();
        foreach(var s in source)
        {
            position[A(s)]=((float3)s.CourseStart).xz;position[B(s)]=((float3)s.CourseEnd).xz;
            foreach(var key in new[]{A(s),B(s)}){degree[key]=degree.TryGetValue(key,out var d)?d+1:1;if(s.Roundabout)fixedNodes.Add(key);}
        }

        // Contract short connectors between junctions (report 64 V1 step 2).
        var parent=new Dictionary<SourceNodeKey,SourceNodeKey>();
        SourceNodeKey Root(SourceNodeKey k){while(parent.TryGetValue(k,out var p)&&p!=k)k=parent[k]=parent.TryGetValue(p,out var q)?q:p;return k;}
        var contracted=new bool[source.Count];
        // A section shorter than MicroLength is one node drawn twice (a portal or
        // tag change beside a junction). The game cannot build a usable edge that
        // short, so its ends become one node, unless another section joins the
        // same two (merged) nodes and would become a loop.
        var micro=Enumerable.Range(0,source.Count).Where(i=>!source[i].Roundabout&&A(source[i])!=B(source[i])&&!fixedNodes.Contains(A(source[i]))&&
            !fixedNodes.Contains(B(source[i]))&&NetworkDesignChecks.HorizontalArcLength(Curve(source[i]))<MicroLength).ToArray();
        foreach(var i in micro)
        {
            token.ThrowIfCancellationRequested();var a=Root(A(source[i]));var b=Root(B(source[i]));if(a==b)continue;
            var loop=false;
            for(var j=0;j<source.Count&&!loop;j++)
                if(j!=i&&!contracted[j]){var x=Root(A(source[j]));var y=Root(B(source[j]));loop=x==a&&y==b||x==b&&y==a;}
            if(loop)continue;
            parent[b]=a;contracted[i]=true;
        }
        // A short section that doubles back at a plain through node (its neighbour
        // there leaves within 60° of it) is a fold, not a route: rail
        // consolidation re-attaching a tail can end a line 4 m past where the
        // next piece starts, a 178° kink that reads as a break. Its ends become
        // one node, under the same loop guard.
        var folds=0;
        double2 Leaving(PreparedSegment s,bool start){var c=Curve(s);var p=start?c.P0:c.P3;return math.normalizesafe(NetworkDesignChecks.Evaluate(c,start?.25:.75).xz-p.xz);}
        var ends=new Dictionary<SourceNodeKey,List<(int Section,bool Start)>>();
        for(var i=0;i<source.Count;i++)foreach(var start in new[]{true,false})
        {var key=start?A(source[i]):B(source[i]);if(!ends.TryGetValue(key,out var list))ends[key]=list=new();list.Add((i,start));}
        for(var i=0;i<source.Count;i++)
        {
            var s=source[i];if(contracted[i]||s.Roundabout||A(s)==B(s)||fixedNodes.Contains(A(s))||fixedNodes.Contains(B(s))||
                NetworkDesignChecks.HorizontalArcLength(Curve(s))>=FoldLength)continue;
            var folded=false;
            foreach(var start in new[]{true,false})
            {
                var key=start?A(s):B(s);if(degree[key]!=2)continue;
                foreach(var (j,atStart) in ends[key])
                    if(j!=i&&!contracted[j]&&math.dot(Leaving(source[j],atStart),Leaving(s,start))>.5)folded=true;
                if(folded)break;
            }
            if(!folded)continue;
            token.ThrowIfCancellationRequested();var a=Root(A(s));var b=Root(B(s));if(a==b)continue;
            var loop=false;
            for(var j=0;j<source.Count&&!loop;j++)
                if(j!=i&&!contracted[j]){var x=Root(A(source[j]));var y=Root(B(source[j]));loop=x==a&&y==b||x==b&&y==a;}
            if(loop)continue;
            parent[b]=a;contracted[i]=true;folds++;
        }
        log.Count("Folded short sections contracted",folds);
        // Two dead ends of one kind and level under MappingGap apart, facing each
        // other (turning under 90°) with any one-way flow running through, are one
        // road broken in the map data; their nodes become one. Nothing else is
        // joined by proximity.
        var deadEnds=new List<(SourceNodeKey Key,int Section,bool AtStart,double3 P,double2 Away)>();
        for(var i=0;i<source.Count;i++)foreach(var start in new[]{true,false})
        {
            var s=source[i];var key=start?A(s):B(s);if(degree[key]!=1||s.Roundabout||fixedNodes.Contains(key))continue;
            var c=Curve(s);var p=start?c.P0:c.P3;var q=NetworkDesignChecks.Evaluate(c,start?.25:.75);
            deadEnds.Add((key,i,start,p,math.normalizesafe(q.xz-p.xz)));
        }
        var gapCells=deadEnds.Select((e,k)=>(e,k)).GroupBy(x=>((int)Math.Floor(x.e.P.x/MappingGap),(int)Math.Floor(x.e.P.z/MappingGap))).ToDictionary(g=>g.Key,g=>g.Select(x=>x.k).ToArray());
        var gapsJoined=0;
        for(var k=0;k<deadEnds.Count;k++)
        {
            var e=deadEnds[k];var cx=(int)Math.Floor(e.P.x/MappingGap);var cz=(int)Math.Floor(e.P.z/MappingGap);
            for(var dz=-1;dz<=1;dz++)for(var dx=-1;dx<=1;dx++)if(gapCells.TryGetValue((cx+dx,cz+dz),out var near))foreach(var m in near)
            {
                if(m<=k)continue;var o=deadEnds[m];var x=source[e.Section];var y=source[o.Section];
                if(x.Kind!=y.Kind||x.Layer!=y.Layer||x.Bridge!=y.Bridge||x.Tunnel!=y.Tunnel||x.OneWay!=y.OneWay||(x.OneWay&&e.AtStart==o.AtStart))continue;
                if(math.distance(e.P.xz,o.P.xz)>=MappingGap||Math.Abs(e.P.y-o.P.y)>=1||math.dot(e.Away,o.Away)>=0)continue;
                var a=Root(e.Key);var b=Root(o.Key);if(a==b)continue;
                parent[b]=a;gapsJoined++;
            }
        }
        log.Count("Map-data breaks joined between facing dead ends",gapsJoined);
        for(var i=0;i<source.Count;i++)
        {
            var s=source[i];if(contracted[i])continue;var limit=VisualCompilerTable.For(s).ClusterLength;
            if(limit<=0||s.Roundabout||NetworkStructure.IsStructure(s)||degree[A(s)]<3||degree[B(s)]<3||fixedNodes.Contains(A(s))||fixedNodes.Contains(B(s)))continue;
            if(NetworkDesignChecks.HorizontalArcLength(Curve(s))>=limit)continue;
            var a=Root(A(s));var b=Root(B(s));if(a!=b)parent[b]=a;contracted[i]=true;
        }
        var majorArms=new Dictionary<SourceNodeKey,int>();
        foreach(var s in source.Where(s=>!s.ProtectedLink))foreach(var key in new[]{A(s),B(s)})majorArms[key]=majorArms.TryGetValue(key,out var c)?c+1:1;
        var canonical=new Dictionary<SourceNodeKey,SourceNodeKey>();
        foreach(var group in position.Keys.GroupBy(Root))
        {
            var best=group.OrderByDescending(k=>ends[k].Any(a=>NativeAssetKeys.IsHighway(source[a.Section])))
                .ThenByDescending(k=>majorArms.TryGetValue(k,out var c)?c:0).ThenBy(k=>k.Id).First();
            foreach(var key in group)canonical[key]=best;
        }
        SourceNodeKey Canon(SourceNodeKey key)=>canonical[key];
        var absorbed=canonical.Where(p=>p.Key!=p.Value).ToDictionary(p=>p.Key,p=>p.Value);

        var arms=new Dictionary<SourceNodeKey,List<Arm>>();
        for(var i=0;i<source.Count;i++)if(!contracted[i])foreach(var start in new[]{true,false})
        {var key=Canon(start?A(source[i]):B(source[i]));if(!arms.TryGetValue(key,out var list))arms[key]=list=new();list.Add(new(i,start));}
        var points=new List<double2>[source.Count];
        var gridShift=new Dictionary<SourceNodeKey,double2>();
        List<double2> Curve2(int i)
        {
            var c=Curve(source[i]);var list=new List<double2>();
            var n=Math.Max(1,Math.Min(64,(int)Math.Ceiling(NetworkDesignChecks.HorizontalArcLength(c)/15)));for(var k=0;k<=n;k++)list.Add(NetworkDesignChecks.Evaluate(c,k/(double)n).xz);
            return list;
        }
        var shift=Separate(source,contracted,Curve2,Canon,token);
        foreach(var pair in shift)position[pair.Key]+=pair.Value;
        // Track cannot bend at a point. A plain main-line rail node beside a short
        // section (under JogLength) that turns the line more than 30°, which no
        // railway curve does within 30 m, is a jog left by consolidation: a 4 m
        // zigzag or hook, or a 13 m detour through a node of a removed parallel
        // track. It moves onto the chord between its neighbours (at most
        // JogLength / 2), over three passes so a detour's shoulders follow its
        // apex. The shift carries linearly along both sections, as for
        // carriageway separation.
        var jogs=0;
        for(var pass=0;pass<3;pass++)foreach(var node in arms)
        {
            var list=node.Value;if(node.Key.Kind!=NetworkKind.Rail||list.Count!=2||fixedNodes.Contains(node.Key)||list[0].Section==list[1].Section||
                list.Any(x=>source[x.Section].Classification!="rail"))continue;
            var p=position[node.Key];
            // A short section's far node, or the point JogLength along a longer one.
            (double2 Point,bool Short) Toward(Arm arm)
            {
                var far=position[Canon(arm.AtStart?B(source[arm.Section]):A(source[arm.Section]))];if(math.distance(p,far)<JogLength)return (far,true);
                var line=Curve2(arm.Section);if(!arm.AtStart)line.Reverse();var walked=0d;
                for(var k=1;k<line.Count;k++){walked+=math.distance(line[k-1],line[k]);if(walked>=JogLength)return (p+line[k]-line[0],false);}
                return (far,false);
            }
            var (a,shortA)=Toward(list[0]);var (b,shortB)=Toward(list[1]);if(!shortA&&!shortB)continue;
            var u=math.normalizesafe(a-p);var v=math.normalizesafe(b-p);
            if(math.dot(u,v)<=-Math.Cos(math.radians(30)))continue;
            // A node projecting near a chord end would land on its neighbour: two
            // distinct nodes 0.4 m apart, which the game joins wrongly and leaves
            // as a break. Such jogs, and moves ending within 2 m of any other rail
            // node, are left as they are.
            var ab=b-a;var t=math.dot(p-a,ab)/Math.Max(1e-9,math.lengthsq(ab));if(t<.15||t>.85)continue;var move=a+ab*t-p;
            if(math.length(move)>JogLength/2)continue;
            var target=p+move;var crowded=false;
            foreach(var other in arms.Keys)if(other.Kind==NetworkKind.Rail&&other!=node.Key&&math.distancesq(position[other],target)<4){crowded=true;break;}
            if(crowded)continue;
            position[node.Key]+=move;shift[node.Key]=shift.TryGetValue(node.Key,out var old)?old+move:move;jogs++;
        }
        log.Count("Rail jogs straightened",jogs);
        List<double2> Points(int i)
        {
            // The section's own curve, already fitted to OSM by the preparation
            // stages (SourceGeometry can span a whole way, beyond this section),
            // carrying its end nodes' carriageway separation shifts linearly.
            if(points[i]!=null)return points[i];var s=source[i];var list=Curve2(i);
            shift.TryGetValue(Canon(A(s)),out var a);shift.TryGetValue(Canon(B(s)),out var b);
            for(var k=0;k<list.Count;k++)list[k]+=math.lerp(a,b,k/(double)Math.Max(1,list.Count-1));
            if(gridShift.Count>0)
            {
                gridShift.TryGetValue(Canon(A(s)),out var gridA);gridShift.TryGetValue(Canon(B(s)),out var gridB);
                for(var k=0;k<list.Count;k++){var t=k/(double)Math.Max(1,list.Count-1);list[k]+=math.lerp(gridA,gridB,t*t*(3-2*t));}
            }
            list[0]=position[Canon(A(s))];list[list.Count-1]=position[Canon(B(s))];return points[i]=list;
        }
        double2 Away(Arm arm)
        {
            var p=Points(arm.Section);if(!arm.AtStart)p=Enumerable.Reverse(p).ToList();
            var walked=0d;for(var k=1;k<p.Count;k++){walked+=math.distance(p[k-1],p[k]);if(walked>=30||k==p.Count-1)return math.normalizesafe(p[k]-p[0]);}
            return default;
        }

        // Pair arm ends by good continuation (Thomson & Richardson 1999).
        bool Mainline(PreparedSegment s)=>s.Classification is "motorway" or "trunk";
        var next=new Dictionary<Arm,Arm>();
        foreach(var node in arms.OrderBy(p=>p.Key.Kind).ThenBy(p=>p.Key.Id))
        {
            token.ThrowIfCancellationRequested();var list=node.Value;if(fixedNodes.Contains(node.Key))continue;var candidates=new List<(double Score,Arm X,Arm Y)>();
            for(var a=0;a<list.Count;a++)for(var b=a+1;b<list.Count;b++)
            {
                var x=source[list[a].Section];var y=source[list[b].Section];
                // At a node of degree two the road goes on whatever its tags say
                // next: the A551 becomes a motorway_link 68 m before it joins the
                // A7, 3 m from its centreline, and ending the stroke there fixed
                // that point, so the merge pushed the link out and back (a hump).
                var plain=list.Count==2;
                if(list[a].Section==list[b].Section||x.Kind!=y.Kind||!plain&&x.ProtectedLink!=y.ProtectedLink||x.OneWay!=y.OneWay||x.Roundabout||y.Roundabout)continue;
                if(!plain&&x.Kind==NetworkKind.Road&&x.Classification!=y.Classification&&!(Mainline(x)&&Mainline(y)))continue;
                if(x.OneWay&&list[a].AtStart==list[b].AtStart)continue;
                // At a junction only a gentle continuation is the same road; at a
                // degree-two node a sharp bend is still one road and gets a fillet.
                var deflection=math.degrees(Math.Acos(math.clamp(-math.dot(Away(list[a]),Away(list[b])),-1,1)));if(deflection>=(list.Count>2?35:150))continue;
                var bonus=x.Reference.Length>0&&x.Reference==y.Reference?10:x.Name.Length>0&&x.Name==y.Name?5:0;
                candidates.Add((deflection-bonus,list[a],list[b]));
            }
            foreach(var c in candidates.OrderBy(c=>c.Score).ThenBy(c=>c.X.Section).ThenBy(c=>c.Y.Section))
                if(!next.ContainsKey(c.X)&&!next.ContainsKey(c.Y)){next[c.X]=c.Y;next[c.Y]=c.X;}
        }
        // The polyline leaving a node along an arm, followed through the
        // sections that continue it, at least length long where it goes on;
        // sections, if given, receives the section of every point.
        List<double2> Chain(Arm arm,double length,List<int>? sections=null)
        {
            var chain=new List<double2>();var current=arm;var walked=0d;var guard=0;
            while(guard++<64)
            {
                var p=Points(current.Section);if(!current.AtStart)p=Enumerable.Reverse(p).ToList();
                foreach(var q in chain.Count==0?p:p.Skip(1)){if(chain.Count>0)walked+=math.distance(chain[chain.Count-1],q);chain.Add(q);sections?.Add(current.Section);}
                if(walked>=length||!next.TryGetValue(new Arm(current.Section,!current.AtStart),out var following))break;
                current=following;
            }
            return chain;
        }

        var output=new List<PreparedSegment>();var visited=new bool[source.Count];long generated=GeneratedNodeBase;
        var nodeY=new Dictionary<SourceNodeKey,float>();var exact=new Dictionary<SourceNodeKey,float3>();
        float JunctionHeight(SourceNodeKey key)
        {
            if(!nodeY.TryGetValue(key,out var y)){var p=position[key];nodeY[key]=y=NetworkProcessor.Sample((float)p.x,(float)p.y,terrain,scale,width);}
            return y;
        }
        // Side-by-side clearance samples, built on first use (SideClear).
        List<(int Section,double2 P,double2 D)>? sideSamples=null;Dictionary<(int,int),List<int>>? sideGrid=null;
        var counts=(Strokes:0,Pieces:0,Corners:0,Merged:0,Projected:0,ProjectionMiss:0,VerticalMisfit:0,Merges:0,Absorbed:0,Rebalanced:0,ShortRuns:0,SidePushes:0,CornersDropped:0,GuardedPoints:0);
        var plans=new List<VisualCompilerProfile.Stroke>();var deferred=new List<Action>();var passthrough=new List<int>();var paths=new List<List<(int Section,bool Forward)>>();
        for(var i=0;i<source.Count;i++)
        {
            if(contracted[i]||visited[i])continue;token.ThrowIfCancellationRequested();
            if(source[i].Roundabout){visited[i]=true;passthrough.Add(i);continue;}
            // Walk back to the stroke start, then forward through paired arms.
            var start=new Arm(i,true);var guard=0;
            while(next.TryGetValue(start,out var previous)&&previous.Section!=i&&guard++<source.Count)start=new(previous.Section,!previous.AtStart);
            var path=new List<(int Section,bool Forward)>();var entry=start;
            while(!visited[entry.Section])
            {
                visited[entry.Section]=true;path.Add((entry.Section,entry.AtStart));
                var exit=new Arm(entry.Section,!entry.AtStart);if(!next.TryGetValue(exit,out var paired)||visited[paired.Section])break;
                entry=paired;
            }
            counts.Strokes++;paths.Add(path);
        }
        // Each section's stroke, and the nodes each stroke passes through (not its
        // ends): a stroke starting or ending on another's interior branches off it.
        var strokeOf=new int[source.Count];Array.Fill(strokeOf,-1);var interior=new HashSet<SourceNodeKey>[paths.Count];var strokeEnds=new (SourceNodeKey Start,SourceNodeKey End)[paths.Count];
        for(var p=0;p<paths.Count;p++)
        {
            var path=paths[p];interior[p]=new();foreach(var (section,_) in path)strokeOf[section]=p;
            for(var k=1;k<path.Count;k++){var s=source[path[k].Section];interior[p].Add(Canon(path[k].Forward?A(s):B(s)));}
            var first=source[path[0].Section];var last=source[path[^1].Section];
            strokeEnds[p]=(Canon(path[0].Forward?A(first):B(first)),Canon(path[^1].Forward?B(last):A(last)));
        }
        bool BranchOf(int t,int s)=>t!=s&&(interior[s].Contains(strokeEnds[t].Start)||interior[s].Contains(strokeEnds[t].End));
        var laneShifted=LaneDatum();
        var streetLines=AlignStreets();
        var twoWayApproaches=0;var carriagewaySplits=0;var approach=MergeApproaches();
        var forks=ForkClearance();var forkPushes=0;
        foreach(var path in paths)EmitStroke(path);
        var roundaboutFloors=RoundaboutWaterFloors();
        VisualCompilerProfile.Solve(plans,terrain,scale,width,log,token,roundaboutFloors);
        // A shared node takes the height its first stroke solved; every stroke was
        // pinned to the same coupled value at the node's own station, so the others
        // agree closely; the mismatch diagnostic reports where they do not.
        foreach(var plan in plans)foreach(var pin in plan.Pins)if(!nodeY.ContainsKey(pin.Node))
        {
            var height=plan.HeightAt(pin.Station);
            if(plan.WaterNodeFloors!=null&&plan.WaterNodeFloors.TryGetValue(pin.Node,out var floor))height=Math.Max(height,floor);
            nodeY[pin.Node]=(float)height;
        }
        foreach(var pair in roundaboutFloors)
            nodeY[pair.Key]=(float)Math.Max(JunctionHeight(pair.Key),pair.Value);
        var mismatch=plans.SelectMany(p=>p.Pins.Select(pin=>Math.Abs(p.HeightAt(pin.Station)-nodeY[pin.Node]))).OrderBy(d=>d).ToArray();
        double MismatchAt(double q)=>mismatch.Length==0?0:mismatch[(int)(q*(mismatch.Length-1))];
        log.Diagnostic($"VISUAL COMPILER junction height mismatch pins={mismatch.Length}; p50={MismatchAt(.5):F3}; p90={MismatchAt(.9):F3}; p99={MismatchAt(.99):F3}; max={MismatchAt(1):F2}; over10cm={mismatch.Count(d=>d>.1)}; over30cm={mismatch.Count(d=>d>.3)}");
        foreach(var emit in deferred)emit();
        foreach(var i in passthrough)output.Add(Passthrough(source[i]));
        log.Count("Visual compiler strokes",counts.Strokes);log.Count("Visual compiler junction nodes absorbed",absorbed.Count);
        log.Diagnostic($"VISUAL COMPILER horizontal inputSections={source.Count}; contracted={contracted.Count(c=>c)}; absorbedNodes={absorbed.Count}; strokes={counts.Strokes}; laneDatumNodesShifted={laneShifted}; rampApproachesAtMergeAngle={counts.Merges}; twoWayApproaches={twoWayApproaches}; carriagewaySplits={carriagewaySplits}; tightForkArms={forks.Count}; forkClearancePointsMoved={forkPushes}; sideClearancePointsMoved={counts.SidePushes}; pushesGuardedFromThirdRoads={counts.GuardedPoints}; pieces={counts.Pieces}; corners={counts.Corners}; cornerVerticesDropped={counts.CornersDropped}; mergedSections={counts.Merged}; shortPiecesAbsorbed={counts.Absorbed}; shortPiecesRebalanced={counts.Rebalanced}; shortRunsBetweenKeptNodes={counts.ShortRuns};projectedPorts={counts.Projected}; projectionMisses={counts.ProjectionMiss}; verticalMisfits={counts.VerticalMisfit}; outputSections={output.Count}");
        // Contracting a short connector can leave two rail routes between the
        // same pair of nodes, rebuilt here onto one stroke; crossovers and
        // connectors inside another route's footprint would overlap it in game.
        var aligned=RailCorridorConsolidator.RedundantParallelSpans(NetworkLaneSlots.Align(output,log),log,token);
        var approaches=ElevateApproaches(RailCorridorConsolidator.RedundantOverlappingRoutes(aligned,log,token,2),terrain,scale,width,log,token);
        var finished=GroundAtJunctions(ElevateApproaches(approaches,terrain,scale,width,log,token,sink:true),log,token);
        log.Count("Former bridge-boundary ports retained as aliases",structureAliases.Count);
        return SourceTopologyObligations.Transfer(structureSource,finished,NetworkKind.Road,structureAliases);

        // Travel-left lane continuity (report 64 V2 step 4). Along a one-way stroke
        // the left edge stays continuous where the native width changes, so a lane
        // drop removes the travel-right lane instead of narrowing both sides. Each
        // section's centreline moves left by half the width it differs from the
        // stroke's first section, eased in over max(60 m, 30·Δ) after each change.
        // The node itself stays put; NetworkLaneSlots offsets the arms there.
        // Right-hand traffic is assumed (both benchmark cities); shifts above 6 m
        // are skipped. Mainlines go first and claim shared junction nodes, so
        // attached ramps follow them.
        int LaneDatum()
        {
            var lane=new Dictionary<SourceNodeKey,double2>();
            foreach(var path in paths.OrderBy(p=>source[p[0].Section].ProtectedLink?1:0))
            {
                var first=source[path[0].Section];if(first.Kind!=NetworkKind.Road||!first.OneWay||path.Count<2)continue;
                var changes=new List<(double Station,double Offset,double Taper)>();var station=0d;var previous=0d;var stations=new double[path.Count+1];
                for(var k=0;k<path.Count;k++)
                {
                    var s=source[path[k].Section];var target=(first.NativeWidth-s.NativeWidth)/2d;
                    if(Math.Abs(target)>6)target=previous;
                    if(Math.Abs(target-previous)>.25)changes.Add((station,target,Math.Max(60,30*Math.Abs(target-previous))));
                    previous=target;station+=NetworkDesignChecks.HorizontalArcLength(Curve(s));stations[k+1]=station;
                }
                if(changes.Count==0)continue;
                double Offset(double at)
                {
                    var value=0d;var from=0d;
                    foreach(var (start,target,taper) in changes)
                    {
                        if(at<=start)break;value=at>=start+taper?target:math.lerp(from,target,(at-start)/taper);from=target;
                    }
                    return value;
                }
                for(var k=0;k<=path.Count;k++)
                {
                    var s=source[path[Math.Min(k,path.Count-1)].Section];var key=Canon(k==path.Count?(path[k-1].Forward?B(source[path[k-1].Section]):A(source[path[k-1].Section])):
                        (path[k].Forward?A(s):B(s)));
                    var offset=Offset(stations[k]);if(Math.Abs(offset)<.01||lane.ContainsKey(key)||fixedNodes.Contains(key))continue;
                    var travel=math.normalizesafe(((float3)s.CourseEnd).xz-((float3)s.CourseStart).xz);
                    lane[key]=new double2(-travel.y,travel.x)*offset;
                }
            }
            foreach(var pair in lane){position[pair.Key]+=pair.Value;shift[pair.Key]=shift.TryGetValue(pair.Key,out var old)?old+pair.Value:pair.Value;}
            Array.Clear(points,0,points.Length);
            return lane.Count;
        }

        IReadOnlyDictionary<int,StreetGridAlignment.Line> AlignStreets()
        {
            var protectedNodes=new HashSet<SourceNodeKey>(arms.Where(n=>n.Value.Any(a=>
                source[a.Section].Kind!=NetworkKind.Road||NativeAssetKeys.IsHighway(source[a.Section])||
                source[a.Section].Roundabout||NetworkStructure.IsStructure(source[a.Section]))).Select(n=>n.Key));
            foreach(var node in position.Keys)if(math.cmax(math.abs(position[node]))>=width/2-16)protectedNodes.Add(node);
            var streets=new List<StreetGridAlignment.Street>();
            for(var id=0;id<paths.Count;id++)
            {
                var path=paths[id];var first=source[path[0].Section];
                if(first.Kind!=NetworkKind.Road||path.Any(p=>NativeAssetKeys.IsHighway(source[p.Section])||source[p.Section].ProtectedLink||
                    source[p.Section].Roundabout||NetworkStructure.IsStructure(source[p.Section])||Math.Abs(source[p.Section].NativeWidth-first.NativeWidth)>.5))continue;
                var line=new List<double2>();var nodes=new List<SourceNodeKey>();var at=new List<int>();var wet=false;
                foreach(var (section,forward) in path)
                {
                    var s=source[section];var p=Points(section);if(!forward)p=p.AsEnumerable().Reverse().ToList();
                    if(line.Count==0){line.Add(p[0]);nodes.Add(Canon(forward?A(s):B(s)));at.Add(0);}
                    line.AddRange(p.Skip(1));nodes.Add(Canon(forward?B(s):A(s)));at.Add(line.Count-1);
                }
                if(water!=null)for(var k=0;k<line.Count&&!wet;k++)
                {
                    var d=math.normalizesafe(line[Math.Min(k+1,line.Count-1)]-line[Math.Max(0,k-1)]);var n=new double2(-d.y,d.x)*(first.NativeWidth/2+12);
                    for(var side=-1;side<=1;side++)if(water(new((float)(line[k].x+n.x*side),0,(float)(line[k].y+n.y*side))).HasValue){wet=true;break;}
                }
                if(!wet)streets.Add(new(id,nodes.ToArray(),at.ToArray(),line.ToArray(),first.NativeWidth));
                else foreach(var node in nodes)protectedNodes.Add(node);
            }
            var result=StreetGridAlignment.Prepare(streets,protectedNodes,log,token);
            foreach(var node in result.Nodes){gridShift[node.Key]=node.Value-position[node.Key];position[node.Key]=node.Value;}
            if(result.Nodes.Count>0)Array.Clear(points,0,points.Length);
            return result.Lines;
        }

        // A one-way road joining or leaving another one-way road meets it at
        // MergeAngle (SplitAngle for a branch of two or more lanes; report 43
        // COR-003, report 64 R3), not tangentially, unless
        // every one of its lanes continues in a lane of its own: lanes after a
        // merge (before a diverge) at least both roads' lanes together. Shared
        // lanes (2 + 2 → 3), a lane drop (2 + 1 → 2) or one lane splitting into
        // two need the angle whatever the roads' classes; at 20° the pieces still
        // looked disconnected in game. The joining road's away direction is the
        // through arm on its side rotated MergeAngle toward it; Clearance is the
        // centreline spacing at which the two roads' edges are 1 m apart.
        Dictionary<Arm,(double2 Direction,double2 Through,double Clearance,List<(double2 Point,double Width)>? Partner)> MergeApproaches()
        {
            var result=new Dictionary<Arm,(double2,double2,double,List<(double2,double)>?)>();var angle=math.radians(MergeAngle);
            double2 Rotate(double2 v,double radians){var c=Math.Cos(radians);var s=Math.Sin(radians);return new(v.x*c-v.y*s,v.x*s+v.y*c);}
            foreach(var node in arms)
            {
                var list=node.Value;if(list.Count<3||fixedNodes.Contains(node.Key))continue;
                var through=list.Where(a=>source[a.Section].Kind==NetworkKind.Road&&source[a.Section].OneWay&&!source[a.Section].Roundabout&&next.TryGetValue(a,out var b)&&list.Contains(b)).ToArray();
                var twoWayThrough=list.Where(a=>source[a.Section].Kind==NetworkKind.Road&&!source[a.Section].OneWay&&!source[a.Section].Roundabout&&next.TryGetValue(a,out var b)&&list.Contains(b)).ToArray();
                // Preserve the established one-way highway merge when a second
                // two-way road also runs through the junction. Use an avenue
                // pair as the fallback, or when it is the highway of the two.
                if(through.Length==0||through.All(a=>!NativeAssetKeys.IsHighway(source[a.Section]))&&twoWayThrough.Length==2&&twoWayThrough.Any(a=>NativeAssetKeys.IsHighway(source[a.Section])))through=twoWayThrough;
                // A two-way road becoming two one-way carriageways (a reversible
                // express road joining both directions, or a divided road
                // starting): OSM often splits them at a sliver angle, which the
                // native node draws as a pinch. The two carriageways open
                // symmetrically about their bisector to MergeAngle between them.
                if(through.Length==0&&list.Count==3)
                {
                    var single=list.Where(a=>source[a.Section].Kind==NetworkKind.Road&&!source[a.Section].OneWay&&!source[a.Section].Roundabout).ToArray();
                    var pair=list.Where(a=>source[a.Section].Kind==NetworkKind.Road&&source[a.Section].OneWay&&!source[a.Section].Roundabout).ToArray();
                    if(single.Length!=1||pair.Length!=2||pair[0].AtStart==pair[1].AtStart)continue;
                    var u=Away(pair[0]);var v=Away(pair[1]);
                    if(math.degrees(Math.Acos(math.clamp(math.dot(u,v),-1,1)))>=MergeAngle)continue;
                    var bisector=math.normalizesafe(u+v);var half=(source[pair[0].Section].NativeWidth+source[pair[1].Section].NativeWidth)/4d+.5;
                    foreach(var arm in pair)
                    {
                        var side=Math.Sign(NetworkDesignChecks.Cross(bisector,Away(arm)));if(side==0)side=arm==pair[0]?1:-1;
                        result[arm]=(Rotate(bisector,side*angle/2),bisector,half,null);
                    }
                    carriagewaySplits++;continue;
                }
                if(through.Length!=2)continue;
                if(!source[through[0].Section].OneWay)
                {
                    foreach(var branch in list.Where(a=>!through.Contains(a)))
                    {
                        var s=source[branch.Section];if(s.Kind!=NetworkKind.Road||s.Roundabout||next.ContainsKey(branch))continue;
                        if(NativeAssetKeys.IsHighway(s)&&!NativeAssetKeys.IsHighway(source[through[0].Section]))continue;
                        var direction=Away(branch);var partner=math.dot(Away(through[0]),direction)>math.dot(Away(through[1]),direction)?through[0]:through[1];
                        var m=Away(partner);if(math.dot(m,direction)<=Math.Cos(angle))continue;
                        var side=Math.Sign(NetworkDesignChecks.Cross(m,direction));
                        if(side==0)
                        {
                            var line=Chain(branch,100);foreach(var p in line.Skip(1))
                            {side=Math.Sign(NetworkDesignChecks.Cross(m,p-line[0]));if(side!=0)break;}
                        }
                        if(side==0)continue;
                        var sections=new List<int>();var lineOfPartner=Chain(partner,HoldReach+100,sections)
                            .Select((p,k)=>(p,(double)source[sections[k]].NativeWidth)).ToList();
                        result[branch]=(Rotate(m,side*math.radians(StreetBranchAngle)),m,(source[partner.Section].NativeWidth+s.NativeWidth)/2d+1,lineOfPartner);
                        twoWayApproaches++;
                    }
                    continue;
                }
                var up=through.FirstOrDefault(a=>!a.AtStart);var down=through.FirstOrDefault(a=>a.AtStart);
                if(up==default||down==default||up.Section==down.Section)continue;
                foreach(var ramp in list)
                {
                    var r=source[ramp.Section];if(ramp==up||ramp==down||r.Kind!=NetworkKind.Road||r.Roundabout||!r.OneWay&&!Mainline(r)&&!r.ProtectedLink)continue;
                    var l=Away(ramp);
                    // A two-way road here is usually a reversible carriageway
                    // (express lanes), which parses as two-way: it meets the through
                    // arm it lies alongside, and always at the full angle.
                    var merge=r.OneWay?!ramp.AtStart:math.dot(Away(up),l)>math.dot(Away(down),l);
                    var lanesUp=source[up.Section].Lanes;var lanesDown=source[down.Section].Lanes;
                    if(r.OneWay&&(merge?lanesDown>=lanesUp+r.Lanes:lanesUp>=lanesDown+r.Lanes))continue;
                    var m=Away(merge?up:down);
                    if(math.degrees(Math.Acos(math.clamp(math.dot(m,l),-1,1)))>=MergeAngle)continue;
                    var side=Math.Sign(NetworkDesignChecks.Cross(m,l));if(side==0)continue;
                    if(!r.OneWay)twoWayApproaches++;
                    var turn=Splits(r)?math.radians(SplitAngle):angle;
                    var sections=new List<int>();var partner=Chain(merge?up:down,HoldReach+100,sections).Select((p,k)=>(p,(double)source[sections[k]].NativeWidth)).ToList();
                    result[ramp]=(Rotate(m,side*turn),m,(source[(merge?up:down).Section].NativeWidth+r.NativeWidth)/2d+1,partner);
                }
            }
            return result;
        }

        // Tight forks: two same-level one-way roads that both leave, or both join,
        // a junction within 35° of each other. OSM draws a ramp leaving the
        // motorway's centreline and staying a few metres away for 100 m or more,
        // but the native motorway is 15–20 m wide, so the ramp was drawn on top
        // of it until it finally cleared. Each such end gets its partner's
        // polyline (followed through continuing sections) and a clearance of half
        // both widths plus 2.5 m; the margin covers the partner's own smoothing
        // drift from its OSM line. A ramp gives way fully to a mainline, and a
        // branch to the road continuing through (a motorway splitting in two);
        // only two branches of equal standing each take half.
        Dictionary<Arm,List<(List<double2> Partner,double Clearance,double Share,double Taper)>> ForkClearance()
        {
            var result=new Dictionary<Arm,List<(List<double2>,double,double,double)>>();
            foreach(var node in arms)
            {
                var list=node.Value;if(list.Count<3||fixedNodes.Contains(node.Key))continue;
                for(var i=0;i<list.Count;i++)for(var j=i+1;j<list.Count;j++)
                {
                    var a=list[i];var b=list[j];var x=source[a.Section];var y=source[b.Section];
                    if(x.Kind!=y.Kind||x.Roundabout||y.Roundabout||x.Layer!=y.Layer)continue;
                    // Rail diverges both ways at a switch; roads only where both
                    // one-way ends leave, or both join.
                    var rail=x.Kind==NetworkKind.Rail;
                    if(!rail&&(!x.OneWay||!y.OneWay||a.AtStart!=b.AtStart))continue;
                    if(next.TryGetValue(a,out var paired)&&paired==b)continue;
                    if(math.degrees(Math.Acos(math.clamp(math.dot(Away(a),Away(b)),-1,1)))>=35)continue;
                    var clearance=(x.NativeWidth+y.NativeWidth)/2d+(rail?1:2.5);
                    // A rail branch, siding or yard track gives way to the through line.
                    // A road that continues through the junction keeps its line
                    // and the branch leaving it moves, as a link does for a
                    // mainline; only two equal branches share the push.
                    bool Branch(Arm arm,PreparedSegment s)=>rail?s.RailService.Length>0||!next.ContainsKey(arm):s.ProtectedLink;
                    var shareA=!rail&&NativeAssetKeys.IsHighway(x)!=NativeAssetKeys.IsHighway(y)?(NativeAssetKeys.IsHighway(x)?0:1):
                        Branch(a,x)&&!Branch(b,y)?1:!Branch(a,x)&&Branch(b,y)?0:
                        rail||next.ContainsKey(a)==next.ContainsKey(b)?.5:next.ContainsKey(a)?0:1;
                    void Add(Arm arm,Arm partner,double share)
                    {if(share<=0)return;if(!result.TryGetValue(arm,out var forks))result[arm]=forks=new();forks.Add((Chain(partner,ForkReach+100),clearance,share,rail?12:8));}
                    Add(a,b,shareA);Add(b,a,1-shareA);
                }
            }
            return result;
        }

        // Push a stroke end clear of a fork partner. The separation it needs grows
        // along the fork's taper (8° for roads, 12° for a rail turnout) from the
        // junction up to the clearance (a gentle merge,
        // not an S-bend), and the push fades out over the last 60 m of ForkReach.
        // The push is to one consistent side, the side where the end is farthest
        // from its partner: OSM often draws a ramp slightly inside the motorway's
        // centreline first, and pushing away from the nearest point would send it
        // the wrong way. Interior junctions stay put, as in Approach.
        void Clear(List<double2> line,bool atStart,(List<double2> Partner,double Clearance,double Share,double Taper) fork,HashSet<int> junctions)
        {
            var count=line.Count;if(count<2||fork.Partner.Count<2)return;var taper=Math.Tan(math.radians(fork.Taper));
            (double Signed,double2 Normal) Offset(double2 p)
            {
                var best=double.PositiveInfinity;var closest=p;var direction=new double2(1,0);
                for(var k=1;k<fork.Partner.Count;k++)
                {
                    var a=fork.Partner[k-1];var ab=fork.Partner[k]-a;var t=math.clamp(math.dot(p-a,ab)/Math.Max(1e-9,math.lengthsq(ab)),0,1);var q=a+ab*t;var d=math.distance(p,q);
                    if(d<best){best=d;closest=q;direction=math.normalizesafe(ab);}
                }
                var normal=new double2(-direction.y,direction.x);return (math.dot(p-closest,normal),normal);
            }
            var reach=new List<int>();var walked=0d;var previous=atStart?line[0]:line[count-1];var side=0d;var widest=0d;
            for(var i=1;i<count;i++)
            {
                var index=atStart?i:count-1-i;walked+=math.distance(previous,line[index]);previous=line[index];
                if(walked>=ForkReach||junctions.Contains(index))break;reach.Add(index);
                var signed=Offset(line[index]).Signed;if(Math.Abs(signed)>widest){widest=Math.Abs(signed);side=Math.Sign(signed);}
            }
            if(side==0)return;walked=0;previous=atStart?line[0]:line[count-1];
            foreach(var index in reach)
            {
                var p=line[index];walked+=math.distance(previous,p);previous=p;
                var required=Math.Min(fork.Clearance,walked*taper);var (signedOffset,normal)=Offset(p);var current=signedOffset*side;
                if(current>=required)continue;
                var fade=walked<=ForkReach-60?1:(ForkReach-walked)/60;
                line[index]=p+normal*side*(required-current)*fork.Share*fade;forkPushes++;
            }
        }

        // Bend a stroke end onto its required approach. In a frame along the
        // through road (along, lateral) the end leaves along the approach
        // direction until it is Clearance beside the through road, then heads
        // straight for its own line and follows it: the required lateral is the
        // upper concave hull of the node, that corner, the source line ahead
        // and the plain clearance at the end of the reach (the corner may stand
        // wider, to fit the straight start and the turn). A ramp drawn close beside
        // the motorway therefore turns out sharply and straightens, and one
        // whose real line diverges meets it along a tangent, rather than
        // running parallel at the clearance and turning out again (a hump), or
        // swinging out past its real line and back (an S). It is never moved
        // inward. The turn at the corner is a circular arc at the class's radius
        // floor up to TurnBackRadius, or the largest that fits; the meeting with the source line is
        // rounded by a smooth maximum. Points within half the reach take
        // the target, and the next two reaches blend back to the source by a
        // smoothstep, which has no tangent break at either end of the blend. The
        // reach is 40 m, or the arc that turns MergeAngle (SplitAngle) at the
        // class's radius floor (131 m for a motorway). It stops at any interior junction, whose
        // shared position must not move.
        static double ApproachReach(PreparedSegment s)=>Math.Max(40,VisualCompilerTable.For(s).RadiusFloor*math.radians(Splits(s)?SplitAngle:MergeAngle));
        void Approach(List<double2> line,bool atStart,(double2 Direction,double2 Through,double Clearance,List<(double2 Point,double Width)>? Partner) approach,double reach,double radius,HashSet<int> junctions,Func<int,double> width)
        {
            var count=line.Count;if(count<2)return;
            var node=atStart?line[0]:line[count-1];
            var m=approach.Through;var normal=math.normalizesafe(approach.Direction-m*math.dot(approach.Direction,m));
            var theta=Math.Acos(math.clamp(math.dot(approach.Direction,m),-1,1));
            radius=Math.Min(radius,TurnBackRadius);var clearance=approach.Clearance;
            var corner=new double2(clearance/Math.Tan(theta),clearance);
            // The frame follows the through road's own polyline where it is
            // known (along it, and out from it on the approach side), so the
            // approach and the hold beyond it measure one offset: in a straight
            // frame from the node, a gently curving motorway put them a metre
            // apart, a bump where one handed over to the other. Needed is the
            // clearance there, half the widths side by side plus 1 m; a point
            // past the polyline's end is not alongside it.
            var partner=approach.Partner;var side=Math.Sign(math.dot(new double2(-m.y,m.x),normal));
            var lengths=new double[partner?.Count??0];for(var k=1;k<lengths.Length;k++)lengths[k]=lengths[k-1]+math.distance(partner![k-1].Point,partner[k].Point);
            (double Along,double Lateral,double2 Normal,double Needed,bool Beyond,double Distance) Frame(int index)
            {
                var p=line[index];
                if(partner==null||partner.Count<2||side==0)return (math.dot(p-node,m),math.dot(p-node,normal),normal,clearance,true,math.distance(p,node));
                var best=double.PositiveInfinity;var nearest=1;var at=0d;
                for(var k=1;k<partner.Count;k++)
                {
                    var a=partner[k-1].Point;var ab=partner[k].Point-a;var t=math.dot(p-a,ab)/Math.Max(1e-9,math.lengthsq(ab));
                    var d=math.distance(p,a+ab*math.clamp(t,0,1));if(d<best){best=d;nearest=k;at=t;}
                }
                var start=partner[nearest-1].Point;var segment=partner[nearest].Point-start;var direction=math.normalizesafe(segment);
                var n=new double2(-direction.y,direction.x)*side;var beyond=nearest==partner.Count-1&&at>1;
                if(!beyond)at=math.clamp(at,0,1);
                return (lengths[nearest-1]+at*math.length(segment),math.dot(p-(start+segment*at),n),n,(partner[nearest].Width+width(index))/2+1,beyond,best);
            }
            // The points this end may move, and the source line in the frame.
            var moved=new List<(int Index,double Walked)>();var source=new List<double2>();var walked=0d;var previous=node;
            for(var i=1;i<count-1;i++)
            {
                var index=atStart?i:count-1-i;var p=line[index];walked+=math.distance(previous,p);previous=p;
                if(walked>=2.5*reach||junctions.Contains(index))break;
                var frame=Frame(index);moved.Add((index,walked));source.Add(new(frame.Along,frame.Lateral));
            }
            if(moved.Count==0)return;
            // Upper concave hull, by a monotone chain, of the corner, the source
            // line ahead of it while it keeps advancing along the through road
            // (a loop ramp curling back is not a line ahead), and the clearance
            // at the end. The required lateral is the lower of the approach line
            // and that hull, so it is concave and never exceeds the approach.
            var ahead=new List<double2>();var furthest=0d;
            foreach(var q in source){if(q.x<furthest-1)break;furthest=Math.Max(furthest,q.x);if(q.x>corner.x)ahead.Add(q);}
            var candidates=new List<double2>{corner};candidates.AddRange(ahead);candidates.Add(new(Math.Max(corner.x,furthest),clearance));
            candidates.Sort((a,b)=>a.x.CompareTo(b.x));var hull=new List<double2>();
            foreach(var q in candidates)
            {
                while(hull.Count>=2&&NetworkDesignChecks.Cross(hull[hull.Count-1]-hull[hull.Count-2],q-hull[hull.Count-2])>=0)hull.RemoveAt(hull.Count-1);
                hull.Add(q);
            }
            double Hull(double along)
            {
                if(along<=hull[0].x)return hull[0].y;
                for(var k=1;k<hull.Count;k++)if(along<=hull[k].x||k==hull.Count-1)
                {var a=hull[k-1];var b=hull[k];return b.x-a.x<1e-9?Math.Max(a.y,b.y):a.y+(b.y-a.y)*math.clamp((along-a.x)/(b.x-a.x),0,1);}
                return clearance;
            }
            // Round the corner between the approach line and the hull's first
            // edge with an arc tangent to both, at distance d either side.
            var arc=(Start:double.NaN,End:double.NaN,Centre:default(double2),Radius:0d);
            var u1=new double2(Math.Cos(theta),Math.Sin(theta));var u2=hull.Count>1?math.normalizesafe(hull[1]-hull[0]):new double2(1,0);
            if(u2.y/Math.Max(1e-9,u2.x)<Math.Tan(theta))
            {
                var turn=Math.Acos(math.clamp(math.dot(u1,u2),-1,1));
                if(turn>1e-4)
                {
                    var d=Math.Min(radius*Math.Tan(turn/2),Math.Min(math.length(corner),hull.Count>1?math.distance(corner,hull[1]):double.PositiveInfinity));
                    var r=d/Math.Tan(turn/2);var t1=corner-u1*d;var t2=corner+u2*d;
                    arc=(t1.x,t2.x,t1+new double2(u1.y,-u1.x)*r,r);
                }
            }
            double Required(double along)
            {
                if(along>=arc.Start&&along<=arc.End)
                    return arc.Centre.y+Math.Sqrt(Math.Max(0,arc.Radius*arc.Radius-(along-arc.Centre.x)*(along-arc.Centre.x)));
                return Math.Min(along*Math.Tan(theta),Hull(along));
            }
            static double SmoothMax(double a,double b,double k){var h=math.clamp(.5+.5*(a-b)/k,0,1);return math.lerp(b,a,h)+k*h*(1-h);}
            // Beyond the blend the end keeps being held at the clearance while
            // its source stays inside it, up to HoldReach; a hold cut short by a
            // junction, the stroke's other end or HoldReach fades out over the
            // last HoldFade metres before it.
            // Signed lateral position describes a side, not a footprint: a
            // loop ramp can be hundreds of metres away on the opposite side.
            // Use the actual distance to the partner, including at its bends.
            bool Inside(int index){var frame=Frame(index);return !frame.Beyond&&frame.Distance<frame.Needed;}
            var hold=new List<(int Index,double Walked)>();var fixedAt=double.NaN;walked=0;previous=node;
            for(var i=1;i<count;i++)
            {
                var index=atStart?i:count-1-i;var p=line[index];walked+=math.distance(previous,p);previous=p;
                if(i==count-1||junctions.Contains(index)||walked>=Math.Max(2.5*reach,HoldReach)){fixedAt=walked;break;}
                if(walked>=2.5*reach&&!Inside(index))break;
                hold.Add((index,walked));
            }
            if(hold.Count>0&&!double.IsNaN(fixedAt)&&!Inside(hold[^1].Index))fixedAt=double.NaN;
            for(var k=0;k<hold.Count;k++)
            {
                var (index,distance)=hold[k];var frame=Frame(index);var lateral=frame.Lateral;
                // A returning through-road segment can put an already clear
                // ramp on its negative side. That half-plane is not occupied
                // pavement: do not push the ramp across the intervening land.
                if(lateral<0&&frame.Distance>=frame.Needed)continue;
                var fade=double.IsNaN(fixedAt)?1:math.clamp((fixedAt-distance)/HoldFade,0,1);
                var held=frame.Beyond?lateral:lateral+Math.Max(0,frame.Needed-lateral)*fade;
                var target=held;
                if(k<moved.Count)
                {
                    var required=Required(Math.Max(0,frame.Along));
                    var u=Math.Max(0,(distance/reach-.5)/2);var weight=1-u*u*(3-2*u);
                    target=math.lerp(held,SmoothMax(lateral,required,math.clamp(required,.1,2)),weight);
                }
                line[index]+=frame.Normal*(target-lateral);
            }
        }

        // Side-by-side clearance. Two same-level strokes of one kind running within
        // 20° of each other (express and local lanes, a collector beside a
        // motorway, opposite carriageways drawn too close, parallel rail lines)
        // overlap wherever their centrelines are closer than half both native
        // widths plus 1 m. Each stroke moves away by its share of that shortfall:
        // a ramp gives way to a mainline and a siding or yard track to a main
        // line; equal routes each take half. The push rises and falls along the
        // same 8° taper as fork clearance (5° for a through road) and is zero at
        // the stroke's ends and interior junctions, so no node moves. Pairs
        // sharing a node are included rather than left to fork clearance,
        // which stops 300 m from the node; with each partner cleared to one
        // side (below), that removed 228 of DC's 1,195 overlaps.
        // The taper keeps the push at zero at the shared node. Partners are
        // measured at their source positions; each partner moves its own share.
        // Every section's source points in SideCell squares, built once.
        const double SideCell=32;
        void EnsureSideGrid()
        {
            if(sideSamples!=null)return;
            sideSamples=new();sideGrid=new();
            for(var i=0;i<source.Count;i++)
            {
                if(contracted[i])continue;var p=Points(i);
                for(var k=0;k<p.Count;k++)
                {
                    var d=math.normalizesafe(p[Math.Min(k+1,p.Count-1)]-p[Math.Max(k-1,0)]);
                    var key=((int)Math.Floor(p[k].x/SideCell),(int)Math.Floor(p[k].y/SideCell));
                    if(!sideGrid.TryGetValue(key,out var list))sideGrid[key]=list=new();list.Add(sideSamples.Count);sideSamples.Add((i,p[k],d));
                }
            }
        }

        // The approach, fork and side pushes each look at one partner. Packed
        // carriageways (the Dulles Toll Road, Access Road and their ramps, four
        // one-way roads within 40 m) let a ramp pushed off one partner land on a
        // third road 25 m away. Wherever a moved point ends up closer to another
        // same-level road of its kind (not one meeting this stroke) than it
        // started, and inside their clearance, its displacement shrinks until it
        // does not; the shrinking eases back in over 2.5% per metre either side
        // so the line keeps its curvature.
        void GuardPushes(List<double2> line,IReadOnlyList<double2> original,List<int> sectionAt,List<(int Section,bool Forward)> path,HashSet<int> junctions)
        {
            var count=line.Count;if(count<3)return;EnsureSideGrid();
            var own=new HashSet<int>(path.Select(p=>p.Section));
            var ownNodes=new HashSet<SourceNodeKey>(path.SelectMany(p=>new[]{Canon(A(source[p.Section])),Canon(B(source[p.Section]))}));
            var factor=new double[count];var limited=new List<int>();
            for(var k=0;k<count;k++)
            {
                // A move under 0.5 m cannot carry a road into another.
                factor[k]=1;if(math.distancesq(line[k],original[k])<.25)continue;var s=source[sectionAt[k]];
                var cx=(int)Math.Floor(line[k].x/SideCell);var cz=(int)Math.Floor(line[k].y/SideCell);
                double Allowed(double f)
                {
                    var p=math.lerp(original[k],line[k],f);var worst=double.PositiveInfinity;
                    for(var dz=-1;dz<=1;dz++)for(var dx=-1;dx<=1;dx++)if(sideGrid!.TryGetValue((cx+dx,cz+dz),out var near))foreach(var index in near)
                    {
                        var q=sideSamples![index];if(own.Contains(q.Section))continue;var o=source[q.Section];
                        if(o.Kind!=s.Kind||o.Layer!=s.Layer||ownNodes.Contains(Canon(A(o)))||ownNodes.Contains(Canon(B(o))))continue;
                        var clearance=(s.NativeWidth+o.NativeWidth)/2d+1;var after=math.distance(p,q.P);if(after>=clearance)continue;
                        worst=Math.Min(worst,after-Math.Min(clearance,math.distance(original[k],q.P)));
                    }
                    return worst;
                }
                if(Allowed(1)>=-.05)continue;
                var lo=0d;var hi=1d;for(var step=0;step<8;step++){var mid=(lo+hi)/2;if(Allowed(mid)>=-.05)lo=mid;else hi=mid;}
                factor[k]=lo;limited.Add(k);
            }
            if(limited.Count==0)return;
            var station=new double[count];for(var k=1;k<count;k++)station[k]=station[k-1]+math.distance(original[k-1],original[k]);
            var eased=new double[count];
            for(var k=0;k<count;k++){var f=1d;foreach(var j in limited)f=Math.Min(f,factor[j]+Math.Abs(station[k]-station[j])*.025);eased[k]=f;}
            for(var k=1;k<count-1;k++)if(!junctions.Contains(k)&&eased[k]<1){line[k]=math.lerp(original[k],line[k],eased[k]);counts.GuardedPoints++;}
        }

        void SideClear(List<double2> line,List<int> sectionAt,List<(int Section,bool Forward)> path,HashSet<int> junctions)
        {
            // A through road that moves (an equal pair) tapers at 5°: at 8° its
            // pushes bent motorways below their radius floor.
            var first=source[path[0].Section];
            const double maximumPush=12;var taper=Math.Tan(math.radians(first.ProtectedLink||first.Kind==NetworkKind.Rail?8:5));var parallel=Math.Cos(math.radians(20));
            EnsureSideGrid();
            var count=line.Count;if(count<3)return;
            var own=new HashSet<int>(path.Select(p=>p.Section));
            bool Minor(PreparedSegment s)=>s.Kind==NetworkKind.Rail?s.RailService.Length>0:s.ProtectedLink;
            var station=new double[count];for(var k=1;k<count;k++)station[k]=station[k-1]+math.distance(line[k-1],line[k]);
            var pushes=new List<(double S,double Need,double2 Direction)>();
            // Each partner section is cleared to one side: the side where this
            // stroke stands farthest from it. A ramp drawn from inside a
            // motorway's centreline crosses it, and pushing each point to the
            // side it happened to be on bent the ramp into a zigzag.
            var sideOf=new Dictionary<int,(double Farthest,double Side)>();
            void Partners(Action<int,double2,(int Section,double2 P,double2 D),PreparedSegment,PreparedSegment> visit)
            {
                for(var k=1;k<count-1;k++)
                {
                    if(junctions.Contains(k))continue;
                    var s=source[sectionAt[k]];var direction=math.normalizesafe(line[k+1]-line[k-1]);var normal=new double2(-direction.y,direction.x);
                    var cx=(int)Math.Floor(line[k].x/SideCell);var cz=(int)Math.Floor(line[k].y/SideCell);
                    for(var dz=-1;dz<=1;dz++)for(var dx=-1;dx<=1;dx++)if(sideGrid!.TryGetValue((cx+dx,cz+dz),out var near))foreach(var index in near)
                    {
                        var q=sideSamples![index];if(own.Contains(q.Section))continue;var o=source[q.Section];
                        if(o.Kind!=s.Kind||o.Layer!=s.Layer||o.Roundabout||s.Roundabout||Math.Abs(math.dot(direction,q.D))<parallel)continue;
                        if(math.distance(line[k],q.P)>=(s.NativeWidth+o.NativeWidth)/2d+1)continue;
                        visit(k,normal,q,s,o);
                    }
                }
            }
            Partners((k,normal,q,s,o)=>
            {
                var signed=math.dot(line[k]-q.P,normal);
                if(!sideOf.TryGetValue(q.Section,out var known)||Math.Abs(signed)>known.Farthest)sideOf[q.Section]=(Math.Abs(signed),signed<0?-1:1);
            });
            var best=new (double Need,double2 Direction)[count];
            Partners((k,normal,q,s,o)=>
            {
                // Equal road pairs keep their designed spacing: only a ramp
                // gives way, and only to a road it runs alongside. Between
                // equal roads, one branching off the other gives way to it.
                if(s.Kind==NetworkKind.Road&&!Minor(s)&&Minor(o))return;
                var self=strokeOf[path[0].Section];var other=strokeOf[q.Section];
                var share=s.Kind==NetworkKind.Road&&NativeAssetKeys.IsHighway(s)!=NativeAssetKeys.IsHighway(o)?(NativeAssetKeys.IsHighway(s)?0:1):
                    Minor(s)&&!Minor(o)?1:!Minor(s)&&Minor(o)?0:
                    s.Kind!=NetworkKind.Road||other<0?.5:BranchOf(other,self)?0:BranchOf(self,other)?1:.5;
                if(share==0)return;
                var side=sideOf[q.Section].Side;var current=math.dot(line[k]-q.P,normal)*side;var clearance=(s.NativeWidth+o.NativeWidth)/2d+1;
                if(current>=clearance)return;
                var need=Math.Min(maximumPush,(clearance-current)*share);
                if(need>best[k].Need)best[k]=(need,normal*side);
            });
            for(var k=1;k<count-1;k++)if(best[k].Need>.05)pushes.Add((station[k],best[k].Need,best[k].Direction));
            if(pushes.Count==0)return;
            var anchors=new List<double>{0,station[count-1]};foreach(var j in junctions)if(j>0&&j<count-1)anchors.Add(station[j]);
            double Limit(int k){var limit=double.PositiveInfinity;foreach(var a in anchors)limit=Math.Min(limit,Math.Abs(station[k]-a)*taper);return limit;}
            var push=new double2[count];
            for(var k=1;k<count-1;k++)
            {
                if(junctions.Contains(k))continue;
                var magnitude=0d;var direction=default(double2);
                foreach(var p in pushes){var m=p.Need-Math.Abs(station[k]-p.S)*taper;if(m>magnitude){magnitude=m;direction=p.Direction;}}
                push[k]=direction*Math.Min(magnitude,Limit(k));
            }
            // A triangular 40 m average rounds the taper's corners, so the moved
            // alignment needs no tight fillet; junction limits apply again after.
            const double window=40;
            for(var k=1;k<count-1;k++)
            {
                if(junctions.Contains(k))continue;var sum=default(double2);var weight=0d;
                for(var j=0;j<count;j++){var w=window-Math.Abs(station[j]-station[k]);if(w>0){sum+=push[j]*w;weight+=w;}}
                var smooth=weight>0?sum/weight:default;var length=math.length(smooth);var limit=Limit(k);
                if(length>limit)smooth*=limit/length;
                if(length>.01){line[k]+=smooth;counts.SidePushes++;}
            }
        }

        PreparedSegment Passthrough(PreparedSegment s)
        {
            var a=Node(A(s));var b=Node(B(s));var c=Curve(s);
            float3 Lift(double3 p,float y)=>new((float)p.x,y,(float)p.z);
            return s with{Start=a,End=b,Control1=Lift(c.P1,math.lerp(a.y,b.y,1f/3)),Control2=Lift(c.P2,math.lerp(a.y,b.y,2f/3)),
                StartCurveOffset=default,EndCurveOffset=default,FinalDeckCurve=null,HorizontalParent=null,HorizontalEmissionReference=null,NativeUnitId=-1};
        }
        float3 Node(SourceNodeKey key){var c=Canon(key);var p=position[c];return new((float)p.x,JunctionHeight(c),(float)p.y);}

        Dictionary<SourceNodeKey,double> RoundaboutWaterFloors()
        {
            var floors=new Dictionary<SourceNodeKey,double>();if(water==null)return floors;
            var seen=new HashSet<int>();var rings=0;
            foreach(var seed in passthrough)
            {
                if(source[seed].Kind!=NetworkKind.Road||source[seed].Tunnel||!seen.Add(seed))continue;
                var ring=new List<int>{seed};var floor=double.NegativeInfinity;
                for(var j=0;j<ring.Count;j++)
                {
                    token.ThrowIfCancellationRequested();var s=source[ring[j]];
                    foreach(var node in new[]{A(s),B(s)})foreach(var arm in arms[node])
                        if(source[arm.Section].Roundabout&&source[arm.Section].Kind==NetworkKind.Road&&!source[arm.Section].Tunnel&&seen.Add(arm.Section))ring.Add(arm.Section);
                    var curve=Curve(s);var count=Math.Max(1,(int)Math.Ceiling(NetworkDesignChecks.HorizontalArcLength(curve)/5));
                    for(var k=0;k<=count;k++)
                    {
                        var t=k/(double)count;var p=NetworkDesignChecks.Evaluate(curve,t);var d=math.normalizesafe(NetworkDesignChecks.Derivative(curve,t).xz);
                        var normal=new double2(-d.y,d.x)*Math.Max(4,s.NativeWidth/2);
                        for(var side=-1;side<=1;side++)
                        {
                            var level=water(new((float)(p.x+normal.x*side),(float)p.y,(float)(p.z+normal.y*side)));
                            if(level.HasValue&&math.isfinite(level.Value))floor=Math.Max(floor,level.Value+(s.Bridge?WaterLevels.RoadBridgeClearanceMeters:WaterLevels.RoadFreeboardMeters));
                        }
                    }
                }
                if(double.IsNegativeInfinity(floor))continue;rings++;
                foreach(var index in ring)foreach(var node in new[]{Canon(A(source[index])),Canon(B(source[index]))})floors[node]=floor;
            }
            log.Diagnostic($"VISUAL COMPILER roundabout water floors rings={rings}; nodes={floors.Count}; circularGeometryPreserved=true; approachFloorsCoupled=true");
            return floors;
        }

        void EmitStroke(List<(int Section,bool Forward)> path)
        {
            // Stroke polyline with the polyline index of every stroke node.
            var line=new List<double2>();var nodes=new List<SourceNodeKey>();var at=new List<int>();var sectionAt=new List<int>();
            foreach(var (section,forward) in path)
            {
                var p=Points(section);if(!forward)p=Enumerable.Reverse(p).ToList();var s=source[section];
                if(line.Count==0){line.Add(p[0]);sectionAt.Add(section);nodes.Add(Canon(forward?A(s):B(s)));at.Add(0);}
                line.AddRange(p.Skip(1));sectionAt.AddRange(Enumerable.Repeat(section,p.Count-1));nodes.Add(Canon(forward?B(s):A(s)));at.Add(line.Count-1);
            }
            if(streetLines.TryGetValue(strokeOf[path[0].Section],out var streetLine))
            {
                for(var k=0;k<line.Count;k++)line[k]=streetLine.Project(line[k]);
                for(var k=0;k<nodes.Count;k++)line[at[k]]=position[nodes[k]];
            }
            var tolerance=VisualCompilerTable.For(source[path[0].Section]);
            var interiorJunctions=new HashSet<int>(Enumerable.Range(1,Math.Max(0,nodes.Count-2)).Where(Junction).Select(k=>at[k]));
            var unpushed=line.ToList();
            // Side clearance first, so an approach (which only ever moves a point
            // outward) has the last word at a merging end; there it also holds
            // the end at its clearance, so fork clearance, with its wider margin,
            // would only add a step and leaves such an end alone.
            SideClear(line,sectionAt,path,interiorJunctions);
            if(approach.TryGetValue(new Arm(path[0].Section,path[0].Forward),out var startDirection)){Approach(line,true,startDirection,ApproachReach(source[path[0].Section]),VisualCompilerTable.For(source[path[0].Section]).RadiusFloor,interiorJunctions,k=>source[sectionAt[k]].NativeWidth);counts.Merges++;}
            var last=path[path.Count-1];
            if(approach.TryGetValue(new Arm(last.Section,!last.Forward),out var endDirection)){Approach(line,false,endDirection,ApproachReach(source[last.Section]),VisualCompilerTable.For(source[last.Section]).RadiusFloor,interiorJunctions,k=>source[sectionAt[k]].NativeWidth);counts.Merges++;}
            if(!approach.ContainsKey(new Arm(path[0].Section,path[0].Forward))&&forks.TryGetValue(new Arm(path[0].Section,path[0].Forward),out var startForks))foreach(var fork in startForks)Clear(line,true,fork,interiorJunctions);
            if(!approach.ContainsKey(new Arm(last.Section,!last.Forward))&&forks.TryGetValue(new Arm(last.Section,!last.Forward),out var endForks))foreach(var fork in endForks)Clear(line,false,fork,interiorJunctions);
            GuardPushes(line,unpushed,sectionAt,path,interiorJunctions);
            bool Junction(int k)=>k==0||k==nodes.Count-1||arms[nodes[k]].Count>=3||fixedNodes.Contains(nodes[k]);
            bool Port(int k)=>!Junction(k)&&(ports.Contains(nodes[k])||absorbed.ContainsValue(nodes[k]));
            double Stub(int k)=>arms[nodes[k]].Count>=3?Math.Max(10,.75*arms[nodes[k]].Max(a=>source[a.Section].NativeWidth)):0;
            // A port that the alignment cannot pass near is pinned like a junction
            // on a second build, so no topology port is ever dropped.
            var forced=new HashSet<int>();var lastResort=false;var built=Build();
            for(var attempt=0;built.Missed.Count>0;attempt++)
            {if(attempt==3)lastResort=true;forced.UnionWith(built.Missed);built=Build();if(lastResort)break;}
            var (pieces,allCuts,_)=built;if(pieces.Count==0)return;
            counts.Pieces+=pieces.Count;counts.ProjectionMiss+=forced.Count;
            var stations=Stations(pieces);

            // Station of every stroke node on the new alignment, searched forward;
            // a cubic's provenance is every source section its interval overlaps.
            var nodeStation=new double[nodes.Count];var cursor=0;
            var cutStation=allCuts.Where(c=>c.StrokeNode>=0).GroupBy(c=>c.StrokeNode).ToDictionary(g=>g.Key,g=>stations[g.First().Piece+1]);
            for(var k=0;k<nodes.Count;k++)
            {
                if(k==0||cutStation.ContainsKey(k))
                {
                    nodeStation[k]=k==0?0:cutStation[k];if(k>0)nodeStation[k]=Math.Max(nodeStation[k],nodeStation[k-1]);
                    var s=Array.BinarySearch(stations,nodeStation[k]);cursor=Math.Max(0,Math.Min(pieces.Count-1,s<0?~s-1:s));continue;
                }
                var target=k==0?pieces[0].P0:k==nodes.Count-1?pieces[pieces.Count-1].P3:line[at[k]];var best=(Piece:cursor,T:0d,Distance:double.PositiveInfinity);
                for(var p=cursor;p<pieces.Count;p++)
                {
                    for(var q=0;q<=16;q++){var d=math.distance(Evaluate(pieces[p],q/16d),target);if(d<best.Distance)best=(p,q/16d,d);}
                    if(stations[p]-stations[best.Piece+1]>200)break;
                }
                cursor=best.Piece;nodeStation[k]=stations[best.Piece]+best.T*(stations[best.Piece+1]-stations[best.Piece]);
                if(k>0)nodeStation[k]=Math.Max(nodeStation[k],nodeStation[k-1]);
            }
            List<int> Covered(double s0,double s1)
            {
                var list=new List<int>();var middle=(s0+s1)/2;var nearest=0;
                for(var k=0;k<path.Count;k++)
                {
                    if(Math.Min(s1,nodeStation[k+1])-Math.Max(s0,nodeStation[k])>.5)list.Add(k);
                    if(nodeStation[k]<=middle)nearest=k;
                }
                if(list.Count==0)list.Add(nearest);return list;
            }

            // Runs between cuts. Provenance comes from the stroke nodes bounding the
            // run, so every source section between two kept nodes is represented once.
            var runs=new List<(int First,int Last,long StartNode,long EndNode,List<int> Path,PreparedSegment Template)>();
            var startPiece=0;var startNode=nodes[0].Id;var startStroke=0;
            foreach(var cut in allCuts)
            {
                var run=startStroke>=0&&cut.StrokeNode>=0&&cut.StrokeNode!=startStroke
                    ?Enumerable.Range(Math.Min(startStroke,cut.StrokeNode),Math.Abs(cut.StrokeNode-startStroke)).ToList()
                    :Covered(stations[startPiece],stations[cut.Piece+1]);
                var template=source[path[run.OrderByDescending(k=>NetworkDesignChecks.HorizontalArcLength(Curve(source[path[k].Section]))).First()].Section];
                runs.Add((startPiece,cut.Piece,startNode,cut.Node,run,template));
                startPiece=cut.Piece+1;startNode=cut.Node;startStroke=cut.StrokeNode;
            }

            // Filling a bridge gap removes its old structure boundary, not its
            // source identity. A former port absorbed inside one emitted run
            // aliases that run's start; no extra node or short edge is needed.
            for(var k=1;k<nodes.Count-1;k++)
            {
                if(!retiredStructurePorts.Contains(nodes[k])||allCuts.Any(c=>c.Node==nodes[k].Id))continue;
                var run=runs.First(r=>nodeStation[k]>=stations[r.First]-1e-6&&nodeStation[k]<=stations[r.Last+1]+1e-6);
                structureAliases[nodes[k].Id]=run.StartNode;
            }

            // Vertical plan: samples every 10 m with the attributes of their run.
            var length=stations[stations.Length-1];var n=Math.Max(2,(int)Math.Ceiling(length/VisualCompilerProfile.Spacing)+1);
            var plan=new VisualCompilerProfile.Stroke{Kind=runs[0].Template.Kind,Class=tolerance,OneWay=runs[0].Template.OneWay,Classification=runs[0].Template.Classification,
                Station=new double[n],Point=new double2[n],Direction=new double2[n],Mode=new sbyte[n],Layer=new int[n],HalfWidth=new float[n],Grade=new float[n],CutFill=new float[n],
                SampleClass=new string[n],PairBand=new float[n],Water=new float[n]};
            for(var k=0;k<n;k++)
            {
                var s=length*k/(n-1);var r=runs.FindIndex(x=>stations[x.Last+1]>=s-1e-6);var t=runs[r<0?runs.Count-1:r].Template;
                plan.Station[k]=s;plan.Point[k]=At(pieces,stations,s);
                plan.Direction[k]=math.normalizesafe(At(pieces,stations,Math.Min(length,s+.5))-At(pieces,stations,Math.Max(0,s-.5)));
                plan.Mode[k]=(sbyte)(t.Tunnel?-1:t.Bridge?1:0);plan.Layer[k]=t.Layer;plan.HalfWidth[k]=Math.Max(4,t.NativeWidth/2);
                plan.Grade[k]=NetworkGeometrySafety.GradeLimit(t);plan.CutFill[k]=NetworkVerticalProfiles.CutFillBudget(t);
                plan.SampleClass[k]=t.Classification;plan.PairBand[k]=t.OneWay&&!t.Roundabout?(float)VisualCompilerTable.For(t).PairSeparation:0;
                plan.Water[k]=float.NaN;
                if(!t.Tunnel&&(t.Bridge||t.Kind==NetworkKind.Road))
                {
                    var q=new float3((float)plan.Point[k].x,0,(float)plan.Point[k].y);var surface=water?.Invoke(q)??float.NaN;
                    if(t.Kind==NetworkKind.Road&&water!=null)
                    {
                        // Cover the gap between profile samples and small lateral
                        // differences between the polyline and its emitted cubic.
                        var direction=new float3((float)plan.Direction[k].x,0,(float)plan.Direction[k].y);
                        var normal=new float3(-direction.z,0,direction.x)*(plan.HalfWidth[k]+(float)VisualCompilerProfile.Spacing*.5f);
                        for(var along=-1;along<=1;along++)for(var side=-1;side<=1;side++)
                        {
                            if(along==0&&side==0)continue;
                            var edge=water(q+normal*side+direction*(along*(float)VisualCompilerProfile.Spacing*.5f));
                            if(edge.HasValue&&math.isfinite(edge.Value))surface=float.IsNaN(surface)?edge.Value:Math.Max(surface,edge.Value);
                        }
                    }
                    if(!float.IsNaN(sea)&&NetworkProcessor.Sample(q.x,q.z,terrain,scale,width)<sea)surface=float.IsNaN(surface)?sea:Math.Max(surface,sea);
                    plan.Water[k]=surface;
                }
            }
            plan.Pins.Add((0,nodes[0],arms[nodes[0]].Count>=3));
            foreach(var cut in allCuts)if(cut.StrokeNode>0&&Junction(cut.StrokeNode))plan.Pins.Add((stations[cut.Piece+1],nodes[cut.StrokeNode],arms[nodes[cut.StrokeNode]].Count>=3));
            plans.Add(plan);

            // Emission waits for the vertical stage; cubics merge greedily up to 400 m.
            deferred.Add(()=>
            {
                // A cubic's height is a Hermite curve between its end heights and
                // grades; reject merges whose height strays more than 0.3 m from
                // the solved profile (report 64 V5). At 0.15 m, and the class merge
                // tolerance alone in plan, straight roads were cut into many short
                // pieces whose joints read as bumps; both are now doubled.
                bool Vertical(Piece c,double s0,double s1,double limit=.3)
                {
                    var y0=plan.HeightAt(s0);var y3=plan.HeightAt(s1);
                    var y1=y0+math.distance(c.P0,c.P1)*plan.GradeAt(s0);var y2=y3-math.distance(c.P2,c.P3)*plan.GradeAt(s1);
                    for(var q=1;q<4;q++){var t=q/4d;var u=1-t;var y=u*u*u*y0+3*u*u*t*y1+3*u*t*t*y2+t*t*t*y3;if(Math.Abs(y-plan.HeightAt(s0+t*(s1-s0)))>limit)return false;}
                    return true;
                }
                foreach(var run in runs)
                {
                    var merge=2*(run.Template.Kind==NetworkKind.Rail?.3:tolerance.MergeTolerance);var runStartNode=run.StartNode;
                    // Long straights are cut into equal parts of at most 200 m, so no
                    // cubic exceeds the native span and each can follow the profile.
                    var queue=new List<(Piece Piece,double S0,double S1)>();
                    for(var p=run.First;p<=run.Last;p++)
                    {
                        var parts=(int)Math.Ceiling((stations[p+1]-stations[p])/200);var rest=pieces[p];
                        for(var q=parts;q>1;q--){var (left,right)=Split(rest,1d/q);var s=stations[p]+(stations[p+1]-stations[p])*(parts-q)/parts;queue.Add((left,s,s+(stations[p+1]-stations[p])/parts));rest=right;}
                        queue.Add((rest,stations[p]+(stations[p+1]-stations[p])*(parts-1)/parts,stations[p+1]));
                    }
                    var list=queue.Select(q=>q.Piece).ToList();var groups=new List<(int A,int B,Piece Piece)>();
                    for(var a=0;a<queue.Count;)
                    {
                        var b=a;var best=queue[a].Piece;
                        while(b+1<queue.Count&&queue[b+1].S1-queue[a].S0<=MaximumSpan&&
                            Fit(list,a,b+1,merge,out var merged)&&Vertical(merged,queue[a].S0,queue[b+1].S1)){b++;best=merged;}
                        groups.Add((a,b,best));a=b+1;
                    }
                    // A cubic shorter than the class minimum joins a neighbour when one
                    // cubic still fits within twice the merge tolerance and 0.6 m of the
                    // profile: a short piece reads as a kink, a decimetre of drift does not.
                    var minimum=VisualCompilerTable.For(run.Template).MinimumSegment;
                    for(var g=0;g<groups.Count;g++)
                    {
                        var (a,b,_)=groups[g];if(queue[b].S1-queue[a].S0>=minimum)continue;
                        foreach(var other in new[]{g-1,g+1})
                        {
                            if(other<0||other>=groups.Count)continue;
                            var lo=Math.Min(a,groups[other].A);var hi=Math.Max(b,groups[other].B);
                            if(queue[hi].S1-queue[lo].S0>MaximumSpan||!Fit(list,lo,hi,2*merge,out var merged)||!Vertical(merged,queue[lo].S0,queue[hi].S1,.6))continue;
                            groups[Math.Min(g,other)]=(lo,hi,merged);groups.RemoveAt(Math.Max(g,other));counts.Absorbed++;g=Math.Min(g,other)-1;break;
                        }
                    }
                    if(groups.Count==1&&queue[queue.Count-1].S1-queue[0].S0<minimum)counts.ShortRuns++;
                    // A short cubic that cannot join a neighbour whole shares its
                    // length instead: the two are cut at the middle of their union
                    // and refitted as two equal cubics, each at least the minimum.
                    // Short pieces between two plain joints of one road were
                    // Marseille's most common short sections.
                    var cubics=groups.Select(g=>(S0:queue[g.A].S0,S1:queue[g.B].S1,Piece:g.Piece,Merged:g.B>g.A)).ToList();
                    for(var g=0;g<cubics.Count;g++)
                    {
                        if(cubics[g].S1-cubics[g].S0>=minimum)continue;
                        foreach(var other in new[]{g+1,g-1}.Where(o=>o>=0&&o<cubics.Count).OrderByDescending(o=>cubics[o].S1-cubics[o].S0))
                        {
                            var u0=Math.Min(cubics[g].S0,cubics[other].S0);var u1=Math.Max(cubics[g].S1,cubics[other].S1);var middle=(u0+u1)/2;
                            if(u1-u0<2*minimum)continue;
                            var left=new List<Piece>();var right=new List<Piece>();var clipped=false;
                            foreach(var item in queue.Where(q=>q.S1>u0+1e-6&&q.S0<u1-1e-6))
                            {
                                // A previous redistribution can put this pair's
                                // outer boundary inside an original source piece.
                                // Trim that piece before fitting; including its
                                // exterior moves the shared node away from the
                                // neighbouring cubic that still owns that node.
                                var lo=Math.Max(u0,item.S0);var hi=Math.Min(u1,item.S1);var piece=item.Piece;
                                clipped|=lo>item.S0+1e-6||hi<item.S1-1e-6;
                                if(hi<item.S1)piece=Split(piece,(hi-item.S0)/(item.S1-item.S0)).Left;
                                if(lo>item.S0)piece=Split(piece,(lo-item.S0)/(hi-item.S0)).Right;
                                if(hi<=middle)left.Add(piece);else if(lo>=middle)right.Add(piece);
                                else{var (l,r)=Split(piece,(middle-lo)/(hi-lo));left.Add(l);right.Add(r);}
                            }
                            if(!Fit(left,0,left.Count-1,2*merge,out var first)||!Vertical(first,u0,middle,.6)||
                               !Fit(right,0,right.Count-1,2*merge,out var second)||!Vertical(second,middle,u1,.6))continue;
                            // A repeated redistribution must not introduce or
                            // deepen a water intrusion while repairing the join.
                            // Inspect only these clipped pairs, on their actual
                            // Hermite curves and across the full road width.
                            if(clipped&&water!=null&&run.Template.Kind==NetworkKind.Road&&!run.Template.Tunnel)
                            {
                                double Freeboard(Piece c,double s0,double s1)
                                {
                                    var y0=plan.HeightAt(s0);var y3=plan.HeightAt(s1);
                                    double3 P(double2 p,double y)=>new(p.x,y,p.y);
                                    var curve=new Cubic3d(P(c.P0,y0),P(c.P1,y0+math.distance(c.P0,c.P1)*plan.GradeAt(s0)),
                                        P(c.P2,y3-math.distance(c.P2,c.P3)*plan.GradeAt(s1)),P(c.P3,y3));
                                    var freeboard=double.PositiveInfinity;var count=Math.Max(1,(int)Math.Ceiling(Length(c)/5));
                                    for(var k=0;k<=count;k++)
                                    {
                                        var t=k/(double)count;var p=NetworkDesignChecks.Evaluate(curve,t);var d=math.normalizesafe(NetworkDesignChecks.Derivative(curve,t).xz);
                                        var normal=new double2(-d.y,d.x)*Math.Max(4,run.Template.NativeWidth/2);
                                        for(var side=-1;side<=1;side++)
                                        {
                                            var level=water(new((float)(p.x+normal.x*side),(float)p.y,(float)(p.z+normal.y*side)));
                                            if(level.HasValue&&math.isfinite(level.Value))freeboard=Math.Min(freeboard,p.y-level.Value);
                                        }
                                    }
                                    return freeboard;
                                }
                                var prior=Math.Min(Freeboard(cubics[g].Piece,cubics[g].S0,cubics[g].S1),Freeboard(cubics[other].Piece,cubics[other].S0,cubics[other].S1));
                                var revised=Math.Min(Freeboard(first,u0,middle),Freeboard(second,middle,u1));
                                var required=run.Template.Bridge?WaterLevels.RoadBridgeClearanceMeters:WaterLevels.RoadFreeboardMeters;
                                if(revised<Math.Min(prior,required)-.01)continue;
                            }
                            var at=Math.Min(g,other);cubics[at]=(u0,middle,first,true);cubics[at+1]=(middle,u1,second,true);counts.Rebalanced++;break;
                        }
                    }
                    for(var g=0;g<cubics.Count;g++)
                    {
                        var (s0,s1,best,merged)=cubics[g];
                        var endNode=g==cubics.Count-1?run.EndNode:generated++;
                        if(merged)counts.Merged++;
                        if(!Vertical(best,s0,s1))counts.VerticalMisfit++;
                        output.Add(Emit(run.Template,run.Path.Select(k=>path[k].Section).ToList(),best,runStartNode,endNode,plan,s0,s1,!path[run.Path[0]].Forward));
                        runStartNode=endNode;
                    }
                }
            });

            (List<Piece> Pieces,List<Cut> Cuts,List<int> Missed) Build()
            {
                // Douglas–Peucker between pinned nodes; other ports are projected later.
                var keep=new bool[line.Count];var pinnedAt=new Dictionary<int,int>();
                for(var k=0;k<nodes.Count;k++)if(Junction(k)||forced.Contains(k)){keep[at[k]]=true;pinnedAt[at[k]]=k;}                var anchors=Enumerable.Range(0,line.Count).Where(j=>keep[j]).ToArray();
                for(var j=1;j<anchors.Length;j++)Simplify(line,anchors[j-1],anchors[j],tolerance.Epsilon,keep);
                var vertices=Enumerable.Range(0,line.Count).Where(j=>keep[j]).ToArray();
                const double cornerRadius=8;
                // A vertex whose fillet cannot fit (a short tangent beside a
                // junction) would become a node corner: a visible kink on an
                // otherwise smooth road. Where the line without it stays within
                // the class deviation of every source point it spans, the vertex
                // is dropped instead; three passes let neighbours follow.
                double FilletRadius(IReadOnlyList<int> list,int v)
                {
                    var previous=line[list[v-1]];var point=line[list[v]];var following=line[list[v+1]];
                    var inLength=math.distance(previous,point);var outLength=math.distance(point,following);
                    var dirIn=(point-previous)/Math.Max(1e-9,inLength);var dirOut=(following-point)/Math.Max(1e-9,outLength);
                    var half=Math.Abs(Math.Atan2(NetworkDesignChecks.Cross(dirIn,dirOut),math.dot(dirIn,dirOut)))/2;
                    if(half<math.radians(.1))return double.PositiveInfinity;if(half>math.radians(85))return 0;
                    double Available(int neighbour,double length)=>pinnedAt.TryGetValue(list[neighbour],out var k)?length-Stub(k):length/2;
                    var tangent=Math.Max(0,Math.Min(Available(v-1,inLength),Available(v+1,outLength)));
                    return Math.Min(tolerance.RadiusTarget,Math.Min(tangent/Math.Tan(half),tolerance.Deviation/(1/Math.Cos(half)-1)));
                }
                var kept=vertices.ToList();
                for(var pass=0;pass<3;pass++)
                {
                    var dropped=false;
                    for(var v=1;v<kept.Count-1;v++)
                    {
                        if(pinnedAt.ContainsKey(kept[v])||FilletRadius(kept,v)>=cornerRadius)continue;
                        var a=line[kept[v-1]];var b=line[kept[v+1]];var ab=b-a;var worst=0d;
                        for(var j=kept[v-1]+1;j<kept[v+1];j++){var t=math.clamp(math.dot(line[j]-a,ab)/Math.Max(1e-9,math.lengthsq(ab)),0,1);worst=Math.Max(worst,math.distance(line[j],a+ab*t));}
                        if(worst>tolerance.Deviation)continue;
                        kept.RemoveAt(v);v--;dropped=true;counts.CornersDropped++;
                    }
                    if(!dropped)break;
                }
                vertices=kept.ToArray();

                // Straights joined by circular fillets (report 64 V2 step 2, rules R1/R5).
                // A curve tighter than the class floor is still drawn as an arc; only
                // a radius too small to read as a curve becomes a node corner.
                var pieces=new List<Piece>();var cuts=new List<Cut>();var current=line[vertices[0]];
                void Straight(double2 a,double2 b){if(math.distancesq(a,b)>1e-6)pieces.Add(new(a,math.lerp(a,b,1/3d),math.lerp(a,b,2/3d),b));}
                void Boundary(long node,int strokeNode){if(pieces.Count>0)cuts.Add(new(pieces.Count-1,node,strokeNode));}
                for(var v=1;v<vertices.Length;v++)
                {
                    var j=vertices[v];var point=line[j];
                    if(pinnedAt.TryGetValue(j,out var pinnedNode))
                    {Straight(current,point);current=point;Boundary(nodes[pinnedNode].Id,pinnedNode);continue;}
                    var previous=line[vertices[v-1]];var following=line[vertices[v+1]];
                    var inLength=math.distance(previous,point);var outLength=math.distance(point,following);
                    var dirIn=(point-previous)/Math.Max(1e-9,inLength);var dirOut=(following-point)/Math.Max(1e-9,outLength);
                    var turn=Math.Atan2(NetworkDesignChecks.Cross(dirIn,dirOut),math.dot(dirIn,dirOut));var half=Math.Abs(turn)/2;
                    if(half<math.radians(.1)){Straight(current,point);current=point;continue;}
                    double Available(int neighbour,double length)=>pinnedAt.TryGetValue(vertices[neighbour],out var k)?length-Stub(k):length/2;
                    var tangent=Math.Max(0,Math.Min(Available(v-1,inLength),Available(v+1,outLength)));
                    var radius=Math.Min(tolerance.RadiusTarget,Math.Min(tangent/Math.Tan(half),tolerance.Deviation/(1/Math.Cos(half)-1)));
                    if(radius<cornerRadius||half>math.radians(85))
                    {Straight(current,point);current=point;Boundary(generated++,-1);counts.Corners++;continue;}
                    var t=radius*Math.Tan(half);var t1=point-dirIn*t;var t2=point+dirOut*t;
                    Straight(current,t1);Arc(pieces,t1,dirIn,radius,turn);current=t2;
                }
                var missed=new List<int>();if(pieces.Count==0)return (pieces,cuts,missed);

                // Degree-two ports (attribute changes, absorbed targets) keep their
                // identity: split the alignment at their nearest point, in order. A
                // port landing on an existing cut moves to the middle of its piece.
                var from=0;
                for(var k=1;k<nodes.Count-1;k++)if(Port(k)&&!forced.Contains(k))
                {
                    var target=line[at[k]];var best=(Piece:-1,T:0d,Distance:double.PositiveInfinity);
                    void Search(int first){for(var p=first;p<pieces.Count;p++)for(var q=0;q<=32;q++)
                        {var d=math.distance(Evaluate(pieces[p],q/32d),target);if(d<best.Distance)best=(p,q/32d,d);}}
                    Search(from);if(best.Distance>2*tolerance.Deviation)Search(0);
                    if(best.Piece<0||best.Distance>2*tolerance.Deviation&&!lastResort){missed.Add(k);continue;}
                    best=Refine(pieces,target,best.Piece,best.T);
                    counts.Projected++;from=best.Piece;
                    var boundary=best.T<=1e-4?best.Piece-1:best.T>=1-1e-4?best.Piece:-2;
                    if(boundary>=0&&cuts.All(c=>c.Piece!=boundary)){cuts.Add(new(boundary,nodes[k].Id,k));continue;}
                    // A corner node carries no identity: the port takes its place
                    // rather than leaving a 0.5 m piece beside it.
                    var corner=boundary>=0?cuts.FindIndex(c=>c.Piece==boundary&&c.StrokeNode<0):-1;
                    if(corner>=0){cuts[corner]=new(boundary,nodes[k].Id,k);continue;}
                    // On an occupied boundary, split just after it so stroke order holds.
                    var piece=boundary>=0?boundary+1:best.Piece;if(piece>=pieces.Count){missed.Add(k);continue;}
                    var split=boundary==-2?best.T:Math.Min(.5,.5/Math.Max(1,Length(pieces[piece])));
                    var (left,right)=Split(pieces[piece],split);pieces[piece]=left;pieces.Insert(piece+1,right);
                    for(var c=0;c<cuts.Count;c++)if(cuts[c].Piece>=piece)cuts[c]=cuts[c] with{Piece=cuts[c].Piece+1};
                    cuts.Add(new(piece,nodes[k].Id,k));
                }
                cuts=cuts.OrderBy(c=>c.Piece).ToList();
                if(cuts.Count==0||cuts[cuts.Count-1].Piece!=pieces.Count-1)cuts.Add(new(pieces.Count-1,nodes[nodes.Count-1].Id,nodes.Count-1));
                return (pieces,cuts,missed);
            }
        }

        PreparedSegment Emit(PreparedSegment template,List<int> covered,Piece c,long startNode,long endNode,VisualCompilerProfile.Stroke plan,double s0,double s1,bool backwards)
        {
            // Heights and handle slopes come from the solved profile, so adjacent
            // cubics share both height and grade at every joint.
            var kind=template.Kind;var y0=NodeHeight(kind,startNode,(float)plan.HeightAt(s0));var y3=NodeHeight(kind,endNode,(float)plan.HeightAt(s1));
            float3 P(double2 p,float y)=>new((float)p.x,y,(float)p.y);
            var p0=Exact(kind,startNode,P(c.P0,y0));var p3=Exact(kind,endNode,P(c.P3,y3));
            var p1=P(c.P1,(float)(y0+math.distance(c.P0,c.P1)*plan.GradeAt(s0)));var p2=P(c.P2,(float)(y3-math.distance(c.P2,c.P3)*plan.GradeAt(s1)));
            if(backwards){(p0,p3)=(p3,p0);(p1,p2)=(p2,p1);(startNode,endNode)=(endNode,startNode);}
            var ways=covered.SelectMany(i=>source[i].SourceWayIds.Count>0?source[i].SourceWayIds:new[]{source[i].WayId}).Distinct().OrderBy(w=>w).ToArray();
            // An earlier alias onto a node this compiler absorbed follows it.
            var aliases=covered.SelectMany(i=>source[i].SourceNodeAliases)
                .Select(a=>absorbed.TryGetValue(new(kind,a.CanonicalNodeId),out var merged)?a with{CanonicalNodeId=merged.Id}:a)
                .Concat(absorbed.Where(p=>p.Value.Kind==kind&&(p.Value.Id==startNode||p.Value.Id==endNode)).Select(p=>new SourceNodeAlias(p.Key.Id,p.Value.Id,ports.Contains(p.Key))))                .GroupBy(a=>a.SourceNodeId).Select(g=>g.Last()).OrderBy(a=>a.SourceNodeId).ToArray();
            var geometry=covered.SelectMany(i=>source[i].SourceGeometry).ToArray();
            // A tunnel's native elevation is its solved depth below the ground, not
            // an earlier stage's value: the game draws it open unless it is deep.
            float Depth(float3 p)=>p.y-NetworkProcessor.Sample(p.x,p.z,terrain,scale,width);
            var startElevation=template.Tunnel?Depth(p0):template.StartElevation;var endElevation=template.Tunnel?Depth(p3):template.EndElevation;
            return template with{StartNodeId=startNode,EndNodeId=endNode,Start=p0,End=p3,Control1=p1,Control2=p2,StartElevation=startElevation,EndElevation=endElevation,
                StartCurveOffset=default,EndCurveOffset=default,FinalDeckCurve=null,HorizontalParent=null,HorizontalEmissionReference=null,
                SourceWayIds=ways,SourceNodeAliases=aliases,SourceGeometry=geometry,NativeUnitId=-1,
                StartTerminal=template.StartTerminal&&startNode==template.StartNodeId,EndTerminal=template.EndTerminal&&endNode==template.EndNodeId,
                StartMiniRoundabout=template.StartMiniRoundabout&&startNode==template.StartNodeId,EndMiniRoundabout=template.EndMiniRoundabout&&endNode==template.EndNodeId};
        }
        float NodeHeight(NetworkKind kind,long node,float fallback)=>nodeY.TryGetValue(new(kind,node),out var y)?y:fallback;
        float3 Exact(NetworkKind kind,long node,float3 p)
        {
            // Every section meeting at a shared node must submit the identical position.
            var key=new SourceNodeKey(kind,node);if(!arms.ContainsKey(key))return p;
            if(!exact.TryGetValue(key,out var value))exact[key]=value=p;return value;
        }
    }
    private static Cubic3d Curve(PreparedSegment s)=>new(s.CourseStart,s.CourseControl1,s.CourseControl2,s.CourseEnd);

    // A motorway or trunk mapped as separate bridges with short ground ways
    // between (the A7 into Marseille, whose embankment the elevation data
    // carries) was drawn viaduct, embankment, viaduct at about one height. A
    // ground run of at most BridgeGap metres of the same class between two of
    // its bridges, through plain two-arm nodes, becomes bridge at their layer:
    // one straight elevated run. Only between viaducts, bridge runs of at
    // least ViaductRun metres on both sides (the A7's are 109–1,002 m): a
    // motorway at grade over a few 20–40 m underpass bridges stays on the ground.
    private const double BridgeGap=500,ViaductRun=100;
    private static IReadOnlyList<PreparedSegment> BridgeGaps(IReadOnlyList<PreparedSegment> source,ImportLog log,CancellationToken token)
    {
        static bool Major(PreparedSegment s)=>s.Kind==NetworkKind.Road&&!s.Roundabout&&s.Classification is "motorway" or "trunk";
        var arms=new Dictionary<SourceNodeKey,List<int>>();
        for(var i=0;i<source.Count;i++)foreach(var id in new[]{source[i].StartNodeId,source[i].EndNodeId})
        {var key=new SourceNodeKey(source[i].Kind,id);if(!arms.TryGetValue(key,out var list))arms[key]=list=new();list.Add(i);}
        bool Ground(int i)=>!source[i].Bridge&&!source[i].Tunnel;
        // Length of the bridge run starting with section first, entered at node.
        double Viaduct(int first,SourceNodeKey node)
        {
            var total=0d;var current=first;var visited=new HashSet<int>();
            while(source[current].Bridge&&visited.Add(current)&&total<ViaductRun)
            {
                total+=NetworkDesignChecks.HorizontalArcLength(Curve(source[current]));
                node=new SourceNodeKey(node.Kind,source[current].StartNodeId==node.Id?source[current].EndNodeId:source[current].StartNodeId);
                if(arms[node].Count!=2)break;current=arms[node][0]==current?arms[node][1]:arms[node][0];
            }
            return total;
        }
        PreparedSegment[]? result=null;var seen=new bool[source.Count];var runs=0;
        for(var i=0;i<source.Count;i++)
        {
            if(seen[i]||!Major(source[i])||!Ground(i))continue;token.ThrowIfCancellationRequested();
            var chain=new List<int>{i};seen[i]=true;var outer=new[]{-1,-1};var entry=new SourceNodeKey[2];
            for(var side=0;side<2;side++)
            {
                var node=new SourceNodeKey(source[i].Kind,side==0?source[i].StartNodeId:source[i].EndNodeId);var current=i;
                while(arms[node].Count==2)
                {
                    var next=arms[node][0]==current?arms[node][1]:arms[node][0];
                    if(!Major(source[next])||source[next].Classification!=source[i].Classification)break;
                    if(!Ground(next)){if(source[next].Bridge){outer[side]=next;entry[side]=node;}break;}
                    if(seen[next])break;seen[next]=true;chain.Add(next);current=next;
                    node=new SourceNodeKey(source[next].Kind,source[next].StartNodeId==node.Id?source[next].EndNodeId:source[next].StartNodeId);
                }
            }
            if(outer[0]<0||outer[1]<0||chain.Sum(k=>NetworkDesignChecks.HorizontalArcLength(Curve(source[k])))>BridgeGap||
                Viaduct(outer[0],entry[0])<ViaductRun||Viaduct(outer[1],entry[1])<ViaductRun)continue;
            result??=source.ToArray();var layer=Math.Min(source[outer[0]].Layer,source[outer[1]].Layer);
            foreach(var k in chain)result[k]=source[k] with{Bridge=true,Layer=Math.Max(source[k].Layer,layer)};
            runs++;
        }
        log.Count("Ground gaps between bridges built as bridge",runs);
        return result??source;
    }

    // Where ground and elevated road pieces meet at one junction (an exit leaving
    // a ground motorway straight onto a viaduct, one ground lane splitting into
    // two elevated ones), the elevated pieces' railings and piers warp the
    // junction. Each elevated arm there keeps its first JunctionGround metres as
    // a ground piece, on its own slope, when at least that much bridge remains
    // beyond. Junctions of only elevated (or only ground) arms are unchanged.
    private const double JunctionGround=24;
    private static IReadOnlyList<PreparedSegment> GroundAtJunctions(IReadOnlyList<PreparedSegment> source,ImportLog log,CancellationToken token)
    {
        var arms=new Dictionary<SourceNodeKey,List<(int Section,bool Start)>>();
        for(var i=0;i<source.Count;i++)foreach(var start in new[]{true,false})
        {var key=new SourceNodeKey(source[i].Kind,start?source[i].StartNodeId:source[i].EndNodeId);if(!arms.TryGetValue(key,out var list))arms[key]=list=new();list.Add((i,start));}
        var cuts=new Dictionary<int,(bool AtStart,bool AtEnd)>();
        foreach(var node in arms)
        {
            token.ThrowIfCancellationRequested();var list=node.Value;
            if(node.Key.Kind!=NetworkKind.Road||list.Count<3||list.Any(a=>source[a.Section].Tunnel))continue;
            if(!list.Any(a=>!source[a.Section].Bridge)||!list.Any(a=>source[a.Section].Bridge))continue;
            foreach(var (i,start) in list)
            {
                var s=source[i];var length=NetworkDesignChecks.HorizontalArcLength(Curve(s));if(!s.Bridge||length<2*JunctionGround)continue;
                // Both ends need three ground lengths, or the two cuts nearly meet.
                cuts.TryGetValue(i,out var c);var both=start?c.AtEnd:c.AtStart;if(both&&length<3*JunctionGround)continue;
                cuts[i]=start?(true,c.AtEnd):(c.AtStart,true);
            }
        }
        if(cuts.Count==0)return source;
        var occupied=new HashSet<long>(source.SelectMany(s=>new[]{s.StartNodeId,s.EndNodeId}));var next=long.MinValue+43000000;
        long Id(){while(occupied.Contains(next))next++;occupied.Add(next);return next++;}
        double At(PreparedSegment s,double metres)
        {
            var curve=Curve(s);var total=NetworkDesignChecks.HorizontalArcLength(curve);var walked=0d;var previous=curve.P0.xz;
            for(var k=1;k<=200;k++){var p=NetworkDesignChecks.Evaluate(curve,k/200d).xz;walked+=math.distance(previous,p);previous=p;if(walked>=metres)return k/200d;}
            return metres/Math.Max(1e-9,total);
        }
        var result=new List<PreparedSegment>();var grounded=0;
        for(var i=0;i<source.Count;i++)
        {
            var s=source[i];if(!cuts.TryGetValue(i,out var c)){result.Add(s);continue;}
            var rest=s;
            if(c.AtStart)
            {
                var (left,right)=NativeDesignUnits.Split(rest,At(rest,JunctionGround),Id());
                result.Add(left with{Bridge=false,FinalDeckCurve=null,StartElevation=null,EndElevation=null});rest=right with{FinalDeckCurve=null};grounded++;
            }
            if(c.AtEnd)
            {
                var keep=NetworkDesignChecks.HorizontalArcLength(Curve(rest))-JunctionGround;
                var (left,right)=NativeDesignUnits.Split(rest,At(rest,keep),Id());
                result.Add(left with{FinalDeckCurve=null});rest=right with{Bridge=false,FinalDeckCurve=null,StartElevation=null,EndElevation=null};grounded++;
            }
            result.Add(rest);
        }
        log.Count("Elevated junction arms given a ground first piece",grounded);
        return result;
    }

    // An approach raised for clearance stayed a ground piece, and the game walls
    // in a ground road held well above the ground beside it (DC's walled edges
    // sat a median 7.3 m up, under low overpasses). From each bridge end
    // through plain two-arm nodes, the approach is built elevated while it
    // stands more than ElevateHeight above the original ground: whole sections,
    // and the last one split where it comes down (an approach above it for
    // under 5 m stays ground). Junctions, roundabouts and tunnels stop it.
    // With sink, the same from each tunnel end: an approach lowered into a
    // cutting before its portal (VisualCompilerProfile.PortalCut) is built as
    // tunnel while it lies more than SinkDepth below the original ground. Beside
    // the other carriageway's tube the roadbed may not cut, and the Vieux-Port
    // tunnel's northbound approach ran 15 m under the ground.
    private const float SinkDepth=4;
    private static IReadOnlyList<PreparedSegment> ElevateApproaches(IReadOnlyList<PreparedSegment> source,ushort[] terrain,float scale,double width,
        ImportLog log,CancellationToken token,bool sink=false)
    {
        var list=source.ToList();var ends=new Dictionary<SourceNodeKey,List<int>>();
        for(var i=0;i<list.Count;i++)foreach(var id in new[]{list[i].StartNodeId,list[i].EndNodeId})
        {var key=new SourceNodeKey(list[i].Kind,id);if(!ends.TryGetValue(key,out var arms))ends[key]=arms=new();arms.Add(i);}
        float Ground(double3 p)=>NetworkProcessor.Sample((float)p.x,(float)p.z,terrain,scale,width);
        float Above(double3 p)=>(float)p.y-Ground(p);
        // Sunk: below the lowest ground around the road out to its half width
        // plus 6 m (a tunnel asset is wider than its road), so a hillside
        // cutting does not become a tube with its downhill side open.
        float Clearance(PreparedSegment s,double3 p)
        {
            if(!sink)return Above(p);var r=s.NativeWidth/2+6d;var low=Ground(p);
            for(var a=0;a<8;a++)low=Math.Min(low,Ground(p+new double3(Math.Cos(a*Math.PI/4)*r,0,Math.Sin(a*Math.PI/4)*r)));
            return low-(float)p.y;
        }
        var threshold=sink?SinkDepth:ElevateHeight;
        PreparedSegment Elevated(PreparedSegment s)=>sink?s with{Tunnel=true,Layer=Math.Min(s.Layer,-1),FinalDeckCurve=null}:
            s with{Bridge=true,FinalDeckCurve=null,StartElevation=Above(s.CourseStart),EndElevation=Above(s.CourseEnd)};
        var occupied=new HashSet<long>(list.SelectMany(s=>new[]{s.StartNodeId,s.EndNodeId}));var next=long.MinValue+(sink?44000000:41000000);
        var queue=list.Where(s=>sink?s.Tunnel:s.Bridge&&!s.Tunnel).SelectMany(s=>new[]{new SourceNodeKey(s.Kind,s.StartNodeId),new SourceNodeKey(s.Kind,s.EndNodeId)}).Distinct().ToList();
        var queued=new HashSet<SourceNodeKey>(queue);var whole=0;var split=0;
        for(var q=0;q<queue.Count;q++)
        {
            token.ThrowIfCancellationRequested();var node=queue[q];if(ends[node].Count!=2)continue;
            foreach(var i in ends[node].ToArray())
            {
                var s=list[i];if(s.Bridge||s.Tunnel||s.Roundabout)continue;
                var fromStart=s.StartNodeId==node.Id;var curve=Curve(s);var length=NetworkDesignChecks.HorizontalArcLength(curve);
                var n=Math.Max(2,(int)Math.Ceiling(length/2));var down=-1;
                for(var k=0;k<=n;k++)if(Clearance(s,NetworkDesignChecks.Evaluate(curve,fromStart?k/(double)n:1-k/(double)n))<threshold){down=k;break;}
                if(down==0||down>0&&down*length/n<5)continue;
                // Neither piece shorter than the class's shortest section: the
                // elevated piece runs on a little lower, or takes the whole section.
                // A tube is never stretched past where the ground covers it: a
                // sunk piece too short, or leaving too short a remainder, stays ground.
                var minimum=VisualCompilerTable.For(s).MinimumSegment;
                if(sink&&down>0&&(down*length/n<minimum||(n-down)*length/n<minimum))continue;
                if(down>0)down=Math.Max(down,(int)Math.Ceiling(minimum/length*n));
                if(down<0||down>=n||(n-down)*length/n<minimum)
                {
                    list[i]=Elevated(s);whole++;
                    var far=new SourceNodeKey(s.Kind,fromStart?s.EndNodeId:s.StartNodeId);if(queued.Add(far))queue.Add(far);
                    continue;
                }
                while(occupied.Contains(next))next++;var cut=next++;occupied.Add(cut);
                var (left,right)=NativeDesignUnits.Split(s,fromStart?down/(double)n:1-down/(double)n,cut);
                var (raised,ground)=fromStart?(left,right):(right,left);
                list[i]=Elevated(raised);list.Add(ground with{FinalDeckCurve=null});split++;
                var farNode=new SourceNodeKey(s.Kind,fromStart?s.EndNodeId:s.StartNodeId);var farArms=ends[farNode];farArms[farArms.IndexOf(i)]=list.Count-1;
                ends[new SourceNodeKey(s.Kind,cut)]=new List<int>{i,list.Count-1};
            }
        }
        if(sink){log.Count("Tunnel approach sections sunk",whole);log.Count("Tunnel approaches split where they come up",split);}
        else{log.Count("Bridge approach sections elevated",whole);log.Count("Bridge approaches split where they come down",split);}
        return list;
    }

    // Opposite carriageways of one divided road closer than half their widths
    // plus a 2 m median are pushed apart symmetrically, each by half the
    // shortfall (at most 4 m), at their nodes. A node closer than 1 m to its
    // partner is where the carriageways meet and stays put.
    private static Dictionary<SourceNodeKey,double2> Separate(IReadOnlyList<PreparedSegment> source,bool[] contracted,Func<int,List<double2>> curve,
        Func<SourceNodeKey,SourceNodeKey> canon,CancellationToken token)
    {
        bool Candidate(PreparedSegment s)=>s.Kind==NetworkKind.Road&&s.OneWay&&!s.ProtectedLink&&!s.Roundabout&&VisualCompilerTable.For(s).PairSeparation>0;
        var segments=new Dictionary<(int,int),List<(int Section,double2 A,double2 B)>>();
        (int,int) Cell(double2 p)=>((int)Math.Floor(p.x/64),(int)Math.Floor(p.y/64));
        for(var i=0;i<source.Count;i++)if(!contracted[i]&&Candidate(source[i]))
        {
            var p=curve(i);for(var k=1;k<p.Count;k++){var c=Cell((p[k-1]+p[k])/2);if(!segments.TryGetValue(c,out var l))segments[c]=l=new();l.Add((i,p[k-1],p[k]));}
        }
        var shift=new Dictionary<SourceNodeKey,double2>();
        for(var i=0;i<source.Count;i++)
        {
            var s=source[i];if(contracted[i]||!Candidate(s))continue;token.ThrowIfCancellationRequested();
            var direction=math.normalizesafe(((float3)s.CourseEnd).xz-((float3)s.CourseStart).xz);
            foreach(var (node,point) in new[]{(canon(new(s.Kind,s.StartNodeId)),(double2)((float3)s.CourseStart).xz),(canon(new(s.Kind,s.EndNodeId)),(double2)((float3)s.CourseEnd).xz)})
            {
                var best=(Distance:double.PositiveInfinity,Closest:default(double2),Width:0f);var c=Cell(point);
                for(var x=c.Item1-1;x<=c.Item1+1;x++)for(var z=c.Item2-1;z<=c.Item2+1;z++)if(segments.TryGetValue((x,z),out var list))
                    foreach(var (j,a,b) in list)
                    {
                        var o=source[j];if(j==i||o.Classification!=s.Classification||o.Layer!=s.Layer||math.dot(math.normalizesafe(b-a),direction)>-.9)continue;
                        var t=math.clamp(math.dot(point-a,b-a)/Math.Max(1e-9,math.lengthsq(b-a)),0,1);var q=a+(b-a)*t;var d=math.distance(point,q);
                        if(d<best.Distance)best=(d,q,o.NativeWidth);
                    }
                var required=(s.NativeWidth+best.Width)/2+2;
                if(best.Distance<1||best.Distance>=required||best.Distance>VisualCompilerTable.For(s).PairSeparation)continue;
                var push=math.normalizesafe(point-best.Closest)*Math.Min(4,(required-best.Distance)/2);
                if(!shift.TryGetValue(node,out var old)||math.lengthsq(push)>math.lengthsq(old))shift[node]=push;
            }
        }
        return shift;
    }
    private static double2 Evaluate(Piece c,double t){var u=1-t;return u*u*u*c.P0+3*u*u*t*c.P1+3*u*t*t*c.P2+t*t*t*c.P3;}
    private static (Piece Left,Piece Right) Split(Piece c,double t)
    {
        var a=math.lerp(c.P0,c.P1,t);var b=math.lerp(c.P1,c.P2,t);var d=math.lerp(c.P2,c.P3,t);
        var e=math.lerp(a,b,t);var f=math.lerp(b,d,t);var m=math.lerp(e,f,t);
        return (new(c.P0,a,e,m),new(m,f,d,c.P3));
    }
    // Closest point to a target near a coarse sample: golden-section search on
    // the sampled piece and its neighbours, which handles boundary minima.
    private static (int Piece,double T,double Distance) Refine(List<Piece> pieces,double2 target,int piece,double t)
    {
        var best=(Piece:piece,T:t,Distance:math.distance(Evaluate(pieces[piece],t),target));
        for(var p=Math.Max(0,piece-1);p<=Math.Min(pieces.Count-1,piece+1);p++)
        {
            double lo=p==piece?Math.Max(0,t-1/32d):0,hi=p==piece?Math.Min(1,t+1/32d):1;const double g=.6180339887498949;
            for(var i=0;i<60;i++){var a=hi-g*(hi-lo);var b=lo+g*(hi-lo);if(math.distance(Evaluate(pieces[p],a),target)<math.distance(Evaluate(pieces[p],b),target))hi=b;else lo=a;}
            var u=(lo+hi)/2;var d=math.distance(Evaluate(pieces[p],u),target);if(d<best.Distance)best=(p,u,d);
        }
        return best;
    }
    private static double Length(Piece c){var l=0d;var p=c.P0;for(var k=1;k<=16;k++){var q=Evaluate(c,k/16d);l+=math.distance(p,q);p=q;}return l;}
    private static double[] Stations(List<Piece> pieces){var s=new double[pieces.Count+1];for(var k=0;k<pieces.Count;k++)s[k+1]=s[k]+Length(pieces[k]);return s;}
    private static double2 At(List<Piece> pieces,double[] stations,double s)
    {
        var k=Array.BinarySearch(stations,s);if(k<0)k=~k-1;k=Math.Max(0,Math.Min(pieces.Count-1,k));
        return Evaluate(pieces[k],math.clamp((s-stations[k])/Math.Max(1e-9,stations[k+1]-stations[k]),0,1));
    }

    // Circular arc from a tangent point, split into parts of at most 90 degrees;
    // handle length (4/3)·tan(φ/4)·R gives radial error ≈ 2.7e-4·R at 90°.
    private static void Arc(List<Piece> pieces,double2 start,double2 direction,double radius,double turn)
    {
        var side=turn>0?1:-1;var normal=new double2(-direction.y,direction.x)*side;var center=start+normal*radius;
        var parts=Math.Max(1,(int)Math.Ceiling(Math.Abs(turn)/(Math.PI/2)));var step=turn/parts;var angle=Math.Atan2(start.y-center.y,start.x-center.x);
        var k=4/3d*Math.Tan(Math.Abs(step)/4)*radius;
        for(var p=0;p<parts;p++)
        {
            var a0=angle+p*step;var a1=a0+step;
            var q0=center+radius*new double2(Math.Cos(a0),Math.Sin(a0));var q3=center+radius*new double2(Math.Cos(a1),Math.Sin(a1));
            var t0=side*new double2(-Math.Sin(a0),Math.Cos(a0));var t3=side*new double2(-Math.Sin(a1),Math.Cos(a1));
            pieces.Add(new(q0,q0+t0*k,q3-t3*k,q3));
        }
    }

    // One cubic over pieces [a,b] with the end points and end tangents fixed;
    // handle lengths by least squares on arc-length samples (Schneider 1990).
    private static bool Fit(List<Piece> pieces,int a,int b,double tolerance,out Piece result)
    {
        result=default;var samples=new List<double2>();
        for(var p=a;p<=b;p++)for(var q=p==a?0:1;q<=12;q++)samples.Add(Evaluate(pieces[p],q/12d));
        var u=new double[samples.Count];for(var k=1;k<samples.Count;k++)u[k]=u[k-1]+math.distance(samples[k-1],samples[k]);
        var total=u[u.Length-1];if(total<=1e-6)return false;for(var k=0;k<u.Length;k++)u[k]/=total;
        var p0=pieces[a].P0;var p3=pieces[b].P3;
        var t0=math.normalizesafe(pieces[a].P1-pieces[a].P0);var t3=math.normalizesafe(pieces[b].P3-pieces[b].P2);
        double c11=0,c12=0,c22=0,x1=0,x2=0;
        for(var k=0;k<samples.Count;k++)
        {
            var s=u[k];var m=1-s;var b0=m*m*m;var b1=3*m*m*s;var b2=3*m*s*s;var b3=s*s*s;
            var a1=b1*t0;var a2=-b2*t3;var r=samples[k]-(b0+b1)*p0-(b2+b3)*p3;
            c11+=math.dot(a1,a1);c12+=math.dot(a1,a2);c22+=math.dot(a2,a2);x1+=math.dot(a1,r);x2+=math.dot(a2,r);
        }
        var det=c11*c22-c12*c12;if(Math.Abs(det)<1e-12)return false;
        var alpha=(x1*c22-x2*c12)/det;var beta=(c11*x2-c12*x1)/det;var chord=math.distance(p0,p3);
        if(!(alpha>chord*.05&&beta>chord*.05&&alpha<chord&&beta<chord))return false;
        var fitted=new Piece(p0,p0+alpha*t0,p3-beta*t3,p3);
        for(var k=0;k<samples.Count;k++)if(math.distance(Evaluate(fitted,u[k]),samples[k])>tolerance)return false;
        result=fitted;return true;
    }

    private static void Simplify(List<double2> line,int first,int last,double epsilon,bool[] keep)
    {
        var stack=new List<(int A,int B)>{(first,last)};
        while(stack.Count>0)
        {
            var (a,b)=stack[stack.Count-1];stack.RemoveAt(stack.Count-1);if(b<=a+1)continue;var worst=-1;var distance=epsilon;
            var d=line[b]-line[a];var length=math.length(d);
            for(var k=a+1;k<b;k++)
            {
                var v=line[k]-line[a];var e=length<1e-9?math.length(v):Math.Abs(NetworkDesignChecks.Cross(d,v))/length;
                if(e>distance){distance=e;worst=k;}
            }
            if(worst<0)continue;keep[worst]=true;stack.Add((a,worst));stack.Add((worst,b));
        }
    }
}
