using System;
using System.Collections.Generic;
using System.Linq;
using System.Threading;
using ImportEarthLocation.Core;
using Unity.Mathematics;

namespace ImportEarthLocation.Processing;

// Vertical stage of the visual compiler (research report 64 §5.3). Heights are
// absolute metres; stations are horizontal metres along each stroke.
//  1. Target τ: terrain across the road width (median of three), shared between
//     opposite carriageways; structure interiors take the chord between ends.
//  2. Coupled points only (junctions, tagged crossings) are solved together as
//     difference constraints: grade between consecutive points, clearance at
//     crossings, tag-directed bounds. Exact monotone closures: lower (tunnel
//     cover), then raise (bridge clearance). No penalty weights, no iterations
//     to convergence; a relaxation budget overrun is a reported conflict.
//  3. Every stroke is then smoothed alone: Σ(y−τ)² + α Σ(Δ²y)² with
//     α = (Λ/2π)⁴/h⁴, a low-pass filter with cutoff wavelength Λ per class,
//     pinned at its coupled points, with grade and junction-plateau penalties.
public static class VisualCompilerProfile
{
    public const double Spacing=10;
    // Leave interpolation headroom below the emitted curve's hard grade limit.
    private const double GradeShare=.85;
    public sealed class Stroke
    {
        public NetworkKind Kind;public VisualCompilerClass Class;public bool OneWay;public string Classification="";
        public double[] Station=Array.Empty<double>();public double2[] Point=Array.Empty<double2>();public double2[] Direction=Array.Empty<double2>();
        // Per sample: 1 bridge, −1 tunnel, 0 ground; layer; half width; hard grade; cut/fill budget.
        public sbyte[] Mode=Array.Empty<sbyte>();public int[] Layer=Array.Empty<int>();public float[] HalfWidth=Array.Empty<float>();
        public float[] Grade=Array.Empty<float>();public float[] CutFill=Array.Empty<float>();
        public string[] SampleClass=Array.Empty<string>();public float[] PairBand=Array.Empty<float>();
        // Water surface across non-tunnel road samples and bridge samples.
        public float[] Water=Array.Empty<float>();
        // Reachable water floors on dry ground leading to a wet road sample.
        internal double[] WaterApproachFloor=Array.Empty<double>();
        // A wet junction keeps its accepted floor through reconciliation and emission.
        internal Dictionary<SourceNodeKey,double>? WaterNodeFloors;
        // Structure samples whose target a cover or water rule moved: only these
        // get a hard bound, so other tunnels and bridges still smooth freely.
        internal bool[] Constrained=Array.Empty<bool>();
        public List<(double Station,SourceNodeKey Node,bool Junction)> Pins=new();
        public double[] Height=Array.Empty<double>();
        public double HeightAt(double s)
        {
            var k=Array.BinarySearch(Station,s);if(k>=0)return Height[k];k=~k;if(k<=0)return Height[0];if(k>=Station.Length)return Height[Height.Length-1];
            return math.lerp(Height[k-1],Height[k],(s-Station[k-1])/Math.Max(1e-9,Station[k]-Station[k-1]));
        }
        // Centred over one sample spacing so both cubics meeting at a joint read
        // the same grade there.
        public double GradeAt(double s)
        {
            var end=Station[Station.Length-1];var a=Math.Max(0,s-Spacing/2);var b=Math.Min(end,s+Spacing/2);
            return b-a<1e-6?0:(HeightAt(b)-HeightAt(a))/(b-a);
        }
    }

    public static void Solve(IReadOnlyList<Stroke> strokes,ushort[] terrain,float scale,double width,ImportLog log,CancellationToken token,
        IReadOnlyDictionary<SourceNodeKey,double>? requiredNodeFloors=null)
    {
        float Ground(double2 p)=>NetworkProcessor.Sample((float)p.x,(float)p.y,terrain,scale,width);
        // 1. Corridor-robust targets.
        var tau=strokes.Select(s=>s.Point.Select((p,k)=>
        {
            var n=new double2(-s.Direction[k].y,s.Direction[k].x)*s.HalfWidth[k];
            var v=new[]{Ground(p),Ground(p+n),Ground(p-n)};Array.Sort(v);return (double)v[1];
        }).ToArray()).ToArray();
        // Track follows the ground only at long wavelengths: rail targets are the
        // terrain averaged over RailSmoothing either side along the stroke. At
        // Saint-Charles the station tracks followed the 14 m elevation data over
        // terraces and walls, 4–5% within 40 m and 7 m apart across the throat.
        for(var i=0;i<strokes.Count;i++)if(strokes[i].Kind==NetworkKind.Rail)tau[i]=Average(strokes[i],tau[i],RailSmoothing);
        var partner=Pair(strokes,tau,token);var shared=partner.Sum(p=>p.Count(q=>q.Stroke>=0));
        // A water edge can fall beneath only one carriageway's sampled width.
        // Keep the conservative surface evidence common to the paired corridor,
        // before either bridge's whole-deck lift is computed.
        for(var i=0;i<strokes.Count;i++)for(var k=0;k<partner[i].Length;k++)
        {
            var s=strokes[i];var (j,q,_)=partner[i][k];
            if(j<0||s.Kind!=NetworkKind.Road||strokes[j].Kind!=NetworkKind.Road||s.Mode[k]<0||strokes[j].Mode[q]<0||k>=s.Water.Length||q>=strokes[j].Water.Length)continue;
            var a=s.Water[k];var b=strokes[j].Water[q];if(float.IsNaN(a)&&float.IsNaN(b))continue;
            var level=float.IsNaN(a)?b:float.IsNaN(b)?a:Math.Max(a,b);s.Water[k]=level;strokes[j].Water[q]=level;
        }
        for(var i=0;i<strokes.Count;i++){Chord(strokes[i],tau[i]);Structures(strokes[i],tau[i],Ground);}
        WaterApproaches(strokes,tau,partner,requiredNodeFloors);

        // 2. Coupled points.
        var vars=new List<(double Tau,double Lo,double Hi)>();var nodeVar=new Dictionary<SourceNodeKey,int>();
        var strokeVars=strokes.Select(_=>new List<(double Station,int Var)>()).ToArray();
        (double,double) Bounds(Stroke s,int k,double t)
        {
            var bounds=s.Mode[k]>0?(t,t+60):s.Mode[k]<0?(t-80,t-4):(t-s.CutFill[k],t+s.CutFill[k]);
            var floor=WaterFloor(s,k);
            if(!double.IsNegativeInfinity(floor))bounds=(Math.Max(bounds.Item1,floor),Math.Max(bounds.Item2,floor));
            return bounds;
        }
        int Sample(Stroke s,double station){var k=Array.BinarySearch(s.Station,station);if(k<0)k=Math.Min(s.Station.Length-1,~k);return k;}
        for(var i=0;i<strokes.Count;i++)foreach(var pin in strokes[i].Pins)
        {
            var s=strokes[i];var k=Sample(s,pin.Station);
            if(!nodeVar.TryGetValue(pin.Node,out var v))
            {var t=tau[i][k];var (lo,hi)=Bounds(s,k,t);nodeVar[pin.Node]=v=vars.Count;vars.Add((t,lo,hi));}
            else{var old=vars[v];var (lo,hi)=Bounds(s,k,old.Tau);vars[v]=(old.Tau,Math.Min(old.Lo,lo),Math.Max(old.Hi,hi));}
            strokeVars[i].Add((pin.Station,v));
        }
        var clearances=Crossings(strokes,token).ToList();var crossingBase=vars.Count;
        var crossingConstraints=new List<(int Over,int Under,double Clearance)>();
        foreach(var c in clearances)
        {
            int Add(int stroke,int k){var s=strokes[stroke];var (lo,hi)=Bounds(s,k,tau[stroke][k]);var v=vars.Count;vars.Add((tau[stroke][k],lo,hi));strokeVars[stroke].Add((s.Station[k],v));return v;}
            crossingConstraints.Add((Add(c.Over,c.OverSample),Add(c.Under,c.UnderSample),c.Clearance));
        }
        var crossingEnd=vars.Count;
        var edges=new List<(int From,int To,double Bound)>();// h[To] ≤ h[From] + Bound
        // A pin on one carriageway gets a twin on the opposite carriageway, held
        // equal within 0.1 m, so both sides are pinned to one height at one place.
        var twins=0;var twinPairs=new List<(int Pin,int Twin)>();
        for(var i=0;i<strokes.Count;i++)foreach(var (station,v) in strokeVars[i].ToArray())
        {
            // The twin stays on the partner's sample: placed exactly across from the
            // pin it added grade-change and terrain issues (DC +50, Marseille +8).
            var k=Sample(strokes[i],station);var (j,q,_)=partner[i][k];if(j<0)continue;
            var (lo,hi)=Bounds(strokes[j],q,tau[j][q]);var w=vars.Count;vars.Add((tau[j][q],lo,hi));strokeVars[j].Add((strokes[j].Station[q],w));
            edges.Add((v,w,.1));edges.Add((w,v,.1));twinPairs.Add((v,w));twins++;
        }
        for(var i=0;i<strokes.Count;i++)
        {
            var list=strokeVars[i].OrderBy(p=>p.Station).ToList();
            for(var j=1;j<list.Count;j++)
            {
                var d=list[j].Station-list[j-1].Station;var k=Sample(strokes[i],(list[j].Station+list[j-1].Station)/2);
                var b=GradeShare*strokes[i].Grade[k]*Math.Max(d,1);edges.Add((list[j-1].Var,list[j].Var,b));edges.Add((list[j].Var,list[j-1].Var,b));
            }
        }
        // A dry arm sharing a water-crossing node must not broaden away the
        // wet arm's floor. Apply it once to the shared variable before closure.
        var waterFloors=new Dictionary<int,double>();
        if(requiredNodeFloors!=null)foreach(var pair in requiredNodeFloors)if(nodeVar.TryGetValue(pair.Key,out var v))
        {
            var old=vars[v];var floor=pair.Value;
            vars[v]=(Math.Max(old.Tau,floor),Math.Max(old.Lo,floor),Math.Max(old.Hi,floor));waterFloors[v]=floor;
        }
        for(var i=0;i<strokes.Count;i++)if(strokes[i].Kind==NetworkKind.Road)foreach(var (station,v) in strokeVars[i])
        {
            var s=strokes[i];var k=Sample(s,station);var floor=WaterFloor(s,k);if(double.IsNegativeInfinity(floor))continue;
            var old=vars[v];
            vars[v]=(Math.Max(old.Tau,floor),Math.Max(old.Lo,floor),Math.Max(old.Hi,floor));
            waterFloors[v]=Math.Max(waterFloors.GetValueOrDefault(v,double.NegativeInfinity),floor);
        }
        foreach(var s in strokes)
        {
            s.WaterNodeFloors=null;
            foreach(var pin in s.Pins)if(waterFloors.TryGetValue(nodeVar[pin.Node],out var floor))
                (s.WaterNodeFloors??=new())[pin.Node]=floor;
        }
        var h=vars.Select(v=>Math.Min(Math.Max(v.Tau,v.Lo),v.Hi)).ToArray();var conflicts=0;
        var upper=vars.Select(v=>v.Hi).ToArray();var lowerBound=vars.Select(v=>v.Lo).ToArray();
        var down=Adjacency(h.Length,edges.Select(e=>(e.From,e.To,e.Bound)));var up=Adjacency(h.Length,edges.Select(e=>(e.To,e.From,e.Bound)));
        // Grades and bounds alone have no negative cycle: both closures terminate.
        Close(h,down,upper,lower:true,null);Close(h,up,lowerBound,lower:false,null);
        // Each crossing is then added alone. One that cannot be met within its
        // relaxation budget closes a contradictory cycle (report 25's class): it
        // is dropped and counted, never smeared across neighbouring heights.
        var budgetConflicts=0;var boundConflicts=0;var preexisting=h.Select((v,i)=>v>upper[i]+.5).Count(o=>o);
        var floors=new HashSet<int>();var ceilings=new HashSet<int>();var recipes=0;
        // A lowering closure (tunnel crossings, and the dip below) spreads through
        // grade, twin and junction edges with no lower bounds of its own. It must
        // not pull a var below its bound or an accepted crossing below its
        // clearance: at I-395 over the GW Parkway a ramp's dip dragged accepted
        // deck floors 9 m down to the parkway.
        var accepted=new List<(int Over,int Under,double Clearance)>();var loweringRejected=0;
        bool Lowered(double[] before)=>h.Where((v,i)=>v<lowerBound[i]-.5&&before[i]>=lowerBound[i]-.5).Any()||
            accepted.Any(a=>h[a.Over]-h[a.Under]<a.Clearance-.05);
        // A crossing that cannot get its full clearance still takes the most it
        // can, at 60% and then 35%: roads left at one height intersect, and the
        // game joins them (a stray roundabout under I-495 at VA-267).
        var partial=0;var snapshot=new double[h.Length];
        foreach(var full in crossingConstraints)
        {
            var met=false;
            foreach(var share in new[]{1,.6,.35})
            {
                var c=full with{Clearance=full.Clearance*share};
                if(!Attempt(c,out var failed))
                {
                    if(share==1){if(failed)budgetConflicts++;else boundConflicts++;}
                    continue;
                }
                met=true;if(share<1)partial++;break;
            }
            if(!met)conflicts++;
        }
        // One backup buffer serves every attempt; it holds the heights before the
        // current one until the next begins.
        int Overshoot(){var n=0;for(var i=0;i<h.Length;i++)if(h[i]>upper[i]+.5&&snapshot[i]<=upper[i]+.5)n++;return n;}
        bool Attempt((int Over,int Under,double Clearance) c,out bool failedBudget)
        {
            // Only the new constraint can be violated, so relaxation starts from
            // its endpoints alone; the snapshot restores a rejected attempt.
            Array.Copy(h,snapshot,h.Length);var tunnel=vars[c.Under].Hi<vars[c.Under].Tau;
            if(tunnel)down[c.Over].Add((c.Under,-c.Clearance));up[c.Under].Add((c.Over,-c.Clearance));
            var failed=(tunnel?Close(h,down,upper,lower:true,new[]{c.Over}):0)+Close(h,up,lowerBound,lower:false,new[]{c.Under});
            failedBudget=failed>0;
            var overshoot=Overshoot();
            if(failed==0&&overshoot==0&&tunnel&&Lowered(snapshot)){overshoot=1;loweringRejected++;}
            if(failed==0&&overshoot==0){floors.Add(c.Over);ceilings.Add(c.Under);accepted.Add(c);return true;}
            Array.Copy(snapshot,h,h.Length);
            // Recipe before rejection (report 64 V3 step 2b): a ground road beneath
            // may dip into a cutting up to twice its cut budget, taking the whole
            // shortfall, so the bridge need not climb more than its grades allow.
            var under=vars[c.Under];
            if(!tunnel&&under.Lo<under.Tau)
            {
                var originalLow=lowerBound[c.Under];var relaxed=under.Lo-(under.Tau-under.Lo);lowerBound[c.Under]=relaxed;
                h[c.Under]=Math.Max(relaxed,h[c.Under]-(c.Clearance-(h[c.Over]-h[c.Under])));
                var retry=Close(h,down,upper,lower:true,new[]{c.Under})+Close(h,up,lowerBound,lower:false,new[]{c.Under});
                if(retry==0&&Overshoot()==0)
                {
                    if(!Lowered(snapshot)){floors.Add(c.Over);ceilings.Add(c.Under);accepted.Add(c);recipes++;return true;}
                    loweringRejected++;
                }
                lowerBound[c.Under]=originalLow;
            }
            Array.Copy(snapshot,h,h.Length);if(tunnel)down[c.Over].RemoveAt(down[c.Over].Count-1);up[c.Under].RemoveAt(up[c.Under].Count-1);
            return false;
        }

        // 3. Each stroke alone, pinned at its coupled points. Opposite carriageways
        // then take one profile: a second pass pulls both sides firmly to the mean
        // of their first-pass heights, which keeps each side's own pins.
        // Junctions and twins are exact pins (arms must meet). A crossing only
        // bounds its strokes: the bridge stays at or above its clearance height
        // and the road beneath at or below, so smoothing rounds the hump instead
        // of being forced through a cone's apex. Rejected crossings bind nothing.
        bool Crossing(int v)=>v>=crossingBase&&v<crossingEnd;
        // A long water deck (WaterDeck) takes one straight line between the
        // heights of its end pins. Pins and crossings inside it move onto the
        // line; where a crossing floor or the water clearance lies above it, the
        // nearer end pin rises (pivoting about the farther one) and the rise
        // spreads through the grade edges like any floor. Held at several
        // heights, a stiff deck bowed into an arch (14th Street bridges, 12 m).
        // A deck with a crossing over it keeps its solved heights.
        var straightDecks=0;var crestDecks=0;
        for(var i=0;i<strokes.Count;i++)
        {
            var s=strokes[i];
            foreach(var (a,b) in DeckRuns(s))
            {
                // Pins up to DeckReach past either end (a bridge pier node, the
                // approach's last port) belong to the same line.
                var lo=Math.Max(0,a-DeckReach);var hi=Math.Min(s.Mode.Length-1,b+DeckReach);var deck=WaterDeck(s,a,b);var length=s.Station[b]-s.Station[a];
                if(!deck&&length<=LevelDeck)continue;
                var run=strokeVars[i].Select(p=>(K:Sample(s,p.Station),p.Var,p.Station)).Where(p=>p.K>=lo&&p.K<=hi).OrderBy(p=>p.Station).ToList();
                if(run.Count<2||run.Any(p=>ceilings.Contains(p.Var)))continue;
                if(!deck)
                {
                    if(CrestDeck(s,tau[i],run,h,floors,lo,hi))crestDecks++;
                    continue;
                }
                var startPin=run[0];var endPin=run[run.Count-1];var span=endPin.Station-startPin.Station;if(span<LevelDeck)continue;
                double W(double station)=>(station-startPin.Station)/span;
                var ya=h[startPin.Var];var yb=h[endPin.Var];
                var needs=run.Where(p=>floors.Contains(p.Var)).Select(p=>(p.Station,Height:h[p.Var]))
                    .Concat(Enumerable.Range(lo,hi-lo+1).Where(k=>k<s.Water.Length&&!float.IsNaN(s.Water[k])).Select(k=>(Station:s.Station[k],Height:(double)s.Water[k]+WaterClearance)));
                foreach(var (station,height) in needs)
                {
                    var w=W(station);if(w<=0||w>=1||height<=ya+(yb-ya)*w)continue;
                    if(w<.5)ya=height-(yb-height)*w/(1-w);else yb=ya+(height-ya)/w;
                }
                foreach(var (pin,height) in new[]{(startPin.Var,ya),(endPin.Var,yb)})
                    if(height>h[pin]+1e-6){lowerBound[pin]=Math.Max(lowerBound[pin],height);Close(h,up,lowerBound,lower:false,new[]{pin});}
                ya=h[startPin.Var];yb=h[endPin.Var];
                for(var k=1;k<run.Count-1;k++)h[run[k].Var]=ya+(yb-ya)*W(run[k].Station);
                for(var k=lo;k<=hi;k++)tau[i][k]=ya+(yb-ya)*W(s.Station[k]);
                straightDecks++;
            }
        }
        log.Count("Visual compiler straight water decks",straightDecks);log.Count("Visual compiler crested land decks",crestDecks);
        var bounds=strokes.Select((s,i)=>strokeVars[i].Where(p=>floors.Contains(p.Var)||ceilings.Contains(p.Var))
            .Select(p=>(Sample(s,p.Station),h[p.Var],floors.Contains(p.Var))).ToList()).ToArray();
        // The junction plateau levels an arm where it ends at a junction, so arms
        // of different grades meet on a level node. A stroke running through the
        // junction keeps one continuous grade there already; levelling it fought
        // its pin on hillsides (Marseille motorway and trunk junctions: arms 2–4 m
        // off the node).
        var plateaus=strokes.Select(s=>s.Pins.Where(p=>p.Junction&&(p.Station<1||p.Station>s.Station[s.Station.Length-1]-1)).Select(p=>p.Station).ToArray()).ToArray();
        var throughJunctions=MainlineJunctions();
        var pinned=strokes.Select((s,i)=>strokeVars[i].Where(p=>!Crossing(p.Var)).Select(p=>(p.Station,h[p.Var])).ToList()).ToArray();
        // A junction takes its height from the most important road running
        // through it, smoothed as if the junction were not there; the roads that
        // end there are pinned to it. Starting from the ground at the node pinned
        // every road through it to that ground, so a motorway followed the
        // elevation data's bumps at each ramp (I-395 before I-695: 3–4% grade
        // swings within 120 m). Ranks: motorway, trunk, primary, secondary,
        // tertiary, others; a link ranks just below its class. Twins on the
        // opposite carriageway follow, and the bounds above still hold.
        int MainlineJunctions()
        {
            double Rank(Stroke s)=>(s.Classification.Replace("_link","") switch{"motorway"=>0,"trunk"=>1,"primary"=>2,"secondary"=>3,"tertiary"=>4,_=>5})+(s.Classification.EndsWith("_link",StringComparison.Ordinal)?.5:0);
            var junctionVars=new HashSet<int>(nodeVar.Values);var twinVars=new HashSet<int>(twinPairs.Select(p=>p.Twin));
            var owner=new Dictionary<int,(int Stroke,double Station,double Rank)>();
            for(var i=0;i<strokes.Count;i++)
            {
                var s=strokes[i];var end=s.Station[s.Station.Length-1];var rank=Rank(s);
                foreach(var (station,v) in strokeVars[i])
                {
                    if(!junctionVars.Contains(v)||station<1||station>end-1)continue;
                    if(!owner.TryGetValue(v,out var o)||rank<o.Rank)owner[v]=(i,station,rank);
                }
            }
            // Opposite carriageways are smoothed together, as in the final pass:
            // each alone, then both pulled to the mean of the two.
            var groups=owner.GroupBy(o=>o.Value.Stroke).ToDictionary(g=>g.Key,g=>g.ToArray());
            var involved=new HashSet<int>(groups.Keys);
            foreach(var i in groups.Keys.ToArray())foreach(var p in partner[i])if(p.Stroke>=0)involved.Add(p.Stroke);
            var free=new Dictionary<int,List<(double,double)>>();var stiff=new Dictionary<int,double[]>();var first=new Dictionary<int,double[]>();
            foreach(var i in involved)
            {
                token.ThrowIfCancellationRequested();var s=strokes[i];
                var owned=groups.TryGetValue(i,out var mine)?new HashSet<int>(mine.Select(o=>o.Key)):new HashSet<int>();
                free[i]=strokeVars[i].Where(p=>!Crossing(p.Var)&&!owned.Contains(p.Var)&&!twinVars.Contains(p.Var)).Select(p=>(p.Station,h[p.Var])).ToList();
                stiff[i]=Enumerable.Repeat(1d,s.Station.Length).ToArray();
                foreach(var (a,b) in DeckRuns(s))if(WaterDeck(s,a,b))for(var k=a;k<=b;k++)stiff[i][k]=DeckStiffness;
                first[i]=Smooth(s,tau[i],null,free[i],bounds[i],plateaus[i],stiff[i]);
            }
            var set=0;
            foreach(var (i,group) in groups)
            {
                token.ThrowIfCancellationRequested();var s=strokes[i];var y=first[i];
                if(partner[i].Any(p=>p.Stroke>=0))
                {
                    var pull=partner[i].Select((p,k)=>p.Stroke<0?double.NaN:(first[i][k]+At(strokes[p.Stroke],first[p.Stroke],p.Station))/2).ToArray();
                    y=Smooth(s,tau[i],pull,free[i],bounds[i],plateaus[i],stiff[i]);
                }
                foreach(var (v,(_,station,_)) in group)
                {
                    var (k,t)=Bracket(s,station);var height=t>0?math.lerp(y[k],y[k+1],t):y[k];
                    h[v]=math.clamp(height,lowerBound[v],upper[v]);set++;
                }
            }
            foreach(var (v,w) in twinPairs)if(owner.ContainsKey(v))h[w]=h[v];
            return set;
        }
        log.Count("Visual compiler junction heights from the road through them",throughJunctions);
        for(var i=0;i<strokes.Count;i++)LiftDecks(strokes[i],tau[i],bounds[i]);
        for(var i=0;i<strokes.Count;i++)CapDecks(strokes[i],tau[i],pinned[i],bounds[i]);
        var stiffness=new double[strokes.Count][];
        void SmoothAll(IReadOnlyCollection<int> which)
        {
            foreach(var i in which)
            {
                var s=strokes[i];stiffness[i]=Enumerable.Repeat(1d,s.Station.Length).ToArray();
                foreach(var (a,b) in DeckRuns(s))if(WaterDeck(s,a,b))for(var k=a;k<=b;k++)stiffness[i][k]=DeckStiffness;
            }
            foreach(var i in which){token.ThrowIfCancellationRequested();strokes[i].Height=Smooth(strokes[i],tau[i],null,pinned[i],bounds[i],plateaus[i],stiffness[i]);}
            var first=strokes.Select(s=>s.Height).ToArray();var firstStiffness=stiffness.Select(a=>a==null?null:(double[])a.Clone()).ToArray();
            foreach(var i in which)
            {
                if(partner[i].All(p=>p.Stroke<0))continue;token.ThrowIfCancellationRequested();
                var pull=partner[i].Select((p,k)=>p.Stroke<0?double.NaN:(first[i][k]+At(strokes[p.Stroke],first[p.Stroke],p.Station))/2).ToArray();
                // Both carriageways bend with the stiffer side's local stiffness, so
                // the pull to one mean profile is not undone by different humps.
                for(var k=0;k<pull.Length;k++)
                {
                    var (j,_,station)=partner[i][k];if(j<0||firstStiffness[j]==null)continue;var (q,t)=Bracket(strokes[j],station);
                    stiffness[i][k]=Math.Max(firstStiffness[i]![k],Math.Max(firstStiffness[j]![q],t>0?firstStiffness[j]![q+1]:0));
                }
                strokes[i].Height=Smooth(strokes[i],tau[i],pull,pinned[i],bounds[i],plateaus[i],stiffness[i]);
            }
        }
        SmoothAll(Enumerable.Range(0,strokes.Count).ToArray());
        // 4. A pin that a stroke cannot reach (a deck lifted for a crossing beside
        // a junction, held by its curvature limit) left the other arms meeting the
        // node at another height: a step at the junction. The node takes the
        // height its least compliant stroke reached, the twin across the road
        // follows, and the strokes holding them smooth again to meet it.
        var reconciled=0;var reconciliationConflicts=0;
        var wetVariables=new HashSet<int>(Enumerable.Range(0,strokes.Count).Where(i=>strokes[i].Kind==NetworkKind.Road&&strokes[i].Water.Any(v=>!float.IsNaN(v)))
            .SelectMany(i=>strokeVars[i].Select(p=>p.Var)));
        var incoming=new List<int>[h.Length];for(var v=0;v<h.Length;v++)incoming[v]=new();
        for(var v=0;v<h.Length;v++)foreach(var edge in up[v])incoming[edge.To].Add(v);
        for(var round=0;round<ReconcileRounds;round++)
        {
            Array.Copy(h,snapshot,h.Length);
            var reached=new Dictionary<int,(int Priority,double Residual,double Height)>();
            for(var i=0;i<strokes.Count;i++)foreach(var (station,v) in strokeVars[i])
            {
                if(Crossing(v))continue;var y=strokes[i].HeightAt(station);var residual=Math.Abs(y-h[v]);
                var s=strokes[i];var k=Sample(s,station);var classification=s.SampleClass[k];
                var priority=s.Kind!=NetworkKind.Road?4:classification=="motorway"?0:classification=="trunk"?1:
                    classification.StartsWith("motorway",StringComparison.Ordinal)?2:classification.StartsWith("trunk",StringComparison.Ordinal)?3:4;
                if(wetVariables.Contains(v))priority=4;
                if(!reached.TryGetValue(v,out var r)||priority<r.Priority||priority==r.Priority&&residual>r.Residual)reached[v]=(priority,residual,y);
            }
            // Revise one pin at a time. A crowded or infeasible junction must
            // not discard independent corrections elsewhere in the network.
            foreach(var candidate in reached.OrderBy(p=>p.Value.Priority).ThenByDescending(p=>p.Value.Residual).ThenBy(p=>p.Key))
            {
                var (v,(_,residual,y))=candidate;if(residual<=ReconcileTolerance)continue;
                var target=waterFloors.TryGetValue(v,out var floor)?Math.Max(y,floor):y;
                if(Math.Abs(target-h[v])<1e-6)continue;
                var prior=new Dictionary<int,double>{{v,h[v]}};h[v]=target;
                var failed=Close(h,down,upper,lower:true,new[]{v},prior);
                var seeds=new HashSet<int>{v};
                foreach(var change in prior)
                {
                    if(h[change.Key]<lowerBound[change.Key])seeds.Add(change.Key);
                    // A lowered destination can violate a constraint from an
                    // unchanged predecessor, including an accepted crossing.
                    if(h[change.Key]<change.Value)foreach(var from in incoming[change.Key])seeds.Add(from);
                }
                failed+=Close(h,up,lowerBound,lower:false,seeds.ToArray(),prior);
                var outside=false;
                foreach(var change in prior)if(h[change.Key]>Math.Max(upper[change.Key]+.5,snapshot[change.Key])+1e-6){outside=true;break;}
                if(failed>0||outside){foreach(var change in prior)h[change.Key]=change.Value;reconciliationConflicts++;}
            }
            var moved=new HashSet<int>();
            for(var v=0;v<h.Length;v++)if(Math.Abs(h[v]-snapshot[v])>1e-6)moved.Add(v);
            var again=new HashSet<int>();
            for(var i=0;i<strokes.Count;i++)if(strokeVars[i].Any(p=>moved.Contains(p.Var)))
            {
                again.Add(i);pinned[i]=strokeVars[i].Where(p=>!Crossing(p.Var)).Select(p=>(p.Station,h[p.Var])).ToList();
                foreach(var p in partner[i])if(p.Stroke>=0)again.Add(p.Stroke);
            }
            if(again.Count>0)SmoothAll(again.OrderBy(i=>i).ToArray());
            reconciled+=moved.Count;if(again.Count==0)break;
        }
        log.Count("Visual compiler junction pins reconciled",reconciled);
        log.Count("Visual compiler junction reconciliation constraint conflicts",reconciliationConflicts);
        log.Diagnostic($"VISUAL COMPILER junction reconciliation revisedPins={reconciled}; correctionsWithheld={reconciliationConflicts}; protectedWetVariables={wetVariables.Count}; perJunctionConstraintClosure=true; failedCorrectionsRolledBack=true; highwayPriorityAtDryJunctions=true");
        if(Environment.GetEnvironmentVariable("VISUAL_DEBUG_PINS")=="1")
        {
            var worst=new List<(double Residual,string Line)>();
            for(var i=0;i<strokes.Count;i++)foreach(var (station,v) in strokeVars[i])
            {
                if(Crossing(v))continue;var s=strokes[i];var y=s.HeightAt(station);var res=Math.Abs(y-h[v]);if(res<.3)continue;
                var (k,_)=Bracket(s,station);var near=strokeVars[i].Count(p=>Math.Abs(p.Station-station)<30)-1;
                var spread=strokeVars[i].Where(p=>Math.Abs(p.Station-station)<60&&!Crossing(p.Var)).Select(p=>h[p.Var]).DefaultIfEmpty(h[v]);
                worst.Add((res,$"res={res:F2} paired={partner[i][k].Stroke>=0} class={s.Classification} kind={s.Kind} mode={s.Mode[k]} station={station:F0}/{s.Station[s.Station.Length-1]:F0} n={s.Station.Length} pinH={h[v]:F2} solved={y:F2} tau={tau[i][k]:F2} pinsWithin30={near} pinSpan60={spread.Max()-spread.Min():F2} grade={s.Grade[Math.Min(k,s.Grade.Length-1)]:F3} plateauGrade={s.Class.PlateauGrade:F3} plateau={s.Class.PlateauLength} floor={floors.Contains(v)} ceiling={ceilings.Contains(v)} lo={lowerBound[v]:F1} at=({s.Point[k].x:F0},{s.Point[k].y:F0})"));
            }
            log.Diagnostic($"VISUAL DEBUG pins over 0.3 m: {worst.Count}");
            foreach(var (_,line) in worst.OrderByDescending(w=>w.Residual).Take(40))log.Diagnostic("VISUAL DEBUG pin "+line);
        }
        // Where does the solved profile itself exceed the class grade-change rate?
        int nearPin=0,nearCrossing=0,nearStructure=0,elsewhere=0;
        var crossingStations=strokes.Select(_=>new List<double>()).ToArray();
        for(var i=0;i<strokes.Count;i++)foreach(var (station,v) in strokeVars[i])if(v>=crossingBase&&v<crossingEnd)crossingStations[i].Add(station);
        for(var i=0;i<strokes.Count;i++)
        {
            var s=strokes[i];
            for(var k=1;k<s.Height.Length-1;k++)
            {
                var g0=(s.Height[k]-s.Height[k-1])/(s.Station[k]-s.Station[k-1]);var g1=(s.Height[k+1]-s.Height[k])/(s.Station[k+1]-s.Station[k]);
                if(Math.Abs(g1-g0)*100<=10/s.Class.K*(s.Station[k+1]-s.Station[k-1])/2/10)continue;
                if(crossingStations[i].Any(c=>Math.Abs(c-s.Station[k])<200))nearCrossing++;
                else if(s.Pins.Any(p=>Math.Abs(p.Station-s.Station[k])<30))nearPin++;
                else if(s.Mode[k]!=0||s.Mode[Math.Max(0,k-3)]!=0||s.Mode[Math.Min(s.Mode.Length-1,k+3)]!=0)nearStructure++;
                else elsewhere++;
            }
        }
        log.Diagnostic($"VISUAL COMPILER profile grade-change samples nearCrossings={nearCrossing}; nearPins={nearPin}; nearStructureEnds={nearStructure}; elsewhere={elsewhere}");
        var gaps=strokes.SelectMany((s,i)=>partner[i].Select((p,k)=>p.Stroke<0?double.NaN:Math.Abs(s.Height[k]-strokes[p.Stroke].HeightAt(p.Station)))).Where(g=>!double.IsNaN(g)).OrderBy(g=>g).ToArray();
        double Quantile(double q)=>gaps.Length==0?0:gaps[(int)(q*(gaps.Length-1))];
        log.Diagnostic($"VISUAL COMPILER vertical strokes={strokes.Count}; coupledPoints={vars.Count}; junctionPoints={nodeVar.Count}; crossings={crossingConstraints.Count}; pairTwins={twins}; sharedPairSamples={shared}; conflicts={conflicts}; pairGapP50={Quantile(.5):F2}; pairGapP90={Quantile(.9):F2}; pairGapP99={Quantile(.99):F2}; pairGapOver03={gaps.Count(g=>g>.3)}");
        log.Count("Visual compiler vertical conflicts",conflicts);
        log.Diagnostic($"VISUAL COMPILER crossing conflicts budget={budgetConflicts}; bound={boundConflicts}; overshootBeforeCrossings={preexisting}; repairedByDippingRoadBeneath={recipes}; loweringRejectedToKeepAcceptedFloors={loweringRejected}; metAtReducedClearance={partial}");
        log.Count("Visual compiler crossings repaired",recipes);
    }

    // Monotone label-correcting closure. Lower: h[to] = min(h[to], h[from]+b) and
    // h ≤ bound; raise: h[to] = max(h[to], h[from]−b) and h ≥ bound. Returns 1 when
    // the relaxation budget is exhausted, which indicates a contradictory cycle.
    private static List<(int To,double Bound)>[] Adjacency(int count,IEnumerable<(int From,int To,double Bound)> constraints)
    {
        var outgoing=Enumerable.Range(0,count).Select(_=>new List<(int To,double Bound)>()).ToArray();
        foreach(var c in constraints)outgoing[c.From].Add((c.To,c.Bound));
        return outgoing;
    }
    private static int Close(double[] h,List<(int To,double Bound)>[] outgoing,double[] bounds,bool lower,int[]? start,Dictionary<int,double>? prior=null)
    {
        // FIFO worklist as a list with a moving head; the game's references define
        // Queue<T> twice, so the generic queue type is avoided here.
        var queue=new List<int>();var head=0;var queued=new bool[h.Length];
        foreach(var v in start??Enumerable.Range(0,h.Length))
        {
            var value=lower?Math.Min(h[v],bounds[v]):Math.Max(h[v],bounds[v]);
            if(prior!=null&&value!=h[v]&&!prior.ContainsKey(v))prior[v]=h[v];
            h[v]=value;queue.Add(v);queued[v]=true;
        }
        // A whole-network closure may touch everything; one added crossing only
        // ever raises a local cone, so its contradiction is found quickly.
        long budget=start==null?200L*(h.Length+1):20000;
        while(head<queue.Count)
        {
            if(--budget<0)return 1;
            var v=queue[head++];queued[v]=false;if(head>4096&&head*2>queue.Count){queue.RemoveRange(0,head);head=0;}
            foreach(var (to,bound) in outgoing[v])
            {
                var candidate=lower?h[v]+bound:h[v]-bound;
                if(lower?candidate<h[to]-1e-6:candidate>h[to]+1e-6)
                {if(prior!=null&&!prior.ContainsKey(to))prior[to]=h[to];h[to]=candidate;if(!queued[to]){queued[to]=true;queue.Add(to);}}
            }
        }
        return 0;
    }

    // Opposite one-way carriageways of one class within the class separation band
    // see one corridor target: the mean of both sides' targets.
    private static (int Stroke,int Sample,double Station)[][] Pair(IReadOnlyList<Stroke> strokes,double[][] tau,CancellationToken token)
    {
        var partners=strokes.Select(s=>Enumerable.Repeat((-1,-1,0d),s.Point.Length).ToArray()).ToArray();
        var grid=new Dictionary<(int,int),List<(int S,int K)>>();
        (int,int) Cell(double2 p)=>((int)Math.Floor(p.x/64),(int)Math.Floor(p.y/64));
        for(var i=0;i<strokes.Count;i++)if(strokes[i].OneWay)
            for(var k=0;k<strokes[i].Point.Length;k++)if(strokes[i].PairBand[k]>0){var c=Cell(strokes[i].Point[k]);if(!grid.TryGetValue(c,out var l))grid[c]=l=new();l.Add((i,k));}
        var result=tau.Select(t=>(double[])t.Clone()).ToArray();
        foreach(var cell in grid)foreach(var (i,k) in cell.Value)
        {
            token.ThrowIfCancellationRequested();var s=strokes[i];var p=s.Point[k];var band=s.PairBand[k];var best=double.PositiveInfinity;var partner=(-1,-1);
            for(var x=cell.Key.Item1-1;x<=cell.Key.Item1+1;x++)for(var z=cell.Key.Item2-1;z<=cell.Key.Item2+1;z++)if(grid.TryGetValue((x,z),out var list))
                foreach(var (j,q) in list)
                {
                    var o=strokes[j];if(j==i||o.SampleClass[q]!=s.SampleClass[k]||o.Layer[q]!=s.Layer[k]||math.dot(o.Direction[q],s.Direction[k])>-.966)continue;
                    var offset=o.Point[q]-p;var along=Math.Abs(math.dot(offset,s.Direction[k]));var lateral=Math.Abs(NetworkDesignChecks.Cross(s.Direction[k],offset));
                    if(along>Spacing||lateral<5||lateral>band||lateral>=best)continue;best=lateral;partner=(j,q);
                }
            if(partner.Item1<0)continue;
            // The partner sample can lie up to Spacing ahead or behind; on a 4%
            // grade 8 m apart is 0.32 m of height that is not a pair gap. The
            // partner is read where it crosses this sample's cross-section: its
            // stations run against this stroke's, so a sample `a` metres ahead
            // along D is `a` metres past that crossing along its own direction.
            var (pj,pq)=partner;var other=strokes[pj];
            var a=math.dot(other.Point[pq]-p,s.Direction[k])/Math.Max(.966,-math.dot(other.Direction[pq],s.Direction[k]));
            var station=Math.Clamp(other.Station[pq]+a,other.Station[0],other.Station[other.Station.Length-1]);
            var (bi,bt)=Bracket(other,station);if(bt>0&&other.Mode[bi]!=other.Mode[bi+1])station=other.Station[pq];
            result[i][k]=(tau[i][k]+At(other,tau[pj],station))/2;partners[i][k]=(pj,pq,station);
        }
        for(var i=0;i<tau.Length;i++)tau[i]=result[i];
        return partners;
    }

    // A per-sample value read at a station by linear interpolation.
    private static double At(Stroke s,double[] value,double station){var (i,t)=Bracket(s,station);return t<=0?value[i]:value[i]+(value[i+1]-value[i])*t;}

    // Half width (metres) of the moving average under rail targets.
    private const double RailSmoothing=150;
    // A centred moving average by station, in linear time.
    private static double[] Average(Stroke s,double[] values,double half)
    {
        var n=values.Length;var result=new double[n];var prefix=new double[n+1];
        for(var k=0;k<n;k++)prefix[k+1]=prefix[k]+values[k];
        for(int k=0,a=0,b=0;k<n;k++)
        {
            while(s.Station[k]-s.Station[a]>half)a++;
            while(b+1<n&&s.Station[b+1]-s.Station[k]<=half)b++;
            result[k]=(prefix[b+1]-prefix[a])/(b-a+1);
        }
        return result;
    }

    // Structure interiors follow the straight chord between their end targets, so
    // a bridge is not pulled into the valley nor a tunnel up the hill it avoids.
    private static void Chord(Stroke s,double[] tau)
    {
        for(var a=0;a<s.Mode.Length;)
        {
            if(s.Mode[a]==0){a++;continue;}
            var b=a;while(b+1<s.Mode.Length&&s.Mode[b+1]==s.Mode[a])b++;
            var lo=Math.Max(0,a-1);var hi=Math.Min(s.Mode.Length-1,b+1);
            for(var k=a;k<=b;k++)tau[k]=math.lerp(tau[lo],tau[hi],(s.Station[k]-s.Station[lo])/Math.Max(1e-9,s.Station[hi]-s.Station[lo]));
            a=b+1;
        }
    }

    // Metres a bridge deck keeps above the water it crosses.
    private const double WaterClearance=WaterLevels.RoadBridgeClearanceMeters;
    // Grade of the land ramp that carries a deck's water lift back to the ground.
    private const double ApproachGrade=.04;

    private static double WaterFloor(Stroke s,int k)
    {
        if(s.Kind!=NetworkKind.Road||s.Mode[k]<0)return double.NegativeInfinity;
        var floor=k<s.Water.Length&&!float.IsNaN(s.Water[k])
            ?s.Water[k]+(double)(s.Mode[k]>0?WaterClearance:WaterLevels.RoadFreeboardMeters):double.NegativeInfinity;
        return k<s.WaterApproachFloor.Length?Math.Max(floor,s.WaterApproachFloor[k]):floor;
    }

    // Carry a shallow-water lift back onto dry ground before smoothing. Paired
    // ground carriageways share the higher floor; otherwise one bank lifts only
    // one side and the pair pull fights two incompatible terrain targets.
    private static void WaterApproaches(IReadOnlyList<Stroke> strokes,double[][] tau,
        (int Stroke,int Sample,double Station)[][] partner,IReadOnlyDictionary<SourceNodeKey,double>? requiredNodeFloors)
    {
        foreach(var s in strokes)s.WaterApproachFloor=Array.Empty<double>();
        if((requiredNodeFloors==null||requiredNodeFloors.Count==0)&&!strokes.Any(s=>s.Kind==NetworkKind.Road&&s.Water.Any(v=>!float.IsNaN(v))))return;
        var floors=strokes.Select(s=>Enumerable.Range(0,s.Mode.Length).Select(k=>
            s.Kind==NetworkKind.Road&&s.Mode[k]==0&&k<s.Water.Length&&!float.IsNaN(s.Water[k])
                ?s.Water[k]+(double)WaterLevels.RoadFreeboardMeters:double.NegativeInfinity).ToArray()).ToArray();
        if(requiredNodeFloors!=null)for(var i=0;i<strokes.Count;i++)foreach(var pin in strokes[i].Pins)
        {
            var s=strokes[i];if(s.Kind!=NetworkKind.Road||!requiredNodeFloors.TryGetValue(pin.Node,out var floor))continue;
            var k=Array.BinarySearch(s.Station,pin.Station);if(k<0)k=Math.Min(s.Station.Length-1,~k);
            if(s.Mode[k]>=0)floors[i][k]=Math.Max(floors[i][k],floor);
        }
        for(var i=0;i<strokes.Count;i++)for(var k=0;k<floors[i].Length;k++)
        {
            var (j,q,_)=partner[i][k];if(j<0||strokes[i].Mode[k]!=0||strokes[j].Mode[q]!=0)continue;
            var floor=Math.Max(floors[i][k],floors[j][q]);floors[i][k]=floor;floors[j][q]=floor;
        }
        for(var i=0;i<strokes.Count;i++)
        {
            var s=strokes[i];var floor=floors[i];
            for(var k=1;k<floor.Length;k++)if(s.Mode[k]==0&&s.Mode[k-1]==0)
                floor[k]=Math.Max(floor[k],floor[k-1]-ApproachGrade*(s.Station[k]-s.Station[k-1]));
            for(var k=floor.Length-2;k>=0;k--)if(s.Mode[k]==0&&s.Mode[k+1]==0)
                floor[k]=Math.Max(floor[k],floor[k+1]-ApproachGrade*(s.Station[k+1]-s.Station[k]));
            for(var k=0;k<floor.Length;k++)tau[i][k]=Math.Max(tau[i][k],floor[k]);
            // A shared dry junction must retain the shallow approach floor
            // through emission. A bound based only on the maximum grade leaves
            // too little room for the emitted vertical curve at the wet edge.
            s.WaterApproachFloor=floor;
        }
    }
    // Weight of a structure sample's target relative to open road (1), so a deck
    // keeps its planned line instead of sagging into the valley or humping.
    private const double StructureWeight=10;
    // Depth of a tunnel deck below the lowest ground across its width. The game
    // draws a submitted tunnel as an open cut unless it lies deep enough: in the
    // 21 October Marseille and DC readbacks every tunnel section under 10 m was
    // drawn open, most at 10–12 m, and only 7 of 913 deeper than 12 m.
    private const double TunnelDepth=13;
    // Steepest descent of a rail tunnel from its portal (8% made sharp vertical
    // curves at the portals, three times the rail limit).
    private const double RailTunnelDescent=.035;
    // Metres inside a portal left to the game's open-cut portal, and the most a
    // portal is lowered so its bore reaches TunnelDepth beyond them.
    private const double PortalOpen=40,PortalCut=8;

    // Bridges: a deck is level at its higher end when both ends are within 2 m,
    // otherwise the straight chord, and it rises as one piece to clear the
    // water under it. Tunnels keep cover below the actual ground, not just below
    // the chord between portals. An abrupt depth at the portal forced grades
    // steeper than the road allows.
    private static void Structures(Stroke s,double[] tau,Func<double2,float> ground)
    {
        s.Constrained=new bool[s.Mode.Length];
        for(var a=0;a<s.Mode.Length;)
        {
            if(s.Mode[a]==0)
            {
                if(s.Kind==NetworkKind.Road&&a<s.Water.Length&&!float.IsNaN(s.Water[a]))tau[a]=Math.Max(tau[a],s.Water[a]+WaterLevels.RoadFreeboardMeters);
                a++;continue;
            }
            var b=a;while(b+1<s.Mode.Length&&s.Mode[b+1]==s.Mode[a])b++;
            var lo=Math.Max(0,a-1);var hi=Math.Min(s.Mode.Length-1,b+1);
            if(s.Mode[a]>0)
            {
                if(Math.Abs(tau[lo]-tau[hi])<=2){var level=(tau[lo]+tau[hi])/2;for(var k=a;k<=b;k++)tau[k]=level;}
                var lift=0d;for(var k=a;k<=b;k++)if(k<s.Water.Length&&!float.IsNaN(s.Water[k]))lift=Math.Max(lift,s.Water[k]+WaterClearance-tau[k]);
                if(lift>0)
                {
                    for(var k=a;k<=b;k++){tau[k]+=lift;s.Constrained[k]=k<s.Water.Length&&!float.IsNaN(s.Water[k]);}
                    // The lift continues onto the land approach as a ramp at
                    // ApproachGrade, so the deck stays straight instead of bowing
                    // up from banks left at ground level (14th Street bridges).
                    for(var side=-1;side<=1;side+=2)
                    {
                        var end=side<0?a:b;
                        for(var k=end+side;k>=0&&k<s.Mode.Length&&s.Mode[k]==0;k+=side)
                        {
                            var raised=tau[end]-ApproachGrade*Math.Abs(s.Station[k]-s.Station[end]);
                            if(raised<=tau[k])break;tau[k]=raised;
                        }
                    }
                }
            }
            else
            {
                // The lowest ground across the bore: on a hillside the downhill
                // edge stood 10 m below the centreline and the bore's side showed.
                // Track descends no faster than RailTunnelDescent; its portals sit
                // in a longer cutting instead.
                var desired=new double[b-a+1];var descent=new double[b-a+1];
                for(var k=a;k<=b;k++)
                {
                    var side=new double2(-s.Direction[k].y,s.Direction[k].x)*s.HalfWidth[k];
                    desired[k-a]=Math.Min(ground(s.Point[k]),Math.Min(ground(s.Point[k]+side),ground(s.Point[k]-side)))-TunnelDepth;
                    descent[k-a]=s.Kind==NetworkKind.Rail?Math.Min(s.Grade[k],RailTunnelDescent):s.Grade[k];
                }
                // A portal too high for its bore to reach TunnelDepth beyond the
                // first PortalOpen metres (which the game draws open at every
                // portal) is lowered by up to PortalCut, the approach descending
                // to it in a cutting. The Vieux-Port tunnel's northbound portal
                // stood at the quay 66 m behind the southbound one, and its bore
                // could not dive deep enough before the harbour: the tube showed
                // under the water, stacked 11 m above the other carriageway.
                foreach(var (portal,step) in new[]{(lo,-1),(hi,1)})
                {
                    if(s.Mode[portal]!=0)continue;
                    var need=double.PositiveInfinity;
                    for(var k=a;k<=b;k++)
                    {
                        var from=Math.Abs(s.Station[k]-s.Station[portal]);
                        if(from>=PortalOpen)need=Math.Min(need,desired[k-a]+descent[k-a]*from);
                    }
                    if(!(need<tau[portal]))continue;
                    var target=Math.Max(need,tau[portal]-PortalCut);
                    for(var j=portal;j>=0&&j<s.Mode.Length&&s.Mode[j]==0;j+=step)
                    {
                        var v=target+.8*s.Grade[j]*Math.Abs(s.Station[portal]-s.Station[j]);
                        if(v>=tau[j])break;tau[j]=v;
                    }
                }
                for(var k=a;k<=b;k++)
                {
                    // As deep as the bore's own grade allows from both portals: the
                    // descent is out of sight, and every metre short of TunnelDepth is
                    // drawn as an open cut.
                    var reachable=Math.Max(tau[lo]-descent[k-a]*(s.Station[k]-s.Station[lo]),tau[hi]-descent[k-a]*(s.Station[hi]-s.Station[k]));
                    var covered=Math.Max(desired[k-a],reachable);
                    if(covered<tau[k]){tau[k]=covered;s.Constrained[k]=true;}
                }
            }
            a=b+1;
        }
    }

    // A crossing floor inside a short bridge (up to LevelDeck metres) raises the
    // whole deck by the largest shortfall, so it stays level instead of humping
    // over the road below. On a longer one only the deck near the crossing
    // rises, falling away at ApproachGrade along the deck and onto the land
    // beyond: lifting a whole river bridge for an overpass at its far end bowed
    // the 14th Street bridges 13 m up mid-river. Each tunnel sample gets a
    // ceiling at its covered target.
    private const double LevelDeck=200;
    private static void LiftDecks(Stroke s,double[] tau,List<(int Sample,double Height,bool Floor)> bounds)
    {
        for(var a=0;a<s.Mode.Length;)
        {
            if(s.Mode[a]==0)
            {
                var floor=WaterFloor(s,a);if(!double.IsNegativeInfinity(floor))bounds.Add((a,floor,true));
                a++;continue;
            }
            var b=a;while(b+1<s.Mode.Length&&s.Mode[b+1]==s.Mode[a])b++;
            if(s.Mode[a]>0)
            {
                if(s.Station[b]-s.Station[a]<=LevelDeck)
                {
                    var lift=0d;foreach(var (k,height,floor) in bounds)if(floor&&k>=a&&k<=b)lift=Math.Max(lift,height-tau[k]);
                    if(lift>0)for(var k=a;k<=b;k++)tau[k]+=lift;
                }
                else if(WaterDeck(s,a,b))StraightDeck(s,tau,bounds,a,b);
                else foreach(var (k,height,floor) in bounds.ToArray())
                {
                    if(!floor||k<a||k>b||height<=tau[k])continue;
                    for(var side=-1;side<=1;side+=2)
                        for(var m=k;m>=0&&m<s.Mode.Length&&s.Mode[m]>=0;m+=side)
                        {
                            var raised=height-ApproachGrade*Math.Abs(s.Station[m]-s.Station[k]);
                            if(raised<=tau[m])break;tau[m]=raised;
                        }
                }
                for(var k=a;k<=b;k++)if(s.Constrained[k])bounds.Add((k,s.Water[k]+WaterClearance,true));
            }
            else for(var k=a;k<=b;k++)if(s.Constrained[k])bounds.Add((k,tau[k],false));
            a=b+1;
        }
    }

    // A long deck over water (beyond LevelDeck, up to WaterDeckLength) reads as
    // one straight span: the 14th Street bridges arched 12 m over a crossing
    // floor near one end and sagged 2.8 m mid-river. Its target is the straight
    // line between its end heights, pivoted up about the farther end through
    // any crossing floor above it (so an end rises at most twice the
    // shortfall); a raised end ramps onto the land at ApproachGrade. Land
    // viaducts keep following the terrain.
    private const double WaterDeckLength=1500;
    // Samples (Spacing apart) past a water deck's ends whose pins join its line.
    private const int DeckReach=10;
    // Curvature stiffness of a straight water deck, against 1 for open road.
    private const double DeckStiffness=1e4;
    // Maximal runs of bridge samples in a stroke. Joining runs across an island
    // (George Mason Bridge) was tried: it drew approach viaducts into one line
    // and bowed the 14th Street Express spans 4–5 m, so each run stands alone.
    internal static IEnumerable<(int A,int B)> DeckRuns(Stroke s)
    {
        for(var a=0;a<s.Mode.Length;)
        {
            if(s.Mode[a]<=0){a++;continue;}
            var b=a;while(b+1<s.Mode.Length&&s.Mode[b+1]>0)b++;
            yield return (a,b);a=b+1;
        }
    }

    internal static bool WaterDeck(Stroke s,int a,int b)
    {
        if(s.Station[b]-s.Station[a]<=LevelDeck||s.Station[b]-s.Station[a]>WaterDeckLength)return false;
        for(var k=a;k<=b&&k<s.Water.Length;k++)if(!float.IsNaN(s.Water[k]))return true;
        return false;
    }
    // A long land deck with a crossing floor on it: pins fixed by the coupled
    // solve before the floor was lifted left a tent, a 6 m rise within 40 m
    // over the road beneath (I-395 by its south portal). Its pins and targets
    // move onto the taut string over its pins and floors (their upper concave
    // hull), a crest that clears every floor with the gentlest grades the end
    // pins allow. Land viaducts without a crossing keep following the terrain.
    private static bool CrestDeck(Stroke s,double[] tau,List<(int K,int Var,double Station)> run,double[] h,HashSet<int> floors,int lo,int hi)
    {
        if(!run.Any(p=>floors.Contains(p.Var)))return false;
        var hull=new List<(double S,double Y)>();
        foreach(var p in run)
        {
            var point=(S:p.Station,Y:h[p.Var]);
            // Pop while the last two points and the new one turn upward (convex).
            while(hull.Count>=2)
            {
                var (s1,y1)=hull[hull.Count-2];var (s2,y2)=hull[hull.Count-1];
                if((s2-s1)*(point.Y-y1)-(y2-y1)*(point.S-s1)>=0)hull.RemoveAt(hull.Count-1);else break;
            }
            if(hull.Count>0&&point.S-hull[hull.Count-1].S<1e-6){if(point.Y>hull[hull.Count-1].Y)hull[hull.Count-1]=point;continue;}
            hull.Add(point);
        }
        double At(double station)
        {
            if(station<=hull[0].S)return hull[0].Y;if(station>=hull[hull.Count-1].S)return hull[hull.Count-1].Y;
            var k=1;while(hull[k].S<station)k++;
            return math.lerp(hull[k-1].Y,hull[k].Y,(station-hull[k-1].S)/Math.Max(1e-9,hull[k].S-hull[k-1].S));
        }
        foreach(var p in run)h[p.Var]=Math.Max(h[p.Var],At(p.Station));
        for(var k=lo;k<=hi;k++)if(s.Station[k]>=run[0].Station&&s.Station[k]<=run[run.Count-1].Station)tau[k]=Math.Max(tau[k],At(s.Station[k]));
        return true;
    }

    private static void StraightDeck(Stroke s,double[] tau,List<(int Sample,double Height,bool Floor)> bounds,int a,int b)
    {
        var lo=Math.Max(0,a-1);var hi=Math.Min(s.Mode.Length-1,b+1);var span=Math.Max(1e-9,s.Station[hi]-s.Station[lo]);
        double W(int k)=>(s.Station[k]-s.Station[lo])/span;
        var ya=tau[lo];var yb=tau[hi];
        foreach(var (k,height,floor) in bounds)
        {
            if(!floor||k<a||k>b)continue;var w=W(k);if(height<=ya+(yb-ya)*w)continue;
            if(w<.5)ya=height-(yb-height)*w/(1-w);else yb=ya+(height-ya)/w;
        }
        for(var k=lo;k<=hi;k++)tau[k]=Math.Max(k==lo||k==hi?tau[k]:double.NegativeInfinity,ya+(yb-ya)*W(k));
        for(var side=-1;side<=1;side+=2)
        {
            var end=side<0?lo:hi;
            for(var k=end+side;k>=0&&k<s.Mode.Length&&s.Mode[k]==0;k+=side)
            {
                var raised=tau[end]-ApproachGrade*Math.Abs(s.Station[k]-s.Station[end]);
                if(raised<=tau[k])break;tau[k]=raised;
            }
        }
    }

    // A bridge never rises above the highest height anything asks of it: its own
    // targets and end samples, pins and crossing floors on it, plus DeckCapMargin. Ground
    // samples are held near the terrain by their cut/fill budget, but a deck had
    // no such bound, and the curvature-stiff smoothing overshot between nearby
    // junction pins like a stiff beam (George Mason Bridge: 12 m above its own
    // targets mid-river).
    private const double DeckCapMargin=2;
    private static void CapDecks(Stroke s,double[] tau,List<(double Station,double Height)> pins,List<(int Sample,double Height,bool Floor)> bounds)
    {
        var floors=bounds.Where(x=>x.Floor).ToArray();
        for(var a=0;a<s.Mode.Length;)
        {
            if(s.Mode[a]<=0){a++;continue;}
            var b=a;while(b+1<s.Mode.Length&&s.Mode[b+1]>0)b++;
            var lo=Math.Max(0,a-1);var hi=Math.Min(s.Mode.Length-1,b+1);
            var cap=double.NegativeInfinity;for(var k=lo;k<=hi;k++)cap=Math.Max(cap,tau[k]);
            foreach(var (station,height) in pins)if(station>=s.Station[lo]&&station<=s.Station[hi])cap=Math.Max(cap,height);
            foreach(var (k,height,_) in floors)if(k>=lo&&k<=hi)cap=Math.Max(cap,height);
            for(var k=a;k<=b;k++)bounds.Add((k,cap+DeckCapMargin,false));
            a=b+1;
        }
    }

    // A structure sample over a lower-layer (or tunnel) sample of another stroke
    // whose footprints overlap: one crossing per stroke pair, at the closest pair.
    private static IEnumerable<(int Over,int OverSample,int Under,int UnderSample,double Clearance)> Crossings(IReadOnlyList<Stroke> strokes,CancellationToken token)
    {
        var grid=new Dictionary<(int,int),List<(int S,int K)>>();
        (int,int) Cell(double2 p)=>((int)Math.Floor(p.x/32),(int)Math.Floor(p.y/32));
        for(var i=0;i<strokes.Count;i++)for(var k=0;k<strokes[i].Point.Length;k++){var c=Cell(strokes[i].Point[k]);if(!grid.TryGetValue(c,out var l))grid[c]=l=new();l.Add((i,k));}
        var hits=new Dictionary<(int,int),List<(double Distance,int A,int B)>>();
        for(var i=0;i<strokes.Count;i++)
        {
            token.ThrowIfCancellationRequested();var s=strokes[i];
            for(var k=0;k<s.Point.Length;k++)
            {
                if(s.Mode[k]<0)continue;var c=Cell(s.Point[k]);
                for(var x=c.Item1-1;x<=c.Item1+1;x++)for(var z=c.Item2-1;z<=c.Item2+1;z++)if(grid.TryGetValue((x,z),out var list))
                    foreach(var (j,q) in list)
                    {
                        var o=strokes[j];if(j==i)continue;
                        var above=s.Layer[k]>o.Layer[q]||o.Mode[q]<0&&s.Mode[k]>=0&&s.Layer[k]>=o.Layer[q];if(!above)continue;
                        var d=math.distance(s.Point[k],o.Point[q]);if(d>s.HalfWidth[k]+o.HalfWidth[q])continue;
                        // Arms of one junction overlap near their shared node; not a crossing.
                        if(s.Pins.Any(p=>Math.Abs(p.Station-s.Station[k])<80&&o.Pins.Any(r=>r.Node==p.Node)))continue;
                        if(!hits.TryGetValue((i,j),out var list2))hits[(i,j)]=list2=new();list2.Add((d,k,q));
                    }
            }
        }
        // Overlaps of one stroke pair more than 60 m apart along the upper stroke
        // are separate crossings; each is constrained at its closest sample pair.
        foreach(var pair in hits.OrderBy(p=>p.Key.Item1).ThenBy(p=>p.Key.Item2))
        {
            var over=strokes[pair.Key.Item1];var group=new List<(double Distance,int A,int B)>();
            foreach(var hit in pair.Value.OrderBy(h=>h.A).Append((0,int.MaxValue,0)))
            {
                if(group.Count>0&&(hit.A==int.MaxValue||over.Station[hit.A]-over.Station[group[group.Count-1].A]>60))
                {
                    var best=group.OrderBy(h=>h.Distance).First();
                    // Deck to deck: 5 m vehicle clearance (6 m over rail) plus about 1.5 m of structure.
                    yield return (pair.Key.Item1,best.A,pair.Key.Item2,best.B,strokes[pair.Key.Item2].Kind==NetworkKind.Rail?7.5:6.5);group.Clear();
                }
                if(hit.A!=int.MaxValue)group.Add(hit);
            }
        }
    }

    // Banded penalised least squares (Reinsch 1967) with half-bandwidth two:
    // weights 1 on the target, α on squared second differences, a stiff weight on
    // pinned samples, and up to five rounds of first-difference penalties for
    // samples exceeding the hard grade or a junction-plateau grade.
    // Local stiffness stops rising once it passes this (fourfold steps: 1, 4, 16, 64, 256).
    private const double StiffnessCap=1e2;
    // A pin missed by more than this (metres) moves to the height its stroke reached.
    private const double ReconcileTolerance=.3;
    // A second round settles arms the first round moved (DC: 130 → 89 arms over 0.3 m).
    private const int ReconcileRounds=2;
    // A profile never swings above (or below) everything near it: each sample
    // stays within EnvelopeMargin of the highest (lowest) target, pin, crossing
    // floor (ceiling) within EnvelopeReach either side. Hills keep their shape,
    // because the envelope rides the terrain targets themselves, but the
    // curvature-stiffened smoothing can no longer swing like a beam: it lifted
    // I-395 by the Pentagon 30 m above its own terrain and bowed the George
    // Mason Bridge 12 m above its ends.
    private const double EnvelopeReach=300,EnvelopeMargin=2;
    private static (double[] Low,double[] High) Envelope(Stroke s,double[] tau,List<(double Station,double Height)> pins,List<(int Sample,double Height,bool Floor)> bounds)
    {
        var n=tau.Length;var high=(double[])tau.Clone();var low=(double[])tau.Clone();
        foreach(var (station,height) in pins)
        {
            var (i,t)=Bracket(s,station);
            foreach(var k in t>0?new[]{i,i+1}:new[]{i}){high[k]=Math.Max(high[k],height);low[k]=Math.Min(low[k],height);}
        }
        foreach(var (k,height,floor) in bounds)if(floor)high[k]=Math.Max(high[k],height);else low[k]=Math.Min(low[k],height);
        var resultHigh=new double[n];var resultLow=new double[n];
        for(int k=0,a=0,b=0;k<n;k++)
        {
            while(s.Station[k]-s.Station[a]>EnvelopeReach)a++;
            while(b+1<n&&s.Station[b+1]-s.Station[k]<=EnvelopeReach)b++;
            var hi=double.NegativeInfinity;var lo=double.PositiveInfinity;
            for(var j=a;j<=b;j++){hi=Math.Max(hi,high[j]);lo=Math.Min(lo,low[j]);}
            resultHigh[k]=hi+EnvelopeMargin;resultLow[k]=lo-EnvelopeMargin;
        }
        return (resultLow,resultHigh);
    }
    // The sample interval holding a station: Station[I] ≤ station < Station[I+1]
    // at fraction T, or T = 0 on a sample and at either end of the stroke.
    private static (int I,double T) Bracket(Stroke s,double station)
    {
        var k=Array.BinarySearch(s.Station,station);if(k>=0)return (k,0);k=~k;
        if(k<=0)return (0,0);if(k>=s.Station.Length)return (s.Station.Length-1,0);
        return (k-1,(station-s.Station[k-1])/Math.Max(1e-9,s.Station[k]-s.Station[k-1]));
    }

    private static double[] Smooth(Stroke s,double[] tau,double[]? pull,List<(double Station,double Height)> pins,
        List<(int Sample,double Height,bool Floor)> bounds,double[] plateau,double[] stiff)
    {
        var envelope=Envelope(s,tau,pins,bounds);
        var boundWeight=new double[bounds.Count];
        var n=s.Station.Length;if(n==1)return new[]{pins.Count>0?pins[0].Height:tau[0]};
        // Every sample stays inside its local envelope (Envelope), enforced like
        // a bound on each sample.
        var bandWeight=new double[n];var bandTarget=new double[n];
        var h=Spacing;var alpha=Math.Pow(s.Class.Wavelength/(2*Math.PI),4)/Math.Pow(h,4);
        var limit=new double[n-1];
        // Water approaches retain their full existing grade allowance so the
        // smoothing target can reach the accepted deck/shore clearance.
        var waterApproach=s.Kind==NetworkKind.Road&&s.Water.Any(v=>!float.IsNaN(v));
        for(var k=0;k<n-1;k++)
        {
            // Match the coupled grade's 15% headroom: cubic emission
            // interpolates between samples and can be steeper than their chord.
            var mid=(s.Station[k]+s.Station[k+1])/2;limit[k]=s.Grade[k]*(waterApproach?1:GradeShare);
            if(plateau.Any(p=>Math.Abs(p-mid)<=s.Class.PlateauLength))limit[k]=Math.Min(limit[k],s.Class.PlateauGrade);
        }
        var penalty=new double[n-1];var target=new double[n-1];double[] y=tau;
        // Where a vertical curve is sharper than the class K allows, the local
        // stiffness rises fourfold per round, to at most 256×, so a forced hump
        // widens only where it must; a 10⁴ cap pulled roads off terrain. A grade change of 1% per K metres is a second difference of
        // 0.01·h²/K over samples h apart. The caller's stiffness array is updated
        // in place.
        var bend=.01*h*h/s.Class.K;
        for(var round=0;round<8;round++)
        {
            var d0=new double[n];var d1=new double[n];var d2=new double[n];var r=new double[n];
            // The pull to the pair's mean profile scales with the local stiffness,
            // so a stiffened hump still follows the mean instead of its own pins.
            // Weight 1000 cut paired-height failing length 10–14% against 500 with
            // neutral totals; 2000 cut it further but added grade-change issues.
            // A straight water deck's own stiffness (DeckStiffness) would make the
            // pull outweigh its pins, so the pull stops at StiffnessCap.
            for(var k=0;k<n;k++){var weight=s.Mode[k]!=0?StructureWeight:1;d0[k]+=weight;r[k]+=weight*tau[k];if(pull!=null&&!double.IsNaN(pull[k])){var w=1000*Math.Min(stiff[k],StiffnessCap);d0[k]+=w;r[k]+=w*pull[k];}}
            // A pin holds the profile at its own station, where HeightAt reads the
            // emitted node: w(a·y[i] + b·y[i+1] − H)², a = 1 − t, b = t, couples only
            // neighbouring samples, so the system stays pentadiagonal. Rounded to the
            // next sample, it left arms up to 10 m off the node at another height.
            foreach(var (station,value) in pins)
            {
                // Rail curvature stiffness can outweigh the old pin weight.
                // Its shared junction must remain at
                // the coupled height; otherwise the neighbouring rail arm gets
                // an artificial grade when both are emitted at one node.
                var w=s.Kind==NetworkKind.Rail?1e8:1e6;var (i,t)=Bracket(s,station);
                if(t<=0){d0[i]+=w;r[i]+=w*value;continue;}
                var a=1-t;d0[i]+=w*a*a;d0[i+1]+=w*t*t;d1[i]+=w*a*t;r[i]+=w*a*value;r[i+1]+=w*t*value;
            }
            for(var b=0;b<bounds.Count;b++)if(boundWeight[b]>0){d0[bounds[b].Sample]+=boundWeight[b];r[bounds[b].Sample]+=boundWeight[b]*bounds[b].Height;}
            for(var k=0;k<n;k++)if(bandWeight[k]>0){d0[k]+=bandWeight[k];r[k]+=bandWeight[k]*bandTarget[k];}
            for(var k=1;k<n-1;k++)
            {
                // α(y[k-1] − 2y[k] + y[k+1])² contributes the stencil (1,−2,1)ᵀ(1,−2,1).
                var a=alpha*stiff[k];
                d0[k-1]+=a;d0[k]+=4*a;d0[k+1]+=a;d1[k-1]+=-2*a;d1[k]+=-2*a;d2[k-1]+=a;
            }
            for(var k=0;k<n-1;k++)if(penalty[k]>0)
            {
                // μ(y[k+1] − y[k] − t)²
                var mu=penalty[k];d0[k]+=mu;d0[k+1]+=mu;d1[k]+=-mu;r[k]-=mu*target[k];r[k+1]+=mu*target[k];
            }
            y=Pentadiagonal(d0,d1,d2,r);
            var violated=false;
            for(var k=0;k<n-1;k++)
            {
                var step=s.Station[k+1]-s.Station[k];var rise=y[k+1]-y[k];var allowed=limit[k]*step;
                if(Math.Abs(rise)>allowed+1e-3){penalty[k]=penalty[k]==0?100*alpha:penalty[k]*10;target[k]=Math.Sign(rise)*allowed;violated=true;}
            }
            for(var b=0;b<bounds.Count;b++)
            {
                var (k,height,floor)=bounds[b];
                if(floor?y[k]<height-.05:y[k]>height+.05){boundWeight[b]=boundWeight[b]==0?10*(1+alpha):boundWeight[b]*10;violated=true;}
            }
            for(var k=0;k<n;k++)
            {
                var edge=y[k]>envelope.High[k]+.05?envelope.High[k]:y[k]<envelope.Low[k]-.05?envelope.Low[k]:double.NaN;
                if(double.IsNaN(edge))continue;
                bandTarget[k]=edge;bandWeight[k]=bandWeight[k]==0?10*(1+alpha):bandWeight[k]*10;violated=true;
            }
            var raised=(double[])stiff.Clone();
            for(var k=1;k<n-1;k++)
                if(Math.Abs(y[k-1]-2*y[k]+y[k+1])>(s.Mode[k-1]<0&&s.Mode[k]<0&&s.Mode[k+1]<0?bend/VisualCompilerTable.TunnelKShare:bend)&&stiff[k]<StiffnessCap){for(var j=Math.Max(1,k-2);j<=Math.Min(n-2,k+2);j++)raised[j]=Math.Max(raised[j],stiff[k]*4);violated=true;}
            Array.Copy(raised,stiff,n);
            if(!violated)break;
        }
        return y;
    }

    // Solves A·x = r for symmetric positive-definite pentadiagonal A with main
    // diagonal d0 and super-diagonals d1, d2, by banded LDLᵀ in O(n).
    private static double[] Pentadiagonal(double[] d0,double[] d1,double[] d2,double[] r)
    {
        var n=d0.Length;var d=new double[n];var l1=new double[n];var l2=new double[n];
        for(var i=0;i<n;i++)
        {
            var a=d0[i];
            if(i>=1)a-=l1[i-1]*l1[i-1]*d[i-1];
            if(i>=2)a-=l2[i-2]*l2[i-2]*d[i-2];
            d[i]=a;
            if(i+1<n){var b=d1[i];if(i>=1)b-=l1[i-1]*l2[i-1]*d[i-1];l1[i]=b/d[i];}
            if(i+2<n)l2[i]=d2[i]/d[i];
        }
        var z=new double[n];
        for(var i=0;i<n;i++){z[i]=r[i];if(i>=1)z[i]-=l1[i-1]*z[i-1];if(i>=2)z[i]-=l2[i-2]*z[i-2];}
        for(var i=0;i<n;i++)z[i]/=d[i];
        var x=new double[n];
        for(var i=n-1;i>=0;i--){x[i]=z[i];if(i+1<n)x[i]-=l1[i]*x[i+1];if(i+2<n)x[i]-=l2[i]*x[i+2];}
        return x;
    }
}
