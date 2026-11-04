using System;
using System.Collections.Generic;
using System.Linq;
using System.Threading;
using ImportEarthLocation.Core;

namespace ImportEarthLocation.Processing;

// Interchange pass of the vertical stage (step 5 of VisualCompilerProfile.Solve).
// Each stroke was smoothed alone against fixed junction heights, and opposite
// carriageways were only pulled toward the mean of their first passes. Where the
// pins and that pull disagree with the class curvature, the stiffness reached its
// cap and the bend stayed. Here each place still bent sharper than its K, whose
// arms miss their node, or whose carriageways are more than 0.3 m apart is
// solved again with its surroundings: the arms of every junction it touches and
// both carriageways, in one least-squares problem (Smooth's objective summed
// over the strokes) whose junction heights are unknowns and whose carriageways
// pull toward each other directly. The rounds are Smooth's, from the stiffness
// each stroke's own smoothing reached. A window keeps its outer two samples
// fixed, so height and grade continue into the profile outside it, and its
// solution is kept only if it has fewer issues than the profile it replaces and
// no new hard violation. Fixed penalty weights on curvature rows instead (an
// active set) spiked heights at conflicting rows: DC bends rose from 4757 to 6938.
public static partial class VisualCompilerProfile
{
    // Samples either side of a seed (250 m); arms reached through a junction
    // get half. A cluster larger than InterchangeUnknowns, or whose factor
    // would exceed InterchangeProfile entries, keeps its per-stroke profile.
    // A carriageway gap seeds half that. Fourteen rounds against ten, and the
    // second pass, each cut 0.3–1% of issues on Marseille and DC; a 400 m
    // reach or a fourfold stiffness cap added issues.
    private const int InterchangeReach=25,InterchangeRounds=14,InterchangeAttempts=3,InterchangeUnknowns=6000,InterchangeProfile=4_000_000;
    private const int InterchangePasses=2;
    private const double JointPinWeight=1e6,JointTwinWeight=1e4,JointPairWeight=1e3;

    private static (int Clusters,int Accepted,int Unknowns,int Before,int After,int Skipped) Interchanges(IReadOnlyList<Stroke> strokes,double[][] tau,
        (int Stroke,int Sample,double Station)[][] partner,List<(double Station,int Var)>[] strokeVars,double[] h,
        Func<int,bool> movable,Func<int,bool> crossing,List<(int Pin,int Twin)> twinPairs,
        List<(int Sample,double Height,bool Floor)>[] bounds,double[][] plateaus,double[][] stiffness,double[] lowerBound,double[] upper,CancellationToken token)
    {
        // Wet samples (a water floor) stay fixed, as do track and roads with a
        // level crossing.
        var eligible=strokes.Select(s=>s.Kind==NetworkKind.Road&&s.Station.Length>=6&&s.LevelCrossings.Count==0).ToArray();
        var dry=strokes.Select(s=>Enumerable.Range(0,s.Station.Length).Select(k=>double.IsNegativeInfinity(WaterFloor(s,k))).ToArray()).ToArray();
        var bend=strokes.Select(BendLimits).ToArray();
        var occurrences=new Dictionary<int,List<(int Stroke,double Station)>>();
        for(var i=0;i<strokes.Count;i++)foreach(var (station,v) in strokeVars[i])
        {
            if(crossing(v))continue;
            if(!occurrences.TryGetValue(v,out var list))occurrences[v]=list=new();list.Add((i,station));
        }
        int Nearest(int i,double station){var (k,t)=Bracket(strokes[i],station);return t>.5?k+1:k;}
        double Read(int i,double[] y,double station){var (k,t)=Bracket(strokes[i],station);return t>0?y[k]+(y[k+1]-y[k])*t:y[k];}
        int Bent(int i,double[] y,int k)=>k>=1&&k<y.Length-1&&Math.Abs(y[k-1]-2*y[k]+y[k+1])>bend[i][k]?1:0;

        // Seeds: samples bent sharper than the class K, nodes whose arms meet
        // more than ReconcileTolerance apart, and carriageways more than 0.3 m
        // apart (paired-height issues DC 547 to 465, Marseille 193 to 185).
        var mark=strokes.Select(s=>new bool[s.Station.Length]).ToArray();
        void Mark(int i,int k,int reach){for(var j=Math.Max(0,k-reach);j<=Math.Min(mark[i].Length-1,k+reach);j++)mark[i][j]=true;}
        for(var i=0;i<strokes.Count;i++)if(eligible[i])for(var k=1;k<strokes[i].Height.Length-1;k++)if(Bent(i,strokes[i].Height,k)>0)Mark(i,k,InterchangeReach);
        foreach(var (v,list) in occurrences)
        {
            if(list.Count<2||!list.All(o=>eligible[o.Stroke]))continue;
            var heights=list.Select(o=>Read(o.Stroke,strokes[o.Stroke].Height,o.Station)).ToArray();
            if(heights.Max()-heights.Min()>ReconcileTolerance)foreach(var o in list)Mark(o.Stroke,Nearest(o.Stroke,o.Station),InterchangeReach);
        }
        for(var i=0;i<strokes.Count;i++)if(eligible[i])for(var k=0;k<strokes[i].Height.Length;k++)
        {
            var (j,_,station)=partner[i][k];if(j<0||!eligible[j])continue;
            if(Math.Abs(strokes[i].Height[k]-Read(j,strokes[j].Height,station))>.3)Mark(i,k,InterchangeReach/2);
        }
        var seeded=mark.Select(m=>(bool[])m.Clone()).ToArray();
        foreach(var list in occurrences.Values)if(list.Any(o=>eligible[o.Stroke]&&seeded[o.Stroke][Nearest(o.Stroke,o.Station)]))
            foreach(var o in list)if(eligible[o.Stroke])Mark(o.Stroke,Nearest(o.Stroke,o.Station),InterchangeReach/2);
        for(var round=0;round<2;round++)for(var i=0;i<strokes.Count;i++)if(eligible[i])for(var k=0;k<mark[i].Length;k++)
        {
            if(!mark[i][k])continue;var (j,q,_)=partner[i][k];if(j<0||!eligible[j])continue;
            mark[j][q]=true;if(q+1<mark[j].Length)mark[j][q+1]=true;
        }

        // Windows are runs of marked samples; a shared node or a carriageway
        // pair joins two windows into one cluster.
        var runs=new List<(int Stroke,int A,int B)>();var runOf=strokes.Select(s=>Enumerable.Repeat(-1,s.Station.Length).ToArray()).ToArray();
        for(var i=0;i<strokes.Count;i++)for(var k=0;k<mark[i].Length;)
        {
            if(!mark[i][k]){k++;continue;}
            var a=k;while(k<mark[i].Length&&mark[i][k])runOf[i][k++]=runs.Count;runs.Add((i,a,k-1));
        }
        var parent=Enumerable.Range(0,runs.Count).ToArray();
        int Root(int r){while(parent[r]!=r)r=parent[r]=parent[parent[r]];return r;}
        void Union(int a,int b){if(a>=0&&b>=0)parent[Root(a)]=Root(b);}
        foreach(var list in occurrences.Values)
        {
            var first=-1;
            foreach(var o in list){var r=runOf[o.Stroke][Nearest(o.Stroke,o.Station)];if(r<0)continue;if(first<0)first=r;else Union(first,r);}
        }
        for(var r=0;r<runs.Count;r++)
        {
            var (i,a,b)=runs[r];
            for(var k=a;k<=b;k++){var (j,q,_)=partner[i][k];if(j>=0)Union(r,runOf[j][q]);}
        }

        var unknown=strokes.Select(s=>Enumerable.Repeat(-1,s.Station.Length).ToArray()).ToArray();
        int clusters=0,accepted=0,unknowns=0,scoreBefore=0,scoreAfter=0,skipped=0;
        foreach(var cluster in Enumerable.Range(0,runs.Count).GroupBy(Root).Select(g=>g.Select(r=>runs[r]).ToArray()))
        {
            token.ThrowIfCancellationRequested();clusters++;
            var members=cluster.Select(r=>r.Stroke).Distinct().ToArray();
            // Unknowns: run samples inside the two fixed at each open window end.
            var m=0;var cells=new List<(int Stroke,int Sample)>();
            foreach(var (i,a,b) in cluster)
            {
                var last=strokes[i].Station.Length-1;var lo=a>0?a+2:a;var hi=b<last?b-2:b;
                for(var k=lo;k<=hi;k++)if(dry[i][k]){unknown[i][k]=m++;cells.Add((i,k));}
            }
            bool Free(int i,double station){var (k,t)=Bracket(strokes[i],station);return unknown[i][k]>=0&&(t<=0||unknown[i][k+1]>=0);}
            var node=new Dictionary<int,int>();
            var touched=new HashSet<int>();
            foreach(var (i,a,b) in cluster)foreach(var (station,v) in strokeVars[i])
            {
                if(crossing(v))continue;var k=Nearest(i,station);if(k<a||k>b)continue;touched.Add(v);
            }
            foreach(var v in touched)if(movable(v)&&occurrences[v].All(o=>eligible[o.Stroke]&&Free(o.Stroke,o.Station)))node[v]=m++;
            if(m==0||m>InterchangeUnknowns){skipped+=m>0?1:0;Reset();continue;}

            Dictionary<int,JointStroke> Fresh()=>members.ToDictionary(i=>i,i=>
            {
                var s=strokes[i];var pins=strokeVars[i].Where(p=>!crossing(p.Var)).Select(p=>(p.Station,h[p.Var])).ToList();
                var js=new JointStroke(s,GradeLimits(s,plateaus[i]),Envelope(s,tau[i],pins,bounds[i]),bounds[i].Count);
                // Stiffness starts where the stroke's own smoothing left it.
                if(stiffness[i]!=null)Array.Copy(stiffness[i],js.Stiff,js.Stiff.Length);
                else foreach(var (a,b) in DeckRuns(s))if(WaterDeck(s,a,b))for(var k=a;k<=b;k++)js.Stiff[k]=DeckStiffness;
                // A pin whose node stays fixed keeps Smooth's pin-side cap.
                foreach(var (station,v) in strokeVars[i])
                {
                    if(crossing(v)||node.ContainsKey(v))continue;var (q,_)=Bracket(s,station);
                    for(var k=Math.Max(0,q-PinReach);k<=Math.Min(s.Station.Length-1,q+1+PinReach);k++)js.Cap[k]=Math.Min(js.Cap[k],PinStiffnessCap);
                }
                return js;
            });
            var stroke=Fresh();
            // Pins of a node whose arms a solve moved apart weigh a hundredfold
            // more in the next attempt.
            var boost=new Dictionary<int,double>();
            var x=new double[m];
            foreach(var (i,k) in cells)x[unknown[i][k]]=strokes[i].Height[k];
            foreach(var (v,u) in node)x[u]=h[v];
            double Y(double[] z,int i,int k){var u=unknown[i][k];return u>=0?z[u]:strokes[i].Height[k];}
            double At(double[] z,int i,double station){var (k,t)=Bracket(strokes[i],station);return t>0?Y(z,i,k)+(Y(z,i,k+1)-Y(z,i,k))*t:Y(z,i,k);}

            double Spread(double[] z,int v)
            {
                var lo=double.PositiveInfinity;var hi=double.NegativeInfinity;
                foreach(var o in occurrences[v]){var y=At(z,o.Stroke,o.Station);lo=Math.Min(lo,y);hi=Math.Max(hi,y);}
                return hi-lo;
            }
            var initial=touched.ToDictionary(v=>v,v=>Spread(x,v));
            bool Split(double spread,int v)=>spread>Math.Max(ReconcileTolerance,initial[v]+.05);
            // Issues inside the cluster: bent samples, carriageways more than
            // 0.3 m apart, arms that miss their node by more than 0.3 m (three
            // per 0.3 m), samples more than 0.3 m off the ground target (a
            // tenth); and hard violations, which must not grow: grade, deck
            // bounds, and a node whose arms move further apart.
            (int Score,int Hard) Score(double[] z)
            {
                double score=0;var hard=0;
                foreach(var (i,a,b) in cluster)
                {
                    var s=strokes[i];var n=s.Station.Length;
                    for(var k=Math.Max(1,a);k<=Math.Min(n-2,b);k++)
                        if(Math.Abs(Y(z,i,k-1)-2*Y(z,i,k)+Y(z,i,k+1))>bend[i][k])score++;
                    for(var k=a;k<=b;k++)
                    {
                        var (j,_,station)=partner[i][k];if(j>=0&&Math.Abs(Y(z,i,k)-At(z,j,station))>.3)score++;
                        if(s.Mode[k]==0&&Math.Abs(Y(z,i,k)-tau[i][k])>.3)score+=.1;
                        if(k<n-1&&Math.Abs(Y(z,i,k+1)-Y(z,i,k))>s.Grade[k]*(s.Station[k+1]-s.Station[k])+1e-3)hard++;
                    }
                    foreach(var (k,height,floor) in bounds[i])if(k>=a&&k<=b&&(floor?Y(z,i,k)<height-.05:Y(z,i,k)>height+.05))hard++;
                }
                foreach(var v in touched)
                {
                    var spread=Spread(z,v);if(spread>ReconcileTolerance)score+=3*spread/ReconcileTolerance;
                    if(Split(spread,v))hard++;
                }
                return ((int)Math.Round(score*10),hard);
            }

            // Every term is w·(Σ c·y − t)² over at most four heights; a fixed
            // height moves into t.
            var normal=new SkylineNormal(m);var ti=new int[4];var tc=new double[4];var tn=0;var tt=0d;
            void Begin(double target){tn=0;tt=target;}
            void Cell(int i,int k,double c){var u=unknown[i][k];if(u>=0){ti[tn]=u;tc[tn++]=c;}else tt-=c*strokes[i].Height[k];}
            void Node(int v,double c){if(node.TryGetValue(v,out var u)){ti[tn]=u;tc[tn++]=c;}else tt-=c*h[v];}
            void Interpolated(int i,double station,double c){var (k,t)=Bracket(strokes[i],station);Cell(i,k,c*(1-t));if(t>0)Cell(i,k+1,c*t);}
            void End(double w){if(tn>0&&w>0)normal.Add(w,ti,tc,tn,tt);}
            var nodeWeight=new Dictionary<int,(double Target,double Weight)>();
            void Terms()
            {
                foreach(var (i,a,b) in cluster)
                {
                    var s=strokes[i];var js=stroke[i];var n=s.Station.Length;
                    for(var k=a;k<=b;k++)
                    {
                        Begin(tau[i][k]);Cell(i,k,1);End(s.Mode[k]!=0?StructureWeight:1);
                        // Smooth's pull weight, toward the other carriageway itself
                        // rather than the mean of two first passes.
                        var (j,_,station)=partner[i][k];
                        if(j>=0){Begin(0);Cell(i,k,1);Interpolated(j,station,-1);End(JointPairWeight*Math.Min(js.Stiff[k],PullStiffnessCap));}
                        if(js.BandWeight[k]>0){Begin(js.BandTarget[k]);Cell(i,k,1);End(js.BandWeight[k]);}
                        if(k>=1&&k<=n-2){Begin(0);Cell(i,k-1,1);Cell(i,k,-2);Cell(i,k+1,1);End(js.Alpha*js.Stiff[k]);}
                        if(k<n-1&&js.GradeWeight[k]>0){Begin(js.GradeTarget[k]);Cell(i,k+1,1);Cell(i,k,-1);End(js.GradeWeight[k]);}
                    }
                    for(var b2=0;b2<bounds[i].Count;b2++)
                    {
                        var (k,height,_)=bounds[i][b2];if(k<a||k>b||js.BoundWeight[b2]==0)continue;
                        Begin(height);Cell(i,k,1);End(js.BoundWeight[b2]);
                    }
                    foreach(var (station,v) in strokeVars[i])
                    {
                        if(crossing(v))continue;var k=Nearest(i,station);if(k<a||k>b)continue;
                        Begin(0);Interpolated(i,station,1);Node(v,-1);End(JointPinWeight*(boost.TryGetValue(v,out var factor)?factor:1));
                    }
                }
                foreach(var (v,w) in twinPairs)if(node.ContainsKey(v)||node.ContainsKey(w)){Begin(0);Node(v,1);Node(w,-1);End(JointTwinWeight);}
                foreach(var (v,u) in node)if(nodeWeight.TryGetValue(v,out var bound)){Begin(bound.Target);Node(v,1);End(bound.Weight);}
            }
            Terms();
            if(!normal.Order(InterchangeProfile)){skipped++;Reset();continue;}
            unknowns+=m;
            // Rounds as in Smooth: grade penalties from 100α, bound and envelope
            // penalties from 10(1 + α), both tenfold while violated, and the
            // local stiffness fourfold where a curve is sharper than its K.
            var solution=x;var solved=false;var before=Score(x);var after=before;
            for(var attempt=0;attempt<InterchangeAttempts;attempt++)
            {
            if(attempt>0){stroke=Fresh();nodeWeight.Clear();}
            solution=x;solved=false;
            for(var round=0;round<InterchangeRounds;round++)
            {
                token.ThrowIfCancellationRequested();
                normal.Clear();Terms();var z=normal.Solve();if(z==null)break;
                solution=z;solved=true;var violated=false;
                foreach(var (i,a,b) in cluster)
                {
                    var s=strokes[i];var js=stroke[i];var n=s.Station.Length;var alpha=js.Alpha;var raised=(double[])js.Stiff.Clone();
                    for(var k=a;k<=b;k++)
                    {
                        var y=Y(z,i,k);
                        if(k<n-1)
                        {
                            var rise=Y(z,i,k+1)-y;var allowed=js.Limit[k]*(s.Station[k+1]-s.Station[k]);
                            if(Math.Abs(rise)>allowed+1e-3){js.GradeWeight[k]=js.GradeWeight[k]==0?100*alpha:js.GradeWeight[k]*10;js.GradeTarget[k]=Math.Sign(rise)*allowed;violated=true;}
                        }
                        if(k>=1&&k<=n-2&&Math.Abs(Y(z,i,k-1)-2*y+Y(z,i,k+1))>bend[i][k]&&js.Stiff[k]<js.Cap[k])
                        {for(var q=Math.Max(1,k-2);q<=Math.Min(n-2,k+2);q++)raised[q]=Math.Max(raised[q],js.Stiff[k]*4);violated=true;}
                        var edge=y>js.Envelope.High[k]+.05?js.Envelope.High[k]:y<js.Envelope.Low[k]-.05?js.Envelope.Low[k]:double.NaN;
                        if(!double.IsNaN(edge)){js.BandTarget[k]=edge;js.BandWeight[k]=js.BandWeight[k]==0?10*(1+alpha):js.BandWeight[k]*10;violated=true;}
                    }
                    Array.Copy(raised,js.Stiff,raised.Length);
                    for(var b2=0;b2<bounds[i].Count;b2++)
                    {
                        var (k,height,floor)=bounds[i][b2];if(k<a||k>b)continue;
                        if(floor?Y(z,i,k)<height-.05:Y(z,i,k)>height+.05){js.BoundWeight[b2]=js.BoundWeight[b2]==0?10*(1+alpha):js.BoundWeight[b2]*10;violated=true;}
                    }
                }
                foreach(var (v,u) in node)
                {
                    var target=z[u]<lowerBound[v]-.05?lowerBound[v]:z[u]>upper[v]+.05?upper[v]:double.NaN;if(double.IsNaN(target))continue;
                    nodeWeight[v]=(target,nodeWeight.TryGetValue(v,out var old)?old.Weight*10:JointPinWeight);violated=true;
                }
                if(!violated)break;
            }
            after=solved?Score(solution):before;
            var split=touched.Where(v=>Split(Spread(solution,v),v)).ToArray();
            if(!solved||split.Length==0)break;
            foreach(var v in split)boost[v]=boost.TryGetValue(v,out var factor)?factor*100:100;
            }
            if(solved&&after.Hard<=before.Hard&&after.Score<before.Score)
            {
                accepted++;scoreBefore+=before.Score;scoreAfter+=after.Score;
                foreach(var (i,k) in cells)strokes[i].Height[k]=solution[unknown[i][k]];
                foreach(var (v,u) in node)h[v]=solution[u];
            }
            Reset();
            void Reset(){foreach(var (i,k) in cells)unknown[i][k]=-1;}
        }
        return (clusters,accepted,unknowns,scoreBefore,scoreAfter,skipped);
    }

    // Largest second difference over one sample spacing that keeps a vertical
    // curve within the class K: 1% grade change per K metres is 0.01·h²/K.
    private static double[] BendLimits(Stroke s)
    {
        var n=s.Station.Length;var limit=new double[n];var bend=.01*Spacing*Spacing/s.Class.K;
        for(var k=1;k<n-1;k++)limit[k]=s.Mode[k-1]<0&&s.Mode[k]<0&&s.Mode[k+1]<0?bend/VisualCompilerTable.TunnelKShare:bend;
        return limit;
    }

    // Per-stroke penalty state of one interchange cluster.
    private sealed class JointStroke
    {
        public readonly double Alpha;public readonly double[] Limit;public readonly (double[] Low,double[] High) Envelope;
        public readonly double[] Stiff,Cap,BandWeight,BandTarget,GradeWeight,GradeTarget,BoundWeight;
        public JointStroke(Stroke s,double[] limit,(double[] Low,double[] High) envelope,int bounds)
        {
            var n=s.Station.Length;Alpha=Math.Pow(s.Class.Wavelength/(2*Math.PI),4)/Math.Pow(Spacing,4);Limit=limit;Envelope=envelope;
            Stiff=Enumerable.Repeat(1d,n).ToArray();Cap=Enumerable.Repeat(StiffnessCap,n).ToArray();
            BandWeight=new double[n];BandTarget=new double[n];GradeWeight=new double[n];GradeTarget=new double[n];BoundWeight=new double[bounds];
        }
    }

    // Normal equations Σ w·c·cᵀ y = Σ w·c·t of a sparse least-squares problem,
    // reordered by reverse Cuthill–McKee and factored by envelope (skyline)
    // Cholesky: each row stores the columns from its first nonzero to the
    // diagonal. Terms added before Order only record the sparsity pattern.
    private sealed class SkylineNormal
    {
        private readonly int n;private readonly List<int>[] adjacency;
        private int[] position=Array.Empty<int>();private int[] first=Array.Empty<int>();private double[][] row=Array.Empty<double[]>();private double[] rhs=Array.Empty<double>();
        private bool ordered;
        public SkylineNormal(int n){this.n=n;adjacency=Enumerable.Range(0,n).Select(_=>new List<int>()).ToArray();}

        public void Add(double w,int[] index,double[] c,int count,double t)
        {
            if(!ordered)
            {
                for(var a=0;a<count;a++)for(var b=a+1;b<count;b++)if(index[a]!=index[b]){adjacency[index[a]].Add(index[b]);adjacency[index[b]].Add(index[a]);}
                return;
            }
            for(var a=0;a<count;a++)
            {
                var pa=position[index[a]];rhs[pa]+=w*c[a]*t;
                for(var b=0;b<count;b++){var pb=position[index[b]];if(pb<=pa)row[pa][pb-first[pa]]+=w*c[a]*c[b];}
            }
        }

        // False when the factor would hold more than limit entries.
        public bool Order(int limit)
        {
            var neighbours=adjacency.Select(a=>a.Distinct().ToArray()).ToArray();var degree=neighbours.Select(a=>a.Length).ToArray();
            // Breadth-first: the order list is its own queue.
            var order=new List<int>(n);var seen=new bool[n];
            foreach(var start in Enumerable.Range(0,n).OrderBy(u=>degree[u]))
            {
                if(seen[start])continue;seen[start]=true;order.Add(start);
                for(var next=order.Count-1;next<order.Count;next++)
                    foreach(var w in neighbours[order[next]].OrderBy(w=>degree[w]))if(!seen[w]){seen[w]=true;order.Add(w);}
            }
            order.Reverse();position=new int[n];for(var p=0;p<n;p++)position[order[p]]=p;
            first=new int[n];long size=0;
            for(var p=0;p<n;p++){var f=p;foreach(var w in neighbours[order[p]])f=Math.Min(f,position[w]);first[p]=f;size+=p-f+1;}
            if(size>limit)return false;
            row=new double[n][];for(var p=0;p<n;p++)row[p]=new double[p-first[p]+1];rhs=new double[n];ordered=true;
            return true;
        }

        public void Clear(){foreach(var r in row)Array.Clear(r,0,r.Length);Array.Clear(rhs,0,n);}

        // Null when the matrix is not numerically positive definite.
        public double[]? Solve()
        {
            for(var r=0;r<n;r++)
            {
                var fr=first[r];var lr=row[r];
                for(var c=fr;c<=r;c++)
                {
                    var fc=first[c];var lc=row[c];var sum=lr[c-fr];
                    for(var k=Math.Max(fr,fc);k<c;k++)sum-=lr[k-fr]*lc[k-fc];
                    if(c<r)lr[c-fr]=sum/lc[c-fc];
                    else{if(!(sum>0))return null;lr[r-fr]=Math.Sqrt(sum);}
                }
            }
            var z=(double[])rhs.Clone();
            for(var r=0;r<n;r++){var fr=first[r];var lr=row[r];var sum=z[r];for(var k=fr;k<r;k++)sum-=lr[k-fr]*z[k];z[r]=sum/lr[r-fr];}
            for(var r=n-1;r>=0;r--){var fr=first[r];var lr=row[r];z[r]/=lr[r-fr];for(var k=fr;k<r;k++)z[k]-=lr[k-fr]*z[r];}
            var x=new double[n];for(var u=0;u<n;u++)x[u]=z[position[u]];
            return x;
        }
    }
}
