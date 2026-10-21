using System;
using System.Collections.Generic;
using System.Linq;
using System.Threading;
using ImportEarthLocation.Core;
using Unity.Mathematics;

namespace ImportEarthLocation.Processing;

// Source-qualified paths are supplied by the native observer. No proximity
// search may substitute a different road for a missing lineage path.
public static class NetworkObservedQuality
{
    public static void Paired(IReadOnlyList<PreparedSegment> sections,Func<PreparedSegment,IReadOnlyList<Cubic3d>?> observe,
        ImportLog log,CancellationToken token,string scope)
    {
        var checkedPairs=0;var expected=0;var failed=0;var unknown=0;var worst=0d;var affected=0d;var witnesses=new List<object>();
        foreach(var group in sections.Where(s=>s.PairedProfileId!=0).GroupBy(s=>(s.PairedProfileId,
            Begin:Math.Round(Math.Min(s.PairedReferenceBegin,s.PairedReferenceEnd),6),End:Math.Round(Math.Max(s.PairedReferenceBegin,s.PairedReferenceEnd),6))))
        {
            token.ThrowIfCancellationRequested();expected++;var pair=group.ToArray();if(pair.Length!=2){unknown++;continue;}
            var a=pair[0];var b=pair[1];var ac=observe(a);var bc=observe(b);
            if(ac==null||bc==null||ac.Count==0||bc.Count==0){unknown++;continue;}
            var sameDirection=Math.Sign(a.PairedReferenceEnd-a.PairedReferenceBegin)==Math.Sign(b.PairedReferenceEnd-b.PairedReferenceBegin);
            var pairWorst=0d;var unavailable=false;
            for(var station=0;station<=16;station++)
            {
                var t=station/16d;var pa=NetworkDesignChecks.Evaluate(new(a.CourseStart,a.CourseControl1,a.CourseControl2,a.CourseEnd),t);
                var pb=NetworkDesignChecks.Evaluate(new(b.CourseStart,b.CourseControl1,b.CourseControl2,b.CourseEnd),sameDirection?t:1-t);
                var ya=Height(ac,pa.xz);var yb=Height(bc,pb.xz);
                if(!ya.HasValue||!yb.HasValue){unavailable=true;continue;}
                pairWorst=Math.Max(pairWorst,Math.Abs(ya.Value-yb.Value));
            }
            worst=Math.Max(worst,pairWorst);
            if(pairWorst>.25){failed++;affected+=NetworkDesignChecks.HorizontalArcLength(new(a.CourseStart,a.CourseControl1,a.CourseControl2,a.CourseEnd));}
            if(unavailable)unknown++;else checkedPairs++;
            if((unavailable||pairWorst>.25)&&witnesses.Count<128)witnesses.Add(new{a.PairedProfileId,a.WayId,otherWay=b.WayId,
                a.StartNodeId,a.EndNodeId,maximumObservedDifferenceMeters=pairWorst,unavailableStations=unavailable,
                stations=17,projectionToleranceMeters=.05,continuousCertificate=false});
        }
        log.Quality(new("native-paired","paired-height",scope,checkedPairs,failed,unknown,worst,affected,
            "17 common planned-reference stations on exact qualified native paths; <=0.25m observed height difference; XZ mismatch/ambiguous height unavailable; sampled, not continuous"){Expected=expected});
        if(log.DiagnosticsEnabled)log.DiagnosticRecords(scope+"-paired-height-witnesses.ndjson",witnesses,critical:true);
    }

    private static double? Height(IReadOnlyList<Cubic3d> curves,double2 point)
    {
        var best=double.PositiveInfinity;var height=0d;var ambiguous=false;
        foreach(var curve in curves)
        {
            if(!NetworkDesignChecks.Finite(curve))continue;
            for(var i=0;i<32;i++)
            {
                var lo=i/32d;var hi=(i+1)/32d;
                var p=NetworkDesignChecks.Evaluate(curve,lo);var q=NetworkDesignChecks.Evaluate(curve,hi);
                var span=q.xz-p.xz;var fraction=math.clamp(math.dot(point-p.xz,span)/Math.Max(1e-20,math.lengthsq(span)),0,1);
                var t=lo+(hi-lo)*fraction;
                for(var step=0;step<6;step++)
                {
                    var x=NetworkDesignChecks.Evaluate(curve,t);var d=NetworkDesignChecks.Derivative(curve,t).xz;
                    var denominator=math.dot(d,d)+math.dot(x.xz-point,NetworkDesignChecks.SecondDerivative(curve,t).xz);
                    if(!(denominator>1e-15))break;
                    t=math.clamp(t-math.dot(x.xz-point,d)/denominator,lo,hi);
                }
                var value=NetworkDesignChecks.Evaluate(curve,t);var distance=math.distance(value.xz,point);
                if(distance<best-1e-6){best=distance;height=value.y;ambiguous=false;}
                else if(Math.Abs(distance-best)<=1e-6&&Math.Abs(height-value.y)>.05)ambiguous=true;
            }
        }
        return best<=.05&&!ambiguous?height:null;
    }

    public static void Ports(SourceTopologyManifest? selected,IReadOnlyList<PreparedSegment> sections,
        Func<SourceNodeKey,int?> degree,ImportLog log,CancellationToken token,string stage,string scope)
    {
        var expected=new Dictionary<SourceNodeKey,HashSet<SourceNodeKey>>();
        if(selected!=null)foreach(var connection in selected.Connections)
        {
            void Add(SourceNodeKey a,SourceNodeKey b){if(!expected.TryGetValue(a,out var peers))expected[a]=peers=new();peers.Add(b);}
            Add(connection.From,connection.To);Add(connection.To,connection.From);
        }
        var actual=sections.SelectMany(s=>new[]{new SourceNodeKey(s.Kind,s.StartNodeId),new SourceNodeKey(s.Kind,s.EndNodeId)}).Distinct().ToArray();
        var aliases=sections.SelectMany(s=>s.SourceNodeAliases.Select(a=>(Source:new SourceNodeKey(s.Kind,a.SourceNodeId),Canonical:new SourceNodeKey(s.Kind,a.CanonicalNodeId))))
            .GroupBy(a=>a.Canonical).ToDictionary(g=>g.Key,g=>g.Select(a=>a.Source).Distinct().ToArray());
        var canonical=sections.SelectMany(s=>s.SourceNodeAliases.Select(a=>(Source:new SourceNodeKey(s.Kind,a.SourceNodeId),Canonical:new SourceNodeKey(s.Kind,a.CanonicalNodeId))))
            .GroupBy(a=>a.Source).Where(g=>g.Select(a=>a.Canonical).Distinct().Count()==1).ToDictionary(g=>g.Key,g=>g.First().Canonical);
        var declaredTerminals=new HashSet<SourceNodeKey>(sections.SelectMany(s=>new[]{(Node:new SourceNodeKey(s.Kind,s.StartNodeId),Terminal:s.StartTerminal),
            (Node:new SourceNodeKey(s.Kind,s.EndNodeId),Terminal:s.EndTerminal)}).Where(p=>p.Terminal).Select(p=>p.Node));
        var failed=0;var unknown=0;var terminals=0;var selectionBoundaries=0;var witnesses=new List<object>();
        foreach(var node in actual)
        {
            token.ThrowIfCancellationRequested();var observed=degree(node);if(!observed.HasValue){unknown++;continue;}
            if(observed.Value>1)continue;
            if(observed.Value==0){failed++;if(witnesses.Count<128)witnesses.Add(new{node.Kind,node.Id,observedDegree=0,classification="no-live-incident-edge"});continue;}
            var sourceKeys=aliases.TryGetValue(node,out var mapped)?mapped.Concat(new[]{node}):new[]{node};
            var required=sourceKeys.Where(expected.ContainsKey).SelectMany(k=>expected[k]).Select(k=>canonical.TryGetValue(k,out var c)?c:k).Where(k=>k!=node).Distinct().Count();
            var declaredTerminal=declaredTerminals.Contains(node);
            if(required>1){failed++;if(witnesses.Count<128)witnesses.Add(new{node.Kind,node.Id,observedDegree=observed,selectedPeers=required,declaredTerminal,classification="nonterminal-orphan"});}
            else if(declaredTerminal)terminals++;
            else if(selected!=null&&required==1)selectionBoundaries++;
            else unknown++;
        }
        log.Quality(new(stage,"nonterminal-orphan-ports",scope,actual.Length,failed,unknown,failed,0,
            "Selected-source connectivity and exact aliases; degree one is not automatically a broken road"));
        log.Quality(new(stage,"declared-terminal-ports",scope,terminals,0,0,0,0,"Explicit prepared terminal markers"));
        log.Quality(new(stage,"selected-graph-boundary-ports",scope,selectionBoundaries,0,0,0,0,
            "One-neighbour boundary in the selected/clipped source graph; omission reason is not inferred from proximity"));
        if(log.DiagnosticsEnabled)log.DiagnosticRecords(scope+"-orphan-witnesses.ndjson",witnesses,critical:true);
    }
}
