using System;
using System.Collections.Generic;
using System.Linq;
using ImportEarthLocation.Core;
using Unity.Mathematics;

namespace ImportEarthLocation.Processing;

// Without streets, OSM's short non-motorway connectors lose what they joined:
// a median U-turn between a divided road's two carriageways, or a stub that
// led to a street that is not imported. Both render as odd little roads. Only
// when streets are off, chains of non-motorway road (connected through plain
// degree-two nodes) are removed when they are
//   a crossover: under 200 m, both ends on opposite one-way carriageways of
//                one road (same name or ref);
//   a stub:      under 150 m, one end attached, the other a dead end away
//                from the map border;
//   a fragment:  under 150 m with both ends dead.
// Motorways and motorway links, including dead-end ramps, are never removed.
public static class NetworkStubs
{
    private const double CrossoverLength=200,StubLength=150,BorderMargin=50;

    public static IReadOnlyList<PreparedSegment> Prune(IReadOnlyList<PreparedSegment> source,ImportRequest request,ImportLog log)
    {
        if((request.RoadLevel&RoadImportLevel.Streets)!=0)return source;
        var half=request.MapSize.WidthMeters/2-BorderMargin;
        SourceNodeKey A(PreparedSegment s)=>new(s.Kind,s.StartNodeId);
        SourceNodeKey B(PreparedSegment s)=>new(s.Kind,s.EndNodeId);
        var incident=new Dictionary<SourceNodeKey,List<int>>();
        for(var i=0;i<source.Count;i++)foreach(var key in new[]{A(source[i]),B(source[i])})
        {if(!incident.TryGetValue(key,out var list))incident[key]=list=new();list.Add(i);}
        bool Candidate(PreparedSegment s)=>s.Kind==NetworkKind.Road&&!s.Classification.StartsWith("motorway",StringComparison.Ordinal);
        var chainOf=new int[source.Count];Array.Fill(chainOf,-1);var chains=new List<List<int>>();
        for(var i=0;i<source.Count;i++)
        {
            if(chainOf[i]>=0||!Candidate(source[i]))continue;
            var chain=new List<int>();var pending=new List<int>{i};chainOf[i]=chains.Count;
            for(var k=0;k<pending.Count;k++)
            {
                var s=source[pending[k]];chain.Add(pending[k]);
                foreach(var key in new[]{A(s),B(s)})
                {
                    var arms=incident[key];if(arms.Count!=2)continue;
                    foreach(var j in arms)if(chainOf[j]<0&&Candidate(source[j])){chainOf[j]=chains.Count;pending.Add(j);}
                }
            }
            chains.Add(chain);
        }
        var removed=new bool[source.Count];int crossovers=0,stubs=0,fragments=0;
        foreach(var chain in chains)
        {
            var members=new HashSet<int>(chain);
            var length=chain.Sum(i=>NetworkDesignChecks.HorizontalArcLength(new(source[i].CourseStart,source[i].CourseControl1,source[i].CourseControl2,source[i].CourseEnd)));
            if(length>=CrossoverLength)continue;
            // End nodes: chain nodes not shared by two chain members.
            var ends=chain.SelectMany(i=>new[]{A(source[i]),B(source[i])}).GroupBy(k=>k).Where(g=>incident[g.Key].Count(j=>members.Contains(j))==1).Select(g=>g.Key).ToArray();
            var attached=ends.Select(k=>incident[k].Where(j=>!members.Contains(j)).ToArray()).ToArray();
            bool Dead(int e)=>attached[e].Length==0&&Math.Abs(source[incident[ends[e]][0]].Start.x)<half&&Math.Abs(source[incident[ends[e]][0]].Start.z)<half&&
                Math.Abs(source[incident[ends[e]][0]].End.x)<half&&Math.Abs(source[incident[ends[e]][0]].End.z)<half;
            var dead=Enumerable.Range(0,ends.Length).Count(Dead);
            string kind="";
            if(ends.Length==2&&dead==2&&length<StubLength)kind="fragment";
            else if(ends.Length==2&&dead==1&&length<StubLength)kind="stub";
            else if(ends.Length==2&&dead==0&&Crossover(attached[0],attached[1]))kind="crossover";
            if(kind.Length==0)continue;
            foreach(var i in chain)removed[i]=true;
            if(kind=="crossover")crossovers++;else if(kind=="stub")stubs++;else fragments++;
        }
        // Two attachments on opposite one-way carriageways of one named road.
        bool Crossover(int[] left,int[] right)
        {
            foreach(var i in left)foreach(var j in right)
            {
                var a=source[i];var b=source[j];if(!a.OneWay||!b.OneWay||a.Kind!=NetworkKind.Road||b.Kind!=NetworkKind.Road)continue;
                var same=a.Name.Length>0&&a.Name==b.Name||a.Reference.Length>0&&a.Reference==b.Reference;
                if(same&&math.dot(math.normalizesafe((a.End-a.Start).xz),math.normalizesafe((b.End-b.Start).xz))<-.5f)return true;
            }
            return false;
        }
        var total=crossovers+stubs+fragments;
        log.Count("Odd connector roads removed",total);
        log.Evidence($"Odd connector roads removed (streets off): {total:N0}; median U-turn crossovers {crossovers:N0}, dead-end stubs {stubs:N0}, fragments {fragments:N0}; motorways and ramps kept.");
        return total==0?source:source.Where((_,i)=>!removed[i]).ToArray();
    }
}
