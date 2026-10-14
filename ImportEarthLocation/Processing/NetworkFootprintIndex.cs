using System;
using System.Collections.Generic;
using System.Threading;
using Unity.Mathematics;

namespace ImportEarthLocation.Processing;

/// <summary>Flat BVH, two initial sorts and stable partitions; dense coverage is never capped.</summary>
public sealed class NetworkFootprintIndex
{
    private readonly record struct Node(double2 Min,double2 Max,int Start,int Count,int Left,int Right);
    private readonly FootprintLeaf[] _leaves;
    private readonly int[] _x,_z,_scratch,_stack;
    private readonly bool[] _left;
    private readonly List<Node> _nodes;
    public long NodeVisits { get; private set; }
    public long CandidateTests { get; private set; }
    public NetworkFootprintIndex(IReadOnlyList<FootprintLeaf> leaves,CancellationToken token)
    {
        _leaves=new FootprintLeaf[leaves.Count];_x=new int[leaves.Count];_z=new int[leaves.Count];
        _scratch=new int[leaves.Count];_left=new bool[leaves.Count];
        _nodes=new List<Node>(checked(Math.Max(1,leaves.Count*2)));
        for(var i=0;i<leaves.Count;i++){_leaves[i]=leaves[i];_x[i]=_z[i]=i;}
        Array.Sort(_x,(a,b)=>Compare(a,b,true));Array.Sort(_z,(a,b)=>Compare(a,b,false));
        if(leaves.Count>0)Build(0,leaves.Count,token);
        _stack=new int[Math.Max(1,_nodes.Count)];
    }
    private int Compare(int a,int b,bool x)
    {
        var pa=(_leaves[a].Min+_leaves[a].Max)*.5;var pb=(_leaves[b].Min+_leaves[b].Max)*.5;
        var order=(x?pa.x:pa.y).CompareTo(x?pb.x:pb.y);
        return order!=0?order:_leaves[a].Ordinal.CompareTo(_leaves[b].Ordinal);
    }
    private int Build(int start,int count,CancellationToken token)
    {
        token.ThrowIfCancellationRequested();var min=_leaves[_x[start]].Min;var max=_leaves[_x[start]].Max;
        for(var i=start+1;i<start+count;i++){min=math.min(min,_leaves[_x[i]].Min);max=math.max(max,_leaves[_x[i]].Max);}
        var node=_nodes.Count;_nodes.Add(new(min,max,start,count,-1,-1));if(count<=8)return node;
        var along=max-min;var selected=along.x>=along.y?_x:_z;var other=ReferenceEquals(selected,_x)?_z:_x;
        var half=count/2;
        for(var i=0;i<count;i++)_left[selected[start+i]]=i<half;
        var l=start;var r=start+half;
        for(var i=start;i<start+count;i++){var ordinal=other[i];_scratch[_left[ordinal]?l++:r++]=ordinal;}
        Array.Copy(_scratch,start,other,start,count);
        var left=Build(start,half,token);var right=Build(start+half,count-half,token);
        _nodes[node]=new(min,max,start,count,left,right);return node;
    }
    private static bool Overlap(double2 a,double2 b,double2 c,double2 d)
        =>a.x<=d.x&&b.x>=c.x&&a.y<=d.y&&b.y>=c.y;
    // One enumerator owns the reusable stack. Intended for one sequential design
    // worker, not concurrent queries; no per-query all-pairs HashSet/list.
    public IEnumerable<(int A,int B)> CandidatePairs(CancellationToken token)
    {
        for(var i=0;i<_leaves.Length;i++)
        {
            token.ThrowIfCancellationRequested();var leaf=_leaves[i];var used=0;
            if(_nodes.Count>0)_stack[used++]=0;
            while(used>0)
            {
                var node=_nodes[_stack[--used]];NodeVisits++;
                if(!Overlap(leaf.Min,leaf.Max,node.Min,node.Max))continue;
                if(node.Left>=0){_stack[used++]=node.Right;_stack[used++]=node.Left;continue;}
                for(var k=node.Start;k<node.Start+node.Count;k++)
                {
                    var j=_x[k];if(j<=i)continue;CandidateTests++;
                    if(Overlap(leaf.Min,leaf.Max,_leaves[j].Min,_leaves[j].Max))yield return(i,j);
                }
            }
        }
    }
}
