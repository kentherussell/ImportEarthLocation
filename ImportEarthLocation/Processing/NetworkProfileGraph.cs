using System;
using System.Collections.Generic;
using System.Threading;

namespace ImportEarthLocation.Processing;

public readonly record struct ProfileGraphArc(int To,double Cost);

/// <summary>Frozen weighted adjacency and reusable value heap for multi-source portal/witness distances.</summary>
public sealed class NetworkProfileGraph
{
    private readonly ProfileGraphArc[][] _adjacency;
    private readonly List<(double Distance,int Node)> _heap=new();
    public long Relaxations { get; private set; }
    public int Nodes=>_adjacency.Length;
    public NetworkProfileGraph(IReadOnlyList<ProfileGraphArc[]> adjacency)
    {
        _adjacency=new ProfileGraphArc[adjacency.Count][];
        for(var i=0;i<adjacency.Count;i++)
        {
            var copy=(ProfileGraphArc[])adjacency[i].Clone();
            foreach(var arc in copy)if(arc.To<0||arc.To>=adjacency.Count||!SparseProfileRows.Finite(arc.Cost)||arc.Cost<0)
                throw new ArgumentException("Invalid frozen graph arc.");
            Array.Sort(copy,(a,b)=>{var c=a.To.CompareTo(b.To);return c!=0?c:a.Cost.CompareTo(b.Cost);});_adjacency[i]=copy;
        }
    }
    private static bool Before((double Distance,int Node) a,(double Distance,int Node) b)
        =>a.Distance<b.Distance||(a.Distance==b.Distance&&a.Node<b.Node);
    private void Push((double Distance,int Node) entry)
    {
        var i=_heap.Count;_heap.Add(entry);
        while(i>0){var parent=(i-1)/2;if(!Before(entry,_heap[parent]))break;_heap[i]=_heap[parent];i=parent;}
        _heap[i]=entry;
    }
    private (double Distance,int Node) Pop()
    {
        var first=_heap[0];var tail=_heap[_heap.Count-1];_heap.RemoveAt(_heap.Count-1);
        if(_heap.Count==0)return first;var i=0;
        while(2*i+1<_heap.Count)
        {
            var child=2*i+1;if(child+1<_heap.Count&&Before(_heap[child+1],_heap[child]))child++;
            if(!Before(_heap[child],tail))break;_heap[i]=_heap[child];i=child;
        }
        _heap[i]=tail;return first;
    }
    public double[] Distances(IReadOnlyList<int> sources,double maximum,CancellationToken token,int[]? predecessor=null)
    {
        if(double.IsNaN(maximum)||maximum<0||predecessor!=null&&predecessor.Length!=Nodes)throw new ArgumentException("Invalid graph search bounds.");
        var distances=new double[Nodes];for(var i=0;i<Nodes;i++){distances[i]=double.PositiveInfinity;if(predecessor!=null)predecessor[i]=-1;}
        _heap.Clear();Relaxations=0;
        foreach(var source in sources)
        {if(source<0||source>=Nodes)throw new ArgumentException("Unknown graph source.");if(distances[source]==0)continue;distances[source]=0;Push((0,source));}
        var visits=0;
        while(_heap.Count>0)
        {
            if((visits++&1023)==0)token.ThrowIfCancellationRequested();
            var current=Pop();if(current.Distance!=distances[current.Node])continue;
            if(current.Distance>maximum)break;
            foreach(var arc in _adjacency[current.Node])
            {
                Relaxations++;var next=current.Distance+arc.Cost;if(next>maximum||next>=distances[arc.To])continue;
                distances[arc.To]=next;if(predecessor!=null)predecessor[arc.To]=current.Node;Push((next,arc.To));
            }
        }
        _heap.Clear();return distances;
    }
}
