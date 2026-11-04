using System;
using System.Linq;
using System.Collections.Generic;
using ImportEarthLocation.Core;

namespace ImportEarthLocation.Processing;

[Flags]
public enum NetworkBoundaryKind
{
    None=0,
    SourcePort=1,
    EndpointMarker=2,
    Structure=4,
    Composition=8,
    Direction=16,
    ProfilePrior=32
}

// A source port preserves native topology/provenance, not necessarily a bend
// in the vertical design. Callers must additionally check incidence and fixed
// native height/slope ownership before sharing a profile through that port.
public static class NetworkBoundaryPolicy
{
    private static readonly NetworkBoundaryKind[] CensusFlags={NetworkBoundaryKind.EndpointMarker,NetworkBoundaryKind.Structure,NetworkBoundaryKind.Composition,NetworkBoundaryKind.Direction,NetworkBoundaryKind.ProfilePrior};
    public static bool SourcePort(PreparedSegment s,bool start)
    {
        var node=start?s.StartNodeId:s.EndNodeId;
        foreach(var alias in s.SourceNodeAliases)if(alias.Mandatory&&alias.CanonicalNodeId==node)return true;
        return false;
    }

    public static bool EndpointMarker(PreparedSegment s,bool start)=>start?
        s.StartTerminal||s.StartLevelCrossing||s.StartMiniRoundabout||s.StartRoundaboutRadius!=0:
        s.EndTerminal||s.EndLevelCrossing||s.EndMiniRoundabout||s.EndRoundaboutRadius!=0;

    public static bool SameCapturedComposition(PreparedSegment a,PreparedSegment b,IReadOnlyDictionary<string,NativeAssetContract>? assets)
        =>assets!=null&&a.NativeAssetKey.Length>0&&a.NativeAssetKey==b.NativeAssetKey&&a.NativeWidth>0&&SparseProfileRows.Finite(a.NativeWidth)&&a.NativeWidth==b.NativeWidth&&
            a.NativeLaneOffsets.Count>0&&a.NativeLaneOffsets.SequenceEqual(b.NativeLaneOffsets)&&
            a.NativeLaneOffsets.All(v=>SparseProfileRows.Finite(v))&&NativeChainCompactor.Compatible(a,b,requireZeroOffsets:false,ignoreNominalLaneCount:true)&&
            NativeAssetContract.Mode(a)==NativeAssetContract.Mode(b)&&assets.TryGetValue(a.NativeAssetKey,out var contract)&&
            contract.LaneOffsets.SequenceEqual(a.NativeLaneOffsets)&&contract.TryEnvelope(NativeAssetContract.Mode(a),out var envelope)&&a.NativeWidth>=envelope.Width;

    public static NetworkBoundaryKind Classify(PreparedSegment a,bool aStart,PreparedSegment b,bool bStart,IReadOnlyDictionary<string,NativeAssetContract>? assets=null)
    {
        var result=NetworkBoundaryKind.None;
        if(SourcePort(a,aStart)||SourcePort(b,bStart))result|=NetworkBoundaryKind.SourcePort;
        if(EndpointMarker(a,aStart)||EndpointMarker(b,bStart)||a.Roundabout||b.Roundabout)result|=NetworkBoundaryKind.EndpointMarker;
        if(a.Layer!=b.Layer||a.Bridge!=b.Bridge||a.Tunnel!=b.Tunnel)result|=NetworkBoundaryKind.Structure;
        if(a.Kind!=b.Kind||a.Classification!=b.Classification||a.ProtectedLink!=b.ProtectedLink||
            a.NativeAssetKey!=b.NativeAssetKey||a.NativeWidth!=b.NativeWidth||
            !a.NativeLaneOffsets.SequenceEqual(b.NativeLaneOffsets)||a.Lanes!=b.Lanes||
            a.PhysicalTracks!=b.PhysicalTracks||a.RailService!=b.RailService)result|=NetworkBoundaryKind.Composition;
        if(a.OneWay!=b.OneWay)result|=NetworkBoundaryKind.Direction;
        if(a.ExplicitIncline!=b.ExplicitIncline||a.SignedInclinePriorAccepted!=b.SignedInclinePriorAccepted)result|=NetworkBoundaryKind.ProfilePrior;
        return result;
    }

    public static bool RequiresProfileBoundary(NetworkBoundaryKind kind)=>(kind&~NetworkBoundaryKind.SourcePort)!=0;

    // Read-only census: these labels explain existing partitions; they never
    // grant permission to remove a native port or change a profile boundary.
    public sealed record Census(int Nodes,int Ends,int Junctions,int DegreeTwo,int RequiredPorts,int ProfileBoundaries,int SameSemanticDegreeTwo);
    public static Census Survey(IReadOnlyList<PreparedSegment> source,SourceTopologyManifest? obligations,ImportLog log,string stage,IReadOnlyDictionary<string,NativeAssetContract>? assets=null)
    {
        var clock=System.Diagnostics.Stopwatch.StartNew();
        var nodes=new Dictionary<SourceNodeKey,List<(PreparedSegment Section,bool Start)>>();
        void Add(PreparedSegment s,bool start)
        {
            var key=new SourceNodeKey(s.Kind,start?s.StartNodeId:s.EndNodeId);
            if(!nodes.TryGetValue(key,out var ports))nodes[key]=ports=new();ports.Add((s,start));
        }
        foreach(var s in source){Add(s,true);Add(s,false);}
        var required=obligations==null?new HashSet<SourceNodeKey>():new HashSet<SourceNodeKey>(obligations.Ports);
        foreach(var s in source)foreach(var alias in s.SourceNodeAliases)if(alias.Mandatory)required.Add(new(s.Kind,alias.CanonicalNodeId));
        var reasons=new SortedDictionary<string,int>(StringComparer.Ordinal);
        void Reason(string reason){reasons[reason]=reasons.TryGetValue(reason,out var count)?count+1:1;}
        var ends=0;var junctions=0;var degreeTwo=0;var requiredPorts=0;var boundaries=0;var observations=0;
        foreach(var pair in nodes)
        {
            var p=pair.Value;if(required.Contains(pair.Key)){requiredPorts++;Reason("mandatory-source-port");}
            if(p.Count==1){ends++;Reason("endpoint");continue;}
            if(p.Count!=2){junctions++;Reason("actual-junction");continue;}
            degreeTwo++;var kind=Classify(p[0].Section,p[0].Start,p[1].Section,p[1].Start,assets);
            foreach(var flag in CensusFlags)
                if((kind&flag)!=0)Reason(flag.ToString());
            var a=p[0].Section;var b=p[1].Section;
            var pairing=a.PairedCorridorId!=b.PairedCorridorId||a.PairedProfileOnly!=b.PairedProfileOnly;
            var direction=a.OneWay&&p[0].Start==p[1].Start;
            if(pairing)Reason("pairing-boundary");if(direction)Reason("incompatible-port-direction");
            if(RequiresProfileBoundary(kind)||pairing||direction)boundaries++;
            else {observations++;Reason("same-semantics-owner-and-metric-unchecked");}
        }
        var census=new Census(nodes.Count,ends,junctions,degreeTwo,requiredPorts,boundaries,observations);
        log.Count("Boundary census "+stage+": nodes",nodes.Count);log.Count("Boundary census "+stage+": required ports",requiredPorts);
        log.Count("Boundary census "+stage+": semantic boundaries",boundaries);log.Count("Boundary census "+stage+": same semantics ownership unchecked",observations);
        foreach(var reason in reasons)log.Count("Boundary census "+stage+": "+reason.Key,reason.Value);
        log.AddDuration("Boundary census "+stage,clock.Elapsed.TotalSeconds);
        log.Diagnostic($"BOUNDARY CENSUS stage={stage}; nodes={census.Nodes}; endpoints={ends}; junctions={junctions}; degreeTwo={degreeTwo}; requiredNativePorts={requiredPorts}; semanticProfileBoundaries={boundaries}; sameSemanticDegreeTwo={observations}; fixedOwnershipAndMetricUnchecked=true; reasonsOverlap=true; analysisCutsAreNotNativeNodes=true; readOnly=true; "+string.Join("; ",reasons.Select(p=>p.Key+"="+p.Value)));
        return census;
    }
}
