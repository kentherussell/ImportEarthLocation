using System;
using System.Collections.Generic;
using System.Threading;
using ImportEarthLocation.Core;
using Unity.Mathematics;

namespace ImportEarthLocation.Processing;

// One fixed clearance model for the joint solve. Full deck rectangles are
// intersected, including parallel decks; no centreline-angle rejection is used.
public static class NetworkPhysicalContacts
{
    public readonly record struct Report(int Leaves,int UncertifiedLeaves,int MissingAssets,long Regions,long Rows,long CandidateTests,
        long LocalTransitionPairs=0,long PairedFormationPairs=0,long GradeContradictionPairs=0,long HorizontalConflictPairs=0,long CoalescedRows=0);
    private readonly record struct Family(long Upper,long Lower,NativeStructureMode UpperMode,NativeStructureMode LowerMode);
    private sealed class Measure { public double Area; public long BoundaryRows; }
    public readonly record struct ContactEvidence(int FirstLeaf,int SecondLeaf,int FirstSection,int SecondSection,
        double FirstBegin,double FirstEnd,double SecondBegin,double SecondEnd,string Classification,long FirstRow,long RowCount,
        double? RequiredGapMeters=null,double? MaximumGradeRiseMeters=null);

    public static Report Append(IReadOnlyList<PreparedSegment> source,IReadOnlyDictionary<string,NativeAssetContract> assets,
        int[] from,int[] c1,int[] c2,int[] to,SparseProfileRows rows,double epsilon,CancellationToken token,Action<string>? progress=null,
        Action<object>? constantWitness=null,Action<object>? horizontalWitness=null,Action<ContactEvidence>? evidence=null)
    {
        if(source.Count==0)return default;
        var leaves=new List<FootprintLeaf>();var envelopes=new NativeCompositionEnvelope[source.Count];
        var missing=0;var uncertified=0;
        for(var i=0;i<source.Count;i++)
        {
            token.ThrowIfCancellationRequested();var s=source[i];
            if((i&127)==0)progress?.Invoke($"Building road/rail deck footprints {i+1}/{source.Count}");
            if(!assets.TryGetValue(s.NativeAssetKey,out var asset)||!asset.TryEnvelope(NativeAssetContract.Mode(s),out envelopes[i])){missing++;continue;}
            var before=leaves.Count;
            NetworkFootprintMath.BuildLeaves(new(s.CourseStart,s.CourseControl1,s.CourseControl2,s.CourseEnd),
                Math.Max(s.NativeWidth,envelopes[i].Width)/2,i,leaves,token);
            for(var j=before;j<leaves.Count;j++)if(!leaves[j].Certified)uncertified++;
        }
        var topology=new NetworkContactTopology(source,envelopes,token,epsilon);
        leaves=topology.SplitFormationLeaves(leaves,envelopes,token);
        var index=new NetworkFootprintIndex(leaves,token);var families=new Dictionary<Family,Measure>();
        var measurementOnly=rows.MeasurementOnly;var unitMeasure=new Measure();
        var localTransitions=0L;var pairedFormations=0L;var contradictions=0L;var horizontalConflicts=0L;var coalesced=0L;
        var witnessed=new HashSet<(int,int)>();
        var rectangle=new double2[16];var clip=new double2[16];var scratch=new double2[16];
        var original=new double2[16];var region=new double2[16];var temp=new double2[16];var clamped=new double2[16];
        var indices=new int[8];var coefficients=new double[8];var hiBasis=new double[4];var loBasis=new double[4];
        long regionCount=0,rowCount=0;var clock=System.Diagnostics.Stopwatch.StartNew();var lastProgress=0d;var visits=0L;
        // Two deterministic enumerations avoid retaining a city-sized contact
        // triangle list. Family weights normalize represented area, not row count.
        // Fixed-vector certification needs every inequality, but not objective
        // family-area normalization. It can emit the identical canonical rows
        // in one pass with unit cost weights; measurement sinks never optimize.
        for(var pass=measurementOnly?1:0;pass<2;pass++)foreach(var pair in index.CandidatePairs(token))
        {
            var classify=pass==0||measurementOnly;
            if((visits++&1023)==0&&clock.Elapsed.TotalSeconds-lastProgress>=2)
            {lastProgress=clock.Elapsed.TotalSeconds;progress?.Invoke($"Checking overlapping deck footprints; pass {(measurementOnly?1:pass+1)}/{(measurementOnly?1:2)}; {visits:N0} candidate leaf pairs processed");}
            var a=leaves[pair.A];var b=leaves[pair.B];if(a.Curve==b.Curve)continue;
            void Evidence(string classification,long firstRow=-1,long rowCount=0)
                =>evidence?.Invoke(new(a.Ordinal,b.Ordinal,a.Curve,b.Curve,a.Begin,a.End,b.Begin,b.End,classification,firstRow,rowCount));
            if(!a.Certified||!b.Certified){if(classify)Evidence("unavailable-footprint");continue;}
            var sa=source[a.Curve];var sb=source[b.Curve];
            if(sa.PairedProfileId!=0&&sa.PairedProfileId==sb.PairedProfileId&&sa.PairedReversed!=sb.PairedReversed)
            {if(classify){pairedFormations++;Evidence("trusted-paired-formation");}continue;}
            if(topology.LocalTransition(a,b)){if(classify){localTransitions++;Evidence("local-junction-or-structure-transition");}continue;}
            var ma=NativeAssetContract.Mode(sa);var mb=NativeAssetContract.Mode(sb);
            // Only immutable source structure evidence establishes an ordering.
            // An overlap on the same level is a horizontal-quality observation,
            // never permission to lift one road over its neighbour.
            var sameLevel=sa.Layer==sb.Layer&&ma==mb;
            // Same-level overlaps only produce observations in pass zero.
            // Re-clipping their polygons cannot contribute family weights or
            // clearance rows in the second pass.
            if(sameLevel&&pass==1&&!measurementOnly)continue;
            var upper=sa.Layer!=sb.Layer?sa.Layer>sb.Layer:ma!=NativeStructureMode.Tunnel&&
                (mb==NativeStructureMode.Tunnel||ma==NativeStructureMode.Elevated);
            var hi=upper?a:b;var lo=upper?b:a;var hs=source[hi.Curve];var ls=source[lo.Curve];
            var origin=a.A;
            var count=NetworkFootprintMath.Rectangle(a,Math.Max(sa.NativeWidth,envelopes[a.Curve].Width)/2,origin,rectangle);
            var nc=NetworkFootprintMath.Rectangle(b,Math.Max(sb.NativeWidth,envelopes[b.Curve].Width)/2,origin,clip);
            count=NetworkFootprintMath.Intersect(rectangle,count,clip,nc,scratch);if(count==0){if(classify)Evidence("no-footprint-intersection");continue;}
            if(sameLevel)
            {
                if(!classify)continue;var area=0d;
                for(var j=2;j<count;j++)area+=NetworkFootprintMath.TriangleArea(rectangle[0],rectangle[j-1],rectangle[j]);
                if(area<=.01){Evidence("same-level-boundary-only");continue;}
                var formation=SharedFormation(sa,sb,envelopes[a.Curve],envelopes[b.Curve]);var outside=formation==null;
                if(formation.HasValue)for(var j=0;j<count;j++)
                    if(math.cmax(math.abs(rectangle[j]+origin-formation.Value.Center))>formation.Value.Radius)outside=true;
                Evidence(outside?"ambiguous-same-level-conflict":"explicit-local-at-grade-formation");
                if(outside&&witnessed.Add(a.Curve<b.Curve?(a.Curve,b.Curve):(b.Curve,a.Curve)))
                {
                    horizontalConflicts++;
                    horizontalWitness?.Invoke(new{firstSection=a.Curve,secondSection=b.Curve,
                        firstWay=sa.WayId,secondWay=sb.WayId,firstStart=sa.StartNodeId,firstEnd=sa.EndNodeId,
                        secondStart=sb.StartNodeId,secondEnd=sb.EndNodeId,classification="same-level-overlap-outside-known-formation",
                        conservativeFootprint=true,requiresReview=true,verticalSeparationRowsAdded=false});
                }
                continue;
            }
            if(classify&&witnessed.Add(a.Curve<b.Curve?(a.Curve,b.Curve):(b.Curve,a.Curve)))
            {
                var center=double2.zero;for(var j=0;j<count;j++)center+=rectangle[j];center=center/count+origin;
                var at=a.Begin+(a.End-a.Begin)*NetworkFootprintMath.Parameter(a,center);
                var bt=b.Begin+(b.End-b.Begin)*NetworkFootprintMath.Parameter(b,center);
                var required=5+envelopes[lo.Curve].SurfaceMax-envelopes[hi.Curve].RangeMin+epsilon;
                var maximumRise=topology.RiseBudget(a.Curve,math.clamp(at,0,1),b.Curve,math.clamp(bt,0,1),token);
                if(maximumRise<required)
                {
                    contradictions++;
                    evidence?.Invoke(new(a.Ordinal,b.Ordinal,a.Curve,b.Curve,a.Begin,a.End,b.Begin,b.End,
                        "clearance-grade-incompatible",-1,0,required,maximumRise));
                }
            }
            Array.Copy(rectangle,original,count);
            var family=new Family(hs.WayId,ls.WayId,NativeAssetContract.Mode(hs),NativeAssetContract.Mode(ls));
            Measure measure;
            if(measurementOnly)measure=unitMeasure;
            else if(families.TryGetValue(family,out var existingMeasure))measure=existingMeasure;
            else families.Add(family,measure=new());
            var firstContactRow=rows.Count;
            // The clamped affine parameter is piecewise affine. Split both
            // parameter domains before constructing polynomial coefficients.
            for(var hc=0;hc<3;hc++)for(var lc=0;lc<3;lc++)
            {
                Array.Copy(original,region,count);var used=count;
                used=ClampRegion(hi,hc,used);used=ClampRegion(lo,lc,used);if(used==0)continue;
                var formation=SharedFormation(hs,ls,envelopes[hi.Curve],envelopes[lo.Curve]);
                var clampedCount=used;Array.Copy(region,clamped,used);
                // Subtract the shared native node's local formation in four
                // disjoint strips. Sharing one node does not exempt an entire
                // long pair of curves from clearance outside that formation.
                for(var part=0;part<(formation.HasValue?4:1);part++)
                {
                used=clampedCount;Array.Copy(clamped,region,used);
                if(formation.HasValue)
                {
                    var center=formation.Value.Center-origin;var radius=formation.Value.Radius;
                    int Cut(double2 normal,double threshold,int length)
                    {var n=NetworkFootprintMath.Clip(region,length,temp,normal,threshold);Array.Copy(temp,region,n);return n;}
                    if(part==0)used=Cut(new(-1,0),-center.x+radius,used);
                    else if(part==1)used=Cut(new(1,0),center.x+radius,used);
                    else
                    {
                        used=Cut(new(1,0),center.x-radius,used);used=Cut(new(-1,0),-center.x-radius,used);
                        used=part==2?Cut(new(0,-1),-center.y+radius,used):Cut(new(0,1),center.y+radius,used);
                    }
                    if(used==0)continue;
                }
                if(classify)regionCount++;
                var triangles=Math.Max(1,used-2);
                for(var triangle=0;triangle<triangles;triangle++)
                {
                    var p=region[0];var q=region[used==1?0:triangle+1];var r=region[used<=2?used-1:triangle+2];
                    var area=NetworkFootprintMath.TriangleArea(p,q,r);var terms=used==1?1:area==0?4:10;
                    if(area==0&&used>1)
                    {
                        // Clipping can retain a collinear interior vertex. A
                        // boundary polynomial must span the extreme pair, not
                        // stop at that middle vertex and miss the far endpoint.
                        var pq=math.lengthsq(p-q);var pr=math.lengthsq(p-r);var qr=math.lengthsq(q-r);
                        if(pr>=pq&&pr>=qr)q=r;else if(qr>pq)p=r;
                        r=q;
                    }
                    if(pass==0){measure.Area+=area;if(area==0)measure.BoundaryRows+=terms;continue;}
                    var weight=measurementOnly?1:area>0?10000*(area/measure.Area)/10:10000d/(100*Math.Max(1,measure.BoundaryRows));
                    var hp=Parameter(hi,hc,p);var hq=Parameter(hi,hc,q);var hr=Parameter(hi,hc,r);
                    var lp=Parameter(lo,lc,p);var lq=Parameter(lo,lc,q);var lr=Parameter(lo,lc,r);
                    var beta=NetworkGeometrySafety.GradeLimit(hs)*hi.MaximumSpeed*hi.Error/hi.Length+
                        NetworkGeometrySafety.GradeLimit(ls)*lo.MaximumSpeed*lo.Error/lo.Length;
                    var gap=5+envelopes[lo.Curve].SurfaceMax-envelopes[hi.Curve].RangeMin+epsilon+beta;
                    for(var k=0;k<terms;k++)
                    {
                        var coefficient=terms==4?k switch{0=>0,1=>1,2=>3,_=>6}:k;
                        NetworkFootprintMath.TriangleBlossom(hp,hq,hr,coefficient,hiBasis);
                        NetworkFootprintMath.TriangleBlossom(lp,lq,lr,coefficient,loBasis);
                        indices[0]=from[hi.Curve];indices[1]=c1[hi.Curve];indices[2]=c2[hi.Curve];indices[3]=to[hi.Curve];
                        indices[4]=from[lo.Curve];indices[5]=c1[lo.Curve];indices[6]=c2[lo.Curve];indices[7]=to[lo.Curve];
                        for(var j=0;j<4;j++){coefficients[j]=-hiBasis[j]/epsilon;coefficients[j+4]=loBasis[j]/epsilon;}
                        var rowIndex=rows.Count;
                        // Only consecutive coefficients of THIS contact triangle
                        // may coalesce. Exact equality preserves the objective,
                        // gradient and physical coverage; no near-row matching.
                        rows.Append(indices,coefficients,8,-gap/epsilon,weight,ProfileRowKind.UpperHingeCost,coalescePrevious:k>0);
                        if(rows.Count>rowIndex)rowCount++;else{coalesced++;rowIndex=rows.Count-1;}
                        if(rows.Row(rowIndex).Count==0&&gap>0)constantWitness?.Invoke(new
                        {
                            row=rowIndex,upperSection=hi.Curve,lowerSection=lo.Curve,
                            upper=new{hs.WayId,hs.StartNodeId,hs.EndNodeId,hs.Layer,hs.PairedProfileId,mode=NativeAssetContract.Mode(hs).ToString()},
                            lower=new{ls.WayId,ls.StartNodeId,ls.EndNodeId,ls.Layer,ls.PairedProfileId,mode=NativeAssetContract.Mode(ls).ToString()},
                            upperBegin=hi.Begin,upperEnd=hi.End,lowerBegin=lo.Begin,lowerEnd=lo.End,
                            hp,hq,hr,lp,lq,lr,gapMeters=gap,weight,
                            origin=DiagnosticArtifacts.Point(new float3((float)origin.x,0,(float)origin.y)),
                            sharedVariables=(int[])indices.Clone(),unmergedCoefficients=(double[])coefficients.Clone(),
                            classification="positive-clearance-demand-with-no-free-coefficients",quality="RequiresReview",
                            objectiveTermRemoved=false,nativePlacementBlocked=false
                        });
                    }
                }
                }
            }

            if(pass==1)Evidence(rows.Count>firstContactRow?"independent-source-ordered-levels":"local-formation-region-subtracted",firstContactRow,rows.Count-firstContactRow);

            int ClampRegion(FootprintLeaf leaf,int part,int used)
            {
                var shift=math.dot(leaf.A-origin,leaf.Tangent);
                int Cut(double2 normal,double threshold,int length)
                {var n=NetworkFootprintMath.Clip(region,length,temp,normal,threshold);Array.Copy(temp,region,n);return n;}
                if(part==0)return Cut(-leaf.Tangent,-shift,used);
                if(part==2)return Cut(leaf.Tangent,shift+leaf.Length,used);
                used=Cut(leaf.Tangent,shift,used);return Cut(-leaf.Tangent,-shift-leaf.Length,used);
            }
            double Parameter(FootprintLeaf leaf,int part,double2 p)=>leaf.Begin+(leaf.End-leaf.Begin)*
                (part==0?0:part==2?1:NetworkFootprintMath.Parameter(leaf,p+origin));
        }
        return new(leaves.Count,uncertified,missing,regionCount,rowCount,index.CandidateTests,localTransitions,pairedFormations,contradictions,horizontalConflicts,coalesced);
    }

    private static (double2 Center,double Radius)? SharedFormation(PreparedSegment a,PreparedSegment b,
        NativeCompositionEnvelope ea,NativeCompositionEnvelope eb)
    {
        if(a.Kind!=b.Kind&&(NetworkStructure.IsStructure(a)||NetworkStructure.IsStructure(b)||a.Layer!=b.Layer))return null;
        var radius=Math.Max(ea.NodeOffset,eb.NodeOffset)+Math.Max(ea.Width,eb.Width)/2;
        foreach(var id in new[]{a.StartNodeId,a.EndNodeId})
        {
            if(id!=b.StartNodeId&&id!=b.EndNodeId)continue;
            return ((double2)(id==a.StartNodeId?a.Start:a.End).xz,radius);
        }
        return null;
    }
}
