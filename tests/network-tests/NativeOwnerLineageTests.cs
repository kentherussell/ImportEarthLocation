using ImportEarthLocation.Core;

internal static class NativeOwnerLineageTests
{
    private readonly record struct Id(int Index,int Version);
    private static void Check(bool value,string message){if(!value)throw new Exception(message);}

    public static void Run()
    {
        var resolver=new NativeOwnerLineage<Id>();
        var root=new Id(160468,3);var lane=new Id(164805,1);var preview=new Id(165331,1);
        var links=new Dictionary<Id,NativeOwnerLink<Id>>
        {
            [root]=new(true,true,false,default),
            [lane]=new(true,false,true,root),
            [preview]=new(true,false,true,lane)
        };
        NativeOwnerLink<Id> Inspect(Id entity)=>links.TryGetValue(entity,out var link)?link:default;
        Check(resolver.Resolve(lane,Inspect,out var proof,out _)&&proof==root,"Captured secondary lane must resolve to its imported road node without a prior Temp identity");
        Check(resolver.Resolve(preview,Inspect,out proof,out _)&&proof==root,"Nested native descendants lost their imported owner lineage");
        var foreign=new Id(900000,1);links[foreign]=new(true,false,false,default);
        Check(!resolver.Resolve(foreign,Inspect,out _,out var reason)&&reason=="unowned-root","Unowned live entity must remain foreign");
        Check(!resolver.Resolve(new Id(root.Index,root.Version+1),Inspect,out _,out _),"Entity index reuse must not inherit another version's ownership");
        var a=new Id(10,1);var b=new Id(11,1);
        links[a]=new(true,false,true,b);links[b]=new(true,false,true,a);
        Check(!resolver.Resolve(a,Inspect,out _,out reason)&&reason=="ownership-cycle","Cyclic ownership must terminate without granting ownership");
        var invalid=new Id(12,1);links[invalid]=new(false,true,false,default);
        Check(!resolver.Resolve(invalid,Inspect,out _,out _),"Deleted/missing/asset entities cannot be trusted roots");

        // A temporarily absent native relationship is not a permanent failure.
        var late=new Id(13,1);links[late]=new(true,false,true,new Id(14,1));
        Check(!resolver.Resolve(late,Inspect,out _,out _),"Missing parent unexpectedly accepted");
        links[new Id(14,1)]=new(true,false,true,root);
        Check(resolver.Resolve(late,Inspect,out proof,out _)&&proof==root,"Negative proof was cached; downstream completion could not resume");
        links[root]=default;
        resolver.BeginObservation();
        Check(!resolver.Resolve(lane,Inspect,out _,out _),"A new observation reused a stale proof of a retired root");
        links[root]=new(true,false,false,default);resolver.BeginObservation();
        Check(!resolver.Resolve(lane,Inspect,out _,out _),"New map with unowned root inherited old-map lineage");
        Console.WriteLine("PASS: log-derived secondary-lane owner lineage, nested descendants, foreign/version/invalid/cycle rejection, transient revalidation and map-observation reset.");
    }
}
