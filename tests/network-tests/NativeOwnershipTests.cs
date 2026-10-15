using ImportEarthLocation.Core;

internal static class NativeOwnershipTests
{
    private readonly record struct Id(int Index,int Version);
    private static void Check(bool valid,string message)
    { if(!valid)throw new Exception(message); }

    public static void Run()
    {
        var ownership=new NativeEntityOwnership<Id>();
        var node=new Id(150620,3);var pillar=new Id(151520,1);var preview=new Id(152467,3);
        var live=new HashSet<Id>{node,pillar};
        ownership.Capture(node,default,false,false,false);
        ownership.Capture(pillar,default,false,false,false);
        Check(!ownership.Contains(pillar),"Unapplied pillar preview must not grant ownership");
        ownership.SettleApplied(live.Contains);
        Check(ownership.Contains(node)&&ownership.Contains(pillar),"Log-derived road node/pillar creation lost generated object ownership");
        Check(!ownership.Contains(new Id(151520,2)),"Recycled entity index must not inherit an old entity version's ownership");
        var foreign=new Id(151521,1);
        Check(!ownership.Contains(foreign),"An unobserved foreign pillar must remain unknown regardless of its type or proximity");

        // The next batch can update the retained pillar; its preview is destroyed
        // by native playback, while the Original remains the live object.
        ownership.Capture(preview,pillar,true,false,false);
        ownership.SettleApplied(live.Contains);
        Check(ownership.Contains(pillar)&&!ownership.Contains(preview),"Ordinary update must retain Original, not its discarded preview");

        var replacement=new Id(152468,3);
        ownership.Capture(replacement,pillar,true,false,false);
        live.Remove(pillar);live.Add(replacement);
        ownership.SettleApplied(live.Contains);
        Check(ownership.Contains(replacement)&&!ownership.Contains(pillar),"Replacement must retain its live output and forget the retired original");

        var deleting=new Id(152469,3);
        ownership.Capture(deleting,replacement,true,true,false);
        live.Remove(replacement);
        ownership.SettleApplied(live.Contains);
        Check(!ownership.Contains(replacement)&&!ownership.Contains(deleting),"Deletion must not promote the delete preview or retain a deleted object");

        var canceled=new Id(152470,3);live.Add(canceled);
        ownership.Capture(canceled,node,true,false,true);
        ownership.SettleApplied(live.Contains);
        Check(ownership.Contains(node)&&!ownership.Contains(canceled),"Canceled output must not acquire ownership or revoke an unchanged original");

        var cleared=new Id(152471,3);live.Add(cleared);
        ownership.Capture(cleared,default,false,false,false);
        ownership.DiscardBatch();ownership.SettleApplied(live.Contains);
        Check(!ownership.Contains(cleared)&&ownership.Contains(node),"Clear/preflight must discard candidates without promoting them or forgetting existing objects");

        // Reconciliation must scale with touched outputs, not all retained trees,
        // lanes, nodes and objects accumulated across previous batches.
        for(var i=0;i<10000;i++)ownership.Capture(new Id(200000+i,1),default,false,false,false);
        ownership.SettleApplied(_=>true);
        ownership.Capture(preview,node,true,false,false);
        var inspected=0;ownership.SettleApplied(id=>{inspected++;return id==node;});
        Check(inspected==2,"Settlement scanned unrelated retained entities instead of this transaction's touched entities");
        ownership.Capture(cleared,default,false,false,false);
        ownership.Clear();ownership.SettleApplied(_=>true);
        Check(ownership.Count==0&&!ownership.Contains(node)&&!ownership.Contains(cleared),"Map/generation reset must clear committed and pending ownership");
        Console.WriteLine("PASS: native ownership retains generated pillars across batches; update/replacement/delete/Clear/cancel/version/reset and touched-only settlement regressions passed. Engine playback still requires an in-game run.");
    }
}
