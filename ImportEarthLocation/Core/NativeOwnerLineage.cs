using System;
using System.Collections.Generic;

namespace ImportEarthLocation.Core;

public readonly record struct NativeOwnerLink<T>(bool Valid, bool Root, bool HasParent, T Parent) where T : notnull;

// Resolve actual native ownership, not geometric proximity or prefab similarity.
// Successful proofs are cached only for one observation pass; failed proofs are
// never cached, because downstream playback can finish between observations.
public sealed class NativeOwnerLineage<T> where T : notnull
{
    private readonly Dictionary<T,T> _roots = new();
    private readonly HashSet<T> _visited = new();
    private readonly List<T> _path = new();
    public void BeginObservation() => _roots.Clear();

    public bool Resolve(T entity, Func<T,NativeOwnerLink<T>> inspect, out T root, out string reason)
    {
        _visited.Clear(); _path.Clear();
        var current=entity;
        while(true)
        {
            if(_roots.TryGetValue(current,out root!))break;
            if(!_visited.Add(current)){root=default!;reason="ownership-cycle";return false;}
            var link=inspect(current);
            if(!link.Valid){root=default!;reason="missing-deleted-or-invalid-owner-reference";return false;}
            _path.Add(current);
            if(link.Root){root=current;break;}
            if(!link.HasParent){root=default!;reason="unowned-root";return false;}
            current=link.Parent;
        }
        foreach(var item in _path)_roots[item]=root;
        reason="native-owner-chain";return true;
    }
}
