using System;
using System.Collections.Generic;
using System.IO;
using System.Linq;
using System.Threading;
using ImportEarthLocation.Core;
using Unity.Mathematics;

namespace ImportEarthLocation.Processing;

// Final coupled solve, after horizontal fitting and the legacy profile passes.
// Roads and rail share clearance constraints. Only explicit, same-level OSM
// crossing anchors may also share elevation; proximity never joins networks.
// Production callers retain finite native-safe geometry as RequiresReview when
// quality constraints fail. Strict offline callers may reject a quality fixture.
public static class NetworkGeometrySafety
{
    public const float DeckGap = NetworkVerticalProfiles.MinimumBridgeClearance + 4f;

    private readonly record struct Key(NetworkKind Kind, long Id);

    private sealed record Contact(
        int A,
        float T,
        int B,
        float U,
        NetworkContactKind Kind = NetworkContactKind.IndependentLevels);

    private readonly record struct Span(int Index, float T, float U, float3 A, float3 B);

    // A bore is out of sight, so tunnels may descend more steeply than open
    // track or road to reach cover quickly; a portal's grade change is still
    // checked against the open approach.
    public const float RoadTunnelGrade = .12f;

    public static float GradeLimit(PreparedSegment segment)
        => segment.Kind == NetworkKind.Rail ? (segment.Tunnel ? .08f : .05f) :
        segment.Tunnel
            ? Math.Max(RoadTunnelGrade, NetworkVerticalProfiles.GradeLimit(segment))
            : NetworkVerticalProfiles.GradeLimit(segment);

    private static float Radius(PreparedSegment segment)
        => segment.NativeWidth > 0
            ? segment.NativeWidth * .5f
            : segment.Kind == NetworkKind.Rail ? 6 : Math.Max(4, segment.Lanes * 1.8f + 2);

    public static void Validate(
        IReadOnlyList<PreparedSegment> sections,
        ImportLog log,
        CancellationToken token,
        bool allowQualityWarnings = false)
    {
        var steep =
            sections
                .Where(segment => !NetworkVerticalProfiles.WithinGradeCeiling(segment, GradeLimit(segment)))
                .ToArray();

        var contacts = Contacts(sections, token);
        var collisions = new List<Contact>();

        foreach (var contact in contacts)
        {
            var gap =
                Math.Abs(
                    NetworkNormalizer.Evaluate(sections[contact.A], contact.T).y
                        - NetworkNormalizer.Evaluate(sections[contact.B], contact.U).y);

            if (contact.Kind != NetworkContactKind.IndependentLevels || !math.isfinite(gap) || gap < MinimumSafeGap)
                collisions.Add(contact);
        }

        log.Diagnostic(
            $"NETWORK SAFETY FINAL sections={sections.Count}; widthAwareContacts={contacts.Count}; clearanceFailures={collisions.Count}; gradeFailures={steep.Length}; minimumClearSpaceMeters=5; deckAllowanceMeters=2; failClosed={!allowQualityWarnings}; iterationMode={allowQualityWarnings}; nativeRenderingStillRequiresVerification=true");

        if (log.DiagnosticsEnabled)
        {
            DescribeQuality(sections, log, token);
            log.DiagnosticRecords("network-clearance-observations.ndjson", contacts
                .OrderBy(
                    contact => Math.Abs(
                        NetworkNormalizer.Evaluate(sections[contact.A], contact.T).y
                            - NetworkNormalizer.Evaluate(sections[contact.B], contact.U).y))
                .Take(64).Select(contact => (object)new
                {
                    constraint = "independent-level-deck-gap",
                    policyStatus = "implementation-assumption-awaiting-game-validation",
                    evidence = "prepared-curve-observation-not-native-readback",
                    nativeRenderingVerified = false,
                    aWay = sections[contact.A].WayId,
                    aStart = sections[contact.A].StartNodeId,
                    aEnd = sections[contact.A].EndNodeId,
                    bWay = sections[contact.B].WayId,
                    bStart = sections[contact.B].StartNodeId,
                    bEnd = sections[contact.B].EndNodeId,
                    aT = contact.T,
                    bT = contact.U,
                    contactKind = contact.Kind.ToString(),
                    aPosition = DiagnosticArtifacts.Point(NetworkNormalizer.Evaluate(sections[contact.A], contact.T)),
                    bPosition = DiagnosticArtifacts.Point(NetworkNormalizer.Evaluate(sections[contact.B], contact.U)),
                    measuredDeckGapMeters = Math.Abs(
                        NetworkNormalizer.Evaluate(sections[contact.A], contact.T).y
                            - NetworkNormalizer.Evaluate(sections[contact.B], contact.U).y),
                    requiredDeckGapMeters = MinimumSafeGap,
                    applicability = contact.Kind == NetworkContactKind.IndependentLevels
                        ? "vertical-clearance"
                        : "horizontal-conflict-needs-review-not-height-stacking",
                    requestedContacts = contacts.Count,
                    sampleSelection = "smallest-gap-first-at-most-64-not-complete-coverage"
                }));
        }

        if (steep.Length > 0 || collisions.Count > 0)
        {
            foreach (var segment in steep.Take(10))
                log.Diagnostic(
                    $"NETWORK SAFETY GRADE way={segment.WayId}; kind={segment.Kind}; bridge={segment.Bridge}; tunnel={segment.Tunnel}; grade={NetworkVerticalProfiles.MaximumGrade(segment):R}; length={NetworkVerticalProfiles.ArcLength(segment):R}; start={segment.Start}; end={segment.End}");

            foreach (var contact in collisions
                .GroupBy(otherContact => (sections[otherContact.A].WayId, sections[otherContact.B].WayId))
                .Select(
                    group => group
                        .OrderBy(
                            otherContact => Math.Abs(
                                NetworkNormalizer.Evaluate(sections[otherContact.A], otherContact.T).y
                                    - NetworkNormalizer.Evaluate(sections[otherContact.B], otherContact.U).y))
                        .First())
                .Take(25))
                log.Diagnostic(
                    $"NETWORK SAFETY COLLISION A={sections[contact.A].WayId}/{sections[contact.A].StartNodeId}-{sections[contact.A].EndNodeId} kind={sections[contact.A].Kind} layer={sections[contact.A].Layer} bridge={sections[contact.A].Bridge} tunnel={sections[contact.A].Tunnel} at={contact.T:R} pos={NetworkNormalizer.Evaluate(sections[contact.A], contact.T)}; B={sections[contact.B].WayId}/{sections[contact.B].StartNodeId}-{sections[contact.B].EndNodeId} kind={sections[contact.B].Kind} layer={sections[contact.B].Layer} bridge={sections[contact.B].Bridge} tunnel={sections[contact.B].Tunnel} at={contact.U:R} pos={NetworkNormalizer.Evaluate(sections[contact.B], contact.U)}");

            var ways =
                steep
                    .Select(segment => segment.WayId)
                    .Concat(
                        collisions.SelectMany(
                            contact => new[] { sections[contact.A].WayId, sections[contact.B].WayId }))
                    .Distinct()
                    .Take(12);

            if (allowQualityWarnings)
            {
                log.Count("Network repair quality observations", collisions.Count + steep.Length);
                log.Warn(
                    $"Iteration quality warning: {collisions.Count} clearance observations and {steep.Length} excessive-grade sections remain after bounded repair. Complete network retained for testing; not production-accepted. Ways: {string.Join(",", ways)}.");

                return;
            }

            throw new InvalidOperationException(
                $"Strict offline network quality validation prevented application: {collisions.Count} clearance failures, {steep.Length} excessive grades. Ways: {string.Join(",", ways)}.");
        }
    }

    private static void DescribeQuality(IReadOnlyList<PreparedSegment> sections, ImportLog log, CancellationToken token)
    {
        var metrics = new List<(PreparedSegment Section, float Length, float Grade, float Radius)>();
        var joins = new Dictionary<Key, List<(PreparedSegment Section, bool Start)>>();

        foreach (var segment in sections)
        {
            token.ThrowIfCancellationRequested();
            var minimumRadius = float.PositiveInfinity;
            var vectorA = segment.CourseStart.xz;
            var vectorB = segment.CourseControl1.xz;
            var c = segment.CourseControl2.xz;
            var d = segment.CourseEnd.xz;

            for (var i = 0; i <= 32; i++)
            {
                var t = i / 32f;
                var u = 1 - t;
                var derivative = 3 * u * u * (vectorB - vectorA) + 6 * u * t * (c - vectorB) + 3 * t * t * (d - c);
                var second = 6 * u * (c - 2 * vectorB + vectorA) + 6 * t * (d - 2 * c + vectorB);
                var speed = math.length(derivative);
                var cross = Math.Abs(derivative.x * second.y - derivative.y * second.x);

                if (speed > .001f && cross > .001f)
                    minimumRadius = Math.Min(minimumRadius, speed * speed * speed / cross);
            }

            metrics.Add(
                (
                    segment,
                    NetworkVerticalProfiles.ArcLength(segment),
                    NetworkVerticalProfiles.MaximumGrade(segment),
                    minimumRadius));

            foreach (var start in new[] { true, false })
            {
                var key = new Key(segment.Kind, start ? segment.StartNodeId : segment.EndNodeId);

                if (!joins.TryGetValue(key, out var list))
                    joins[key] = list = new();

                list.Add((segment, start));
            }
        }

        log.DiagnosticRecords("network-constraint-observations.ndjson", metrics
            .OrderByDescending(item => item.Grade / Math.Max(.000001f, GradeLimit(item.Section))).Take(64)
            .Select(item => (object)new
            {
                constraint = "longitudinal-grade-and-sampled-curvature",
                policyStatus = "implementation-assumption-awaiting-game-validation",
                evidence = "prepared-curve-observation-not-native-readback",
                nativeRenderingVerified = false,
                item.Section.WayId,
                item.Section.StartNodeId,
                item.Section.EndNodeId,
                item.Section.SourceWayIds,
                kind = item.Section.Kind.ToString(),
                item.Section.Classification,
                item.Section.Bridge,
                item.Section.Tunnel,
                item.Section.Lanes,
                item.Section.OneWay,
                item.Section.NativeWidth,
                measuredMaximumGrade = item.Grade,
                desiredGrade = NetworkVerticalProfiles.DesiredGrade(item.Section),
                exceptionalGrade = GradeLimit(item.Section),
                exceptionalResidual = item.Grade - GradeLimit(item.Section),
                exceptionalContinuousCheckPassed = NetworkVerticalProfiles.WithinGradeCeiling(
                    item.Section,
                    GradeLimit(item.Section)),
                sampledMinimumRadiusMeters = float.IsInfinity(item.Radius) ? (float?)null : item.Radius,
                curvatureEvidence = "33-parameter-samples-not-continuous-certification",
                lengthMeters = item.Length,
                deckStart = DiagnosticArtifacts.Point(item.Section.CourseStart),
                deckControl1 = DiagnosticArtifacts.Point(item.Section.CourseControl1),
                deckControl2 = DiagnosticArtifacts.Point(item.Section.CourseControl2),
                deckEnd = DiagnosticArtifacts.Point(item.Section.CourseEnd),
                requestedSections = sections.Count,
                sampleSelection = "highest-grade-ratio-first-at-most-64-not-complete-coverage"
            }));

        foreach (var group in metrics.GroupBy(
            item => item.Section.Kind == NetworkKind.Rail
                ? "rail"
                : item.Section.ProtectedLink
                    ? "ramps"
                    : item.Section.Classification == "motorway" || item.Section.Classification == "trunk"
                        ? "mainline"
                        : "streets"))
        {
            var above = group.Count(item => item.Grade > NetworkVerticalProfiles.DesiredGrade(item.Section) + .001f);
            log.Diagnostic(
                $"NETWORK QUALITY class={group.Key}; sections={group.Count()}; spansBelow12m={group.Count(item => item.Length < 12)}; spansBelow24m={group.Count(item => item.Length < 24)}; desiredGradeExceeded={above}; maximumGrade={group.Max(item => item.Grade):R}; tightestSampledRadiusMeters={group.Min(item => item.Radius):R}; desiredNotExceptionalAcceptance=true; legitimateShortJunctionsAndTerminalsPreserved=true; curvatureIsSampledNotProof=true");

            foreach (var item in group
                .Where(otherItem => otherItem.Grade > NetworkVerticalProfiles.DesiredGrade(otherItem.Section) + .001f)
                .OrderByDescending(otherItem => otherItem.Grade)
                .Take(5))
                log.Diagnostic(
                    $"NETWORK QUALITY OUTLIER way={item.Section.WayId}; sourceIds={string.Join(",", item.Section.SourceWayIds)}; nodes={item.Section.StartNodeId},{item.Section.EndNodeId}; class={group.Key}; length={item.Length:R}; grade={item.Grade:R}; target={NetworkVerticalProfiles.DesiredGrade(item.Section):R}; radius={item.Radius:R}; bridge={item.Section.Bridge}; tunnel={item.Section.Tunnel}");
        }

        var tangentBreaks = 0;
        var gradeBreaks = 0;
        var maximumAngle = 0f;
        var maximumGradeJump = 0f;
        var joinObservations = new List<(object Record, float Severity)>();
        var joinCount = 0;

        foreach (var pair in joins.Values.Where(items => items.Count == 2 && items[0].Section != items[1].Section))
        {
            var left = pair[0];
            var right = pair[1];
            var vectorA =
                left.Start
                    ? left.Section.CourseStart - left.Section.CourseControl1
                    : left.Section.CourseEnd - left.Section.CourseControl2;

            var vectorB =
                right.Start
                    ? right.Section.CourseControl1 - right.Section.CourseStart
                    : right.Section.CourseControl2 - right.Section.CourseEnd;

            var angle =
                (float)Math.Acos(
                    math.clamp(math.dot(math.normalizesafe(vectorA.xz), math.normalizesafe(vectorB.xz)), -1, 1))
                    * 180
                    / math.PI;

            var leftPoint = left.Start ? left.Section.CourseStart : left.Section.CourseEnd;
            var rightPoint = right.Start ? right.Section.CourseStart : right.Section.CourseEnd;
            var leftReach = math.length(vectorA.xz);
            var rightReach = math.length(vectorB.xz);
            double? measuredJump = leftReach > 1e-8f && rightReach > 1e-8f
                ? Math.Abs(vectorA.y / leftReach - vectorB.y / rightReach) : null;

            joinCount++;
            var severity = Math.Max(angle / 15, math.distance(leftPoint, rightPoint) / .05f);

            if (measuredJump.HasValue && angle <= 30)
                severity = Math.Max(severity, (float)(measuredJump.Value / .01));

            joinObservations.Add((new
            {
                constraint = "degree-two-join-continuity",
                policyStatus = "implementation-assumption-awaiting-game-validation",
                evidence = "prepared-port-pair-not-native-lanes",
                nativeLaneConnectivityVerified = false,
                leftWay = left.Section.WayId,
                rightWay = right.Section.WayId,
                node = left.Start ? left.Section.StartNodeId : left.Section.EndNodeId,
                kind = left.Section.Kind.ToString(),
                angleDegrees = angle,
                gradeJump = measuredJump,
                endpointDistanceMeters = math.distance(leftPoint, rightPoint),
                leftReachMeters = leftReach,
                rightReachMeters = rightReach,
                tangentWarningDegrees = 15,
                throughJoinAngleCeilingDegrees = 30,
                gradeJumpWarning = .01,
                gradeJumpApplicable = angle <= 30 && measuredJump.HasValue,
                interpretation = "source-corner-may-be-intentional-inspect-before-changing-policy",
                leftLanes = left.Section.Lanes,
                rightLanes = right.Section.Lanes,
                leftOneWay = left.Section.OneWay,
                rightOneWay = right.Section.OneWay
            }, severity));

            // Keep only the worst 64 observations, not a second whole-network dump.
            if (joinObservations.Count > 64)
            {
                var least = 0;

                for (var j = 1; j < joinObservations.Count; j++)
                    if (joinObservations[j].Severity < joinObservations[least].Severity)
                        least = j;

                joinObservations.RemoveAt(least);
            }

            if (angle > 15)
            {
                tangentBreaks++;
                maximumAngle = Math.Max(maximumAngle, angle);
            }

            if (angle > 30)
                continue; // An intentional corner is not a grade-continuity claim.

            var jump =
                Math.Abs(
                    vectorA.y / Math.Max(.001f, math.length(vectorA.xz))
                        - vectorB.y / Math.Max(.001f, math.length(vectorB.xz)));

            if (jump > .01f)
            {
                gradeBreaks++;
                maximumGradeJump = Math.Max(maximumGradeJump, jump);
            }
        }

        log.Diagnostic(
            $"NETWORK QUALITY JOINS degreeTwoTangentBreaks={tangentBreaks}; maximumAngleDegrees={maximumAngle:R}; continuousDirectionGradeJumps={gradeBreaks}; maximumGradeJump={maximumGradeJump:R}; multiArmJunctionsNotClassifiedAsThroughJoins=true; sourceCornersMayBeIntentional=true");

        log.DiagnosticRecords(
            "network-join-observations.ndjson",
            joinObservations.OrderByDescending(item => item.Severity).Select(item => item.Record));

        log.Diagnostic(
            $"NETWORK JOIN EVIDENCE eligibleJoins={joinCount}; records={joinObservations.Count}; sampleSelection=worst-64; completeCoverage={joinCount <= 64}; zeroReachGradeNotInvented=true");

        log.Diagnostic(
            $"NETWORK CONSTRAINT EVIDENCE policyStatus=implementation-assumption-awaiting-game-validation; evidence=prepared-geometry; tangentWarningDegrees=15; throughJoinAngleCeilingDegrees=30; gradeJumpWarning=0.01; clearanceDeckGapMeters={MinimumSafeGap:R}; nativeRenderingVerified=false; sampledPassDoesNotValidatePhysicalPolicy=true; qualityFailureAction=RequiresReview-in-production");
    }

    private const float MinimumSafeGap = NetworkVerticalProfiles.MinimumBridgeClearance + 2 + .01f;

    public static void ValidateTunnelCover(
        IReadOnlyList<PreparedSegment> sections,
        float[] raster,
        float scale,
        double width,
        CancellationToken token,
        ImportLog? log = null,
        IReadOnlyDictionary<string, NativeAssetContract>? assets = null)
    {
        foreach (var sample in TunnelCoverSamples(sections, raster, scale, width, token, assets))
        {
            var segment = sections[sample.index];

            if (NetworkNormalizer.Evaluate(segment, sample.t).y > sample.ceiling + .05f)
            {
                // No-log calls are the existing strict offline audit contract.
                // Production supplies a log and retains safe inspectable output.
                if (log == null)
                    throw new InvalidOperationException(
                        $"Strict offline tunnel cover audit failed for way {segment.WayId} at {sample.t:R}.");

                log.Count("Final exposed tunnel observations");

                if (log.Metric("Final exposed tunnel observations") == 1)
                    log.Warn(
                        "Final terrain exposes tunnel geometry; retained as RequiresReview. Enable diagnostics for measured cover residuals.");

                if (log.DiagnosticsEnabled && log.Metric("Final exposed tunnel observations") <= 25)
                    log.Diagnostic(
                        $"TUNNEL COVER OBSERVATION way={segment.WayId}; nodes={segment.StartNodeId},{segment.EndNodeId}; t={sample.t:R}; deckY={NetworkNormalizer.Evaluate(segment, sample.t).y:R}; ceilingY={sample.ceiling:R}; residualMeters={NetworkNormalizer.Evaluate(segment, sample.t).y - sample.ceiling:R}; toleranceMeters=0.05; policyStatus=implementation-assumption-awaiting-game-validation; evidence=final-prepared-raster-not-native-readback; qualityFailureAction=RequiresReview");
            }
        }
    }

    internal static IEnumerable<(int index, float t, float ceiling)> TunnelCoverSamples(
        IReadOnlyList<PreparedSegment> sections,
        float[] raster,
        float scale,
        double width,
        CancellationToken token,
        IReadOnlyDictionary<string, NativeAssetContract>? assets = null)
    {
        var n = (int)Math.Sqrt(raster.Length);

        if (n * n != raster.Length || n < 2 || scale <= 0 || width <= 0)
            throw new InvalidOperationException("Invalid final terrain for tunnel cover audit.");

        var surface = new HashSet<Key>();
        var adjacency = new Dictionary<Key, List<(Key node, float length)>>();

        foreach (var segment in sections)
            foreach (var reverse in new[] { false, true })
            {
                var firstKey = new Key(segment.Kind, reverse ? segment.EndNodeId : segment.StartNodeId);
                var secondKey = new Key(segment.Kind, reverse ? segment.StartNodeId : segment.EndNodeId);

                if (!segment.Tunnel)
                {
                    surface.Add(firstKey);
                    continue;
                }

                if (!adjacency.TryGetValue(firstKey, out var list))
                    adjacency[firstKey] = list = new();

                list.Add((secondKey, NetworkVerticalProfiles.ArcLength(segment)));
            }

        var distance = new Dictionary<Key, float>();
        var queue = new List<Key>();

        foreach (var node in surface.Where(adjacency.ContainsKey))
        {
            distance[node] = 0;
            queue.Add(node);
        }

        for (var cursor = 0; cursor < queue.Count; cursor++)
        {
            token.ThrowIfCancellationRequested();
            var node = queue[cursor];

            foreach (var edge in adjacency[node])
            {
                var d = distance[node] + edge.length;

                if (d > 160 || distance.TryGetValue(edge.node, out var old) && old <= d)
                    continue;

                distance[edge.node] = d;
                queue.Add(edge.node);
            }
        }

        float Ground(float3 point) => NetworkProcessor.Sample(point.x, point.z, raster, width);

        for (var index = 0; index < sections.Count; index++)
        {
            var segment = sections[index];

            if (!segment.Tunnel)
                continue;

            var roofHeight =
                assets != null
                    && assets.TryGetValue(segment.NativeAssetKey, out var asset)
                    && asset.TryEnvelope(NativeStructureMode.Tunnel, out var envelope)
                    ? envelope.RangeMax
                    : 0;

            var length = NetworkVerticalProfiles.ArcLength(segment);
            var count = Math.Max(16, (int)Math.Ceiling(length / 4));
            var a =
                distance.TryGetValue(new(segment.Kind, segment.StartNodeId), out var da) ? da : float.PositiveInfinity;

            var b =
                distance.TryGetValue(new(segment.Kind, segment.EndNodeId), out var db) ? db : float.PositiveInfinity;

            for (var i = 0; i <= count; i++)
            {
                token.ThrowIfCancellationRequested();
                var t = i / (float)count;
                var point = NetworkNormalizer.Evaluate(segment, t);
                var portalDistance = Math.Min(a + length * t, b + length * (1 - t));
                var fraction = math.clamp(portalDistance / 160, 0, 1);
                var cover = 15 * fraction * fraction * (3 - 2 * fraction);
                var tangent =
                    math.normalizesafe(
                        (NetworkNormalizer.Evaluate(segment, Math.Min(1, t + .001f))
                            - NetworkNormalizer.Evaluate(segment, Math.Max(0, t - .001f))).xz);

                var side = new float3(-tangent.y, 0, tangent.x) * Radius(segment);
                var fullRoof = Math.Min(Ground(point), Math.Min(Ground(point + side), Ground(point - side)));
                // An open portal has no full-width roof at its boundary. Blend
                // into full-width auditing over one physical road width, not
                // an arbitrary 100m exemption. Interiors retain the full check.
                var roof =
                    math.lerp(
                        Ground(point),
                        fullRoof,
                        math.clamp(portalDistance / Math.Max(8, 2 * Radius(segment)), 0, 1));

                // Composition roof height never fades at a portal. Only the
                // ground allowance and requested cover have transition ramps.
                yield return (index, t, roof - cover - roofHeight);
            }
        }
    }

    public static void ValidateEnvelope(
        IReadOnlyList<PreparedSegment> sections,
        double width,
        float scale,
        CancellationToken token,
        ImportLog? log = null)
    {
        var half = (float)width / 2;

        foreach (var segment in sections)
        {
            token.ThrowIfCancellationRequested();

            // The Bezier is inside the convex hull of these four points. Check
            // the actual native course, not only its shared node anchors.
            foreach (var point in new[] {
                segment.CourseStart,
                segment.CourseControl1,
                segment.CourseControl2,
                segment.CourseEnd
                })
                if (!math.all(math.isfinite(point))
                    || Math.Abs(point.x) > half + .01f
                    || Math.Abs(point.z) > half + .01f
                    || point.y < 0
                    || point.y > scale)
                {
                    if (log == null)
                        throw new InvalidOperationException(
                            $"Strict offline native envelope audit failed for way {segment.WayId}.");

                    log.Count("Network envelope observations");

                    if (log.Metric("Network envelope observations") == 1)
                        log.Warn(
                            "Network envelope observations require local representability checks before native placement; see diagnostics.");

                    if (log.DiagnosticsEnabled && log.Metric("Network envelope observations") <= 25)
                        log.Diagnostic(
                            $"NETWORK ENVELOPE way={segment.WayId}; nodes={segment.StartNodeId},{segment.EndNodeId}; point={point}; finite={math.all(math.isfinite(point))}; halfWidthMeters={half:R}; heightScaleMeters={scale:R}; boundsToleranceMeters=0.01; evidence=prepared-control-hull; nativeRepresentabilityVerified=false; localNativeSafetyCheckRequired=true");
                }
        }
    }

    private static List<Contact> Contacts(IReadOnlyList<PreparedSegment> sections, CancellationToken token)
    {
        var grid = new Dictionary<(int, int), List<Span>>();
        var contacts = new Dictionary<(int, int, int, int), Contact>();
        var adjacent = new Dictionary<Key, List<(Key node, float length)>>();
        var lengths = sections.Select(NetworkVerticalProfiles.ArcLength).ToArray();
        var radii = sections.Select(Radius).ToArray();
        var searchLimit = radii.DefaultIfEmpty(0).Max() * 12 + 2;

        for (var i = 0; i < sections.Count; i++)
            foreach (var reverse in new[] { false, true })
            {
                var segment = sections[i];
                var firstKey = new Key(segment.Kind, reverse ? segment.EndNodeId : segment.StartNodeId);
                var secondKey = new Key(segment.Kind, reverse ? segment.StartNodeId : segment.EndNodeId);

                if (!adjacent.TryGetValue(firstKey, out var list))
                    adjacent[firstKey] = list = new();

                list.Add((secondKey, lengths[i]));
            }

        var paths = new Dictionary<Key, Dictionary<Key, float>>();

        Dictionary<Key, float> Paths(Key origin)
        {
            if (paths.TryGetValue(origin, out var cached))
                return cached;

            var best = new Dictionary<Key, float> { { origin, 0 } };
            var queue = new List<Key> { origin };

            for (var cursor = 0; cursor < queue.Count; cursor++)
            {
                token.ThrowIfCancellationRequested();
                var node = queue[cursor];

                if (!adjacent.TryGetValue(node, out var edges))
                    continue;

                foreach (var edge in edges)
                {
                    var distance = best[node] + edge.length;

                    if (distance > searchLimit || best.TryGetValue(edge.node, out var old) && old <= distance)
                        continue;

                    best[edge.node] = distance;
                    queue.Add(edge.node);
                }
            }

            paths[origin] = best;

            return best;
        }

        bool LocalJunction(int ai, float t, int bi, float u)
        {
            var firstSegment = sections[ai];
            var secondSegment = sections[bi];

            if (firstSegment.Kind != secondSegment.Kind)
                return false;

            float2 Tangent(PreparedSegment segment, float at)
                => math.normalizesafe(
                    (NetworkNormalizer.Evaluate(segment, Math.Min(1, at + .001f))
                        - NetworkNormalizer.Evaluate(segment, Math.Max(0, at - .001f))).xz);

            var parallel = Math.Abs(math.dot(Tangent(firstSegment, t), Tangent(secondSegment, u))) > .8f;
            var limit = (radii[ai] + radii[bi]) * (parallel ? 6 : 4) + 2;

            bool Via(long origin, float distance)
            {
                if (distance > limit)
                    return false;

                var best = Paths(new(firstSegment.Kind, origin));

                return best.TryGetValue(new(secondSegment.Kind, secondSegment.StartNodeId), out var left)
                    && distance + left + lengths[bi] * u <= limit
                    || best.TryGetValue(new(secondSegment.Kind, secondSegment.EndNodeId), out var right)
                           && distance + right + lengths[bi] * (1 - u) <= limit;
            }

            if (Via(firstSegment.StartNodeId, lengths[ai] * t) || Via(firstSegment.EndNodeId, lengths[ai] * (1 - t)))
                return true;

            return false;
        }

        var seen = new HashSet<Span>();

        for (var i = 0; i < sections.Count; i++)
        {
            token.ThrowIfCancellationRequested();
            var segment = sections[i];
            var steps = Math.Max(8, (int)Math.Ceiling(lengths[i] / 8));

            for (var j = 0; j < steps; j++)
            {
                var t = j / (float)steps;
                var u = (j + 1) / (float)steps;
                var vectorA = NetworkNormalizer.Evaluate(segment, t);
                var vectorB = NetworkNormalizer.Evaluate(segment, u);
                var span = new Span(i, t, u, vectorA, vectorB);
                var radius = Radius(segment) + .5f;
                var low = math.min(vectorA.xz, vectorB.xz) - radius;
                var high = math.max(vectorA.xz, vectorB.xz) + radius;
                seen.Clear();

                for (var z = (int)Math.Floor(low.y / 64); z <= (int)Math.Floor(high.y / 64); z++)
                    for (var x = (int)Math.Floor(low.x / 64); x <= (int)Math.Floor(high.x / 64); x++)
                    {
                        var cell = (x, z);

                        if (!grid.TryGetValue(cell, out var nearby))
                            grid[cell] = nearby = new();

                        foreach (var other in nearby)
                        {
                            if (other.Index == i || !seen.Add(other))
                                continue;

                            var otherSegment = sections[other.Index];
                            var separation = radii[i] + radii[other.Index] + .5f;

                            if (math.any(
                                math.max(vectorA.xz, vectorB.xz) + separation < math.min(other.A.xz, other.B.xz))
                                || math.any(
                                   math.max(other.A.xz, other.B.xz) + separation < math.min(vectorA.xz, vectorB.xz)))
                                continue;

                            // Connected decks can still cross remotely: only local
                            // junction geometry is exempt, never the whole component.
                            // Sharing an endpoint does not exempt a remote crossing
                            // of these same long curves. LocalJunction below measures
                            // actual distance to the connection at this contact.
                            Closest(vectorA.xz, vectorB.xz, other.A.xz, other.B.xz, out var at, out var bt);
                            var point = math.lerp(vectorA, vectorB, at);
                            var otherPoint = math.lerp(other.A, other.B, bt);

                            if (math.distance(point.xz, otherPoint.xz) > Radius(segment) + Radius(otherSegment) + .5f)
                                continue;

                            var st = math.lerp(t, u, at);
                            var ot = math.lerp(other.T, other.U, bt);
                            var headingA = (vectorB - vectorA).xz;
                            var headingB = (other.B - other.A).xz;
                            var kind =
                                NetworkContactPolicy.Classify(
                                    segment,
                                    otherSegment,
                                    point,
                                    otherPoint,
                                    headingA,
                                    headingB,
                                    LocalJunction(other.Index, ot, i, st));

                            if (kind == NetworkContactKind.Junction || kind == NetworkContactKind.LevelCrossing)
                                continue;

                            // Same formation tracks/carriageways are horizontal-spacing
                            // conflicts, never instructions to stack each track vertically.
                            // Ambiguous same-level structures need local reconstruction,
                            // not a fabricated layer order derived from source IDs.
                            var key = (other.Index, i, (int)(ot * 32), (int)(st * 32));
                            contacts[key] = new(other.Index, ot, i, st, kind);
                        }

                        nearby.Add(span);
                    }
            }
        }

        return contacts.Values.ToList();
    }

    // Closest points of two finite segments, including nearly parallel decks.
    private static void Closest(float2 vectorA, float2 vectorB, float2 c, float2 d, out float t, out float u)
    {
        var r = vectorB - vectorA;
        var s = d - c;
        var w = vectorA - c;
        var aa = math.dot(r, r);
        var bb = math.dot(r, s);
        var cc = math.dot(s, s);
        var dd = math.dot(r, w);
        var ee = math.dot(s, w);
        var determinant = aa * cc - bb * bb;
        t = determinant > 1e-8f ? math.clamp((bb * ee - cc * dd) / determinant, 0, 1) : 0;
        u = cc > 1e-8f ? math.clamp((bb * t + ee) / cc, 0, 1) : 0;
        t = aa > 1e-8f ? math.clamp((bb * u - dd) / aa, 0, 1) : 0;
    }
}
