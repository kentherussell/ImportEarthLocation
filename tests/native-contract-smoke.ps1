param([string]$ManagedPath = 'C:\Program Files (x86)\Steam\steamapps\common\Cities Skylines II\Cities2_Data\Managed')
$ErrorActionPreference = 'Stop'
[Reflection.Assembly]::LoadFrom((Join-Path $ManagedPath 'Colossal.Mono.Cecil.dll')) | Out-Null
$contractAssembly = [Colossal.Mono.Cecil.AssemblyDefinition]::ReadAssembly((Join-Path $ManagedPath 'Game.dll'))
try {
    $contractTypes = $contractAssembly.MainModule.Types
    $nodeGenerator=$contractTypes | Where-Object FullName -eq 'Game.Tools.GenerateNodesSystem'
    $nodeKey=$nodeGenerator.NestedTypes | Where-Object Name -eq 'NodeKey'
    $nodeEquals=$nodeKey.Methods | Where-Object Name -eq 'Equals'
    if (!(($nodeEquals.Body.Instructions | ForEach-Object ToString) -match 'float3::Equals')) { throw 'Native exact-position node identity contract changed' }
    $course=$contractTypes | Where-Object FullName -eq 'Game.Tools.CoursePos'
    foreach($fieldName in @('m_Entity','m_Position','m_Elevation','m_Flags')) {
        if(!($course.Fields | Where-Object Name -eq $fieldName)){throw "Native portal contract missing: $fieldName"}
    }
    $fillNodes=$nodeGenerator.NestedTypes | Where-Object Name -eq 'FillNodeMapJob'
    $addNode=$fillNodes.Methods | Where-Object Name -eq 'AddNode'
    if((($addNode.Body.Instructions | ForEach-Object ToString) -join "`n") -notmatch 'CoursePos::m_Entity'){throw 'Explicit committed-node attachment contract changed'}
    $netApply=$contractTypes | Where-Object FullName -eq 'Game.Tools.ApplyNetSystem'
    $netJob=$netApply.NestedTypes | Where-Object Name -eq 'HandleTempEntitiesJob'
    $netCreate=$netJob.Methods | Where-Object Name -eq 'Create'
    $removeTemp=$netCreate.Body.Instructions | Where-Object { $_.Operand -and $_.Operand.ToString() -match 'RemoveComponent<Game.Tools.Temp>' } | Select-Object -First 1
    if(!$removeTemp -or $removeTemp.Previous.OpCode.Name -ne 'ldarg.2'){throw 'Native Create no longer retains the generated entity identity'}
    $netUpdate=($netJob.Methods | Where-Object Name -eq 'Update').Body.Instructions | ForEach-Object ToString
    if(!($netUpdate -match 'Temp::m_Original') -or !($netUpdate -match 'm_ApplyUpdatedTypes') -or !($netUpdate -match 'Game.Common.Deleted')){throw 'Native ordinary-update lineage contract changed'}
    $tempFlags=$contractTypes | Where-Object FullName -eq 'Game.Tools.TempFlags'
    foreach($flagContract in @(@('Delete',2),@('Replace',256),@('Combine',4096))) {
        $flag=$tempFlags.Fields | Where-Object Name -eq $flagContract[0]
        if(!$flag -or [int]$flag.Constant -ne $flagContract[1]){throw 'Native replacement flag contract changed'}
    }
    $netExecute=($netJob.Methods | Where-Object { $_.Name -eq 'Execute' -and $_.HasBody } | Select-Object -First 1).Body.Instructions | ForEach-Object ToString
    foreach($mask in @('ldc.i4 256','ldc.i4 4352')) { if(!($netExecute -match [regex]::Escape($mask))){throw 'Native node/edge replacement execution contract changed'} }
    $netPatch=$netApply.NestedTypes | Where-Object Name -eq 'PatchTempReferencesJob'
    $patchExecute=($netPatch.Methods | Where-Object { $_.Name -eq 'Execute' -and $_.HasBody } | Select-Object -First 1).Body.Instructions | ForEach-Object ToString
    foreach($symbol in @('Temp::m_Original','Edge::m_Start','Edge::m_End','ConnectedEdge','ldc.i4 258')) { if(!($patchExecute -match [regex]::Escape($symbol))){throw 'Native incident-buffer/reference patch contract changed'} }
    $applyMode=$contractTypes | Where-Object FullName -eq 'Game.Tools.ApplyMode'
    foreach($expectedMode in @(@('Apply',1),@('Clear',2))){
        $value=$applyMode.Fields | Where-Object Name -eq $expectedMode[0]
        if(!$value -or [int]$value.Constant -ne $expectedMode[1]){throw 'Native commit/cancel mode contract changed'}
    }
    foreach($networkContract in @(
        @('Game.Net.Upgraded','m_Flags'),
        @('Game.Prefabs.CompositionFlags','m_General'),
        @('Game.Prefabs.CompositionFlags','m_Left'),
        @('Game.Prefabs.CompositionFlags','m_Right'),
        @('Game.Net.Roundabout','m_Radius'),
        @('Game.Net.Node','m_Position'),
        @('Game.Net.ConnectedEdge','m_Edge')
    )) {
        $networkType=$contractTypes | Where-Object FullName -eq $networkContract[0]
        if(!$networkType -or !($networkType.Fields | Where-Object Name -eq $networkContract[1])) { throw "Native wall/roundabout contract missing: $($networkContract -join '.')" }
    }
    $composition=$contractTypes | Where-Object FullName -eq 'Game.Prefabs.CompositionFlags'
    foreach($networkFlag in @(@('General','Roundabout',64),@('Side','Lowered',2))) {
        $enum=$composition.NestedTypes | Where-Object Name -eq $networkFlag[0]
        $flag=$enum.Fields | Where-Object Name -eq $networkFlag[1]
        if(!$flag -or [int]$flag.Constant -ne $networkFlag[2]) { throw "Native wall/roundabout flag changed: $($networkFlag[0]).$($networkFlag[1])" }
    }
    $placementSource=Get-Content -Raw (Join-Path $PSScriptRoot '..\ImportEarthLocation\Systems\NativeNetworkPlacement.cs')
    if($placementSource -notmatch 'm_Flags = structure \? CreationFlags.SubElevation : 0' -or $placementSource -notmatch 'CoursePosFlags.FreeHeight'){
        throw 'Ground courses must not use elevation-edit SubElevation; FreeHeight must remain supported'
    }
    $bootstrap=Get-Content -Raw (Join-Path $PSScriptRoot '..\ImportEarthLocation\Integration\NativeUiBootstrap.cs')
    foreach($guard in @('Core.UiPackageDiscovery.Find','RemoveActiveUIModLocation(new[]{s_RegisteredModule})','RemoveHostLocation("ui-mods",s_RegisteredDirectory)','ReferenceEquals(s_RegisteredHost,host)')){
        if(!$bootstrap.Contains($guard)){throw "Standalone UI package/owned cleanup contract missing: $guard"}
    }
    if($bootstrap.Contains('Path.GetDirectoryName(typeof(Mod).Assembly.Location)')){throw 'UI bootstrap again converts a byte-loaded assembly path before guarded discovery'}
    $appBindings=$contractTypes | Where-Object FullName -eq 'Game.UI.AppBindings'
    foreach($method in @('AddActiveUIModLocation','RemoveActiveUIModLocation')){
        if(!($appBindings.Methods | Where-Object { $_.Name -eq $method -and $_.IsPublic })){throw "Standalone UI public binding contract unavailable: $method"}
    }
    # Regression: one protected/unbound end does NOT disable the engine's
    # overlap optimizer. Verify the installed IL contract and the source policy
    # separately; neither check claims an actual imported network was published.
    $courseFlags=$contractTypes | Where-Object FullName -eq 'Game.Tools.CoursePosFlags'
    $disableMerge=$courseFlags.Fields | Where-Object Name -eq 'DisableMerge'
    if(!$disableMerge -or [int]$disableMerge.Constant -ne 4096){throw 'Native DisableMerge bit changed'}
    $split=$contractTypes | Where-Object FullName -eq 'Game.Tools.CourseSplitSystem'
    $overlapJob=$split.NestedTypes | Where-Object Name -eq 'FindOverlapsJob'
    $overlapExecute=$overlapJob.Methods | Where-Object { $_.Name -eq 'Execute' -and $_.HasBody } | Select-Object -First 1
    if(!$overlapExecute){throw 'Native overlap protection job unavailable'}
    $guard=@($overlapExecute.Body.Instructions | Select-Object -First 25 | ForEach-Object ToString)
    if(($guard -match 'CoursePos::m_Flags').Count -ne 2 -or ($guard -match '\band$').Count -lt 2 -or !($guard -match 'ldc.i4 4096') -or !($guard -match '\bret$')){throw 'Native BOTH-end DisableMerge early-return contract changed; inspect IL before trusting source guard'}
    foreach($endpoint in @('startPosition','endPosition')) {
        if($placementSource -notmatch ([regex]::Escape($endpoint)+ '\.m_Flags\s*\|=\s*CoursePosFlags\.DisableMerge\s*;')){throw "Imported course leaves $endpoint vulnerable to overlap coalescence"}
    }
    if($placementSource -match '_protectUnboundEndpoints' -or $placementSource -notmatch 'disableOverlapMergeBothEnds=true'){throw 'Native continuation protection reverted to unbound/retry-only policy'}
    if($placementSource -match 'ForceElevatedEdge.*IsStructure|ForceElevatedNode.*Abs\(') { throw 'Tunnel placement again forces positive elevated composition' }
    foreach ($contractName in @('Game.Prefabs.DefaultNetLane','Game.Prefabs.TrackLaneData')) {
        $laneType = $contractTypes | Where-Object FullName -eq $contractName
        if (!$laneType) { throw "Single-track verification contract unavailable: $contractName" }
    }
    $defaultLane = $contractTypes | Where-Object FullName -eq 'Game.Prefabs.DefaultNetLane'
    foreach ($fieldName in @('m_Lane','m_Position')) { if (!($defaultLane.Fields | Where-Object Name -eq $fieldName)) { throw "Track readback field unavailable: $fieldName" } }
    $trackLane = $contractTypes | Where-Object FullName -eq 'Game.Prefabs.TrackLaneData'
    if (!($trackLane.Fields | Where-Object Name -eq 'm_TrackTypes')) { throw 'Track-type readback unavailable' }
    foreach ($railContract in @(
        @('Game.Prefabs.NetGeometryPrefab','m_Sections'),
        @('Game.Prefabs.NetSectionInfo','m_Section'),
        @('Game.Prefabs.NetSectionPrefab','m_Pieces'),
        @('Game.Prefabs.NetSectionPrefab','m_SubSections'),
        @('Game.Prefabs.NetPieceInfo','m_Piece'),
        @('Game.Prefabs.NetSubSectionInfo','m_Section'),
        @('Game.Prefabs.NetPieceLanes','m_Lanes'),
        @('Game.Prefabs.NetLaneInfo','m_Lane'),
        @('Game.Prefabs.NetLaneInfo','m_Position'),
        @('Game.Prefabs.TrackLane','m_TrackType')
    )) {
        $railType = $contractTypes | Where-Object FullName -eq $railContract[0]
        if (!$railType -or !($railType.Fields | Where-Object Name -eq $railContract[1])) { throw "Rail definition hierarchy contract missing: $($railContract -join '.')" }
    }
    $paintType = $contractTypes | Where-Object FullName -eq 'Game.Rendering.TerrainMaterialSystem'
    $materialEnum = $contractTypes | Where-Object FullName -eq 'Game.Prefabs.TerrainMaterialType'
    foreach($slot in 1..4) {
        $slotField = $materialEnum.Fields | Where-Object Name -eq "Extra$slot"
        if(!$slotField -or [int]$slotField.Constant -ne (3+$slot)) { throw "Native brush selection changed: Extra$slot" }
    }
    $paintMethod=$paintType.Methods | Where-Object Name -eq 'ApplyToSplatmap'
    if(($paintMethod.Parameters.ParameterType.FullName -join ',') -ne 'UnityEngine.RenderTexture,Game.Tools.Brush,UnityEngine.Texture,Unity.Mathematics.float2,Unity.Mathematics.float2') { throw 'Native compute brush invocation signature changed' }
    $paintInstructions=($paintMethod.Body.Instructions | ForEach-Object ToString) -join "`n"
    foreach($symbol in @('m_PaintKernel','m_EraseKernel','_BrushChannels','_BrushOpacity')) {
        if($paintInstructions -notmatch [regex]::Escape($symbol)) { throw "Native paint/erase strength contract changed: $symbol" }
    }
    $paintSource=Get-Content -Raw (Join-Path $PSScriptRoot '..\ImportEarthLocation\Integration\NativeSurfaceBridge.cs')
    if($paintSource -notmatch 'SurfacePaint.VanillaMaterial' -or $paintSource -match 'name.Contains\(' -or $paintSource -match 'used.Length == 0.*return') { throw 'Surface mapping again guesses brush names or fails to clear Default' }
    if($paintSource -notmatch 'GraphicsFormat.R8G8_UNorm' -or $paintSource -notmatch 'TextureFormat.RG16' -or $paintSource -notmatch 'GetRawTextureData' -or $paintSource -notmatch 'paint.Invoke' -or $paintSource -match 'target.format != RenderTextureFormat.ARGB32') { throw 'Surface bridge does not use native RG16 shader encoding/readback' }
    $paintCreate=($paintType.Methods | Where-Object Name -eq 'OnCreate').Body.Instructions
    foreach($kernel in @('PaintDither','EraseDither')) { if(!(($paintCreate | ForEach-Object ToString) -match $kernel)) { throw "Native paint kernel missing: $kernel" } }
    foreach($textureName in @('UserSplatmap','UserSplatmapWorld')) {
        $nameInstruction=$paintCreate | Where-Object { $_.OpCode.Name -eq 'ldstr' -and $_.Operand -eq $textureName } | Select-Object -First 1
        if(!$nameInstruction -or $nameInstruction.Previous.Previous.Previous.OpCode.Name -ne 'ldc.i4.6') { throw "Native RG16 constructor changed: $textureName" }
    }
    $nativeBrush=$contractTypes | Where-Object FullName -eq 'Game.Tools.Brush'
    foreach($brushField in @('m_Tool','m_Position','m_Size','m_Angle','m_Strength')) { if(!($nativeBrush.Fields | Where-Object Name -eq $brushField)) { throw "Native compute brush field missing: $brushField" } }
    foreach ($fieldName in @('m_UserSplatmap', 'm_UserSplatmapWorld')) {
        $paintField = $paintType.Fields | Where-Object Name -eq $fieldName
        if (!$paintField -or $paintField.FieldType.FullName -ne 'UnityEngine.RenderTexture') { throw "Native paint field contract changed: $fieldName" }
    }
    foreach ($getterName in @('get_UserSplatmap', 'get_UserSplatmapWorld')) {
        $paintGetter = $paintType.Methods | Where-Object Name -eq $getterName
        if (!(($paintGetter.Body.Instructions | ForEach-Object ToString) -match 'Texture2D::get_blackTexture')) { throw "Native uninitialized paint fallback contract changed: $getterName" }
    }
    $waterApply = $contractTypes | Where-Object FullName -eq 'Game.Tools.ApplyWaterSourcesSystem'
    $waterJob = $waterApply.NestedTypes | Where-Object Name -eq 'HandleTempEntitiesJob'
    $waterCreate = $waterJob.Methods | Where-Object Name -eq 'Create'
    if (!(($waterCreate.Body.Instructions | ForEach-Object ToString) -match 'AddComponent<Game.Common.Created>')) { throw 'Native water creation lifecycle contract changed' }
    $waterUtils = $contractTypes | Where-Object FullName -eq 'Game.Simulation.WaterUtils'
    if (!($waterUtils.Methods | Where-Object { $_.Name -eq 'SampleDepth' -and $_.Parameters.Count -eq 2 })) { throw 'Native water depth readback unavailable' }
    $waterSystem = $contractTypes | Where-Object FullName -eq 'Game.Simulation.WaterSystem'
    foreach ($waterProperty in @('CellSize','IsNewMap','LastFrameSourceCache','CurrentJobSourceCache')) {
        if (!($waterSystem.Properties | Where-Object Name -eq $waterProperty)) { throw "Native river simulation diagnostic contract missing: $waterProperty" }
    }
    $sourceSimulation = $contractTypes | Where-Object FullName -eq 'Game.Simulation.WaterSimulation'
    $sourceJob = $sourceSimulation.NestedTypes | Where-Object Name -eq 'SourceJob'
    if (!(($sourceJob.Methods | Where-Object Name -eq 'Execute').Body.Instructions | Where-Object { $_.Operand -and $_.Operand.ToString() -match 'Game.Objects.Transform' })) { throw 'Native water collector no longer uses Transform' }
    $terrainType=$contractTypes | Where-Object FullName -eq 'Game.Simulation.TerrainSystem'
    foreach($signature in @(
        @('SetHeightmap','UnityEngine.Texture2D'),
        @('SetWorldHeightmap','UnityEngine.Texture2D,System.Boolean'),
        @('FinalizeTerrainData','UnityEngine.Texture2D,UnityEngine.Texture2D,Unity.Mathematics.float2,Unity.Mathematics.float2,Unity.Mathematics.float2,Unity.Mathematics.float2,Unity.Mathematics.float2,Unity.Mathematics.float2')
    )) {
        if(!($terrainType.Methods | Where-Object { $_.Name -eq $signature[0] -and ($_.Parameters.ParameterType.FullName -join ',') -eq $signature[1] })) { throw "Guarded native terrain stage changed: $($signature[0])" }
    }
    foreach($property in @('heightmap','worldHeightmap','heightScaleOffset','playableOffset','playableArea','worldOffset','worldSize','worldHeightMinMax','worldMapAsset')) {
        if(!($terrainType.Properties | Where-Object Name -eq $property)){throw "Terrain snapshot/transform contract unavailable: $property"}
    }
    $worldSetter=($terrainType.Methods | Where-Object Name -eq 'SetWorldHeightmap').Body.Instructions | ForEach-Object ToString
    if(!($worldSetter -match 'Graphics::CopyTexture') -or !($worldSetter -match 'm_WorldMapEditable')){throw 'Owned world-terrain restore contract changed'}
    foreach($fieldContract in @(
        @('Game.Net.SubLane','m_SubLane'),@('Game.Net.Lane','m_StartNode'),@('Game.Net.Lane','m_EndNode'),
        @('Game.Net.CarLane','m_Flags'),@('Game.Net.TrackLane','m_Flags'),@('Game.Common.Owner','m_Owner'),
        @('Game.Net.Composition','m_Edge'),@('Game.Prefabs.NetCompositionData','m_Width'),
        @('Game.Prefabs.NetCompositionData','m_Flags'),@('Game.Prefabs.DlcRequirement','m_Dlc'),
        @('Game.Prefabs.ContentPrerequisite','m_ContentPrerequisite')
    )) {
        $nativeType=$contractTypes | Where-Object FullName -eq $fieldContract[0]
        if(!$nativeType -or !($nativeType.Fields | Where-Object Name -eq $fieldContract[1])){throw "Native readback/vegetation contract missing: $($fieldContract -join '.')"}
    }
    foreach($flagContract in @(@('Game.Net.CarLaneFlags','Invert',4),@('Game.Net.CarLaneFlags','Twoway',128),@('Game.Net.TrackLaneFlags','Invert',1),@('Game.Net.TrackLaneFlags','Twoway',2))) {
        $flagType=$contractTypes | Where-Object FullName -eq $flagContract[0]
        $flag=$flagType.Fields | Where-Object Name -eq $flagContract[1]
        if(!$flag -or [int]$flag.Constant -ne $flagContract[2]){throw "Native movement direction flag changed: $($flagContract[0]).$($flagContract[1])"}
    }
    foreach($typeName in @('Game.Objects.Plant','Game.Prefabs.PlantData','Game.Prefabs.TreeData')) {
        if(!($contractTypes | Where-Object FullName -eq $typeName)){throw "Native vegetation contract unavailable: $typeName"}
    }
    $sourceRoot=Join-Path $PSScriptRoot '..\ImportEarthLocation'
    $terrainSource=Get-Content -Raw (Join-Path $sourceRoot 'Integration\TerrainBridge.cs')
    if($terrainSource -match '_replaceHeightmap\.Invoke' -or $terrainSource -notmatch 'var backup=Capture' -or $terrainSource -notmatch 'Finalize\(backup,oldScale\)' -or $terrainSource -notmatch 'worldBackup,true' -or $terrainSource -notmatch 'Resample\(prepared.Heightmap, backup.width, backup.height\)'){throw 'Terrain replacement lacks transform-preserving snapshot/owned rollback'}
    $commitSource=Get-Content -Raw (Join-Path $sourceRoot 'Systems\NetworkCommitSystem.cs')
    if($commitSource -notmatch 'ReadOnly<Game.Objects.Plant>' -or $commitSource -notmatch 's_Submitted.Contains\(entity\)' -or $commitSource -notmatch 's_SettledFrames>=2' -or $commitSource -notmatch 'CanStartExclusive'){throw 'Native commit again completes on request, ignores bush-only batches or lacks unrelated-preview preflight'}
    if($commitSource.IndexOf('native-commit/before-apply') -lt 0 -or $commitSource.IndexOf('native-commit/before-apply') -gt $commitSource.IndexOf('_applyModeSetter.Invoke(lastTool, new object[] { ApplyMode.Apply })')){throw 'Native diagnostic checkpoint is not recorded before requesting Apply'}
    if($commitSource -notmatch 'ReadOnly<Node>' -or $commitSource.IndexOf('s_BeforeApply?.Invoke') -lt 0 -or $commitSource.IndexOf('s_BeforeApply?.Invoke') -gt $commitSource.IndexOf('_applyModeSetter.Invoke(lastTool, new object[] { ApplyMode.Apply })')){throw 'Native graph lineage is not captured before Apply or node retirement is omitted'}
    foreach($guard in @('NativeBatchBindings.Match','TempFlags.Replace','TempFlags.Combine','HasBuffer<ConnectedEdge>','m_Entity=existingStart','NativeBatchBindings.Rebase','live.m_Start!=e.StartNode')) {
        if($placementSource -notmatch [regex]::Escape($guard)){throw "Native batch lineage/continuation guard missing: $guard"}
    }
    foreach($guard in @('NativeBatchBindings.RebaseBound','CurveStart=curve.a','_submittedCourses','ArtifactStem+"-owners.ndjson"','_verifiedPaths','BindVerified')) {
        if($placementSource -notmatch [regex]::Escape($guard)){throw "Native center/port provenance or retained batch evidence missing: $guard"}
    }
    if($commitSource -notmatch 's_Recoverable=!s_Applied&&exception is NativePreviewException' -or $commitSource -notmatch 'CancelOwnedPreview'){throw 'Native preview recovery can retry an applied/unclassified failure or lacks owned preview cleanup'}
    $applySource=Get-Content -Raw (Join-Path $sourceRoot 'Systems\ImportApplySystem.cs')
    if($applySource -notmatch 'TryCaptureCommittedNodes' -or $applySource -notmatch '_readbackFrames>=180' -or $applySource -notmatch '_phase >= 3 && _phase <= 5'){throw 'Live batch readiness or failure-stage routing guard missing'}
    foreach($guard in @('NativePreviewRecovery.NextBatchSize','_phase==8','_recoveryWaitFrames<180','_networkIndex-=_networkBatch.Count','CanStartExclusive','_recoveryUntilIndex')) {
        if($applySource -notmatch [regex]::Escape($guard)){throw "Bounded pre-commit recovery/progress guard missing: $guard"}
    }
    $logSource=Get-Content -Raw (Join-Path $sourceRoot 'Core\ImportLog.cs')
    if($logSource -notmatch 'catch \(Exception error\)' -or $logSource -notmatch 'disk logging is not required for generation'){throw 'Diagnostic artifact failure can change import acceptance'}
    $vegetationSource=Get-Content -Raw (Join-Path $sourceRoot 'Systems\NativeVegetationPlacement.cs')
    if($vegetationSource -notmatch 'isBuiltin' -or $vegetationSource -notmatch 'RequiresPaidContent' -or $vegetationSource -notmatch 'isSubscribedMod'){throw 'Vanilla vegetation palette lacks content-origin guards'}
    $readbackSource=Get-Content -Raw (Join-Path $sourceRoot 'Integration\NativeNetworkReadback.cs')
    if($readbackSource -notmatch 'actualVia.Count!=1' -or $readbackSource -notmatch 'TrackLaneFlags.Twoway' -or $readbackSource -notmatch 'CarLaneFlags.Invert'){throw 'Native movement audit lacks junction/direction qualification'}
    if($readbackSource -notmatch 'VerifiedPaths.TryGetValue' -or $readbackSource -notmatch 'byEntity.TryGetValue' -or $readbackSource -notmatch 'cursor==b'){throw 'Native readback again treats recentered/subdivided verified chains as missing single edges'}
    $roadProfile=Get-Content -Raw (Join-Path $sourceRoot 'Processing\NetworkVerticalProfiles.cs')
    if($roadProfile -match 'portals=new\[\]\{first,last\}' -or $roadProfile -notmatch 'wholeInterchangePlane=false'){throw 'Branched bridge network again uses one farthest-node plane'}
    $railProfile=Get-Content -Raw (Join-Path $sourceRoot 'Processing\RailVerticalProfiles.cs')
    if($railProfile -match 'Any\(s=>s.Bridge\)\)heights\[node.Key\]\+=8' -or $railProfile -notmatch 'blanketBridgePortalLift=false'){throw 'Rail bridge portals again receive an unconditional lift'}
    $terrainBuilder=Get-Content -Raw (Join-Path $sourceRoot 'Processing\TerrainBuilder.cs')
    $tileClient=Get-Content -Raw (Join-Path $sourceRoot 'Data\TerrainTileClient.cs')
    $uiSource=Get-Content -Raw (Join-Path $sourceRoot 'Systems\ImportEarthLocationUISystem.cs')
    if($terrainBuilder -notmatch 'sourceOriginX: cropLeft' -or $tileClient -notmatch 'unavailable-fine-tile-parent' -or $uiSource -notmatch 'PREPARATION DATA UNAVAILABLE'){throw 'DEM crop origin or typed bounded recovery outcome is missing'}
    $waterDiagnostic=Get-Content -Raw (Join-Path $sourceRoot 'Integration\NativeDiagnostics.cs')
    $waterBridge=Get-Content -Raw (Join-Path $sourceRoot 'Integration\NativeWaterBridge.cs')
    if($waterDiagnostic -notmatch 'PlannedWater.GroupBy' -or $waterDiagnostic -notmatch 'missingPlannedControls' -or $waterBridge -notmatch 'WATER APPLY BODY'){throw 'Water audit loses planned bodies with no safe native control'}
    if($vegetationSource -match 'palette=palettes\[VegetationForm.Broadleaf\]'){throw 'Unavailable stock growth form silently becomes unrelated foliage'}
    Write-Output 'PASS: installed-game metadata and source guards confirm terrain snapshot/transform/owned rollback stages, native commit retirement, stock vegetation provenance, directed road/rail lane readback, sided walls, roundabouts, signed tunnels, existing paint and water lifecycle contracts. Game not launched; this is not visual or simulation acceptance.'
}
finally { $contractAssembly.Dispose() }
