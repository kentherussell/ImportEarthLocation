import React, { useEffect, useRef, useState } from "react";
import { createPortal } from "react-dom";
import { bindValue, trigger, useValue } from "cs2/api";
import { Button, Panel, Portal, Scrollable, Tooltip } from "cs2/ui";
import styles from "./import-earth-location.module.scss";
import { Location, LocationPicker } from "./location-picker";
import importIcon from "../import-earth-icon-generated.png";

const group = "ImportEarthLocation";
const isEditor$ = bindValue<boolean>(group, "isEditor", false);
const mapExt$ = bindValue<boolean>(group, "mapExtDetected", false);
const mapSize$ = bindValue<string>(group, "mapSize", "vanilla");
const loadedMapSize$ = bindValue<string>(group, "loadedMapSize", "");
const panelOpen$ = bindValue<boolean>(group, "panelOpen", false);
export const ImportLauncher = ({ theme, className }: { theme?: any; className?: string }) => {
  const editor = useValue(isEditor$);
  const open = useValue(panelOpen$);
  if (!editor) return null;
  return <Tooltip tooltip={<span className={styles.brandText}>Import Earth</span>} direction="up"><Button variant="icon"
    focusKey="import-earth-location-launcher" className={className} theme={theme}
    selected={open}
    onSelect={() => trigger(group, "setPanelOpen", !open)} aria-label="Import Earth"><img className={styles.launcherIcon} src={importIcon} alt="" width={32} height={32} style={{ width: "32rem", height: "32rem", maxWidth: "32rem", maxHeight: "32rem", flexShrink: 0, objectFit: "contain" }} /></Button></Tooltip>;
};
const AnchoredLauncher = () => {
  const editor = useValue(isEditor$);
  const [host, setHost] = useState<HTMLElement | null>(null);
  const [nativeButtonClass, setNativeButtonClass] = useState("");
  useEffect(() => {
    if (!editor) return;
    const container = document.createElement("div");
    container.id = "import-earth-location-launcher-host";
    let scheduled: number | null = null;
    let stopped = false;
    let lastMode = "";
    const place = () => {
      scheduled = null;
      if (stopped) return;
      const found = document.querySelector<HTMLElement>('[class*="editor-toolbar_"]');
      const rectangle = found?.getBoundingClientRect();
      const toolbar = found && rectangle && rectangle.width > 0 && rectangle.height > 0 ? found : null;
      const buttons = toolbar ? Array.from(toolbar.querySelectorAll<HTMLElement>("button"))
        .filter(button => !container.contains(button)) : [];
      let row = buttons.length > 1 ? buttons[0].parentElement : toolbar;
      const last = buttons[buttons.length - 1];
      // Reuse only native button style classes, never a neighbor's selected,
      // disabled, or mod-specific state. Native rules own colors and hover state.
      const peer = buttons.find(button => Array.from(button.classList).some(name => /^(button|tool-button|icon-button)_/.test(name)));
      const peerClasses = peer ? Array.from(peer.classList).filter(name => /^(button|tool-button|icon-button)_/.test(name)).join(" ") : "";
      setNativeButtonClass(previous => previous === peerClasses ? previous : peerClasses);
      if (peer) {
        const size = window.getComputedStyle(peer);
        container.style.width = size.width;
        container.style.height = size.height;
      } else {
        container.style.width = "48rem";
        container.style.height = "48rem";
      }
      while (row && toolbar && last && !row.contains(last)) row = row.parentElement;
      if (row && toolbar && !toolbar.contains(row)) row = toolbar;
      const target = row || toolbar || document.body;
      const mode = toolbar ? "toolbar" : "fallback";
      container.className = toolbar ? styles.launcherHostInline : styles.launcherHostFallback;
      if (container.parentElement !== target) target.appendChild(container);
      if (mode !== lastMode) {
        console.info("ImportEarthLocation launcher placement", { mode, toolbarFound: !!toolbar, nativeButtons: buttons.length });
        trigger(group, "frontendState", JSON.stringify({ event: "launcher-placement", mode, nativeButtons: buttons.length }));
        lastMode = mode;
      }
    };
    const schedule = () => {
      if (!stopped && scheduled === null) scheduled = window.requestAnimationFrame(place);
    };
    place(); setHost(container);
    // Polling is also retained: some Gameface versions/mods replace UI trees
    // without reliable DOM observation. Only our own host is moved/removed.
    const observer = typeof MutationObserver !== "undefined" ? new MutationObserver(schedule) : null;
    observer?.observe(document.body, { childList: true, subtree: true });
    const interval = window.setInterval(schedule, 1000);
    window.addEventListener("resize", schedule);
    return () => {
      stopped = true; observer?.disconnect(); window.clearInterval(interval);
      window.removeEventListener("resize", schedule);
      if (scheduled !== null) window.cancelAnimationFrame(scheduled);
      container.remove();
    };
  }, [editor]);
  return editor && host ? createPortal(<ImportLauncher className={`${nativeButtonClass} ${styles.launcherButton}`} />, host) : null;
};
// A share from 0 (off) to 1 in tenths: click or drag the track, or step with the
// buttons, which also serve keyboard and controller navigation.
export const DensitySlider = ({ label, value, disabled, focusKey, onChange }:
  { label: string; value: number; disabled: boolean; focusKey: string; onChange: (value: number) => void }) => {
  const step = (v: number) => Math.round(Math.max(0, Math.min(1, v)) * 10) / 10;
  const fromPointer = (track: HTMLElement, x: number) => {
    const box = track.getBoundingClientRect();
    return step(box.width > 0 ? (x - box.left) / box.width : value);
  };
  const begin = (event: React.MouseEvent<HTMLDivElement>) => {
    if (disabled) return;
    const track = event.currentTarget;
    let last = fromPointer(track, event.clientX);
    onChange(last);
    const move = (e: MouseEvent) => { const next = fromPointer(track, e.clientX); if (next !== last) { last = next; onChange(next); } };
    const end = () => { window.removeEventListener("mousemove", move); window.removeEventListener("mouseup", end); };
    window.addEventListener("mousemove", move);
    window.addEventListener("mouseup", end);
  };
  return <div className={`${styles.slider} ${disabled ? styles.sliderDisabled : ""}`} role="slider" aria-label={label}
    aria-valuemin={0} aria-valuemax={100} aria-valuenow={Math.round(value * 100)}>
    <span className={styles.sliderLabel}>{label}</span>
    <Button variant="flat" className={styles.sliderStep} focusKey={`${focusKey}-down`} disabled={disabled || value <= 0} onSelect={() => onChange(step(value - .1))}>−</Button>
    <div className={styles.sliderTrack} onMouseDown={begin}><div className={styles.sliderFill} style={{ width: `${Math.round(value * 100)}%` }} /></div>
    <Button variant="flat" className={styles.sliderStep} focusKey={`${focusKey}-up`} disabled={disabled || value >= 1} onSelect={() => onChange(step(value + .1))}>+</Button>
    <span className={styles.sliderValue}>{value <= 0 ? "Off" : `${Math.round(value * 100)}%`}</span>
  </div>;
};
const busy$ = bindValue<boolean>(group, "busy", false);
const canResume$ = bindValue<boolean>(group, "canResume", false);
const ready$ = bindValue<boolean>(group, "ready", false);
const stage$ = bindValue<number>(group, "stage", 0);
const status$ = bindValue<string>(group, "status", "");
const error$ = bindValue<string>(group, "error", "");
const summary$ = bindValue<string>(group, "summary", "");
const log$ = bindValue<string>(group, "log", "");
const location$ = bindValue<string>(group, "location", "{}");
const roads$ = bindValue<string>(group, "roadLevel", "none");
const rail$ = bindValue<boolean>(group, "importRail", true);
const forests$ = bindValue<boolean>(group, "importForests", true);
const rocks$ = bindValue<boolean>(group, "importRocks", false);
const vegetationDensity$ = bindValue<number>(group, "vegetationDensity", 1);
const rockDensity$ = bindValue<number>(group, "rockDensity", 0);
const hdTerrain$ = bindValue<boolean>(group, "hdTerrain", false);
// Hover help for each option: a title and one short, plain-language line.
const tips: Record<string, string> = {
  "Highways": "Motorways and their on and off ramps.",
  "Main Roads": "Major through roads between towns and districts.",
  "Streets": "Local streets. Takes longer and is still being improved.",
  "Rail": "Train lines and tracks.",
  "Vegetation": "Forests, parks, trees and bushes. Lower values plant fewer; Off skips them.",
  "Rocks": "Rock clumps below cliffs and rocky slopes. Lower values place fewer; Off skips them.",
  "HD Terrain": "Sharper hills, valleys and coastlines. Downloads more, so preparing takes a little longer.",
  "Air Routes": "Lets planes fly in and out of your map.",
  "Ship Routes": "Lets ships sail in and out of your map.",
  "Surfaces": "Paints sand, grass, dirt and rock on the ground.",
  "Diagnostics": "Saves detailed logs for troubleshooting. Slower, so leave it off normally."
};
// The panel sits above the game's tooltip layer, so native tooltips drew under
// it. The hover help is drawn by the mod instead, beside the panel, as a
// viewport overlay in the same Portal.
type HoverTip = { label: string; top: number; left: number };
const airConnections$ = bindValue<boolean>(group, "createAirConnections", false);
const seaConnections$ = bindValue<boolean>(group, "createSeaConnections", false);
const diagnostics$ = bindValue<boolean>(group, "diagnosticsEnabled", false);
const surfaces$ = bindValue<boolean>(group,"applyGroundSurfaces",true);
const session$ = bindValue<string>(group, "mapSession", "");
const actionRevision$ = bindValue<string>(group, "actionRevision", "0");
const serviceUrl$ = bindValue<string>(group, "serviceUrl", "");
const serviceStatus$ = bindValue<string>(group, "serviceStatus", "");
const serviceRevision$ = bindValue<number>(group, "serviceRevision", 0);

class ImportPanelBoundary extends React.Component<React.PropsWithChildren<{}>, { failed: boolean }> {
  state = { failed: false };

  static getDerivedStateFromError() { return { failed: true }; }

  componentDidCatch(error: Error) {
    console.error("Import Earth panel failed:", error.message);
    trigger(group, "frontendState", JSON.stringify({ event: "render-error", error: error.message }));
  }

  render() {
    if (!this.state.failed) return this.props.children;
    return <div className={styles.panel}>
      <div className={styles.error}>The import panel could not be displayed. Close it and review UI.log.</div>
      <Button variant="flat" className={`${styles.actionButton} ${styles.utility}`} onSelect={() => { trigger(group, "setPanelOpen", false); this.setState({ failed: false }); }}>Close</Button>
    </div>;
  }
}

export const ImportEarthLocation = () => <ImportPanelBoundary><ImportEarthLocationContent /></ImportPanelBoundary>;

export const ImportEarthLocationContent = () => {
  const isEditor = useValue(isEditor$);
  const mapExt = useValue(mapExt$);
  const panelOpen = useValue(panelOpen$);
  const busy = useValue(busy$);
  const canResume = useValue(canResume$);
  const ready = useValue(ready$);
  const applying = useValue(stage$) === 6;
  const status = useValue(status$);
  const error = useValue(error$);
  const summary = useValue(summary$);
  const log = useValue(log$);
  const [latitude, setLatitude] = useState("");
  const [longitude, setLongitude] = useState("");
  const mapSize = useValue(mapSize$);
  const loadedMapSize = useValue(loadedMapSize$);
  const rawRoadLevel = useValue(roads$);
  const roadGroups = rawRoadLevel.split("|").filter(id => ["highways", "routes", "streets"].includes(id));
  // Toggle atomically in the backend; rapid clicks on different checkboxes
  // must not overwrite a previous selection before its binding update arrives.
  const toggleRoadGroup = (id: string) => trigger(group, "toggleRoadGroup", id);
  const rail = useValue(rail$);
  const forests = useValue(forests$);
  const rocks = useValue(rocks$);
  const vegetationDensity = useValue(vegetationDensity$);
  const rockDensity = useValue(rockDensity$);
  const hdTerrain = useValue(hdTerrain$);
  const mapSession = useValue(session$);
  const actionRevision = useValue(actionRevision$);
  const actionGate = useRef<string | null>(null);
  const [starting, setStarting] = useState(false);
  const surfaces = useValue(surfaces$);
  const airConnections = useValue(airConnections$);
  const seaConnections = useValue(seaConnections$);
  const diagnostics = useValue(diagnostics$);
  const [hoverTip, setHoverTip] = useState<HoverTip | null>(null);
  const withTip = (label: string, element: JSX.Element, key?: string, wide?: boolean) =>
    <div key={key} className={wide ? `${styles.tipHost} ${styles.wide}` : styles.tipHost} data-tip={tips[label]}
      onMouseEnter={event => {
        const rect = event.currentTarget.getBoundingClientRect();
        const panel = event.currentTarget.closest(`.${styles.panel}`);
        setHoverTip({ label, top: rect.top, left: panel ? panel.getBoundingClientRect().left : rect.left });
      }}
      onMouseLeave={() => setHoverTip(null)}>{element}</div>;
  const [confirming, setConfirming] = useState(false);
  const [showLog, setShowLog] = useState(false);
  const [pickerOpen, setPickerOpen] = useState(false);
  const locationText = useValue(location$);
  const serviceUrl = useValue(serviceUrl$);
  const serviceStatus = useValue(serviceStatus$);
  const serviceRevision = useValue(serviceRevision$);
  const [serviceOpen, setServiceOpen] = useState(false);
  const [serviceDraft, setServiceDraft] = useState("");
  useEffect(() => { setServiceDraft(serviceUrl); }, [serviceUrl]);
  useEffect(() => { setServiceOpen(false); }, [serviceRevision]);
  useEffect(() => { if (panelOpen && !serviceUrl && !busy) setServiceOpen(true); }, [panelOpen]);
  useEffect(() => {
    console.info("ImportEarthLocation UI mounted", { isEditor, mapSession });
    if (mapSession) trigger(group, "frontendState", JSON.stringify({ event: "mounted", isEditor, mapSession }));
  }, [isEditor, mapSession]);
  useEffect(() => { actionGate.current = null; setStarting(false); }, [busy, ready, error, actionRevision]);
  useEffect(() => { setConfirming(false); setPickerOpen(false); setShowLog(false); actionGate.current = null; setStarting(false); }, [mapSession]);
  useEffect(() => { if (!ready) setConfirming(false); }, [ready]);
  useEffect(() => {
    try {
      const value = JSON.parse(locationText);
      setLatitude(typeof value?.latitude === "number" && isFinite(value.latitude) ? String(value.latitude) : "");
      setLongitude(typeof value?.longitude === "number" && isFinite(value.longitude) ? String(value.longitude) : "");
    } catch { setLatitude(""); setLongitude(""); }
  }, [locationText]);

  const closePanel = () => {
    setHoverTip(null);
    setConfirming(false);
    setPickerOpen(false);
    setServiceOpen(false);
    trigger(group, "setPanelOpen", false);
  };

  useEffect(() => {
    if (!panelOpen) { setConfirming(false); return; }
    const onKeyDown = (event: KeyboardEvent) => {
      if (event.key !== "Escape" && event.keyCode !== 27) return;
      event.preventDefault();
      event.stopPropagation();
      if (serviceOpen) setServiceOpen(false);
      else if (pickerOpen) setPickerOpen(false);
      else if (confirming) setConfirming(false);
      else closePanel();
    };
    window.addEventListener("keydown", onKeyDown, true);
    return () => window.removeEventListener("keydown", onKeyDown, true);
  }, [panelOpen, confirming, pickerOpen, serviceOpen]);

  if (!isEditor) return null;

  const hasLocation = latitude !== "" && longitude !== "";
  const locationSummary = [mapSize === "mapext57" ? "57km" : mapSize === "mapext28" ? "28km" : "14km",
    latitude !== "" ? `Lat ${latitude}` : "", longitude !== "" ? `Lon ${longitude}` : ""].filter(Boolean).join(" / ");

  return (
    <Portal>
      <div className={styles.host}>
        <AnchoredLauncher />

        {panelOpen && (
          <Panel header={<span className={styles.brand}><img src={importIcon} alt="" /><span className={styles.brandText}>Import Earth</span></span>} className={styles.panel} theme={{ header: styles.panelHeader, titleBar: styles.panelTitleBar, title: styles.panelTitle, iconSpace: styles.panelIconSpace, closeButton: styles.panelCloseButton, content: styles.panelContent }} onClose={closePanel}>
            <Scrollable vertical className={styles.panelBody}>
            <div className={styles.section}>
              {!serviceUrl && <div className={styles.notice}>No personal data service configured. Public backup downloads could take hours. Set up a free NextGIS service for faster access.</div>}
              <div className={styles.locationActions}>
                <Button variant="flat" className={`${styles.actionButton} ${styles.utility}`} disabled={busy || starting} onSelect={() => { setServiceDraft(serviceUrl); setServiceOpen(true); }}>Configure Data Source</Button>
                <Button variant="flat" className={`${styles.actionButton} ${styles.secondary}`} disabled={busy || starting} onSelect={() => setPickerOpen(true)}>Select Location on Map</Button>
              </div>
              <div className={styles.locationSummary}>{locationSummary}</div>
              <div className={styles.field} role="group" aria-label="Road groups">
                <span className={styles.sectionTitle}>Road Groups</span>
                <div className={styles.options}>
                {[["highways", "Highways"], ["routes", "Main Roads"], ["streets", "Streets"]].map(([id,label]) => withTip(label,
                  <Button variant="flat" className={styles.check} focusKey={`iel-roads-${id}`} role="checkbox" aria-checked={roadGroups.includes(id)} disabled={busy || starting} onSelect={() => toggleRoadGroup(id)}>
                    <span className={styles.checkbox}>{roadGroups.includes(id) ? "✓" : ""}</span><span>{label}{id === "streets" && <span className={styles.experimental}>Experimental</span>}</span>
                  </Button>, id))}
                </div>
              </div>
              <div className={styles.sectionTitle}>Options</div>
              <div className={styles.options} role="group" aria-label="Options">
              {withTip("Vegetation", <DensitySlider label="Vegetation" focusKey="iel-forests" value={forests ? vegetationDensity : 0} disabled={busy || starting}
                onChange={value => trigger(group, "setVegetationDensity", value)} />, undefined, true)}
              {withTip("Rocks", <DensitySlider label="Rocks" focusKey="iel-rocks" value={rocks ? rockDensity : 0} disabled={busy || starting}
                onChange={value => trigger(group, "setRockDensity", value)} />, undefined, true)}
              {withTip("Rail", <Button variant="flat" className={styles.check} focusKey="iel-rail" role="checkbox" aria-checked={rail} disabled={busy || starting} onSelect={() => trigger(group, "setImportRail", !rail)}>
                <span className={styles.checkbox}>{rail ? "✓" : ""}</span><span>Rail</span>
              </Button>)}
              {withTip("HD Terrain", <Button variant="flat" className={styles.check} focusKey="iel-hd-terrain" role="checkbox" aria-checked={hdTerrain} disabled={busy || starting} onSelect={() => trigger(group, "setHdTerrain", !hdTerrain)}>
                <span className={styles.checkbox}>{hdTerrain ? "✓" : ""}</span><span>HD Terrain</span>
              </Button>)}
              {withTip("Air Routes", <Button variant="flat" className={styles.check} role="checkbox" aria-checked={airConnections} disabled={busy || starting} onSelect={() => {
                trigger(group, "setAirConnections", !airConnections);
              }}><span className={styles.checkbox}>{airConnections ? "✓" : ""}</span><span>Air Routes</span></Button>)}
              {withTip("Ship Routes", <Button variant="flat" className={styles.check} role="checkbox" aria-checked={seaConnections} disabled={busy || starting} onSelect={() => trigger(group, "setSeaConnections", !seaConnections)}>
                <span className={styles.checkbox}>{seaConnections ? "✓" : ""}</span><span>Ship Routes</span>
              </Button>)}
              {withTip("Surfaces", <Button variant="flat" className={styles.check} role="checkbox" aria-checked={surfaces} disabled={busy || starting} onSelect={() => trigger(group, "setGroundSurfaces", !surfaces)}><span className={styles.checkbox}>{surfaces ? "✓" : ""}</span><span>Surfaces</span></Button>)}
              {withTip("Diagnostics", <Button variant="flat" className={styles.check} role="checkbox" aria-checked={diagnostics} disabled={busy || starting} onSelect={() => trigger(group,"setDiagnostics",!diagnostics)}>
                <span className={styles.checkbox}>{diagnostics ? "✓" : ""}</span><span>Diagnostics</span>
              </Button>)}
              </div>
            </div>

            <div className={styles.actionSection}>
              <div className={styles.actions}>
                {!ready && !busy && (
                  <Button variant="flat" className={`${styles.actionButton} ${styles.primary}`} focusKey="iel-prepare" disabled={busy || starting || !hasLocation} onSelect={() => { if(actionGate.current || !hasLocation) return; actionGate.current = "prepare"; setStarting(true); trigger(group, "prepare"); }}>
                    Prepare import
                  </Button>
                )}
                {busy && !applying && (
                  <Button variant="flat" className={`${styles.actionButton} ${styles.utility}`} focusKey="iel-cancel" onSelect={() => trigger(group, "cancel")}>Cancel preparation</Button>
                )}
                {busy && applying && canResume && (
                  <Button variant="flat" className={styles.actionButton} focusKey="iel-resume" onSelect={() => trigger(group, "resumeGeneration")}>Resume</Button>
                )}
                {busy && applying && (
                  <Button variant="flat" className={`${styles.actionButton} ${styles.danger}`} focusKey="iel-stop" onSelect={() => trigger(group, "stopGeneration")}>Stop generation</Button>
                )}
                {ready && !busy && (
                  <Button variant="flat" focusKey="iel-replace" className={`${styles.actionButton} ${styles.danger}`} disabled={busy || starting} onSelect={() => setConfirming(true)}>
                    Replace current map…
                  </Button>
                )}
                <Button variant="flat" className={`${styles.actionButton} ${styles.utility}`} focusKey="iel-log" onSelect={() => setShowLog(value => !value)}>
                  {showLog ? "Hide log" : "View log"}
                </Button>
              </div>
              {!ready && !busy && !hasLocation && <div className={styles.helper} role="note">Select a location on the map to prepare an import.</div>}
              {showLog && <Scrollable className={styles.logViewport} vertical horizontal={false} trackVisibility="always">
                <pre className={styles.log}>{log || "No log entries yet."}</pre>
              </Scrollable>}
              {/* Scrolls without a visible bar: Gameface draws none for overflow. */}
              <div className={styles.feedback}>
                <div className={styles.status} role="status">{status}</div>
                {summary && <div className={styles.summary}>{summary}</div>}
                {error && <div className={styles.error} role="alert">{error}</div>}
              </div>
            </div>

            </Scrollable>
          </Panel>
        )}
      </div>
      {/* Keep viewport overlays outside Panel's native clipping/containing block. */}
      {panelOpen && hoverTip && !confirming && !serviceOpen && !pickerOpen &&
        <div className={styles.tip} role="tooltip" style={{ top: `${hoverTip.top}px`, left: `${hoverTip.left}px` }}>
          <div className={styles.tipTitle}>{hoverTip.label}</div><div className={styles.tipText}>{tips[hoverTip.label]}</div>
        </div>}
      {panelOpen && serviceOpen && <div className={styles.confirmBackdrop}>
        <Panel className={styles.servicePanel} role="dialog" aria-modal="true" aria-labelledby="iel-service-title"
          header={<span id="iel-service-title" className={styles.brandText}>Configure Data Source</span>}
          theme={{ header: styles.panelHeader, titleBar: styles.panelTitleBar, title: styles.panelTitle, iconSpace: styles.panelIconSpace, closeButton: styles.panelCloseButton, content: styles.panelContent }} onClose={() => setServiceOpen(false)}>
          <div className={styles.section}>
          <Scrollable vertical horizontal={false} trackVisibility="scrollable" className={styles.confirmBody}>
            <p>Sign up for free at nextgis.com. Go to Profile → Software and services → Services. Find Overpass API, create an API key if needed, and copy your service URL.</p>
            <p>Without a service URL, public backup downloads could take hours. You can still continue using backups.</p>
            <label className={styles.field}><span>NextGIS Overpass service URL</span><input type="password" autoComplete="off" value={serviceDraft} disabled={busy} placeholder="Paste your service URL" onChange={event => setServiceDraft(event.target.value)} /></label>
            <p>The URL contains your personal key. It is saved locally, not included in import diagnostics. Leave it empty to use public backups.</p>
            <div role="status">{serviceStatus}</div>
          </Scrollable>
          <div className={`${styles.actions} ${styles.confirmFooter}`}>
            <Button variant="flat" className={`${styles.actionButton} ${styles.utility}`} onSelect={() => setServiceOpen(false)}>{serviceUrl ? "Cancel" : "Continue with backups"}</Button>
            <Button variant="flat" className={`${styles.actionButton} ${styles.primary}`} disabled={busy} onSelect={() => trigger(group, "saveServiceUrl", serviceDraft)}>Save</Button>
          </div>
          </div>
        </Panel>
      </div>}
      {panelOpen && confirming && (
        <div className={styles.confirmBackdrop}>
          <div className={styles.confirmBox} role="dialog" aria-modal="true" aria-labelledby="iel-confirm-title">
            <h3 id="iel-confirm-title">Replace current map contents?</h3>
            <Scrollable vertical horizontal={false} className={styles.confirmBody}>
              <p>This replaces terrain and removes existing roads, rail, other networks, and water sources from the open map—even when new roads or rail are disabled. Optional forest trees are added where supported. The map file and name are not deleted.</p>
              <p><strong>Save a backup first.</strong></p>
              <p>The game may appear to freeze for a few seconds at a time while replacing the map. Please wait for it to finish.</p>
            </Scrollable>
            <div className={`${styles.actions} ${styles.confirmFooter}`}>
              <Button variant="flat" focusKey="iel-confirm-cancel" className={`${styles.actionButton} ${styles.utility}`} onSelect={() => setConfirming(false)}>Cancel</Button>
              <Button variant="flat" focusKey="iel-confirm-replace" disabled={busy || starting} className={`${styles.actionButton} ${styles.danger}`} onSelect={() => { if(actionGate.current) return; actionGate.current = "confirm"; setStarting(true); setConfirming(false); trigger(group, "confirm"); }}>
                Replace map
              </Button>
            </div>
          </div>
        </div>
      )}
      {panelOpen && pickerOpen && <LocationPicker mapExt={mapExt} loadedSize={loadedMapSize} initial={hasLocation ? { latitude: Number(latitude), longitude: Number(longitude), mapSize } : null} onClose={() => setPickerOpen(false)} onCommit={(value: Location) => {
        trigger(group, "commitLocation", JSON.stringify(value)); setPickerOpen(false);
      }} />}
    </Portal>
  );
};
