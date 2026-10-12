import React, { useEffect, useRef, useState } from "react";
import { createPortal } from "react-dom";
import { bindValue, trigger, useValue } from "cs2/api";
import * as UI from "cs2/ui";
import { Button, Dropdown, DropdownToggle, Panel, Portal, Scrollable, Tooltip, DropdownItemProps } from "cs2/ui";
import styles from "./import-earth-location.module.scss";
import { Location, LocationPicker } from "./location-picker";
import importIcon from "../import-earth-icon-generated.png";

const group = "ImportEarthLocation";
// The stock declarations also use DropdownItem as a data-interface name.
// The installed cs2/ui runtime exports the component under this same name.
const DropdownItem = (UI as unknown as { DropdownItem: React.ComponentType<React.PropsWithChildren<DropdownItemProps<string>>> }).DropdownItem;
const isEditor$ = bindValue<boolean>(group, "isEditor", false);
const mapExt$ = bindValue<boolean>(group, "mapExtDetected", false);
const mapSize$ = bindValue<string>(group, "mapSize", "vanilla");
const panelOpen$ = bindValue<boolean>(group, "panelOpen", false);
export const ImportLauncher = ({ theme, className }: { theme?: any; className?: string }) => {
  const editor = useValue(isEditor$);
  const open = useValue(panelOpen$);
  if (!editor) return null;
  return <Tooltip tooltip="Import Earth" direction="up"><Button variant="icon"
    focusKey="import-earth-location-launcher" className={className} theme={theme}
    style={{ width: "44rem", height: "44rem", minWidth: "44rem", maxWidth: "44rem", minHeight: "44rem", maxHeight: "44rem", overflow: "hidden", flexShrink: 0 }}
    selected={open}
    onSelect={() => trigger(group, "setPanelOpen", !open)} aria-label="Import Earth"><img className={styles.launcherIcon} src={importIcon} alt="" width={32} height={32} style={{ width: "32rem", height: "32rem", maxWidth: "32rem", maxHeight: "32rem", flexShrink: 0, objectFit: "contain" }} /></Button></Tooltip>;
};
const AnchoredLauncher = () => {
  const editor = useValue(isEditor$);
  const [host, setHost] = useState<HTMLElement | null>(null);
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
      // Gameface cannot resolve CSS var() fallbacks reliably. Keep both the
      // button and image explicitly bounded instead of inheriting auto sizes.
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
  return editor && host ? createPortal(<ImportLauncher className={styles.launcherButton} />, host) : null;
};
const busy$ = bindValue<boolean>(group, "busy", false);
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
  const ready = useValue(ready$);
  const applying = useValue(stage$) === 6;
  const status = useValue(status$);
  const error = useValue(error$);
  const summary = useValue(summary$);
  const log = useValue(log$);
  const [latitude, setLatitude] = useState("");
  const [longitude, setLongitude] = useState("");
  const mapSize = useValue(mapSize$);
  const rawRoadLevel = useValue(roads$);
  const roadGroups = rawRoadLevel.split("|").filter(id => ["highways", "routes", "streets"].includes(id));
  // Toggle atomically in the backend; rapid clicks on different checkboxes
  // must not overwrite a previous selection before its binding update arrives.
  const toggleRoadGroup = (id: string) => trigger(group, "toggleRoadGroup", id);
  const rail = useValue(rail$);
  const forests = useValue(forests$);
  const mapSession = useValue(session$);
  const actionRevision = useValue(actionRevision$);
  const actionGate = useRef<string | null>(null);
  const [starting, setStarting] = useState(false);
  const surfaces = useValue(surfaces$);
  const airConnections = useValue(airConnections$);
  const seaConnections = useValue(seaConnections$);
  const diagnostics = useValue(diagnostics$);
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
    try { const value = JSON.parse(locationText); if (typeof value.latitude === "number") { setLatitude(String(value.latitude)); setLongitude(String(value.longitude)); } } catch { }
  }, [locationText]);

  const closePanel = () => {
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

  const updateLatitude = (value: string) => { setLatitude(value); trigger(group, "setLatitude", value); };
  const updateLongitude = (value: string) => { setLongitude(value); trigger(group, "setLongitude", value); };
  const updateMapSize = (value: string) => trigger(group, "setMapSize", value);

  return (
    <Portal>
      <div className={styles.host}>
        <AnchoredLauncher />

        {panelOpen && (
          <Panel header="Import Earth" className={styles.panel} onClose={closePanel}>
            <Scrollable vertical className={styles.panelBody}>
            <div className={styles.section}>
              <div className={styles.notice}>Editor-only full-map import. Roads and rail are independently selectable.</div>
              {!serviceUrl && <div className={styles.notice}>No personal data service configured. Public backup downloads could take hours. Set up a free NextGIS service for faster access.</div>}
              <Button variant="flat" className={`${styles.actionButton} ${styles.utility}`} disabled={busy || starting} onSelect={() => { setServiceDraft(serviceUrl); setServiceOpen(true); }}>Configure data service…</Button>
              <Button variant="flat" className={`${styles.actionButton} ${styles.secondary}`} disabled={busy || starting} onSelect={() => setPickerOpen(true)}>Select location on map…</Button>
              <label className={styles.field}>
                <span>Center latitude</span>
                <input value={latitude} disabled={busy} placeholder="43.720000" onChange={event => updateLatitude(event.target.value)} />
              </label>
              <label className={styles.field}>
                <span>Center longitude</span>
                <input value={longitude} disabled={busy} placeholder="15.820000" onChange={event => updateLongitude(event.target.value)} />
              </label>
              <div className={styles.field}>
                <span>Map size</span>
                <Tooltip tooltip="Other map sizes are not available in the alpha version." direction="up"><div>
                <Dropdown theme={{ dropdownToggle: styles.sizeToggle, label: styles.sizeLabel, dropdownPopup: styles.sizePopup, dropdownMenu: styles.sizeMenu, dropdownItem: styles.sizeItem }} content={<>
                  <DropdownItem value="vanilla" disabled selected={mapSize === "vanilla"} onChange={updateMapSize}>14.336 km (Vanilla)</DropdownItem>
                  {mapExt && <DropdownItem value="mapext28" disabled selected={mapSize === "mapext28"} onChange={updateMapSize}>28.672 km (MapExt)</DropdownItem>}
                  {mapExt && <DropdownItem value="mapext57" disabled selected={mapSize === "mapext57"} onChange={updateMapSize}>57.344 km (MapExt)</DropdownItem>}
                </>}>
                  <DropdownToggle disabled>{mapSize === "mapext57" ? "57.344 km (MapExt)" : mapSize === "mapext28" ? "28.672 km (MapExt)" : "14.336 km (Vanilla)"}</DropdownToggle>
                </Dropdown>
                </div></Tooltip>
              </div>
              <div className={styles.field} role="group" aria-label="Road groups">
                <span>Road groups</span>
                {[["highways", "Highways"], ["routes", "Main Roads"], ["streets", "Streets — experimental"]].map(([id,label]) =>
                  <Button key={id} variant="flat" className={styles.check} focusKey={`iel-roads-${id}`} role="checkbox" aria-checked={roadGroups.includes(id)} selected={roadGroups.includes(id)} disabled={busy || starting} onSelect={() => toggleRoadGroup(id)}>
                    <span className={styles.checkbox}>{roadGroups.includes(id) ? "✓" : ""}</span><span>{label}</span>
                  </Button>)}
              </div>
              <Button variant="flat" className={styles.check} focusKey="iel-rail" role="checkbox" aria-checked={rail} selected={rail} disabled={busy || starting} onSelect={() => trigger(group, "setImportRail", !rail)}>
                <span className={styles.checkbox}>{rail ? "✓" : ""}</span><span>Import rail</span>
              </Button>
              <Button variant="flat" className={styles.check} focusKey="iel-forests" role="checkbox" aria-checked={forests} selected={forests} disabled={busy || starting} onSelect={() => trigger(group, "setImportForests", !forests)}>
                <span className={styles.checkbox}>{forests ? "✓" : ""}</span><span>Import forests</span>
              </Button>
              <Button variant="flat" className={styles.check} role="checkbox" aria-checked={surfaces} selected={surfaces} disabled={busy} onSelect={() => {
                trigger(group, "setGroundSurfaces", !surfaces);
              }}><span className={styles.checkbox}>{surfaces ? "✓" : ""}</span><span>Apply real-world ground surfaces</span></Button>
              <Button variant="flat" className={styles.check} role="checkbox" aria-checked={airConnections} selected={airConnections} disabled={busy} onSelect={() => {
                trigger(group, "setAirConnections", !airConnections);
              }}><span className={styles.checkbox}>{airConnections ? "✓" : ""}</span><span>Create air connections</span></Button>
              <Button variant="flat" className={styles.check} role="checkbox" aria-checked={seaConnections} selected={seaConnections} disabled={busy} onSelect={() => trigger(group, "setSeaConnections", !seaConnections)}>
                <span className={styles.checkbox}>{seaConnections ? "✓" : ""}</span><span>Create water connections</span>
              </Button>
              <Button variant="flat" className={styles.check} role="checkbox" aria-checked={diagnostics} selected={diagnostics} disabled={busy || starting} onSelect={() => trigger(group,"setDiagnostics",!diagnostics)}>
                <span className={styles.checkbox}>{diagnostics ? "✓" : ""}</span><span>Enable diagnostics</span>
              </Button>
              <div className={styles.helper}>Detailed generation logs and diagnostic files. Off by default; status updates and important messages remain available.</div>
            </div>

            <div className={styles.section}>
              <div className={styles.status} role="status">{status}</div>
              {summary && <Scrollable vertical className={styles.summary}>{summary}</Scrollable>}
              {error && <div className={styles.error}>{error}</div>}
              <div className={styles.actions}>
                {!ready && (
                  <Button variant="flat" className={`${styles.actionButton} ${styles.primary}`} focusKey="iel-prepare" disabled={busy || starting} onSelect={() => { if(actionGate.current) return; actionGate.current = "prepare"; setStarting(true); trigger(group, "prepare"); }}>
                    {busy ? "Preparing…" : "Prepare import"}
                  </Button>
                )}
                {busy && !applying && (
                  <Button variant="flat" className={`${styles.actionButton} ${styles.utility}`} focusKey="iel-cancel" onSelect={() => trigger(group, "cancel")}>Cancel preparation</Button>
                )}
                {busy && applying && (
                  <Button variant="flat" className={`${styles.actionButton} ${styles.danger}`} focusKey="iel-stop" onSelect={() => trigger(group, "stopGeneration")}>Stop generation</Button>
                )}
                {ready && (
                  <Button variant="flat" focusKey="iel-replace" className={`${styles.actionButton} ${styles.danger}`} disabled={busy || starting} onSelect={() => setConfirming(true)}>
                    Replace current map…
                  </Button>
                )}
                <Button variant="flat" className={`${styles.actionButton} ${styles.utility}`} focusKey="iel-log" onSelect={() => setShowLog(value => !value)}>
                  {showLog ? "Hide log" : "View log"}
                </Button>
              </div>
              {showLog && <Scrollable className={styles.logViewport} vertical horizontal={false} trackVisibility="always">
                <pre className={styles.log}>{log || "No log entries yet."}</pre>
              </Scrollable>}
            </div>

            </Scrollable>
          </Panel>
        )}
      </div>
      {/* Keep viewport overlays outside Panel's native clipping/containing block. */}
      {panelOpen && serviceOpen && <div className={styles.confirmBackdrop}>
        <div className={styles.confirmBox} role="dialog" aria-modal="true" aria-labelledby="iel-service-title">
          <h3 id="iel-service-title">Set up your data service</h3>
          <Scrollable vertical horizontal={false} className={styles.confirmBody}>
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
      </div>}
      {panelOpen && confirming && (
        <div className={styles.confirmBackdrop}>
          <div className={styles.confirmBox} role="dialog" aria-modal="true" aria-labelledby="iel-confirm-title">
            <h3 id="iel-confirm-title">Replace current map contents?</h3>
            <Scrollable vertical horizontal={false} className={styles.confirmBody}>
              <p>This replaces terrain and removes existing roads, rail, other networks, and water sources from the open map—even when new roads or rail are disabled. Optional forest trees are added where supported. The map file and name are not deleted.</p>
              <p><strong>Save a backup first.</strong></p>
              <p>The game may appear to freeze for up to 5 minutes while replacing the map, depending on its complexity. Please wait for it to finish.</p>
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
      {panelOpen && pickerOpen && <LocationPicker mapExt={mapExt} initial={{ latitude: Number(latitude) || 0, longitude: Number(longitude) || 0, mapSize }} onClose={() => setPickerOpen(false)} onCommit={(value: Location) => {
        trigger(group, "commitLocation", JSON.stringify(value)); setPickerOpen(false);
      }} />}
    </Portal>
  );
};
