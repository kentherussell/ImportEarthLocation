import React, { useEffect, useRef, useState } from "react";
import { bindValue, trigger, useValue } from "cs2/api";
import * as UI from "cs2/ui";
import { Button, Dropdown, DropdownToggle, Panel, Portal, Scrollable, Tooltip, DropdownItemProps } from "cs2/ui";
import styles from "./import-earth-location.module.scss";
import { Location, LocationPicker } from "./location-picker";

const group = "ImportEarthLocation";
// The stock declarations also use DropdownItem as a data-interface name.
// The installed cs2/ui runtime exports the component under this same name.
const DropdownItem = (UI as unknown as { DropdownItem: React.ComponentType<React.PropsWithChildren<DropdownItemProps<string>>> }).DropdownItem;
const isEditor$ = bindValue<boolean>(group, "isEditor", false);
const mapExt$ = bindValue<boolean>(group, "mapExtDetected", false);
const mapSize$ = bindValue<string>(group, "mapSize", "vanilla");
const panelOpen$ = bindValue<boolean>(group, "panelOpen", false);
const busy$ = bindValue<boolean>(group, "busy", false);
const ready$ = bindValue<boolean>(group, "ready", false);
const stage$ = bindValue<number>(group, "stage", 0);
const status$ = bindValue<string>(group, "status", "");
const error$ = bindValue<string>(group, "error", "");
const summary$ = bindValue<string>(group, "summary", "");
const log$ = bindValue<string>(group, "log", "");
const location$ = bindValue<string>(group, "location", "{}");
const roads$ = bindValue<string>(group, "roadLevel", "primary");
const rail$ = bindValue<boolean>(group, "importRail", true);
const session$ = bindValue<string>(group, "mapSession", "");
const actionRevision$ = bindValue<string>(group, "actionRevision", "0");

class ImportPanelBoundary extends React.Component<React.PropsWithChildren<{}>, { failed: boolean }> {
  state = { failed: false };

  static getDerivedStateFromError() { return { failed: true }; }

  componentDidCatch(error: Error) {
    console.error("Import Earth Location panel failed:", error.message);
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
  const roadLevel = ["none", "highways", "primary", "secondary"].includes(rawRoadLevel) ? rawRoadLevel : "primary";
  const rail = useValue(rail$);
  const mapSession = useValue(session$);
  const actionRevision = useValue(actionRevision$);
  const actionGate = useRef<string | null>(null);
  const [starting, setStarting] = useState(false);
  const [surfaces, setSurfaces] = useState(false);
  const [airConnections, setAirConnections] = useState(false);
  const [confirming, setConfirming] = useState(false);
  const [showLog, setShowLog] = useState(false);
  const [pickerOpen, setPickerOpen] = useState(false);
  const locationText = useValue(location$);
  useEffect(() => { actionGate.current = null; setStarting(false); }, [busy, ready, status, error, actionRevision]);
  useEffect(() => { setConfirming(false); setPickerOpen(false); setShowLog(false); actionGate.current = null; setStarting(false); }, [mapSession]);
  useEffect(() => { if (!ready) setConfirming(false); }, [ready]);
  useEffect(() => {
    try { const value = JSON.parse(locationText); if (typeof value.latitude === "number") { setLatitude(String(value.latitude)); setLongitude(String(value.longitude)); } } catch { }
  }, [locationText]);

  const closePanel = () => {
    setConfirming(false);
    setPickerOpen(false);
    trigger(group, "setPanelOpen", false);
  };

  useEffect(() => {
    if (!panelOpen) { setConfirming(false); return; }
    const onKeyDown = (event: KeyboardEvent) => {
      if (event.key !== "Escape" && event.keyCode !== 27) return;
      event.preventDefault();
      event.stopPropagation();
      if (pickerOpen) setPickerOpen(false);
      else if (confirming) setConfirming(false);
      else closePanel();
    };
    window.addEventListener("keydown", onKeyDown, true);
    return () => window.removeEventListener("keydown", onKeyDown, true);
  }, [panelOpen, confirming, pickerOpen]);

  if (!isEditor) return null;

  const updateLatitude = (value: string) => { setLatitude(value); trigger(group, "setLatitude", value); };
  const updateLongitude = (value: string) => { setLongitude(value); trigger(group, "setLongitude", value); };
  const updateMapSize = (value: string) => trigger(group, "setMapSize", value);

  return (
    <Portal>
      <div className={styles.host}>
        <Tooltip tooltip="Import Earth Location" direction="up">
          <Button
            variant="icon"
            focusKey="import-earth-location-launcher"
            className={styles.launcher}
            src="Media/Editor/AssetImport.svg"
            tinted
            selected={panelOpen}
            onSelect={() => trigger(group, "setPanelOpen", !panelOpen)}
            aria-label="Import Earth Location"
          />
        </Tooltip>

        {panelOpen && (
          <Panel header="Import Earth Location" className={styles.panel} onClose={closePanel}>
            <Scrollable vertical className={styles.panelBody}>
            <div className={styles.section}>
              <div className={styles.notice}>Editor-only full-map import. Roads and rail are independently selectable.</div>
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
                <Dropdown theme={{ dropdownToggle: styles.sizeToggle, label: styles.sizeLabel, dropdownPopup: styles.sizePopup, dropdownMenu: styles.sizeMenu, dropdownItem: styles.sizeItem }} content={<>
                  <DropdownItem value="vanilla" disabled={busy} selected={mapSize === "vanilla"} onChange={updateMapSize}>14.336 km (Vanilla)</DropdownItem>
                  {mapExt && <DropdownItem value="mapext28" disabled={busy} selected={mapSize === "mapext28"} onChange={updateMapSize}>28.672 km (MapExt)</DropdownItem>}
                  {mapExt && <DropdownItem value="mapext57" disabled={busy} selected={mapSize === "mapext57"} onChange={updateMapSize}>57.344 km (MapExt)</DropdownItem>}
                </>}>
                  <DropdownToggle disabled={busy}>{mapSize === "mapext57" ? "57.344 km (MapExt)" : mapSize === "mapext28" ? "28.672 km (MapExt)" : "14.336 km (Vanilla)"}</DropdownToggle>
                </Dropdown>
              </div>
              <div className={styles.field} role="radiogroup" aria-label="Road import level">
                <span>Road import level</span>
                {[["none", "None"], ["highways", "Highways only"], ["primary", "Primary roads & above"], ["secondary", "Secondary roads & above"]].map(([id,label]) =>
                  <Button key={id} variant="flat" className={styles.configChoice} focusKey={`iel-roads-${id}`} role="radio" aria-checked={roadLevel === id} selected={roadLevel === id} disabled={busy || starting} onSelect={() => trigger(group, "setRoadLevel", id)}>
                    <span className={styles.checkbox}>{roadLevel === id ? "●" : "○"}</span><span>{label}</span>
                  </Button>)}
                <span className={styles.helper}>Primary includes highways. Secondary includes primary and highways.</span>
              </div>
              <Button variant="flat" className={styles.check} focusKey="iel-rail" role="checkbox" aria-checked={rail} selected={rail} disabled={busy || starting} onSelect={() => trigger(group, "setImportRail", !rail)}>
                <span className={styles.checkbox}>{rail ? "✓" : ""}</span><span>Import rail</span>
              </Button>
              <Button variant="flat" className={styles.check} role="checkbox" aria-checked={surfaces} selected={surfaces} disabled={busy} onSelect={() => {
                setSurfaces(!surfaces); trigger(group, "setGroundSurfaces", !surfaces);
              }}><span className={styles.checkbox}>{surfaces ? "✓" : ""}</span><span>Apply real-world ground surfaces</span></Button>
              <Button variant="flat" className={styles.check} role="checkbox" aria-checked={airConnections} selected={airConnections} disabled={busy} onSelect={() => {
                setAirConnections(!airConnections); trigger(group, "setAirConnections", !airConnections);
              }}><span className={styles.checkbox}>{airConnections ? "✓" : ""}</span><span>Create air connections to nearby major cities</span></Button>
            </div>

            <div className={styles.section}>
              <div className={styles.status}>{status}</div>
              {summary && <Scrollable vertical className={styles.summary}>{summary}</Scrollable>}
              {error && <div className={styles.error}>{error}</div>}
              <div className={styles.actions}>
                {!ready && (
                  <Button variant="flat" className={`${styles.actionButton} ${styles.primary}`} focusKey="iel-prepare" disabled={busy || starting} onSelect={() => { if(actionGate.current) return; actionGate.current = "prepare"; setStarting(true); setShowLog(true); trigger(group, "prepare"); }}>
                    {busy ? "Preparing…" : "Prepare import"}
                  </Button>
                )}
                {busy && !applying && (
                  <Button variant="flat" className={`${styles.actionButton} ${styles.utility}`} focusKey="iel-cancel" onSelect={() => trigger(group, "cancel")}>Cancel preparation</Button>
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
            {confirming && (
              <div className={styles.confirmBackdrop}>
                <div className={styles.confirmBox}>
                  <h3>Replace current map contents?</h3>
                  <Scrollable vertical horizontal={false} className={styles.confirmBody}>
                    <p>This replaces terrain and removes existing roads, rail, other networks, and water sources from the open map—even when new roads or rail are disabled. Optional forest trees are added where supported. The map file and name are not deleted.</p>
                    <p><strong>Save a backup first.</strong> Once application starts it cannot be cancelled.</p>
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
          </Panel>
        )}
      </div>
      {panelOpen && pickerOpen && <LocationPicker mapExt={mapExt} initial={{ latitude: Number(latitude) || 0, longitude: Number(longitude) || 0, mapSize }} onClose={() => setPickerOpen(false)} onCommit={(value: Location) => {
        trigger(group, "commitLocation", JSON.stringify(value)); setPickerOpen(false);
      }} />}
    </Portal>
  );
};
