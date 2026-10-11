import React, { useEffect, useRef, useState } from "react";
import { bindValue, trigger, useValue } from "cs2/api";
import { Button, Panel, Scrollable } from "cs2/ui";
import styles from "./import-earth-location.module.scss";

const group = "ImportEarthLocation";
const places$ = bindValue<string>(group, "placeResults", "{}");
const tiles$ = bindValue<string>(group, "displayTiles", "{}");
export type Location = { latitude: number; longitude: number; mapSize: string };
export function parseCoordinate(text: string, limit: number): number | null {
  if (!text.trim() || !/^[+-]?(?:\d+(?:\.\d*)?|\.\d+)$/.test(text.trim())) return null;
  const value = Number(text);
  return isFinite(value) && Math.abs(value) <= limit ? value : null;
}
export const mapWidth = (id: string) => id === "mapext57" ? 57344 : id === "mapext28" ? 28672 : 14336;
export function project(lat: number, lon: number, zoom: number) {
  const size = 256 * Math.pow(2, zoom);
  const sin = Math.sin(Math.max(-85.05112878, Math.min(85.05112878, lat)) * Math.PI / 180);
  return { x: (lon + 180) / 360 * size, y: (0.5 - Math.log((1 + sin) / (1 - sin)) / (4 * Math.PI)) * size };
}
export function unproject(x: number, y: number, zoom: number) {
  const size = 256 * Math.pow(2, zoom);
  return { latitude: Math.atan(Math.sinh(Math.PI * (1 - 2 * Math.max(0, Math.min(size, y)) / size))) * 180 / Math.PI,
    longitude: ((x / size * 360) % 360 + 360) % 360 - 180 };
}
const json = (text: string) => { try { return JSON.parse(text); } catch { return {}; } };
// Unique across picker reopenings: an old response must never match a new draft.
let requestSerial = 0;
const nextRevision = () => ++requestSerial;

export const LocationPicker = ({ initial, mapExt, onCommit, onClose }: { initial: Location; mapExt: boolean; onCommit: (value: Location) => void; onClose: () => void }) => {
  const [center, setCenter] = useState({ latitude: initial.latitude, longitude: initial.longitude });
  const [latitude, setLatitude] = useState(String(initial.latitude));
  const [longitude, setLongitude] = useState(String(initial.longitude));
  const [size, setSize] = useState(mapExt ? initial.mapSize : "vanilla");
  const [zoom, setZoom] = useState(9);
  const [query, setQuery] = useState("");
  const [error, setError] = useState("");
  const [searching, setSearching] = useState(false);
  const [searchRevision, setSearchRevision] = useState(0);
  const [tileRevision, setTileRevision] = useState(0);
  const committed = useRef(false);
  const [committing, setCommitting] = useState(false);
  const drag = useRef<{ x: number; y: number; worldX: number; worldY: number; moved: boolean } | null>(null);
  const rawPlaces = json(useValue(places$));
  const rawTiles = json(useValue(tiles$));
  const places = rawPlaces.revision === searchRevision ? rawPlaces : {};
  const tiles = rawTiles.revision === tileRevision ? rawTiles : {};
  const world = project(center.latitude, center.longitude, zoom);
  const viewport = { width: 400, height: 260 };
  const tileCount = Math.pow(2, zoom);
  const visible: { key: string; left: number; top: number }[] = [];
  for (let y = Math.floor((world.y - 130) / 256); y <= Math.floor((world.y + 130) / 256); y++) {
    if (y < 0 || y >= tileCount) continue;
    for (let x = Math.floor((world.x - 200) / 256); x <= Math.floor((world.x + 200) / 256); x++) {
      visible.push({ key: `${zoom}/${(x % tileCount + tileCount) % tileCount}/${y}`, left: x * 256 - world.x + 200, top: y * 256 - world.y + 130 });
    }
  }
  const keys = visible.map(t => t.key).join(",");
  useEffect(() => {
    if (Math.abs(center.latitude) > 85.05112878) return;
    // Debounce drags. Only the current viewport is requested, never an import region.
    const timer = window.setTimeout(() => {
      const revision = nextRevision();
      setTileRevision(revision);
      trigger(group, "previewTiles", JSON.stringify({ revision, keys: keys.split(",") }));
    }, 300);
    return () => window.clearTimeout(timer);
  }, [keys, center.latitude]);
  useEffect(() => () => trigger(group, "cancelLocationServices"), []);
  useEffect(() => { if (rawPlaces.revision === searchRevision) setSearching(false); }, [rawPlaces.revision, searchRevision]);
  useEffect(() => { if (!mapExt) setSize("vanilla"); }, [mapExt]);
  const recenter = (value: { latitude: number; longitude: number }) => {
    setCenter(value); setLatitude(value.latitude.toFixed(6)); setLongitude(value.longitude.toFixed(6)); setError("");
  };
  const manual = () => {
    const lat = parseCoordinate(latitude, 90), lon = parseCoordinate(longitude, 180);
    if (lat === null || lon === null) { setError("Enter valid latitude (−90 to 90) and longitude (−180 to 180)."); return null; }
    const value = { latitude: lat, longitude: lon };
    setCenter(value); setError(""); return value;
  };
  // Identical equirectangular footprint calculation to GeoMath.BoundsFromCenter.
  const halfLat = mapWidth(size) / 2 / 6378137 * 180 / Math.PI;
  const halfLon = halfLat / Math.max(0.01, Math.cos(center.latitude * Math.PI / 180));
  const nw = project(center.latitude + halfLat, center.longitude - halfLon, zoom);
  const se = project(center.latitude - halfLat, center.longitude + halfLon, zoom);
  const mapUnavailable = Math.abs(center.latitude) > 85.05112878 || tiles.error;
  const search = () => {
    if (searching || query.trim().length < 2) return;
    const revision = nextRevision(); setSearchRevision(revision); setSearching(true);
    trigger(group, "searchPlace", JSON.stringify({ revision, query }));
  };
  return <Panel header="Select Earth location" className={styles.picker} onClose={onClose}>
    <div className={styles.section}>
      <div className={styles.search}><input aria-label="City or place" placeholder="City or place" value={query} onChange={e => {
        setQuery(e.target.value); setSearchRevision(nextRevision()); setSearching(false);
      }} onKeyDown={e => {
        if (e.key !== "Enter" || e.nativeEvent.isComposing || e.repeat) return;
        e.preventDefault(); e.stopPropagation(); search();
      }} /><Button variant="flat" className={`${styles.actionButton} ${styles.secondary}`} disabled={searching || query.trim().length < 2} onSelect={search}>Search</Button></div>
      {places.message && <div className={styles.notice}>{places.message}</div>}
      {places.results?.length > 0 && <Scrollable vertical className={styles.results}>{places.results.map((p: any, i: number) =>
        <Button key={i} variant="flat" className={`${styles.actionButton} ${styles.secondary}`} onSelect={() => recenter(p)}>{p.label}</Button>)}</Scrollable>}
      <div className={styles.map} onWheel={e => { e.preventDefault(); setZoom(z => Math.max(1, Math.min(16, z + (e.deltaY < 0 ? 1 : -1)))); }}
        onMouseDown={e => { if (e.button !== 0) return; e.preventDefault(); drag.current = { x: e.clientX, y: e.clientY, worldX: world.x, worldY: world.y, moved: false }; }}
        onMouseMove={e => {
          const d = drag.current; if (!d) return;
          const rect = e.currentTarget.getBoundingClientRect();
          const dx = (e.clientX - d.x) * viewport.width / rect.width, dy = (e.clientY - d.y) * viewport.height / rect.height;
          if (Math.abs(dx) + Math.abs(dy) > 4) d.moved = true;
          if (d.moved) recenter(unproject(d.worldX - dx, d.worldY - dy, zoom));
        }} onMouseLeave={() => { drag.current = null; }} onMouseUp={e => {
          const d = drag.current; drag.current = null; if (!d || d.moved) return;
          const rect = e.currentTarget.getBoundingClientRect();
          recenter(unproject(world.x + (e.clientX - rect.left) / rect.width * 400 - 200, world.y + (e.clientY - rect.top) / rect.height * 260 - 130, zoom));
        }}>
        {!mapUnavailable && visible.map(t => tiles.tiles?.[t.key] && <img key={t.key} draggable={false} src={tiles.tiles[t.key]} style={{ left: `${t.left}rem`, top: `${t.top}rem` }} />)}
        <div className={styles.footprint} style={{ left: `${nw.x - world.x + 200}rem`, top: `${nw.y - world.y + 130}rem`, width: `${se.x - nw.x}rem`, height: `${se.y - nw.y}rem` }} />
        <div className={styles.reticle}>+</div>
        {mapUnavailable && <div className={styles.mapError}>Map preview unavailable. Enter coordinates manually.</div>}
        <div className={styles.attribution}>© OpenStreetMap contributors</div>
      </div>
      <div className={styles.actions}><Button variant="flat" className={`${styles.actionButton} ${styles.secondary}`} onSelect={() => setZoom(z => Math.min(16, z + 1))}>Zoom +</Button><Button variant="flat" className={`${styles.actionButton} ${styles.secondary}`} onSelect={() => setZoom(z => Math.max(1, z - 1))}>Zoom −</Button></div>
      <div className={styles.coordinates}>
        <label className={styles.field}>Latitude<input value={latitude} onChange={e => setLatitude(e.target.value)} onBlur={manual} /></label>
        <label className={styles.field}>Longitude<input value={longitude} onChange={e => setLongitude(e.target.value)} onBlur={manual} /></label>
      </div>
      <div className={styles.actions}>{["vanilla", ...(mapExt ? ["mapext28", "mapext57"] : [])].map(id =>
        <Button key={id} variant="flat" className={styles.configChoice} selected={id === size} onSelect={() => setSize(id)}>{mapWidth(id) / 1000} km</Button>)}</div>
      <div className={styles.notice}>Search: Photon / OpenStreetMap; fallback Open-Meteo / GeoNames. Terrain import currently supports latitudes ±85° and does not cross the date line.</div>
      {error && <div className={styles.error}>{error}</div>}
      <div className={styles.actions}><Button variant="flat" className={`${styles.actionButton} ${styles.primary}`} disabled={committing} onSelect={() => {
        if(committed.current) return;
        const value = manual(); if(!value) return;
        committed.current = true; setCommitting(true);
        try { onCommit({ ...value, mapSize: size }); }
        catch { committed.current = false; setCommitting(false); setError("Location could not be accepted. Please try again."); }
      }}>Use This Location</Button><Button variant="flat" className={`${styles.actionButton} ${styles.utility}`} onSelect={onClose}>Cancel</Button></div>
    </div>
  </Panel>;
};
