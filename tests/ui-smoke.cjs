const fs = require('fs');
const vm = require('vm');
const assert = require('assert');
const root = require('path').resolve(__dirname, '..', 'ImportEarthLocation.UI');
const ts = require(root + '/node_modules/typescript');
const React = require(root + '/node_modules/react');
const bindings = { isEditor: false, mapSession: 'standalone-session', panelOpen: false, busy: false, ready: false, stage: 0, mapExtDetected: false };
const events = [];
const hooks = [];
let hookIndex = 0;
let cleanup = [];
let dirty = false;
let acknowledgeActions = true;
const handlers = new Map();
const react = { ...React,
  useState(initial) { const i = hookIndex++; if (!(i in hooks)) hooks[i] = initial; return [hooks[i], value => { const next = typeof value === 'function' ? value(hooks[i]) : value; if (next !== hooks[i]) {hooks[i] = next; dirty = true;} }]; },
  useRef(initial) { const i = hookIndex++; if (!(i in hooks)) hooks[i] = {current:initial}; return hooks[i]; },
  useEffect(effect, deps) { const i = hookIndex++; const old = hooks[i]; if (!old || deps.some((v,j) => v !== old.deps[j])) cleanup.push(() => { old?.cleanup?.(); hooks[i] = {deps, cleanup: effect()}; }); }
};
const ui = Object.fromEntries(['Button','Dropdown','DropdownItem','DropdownToggle','Panel','Portal','Scrollable','Tooltip'].map(name => [name, name]));
// The real cs2/ui runtime does not export a ScrollController constructor.
assert.equal(ui.ScrollController,undefined);
const output = { exports: {} };
const code = ts.transpileModule(fs.readFileSync(root + '/src/mods/import-earth-location.tsx', 'utf8'), { compilerOptions: { jsx: ts.JsxEmit.ReactJSX, module: ts.ModuleKind.CommonJS, target: ts.ScriptTarget.ES2020, esModuleInterop: true } }).outputText;
vm.runInNewContext(code, { exports: output.exports, console,
  window: { addEventListener: (name, cb) => handlers.set(name, cb), removeEventListener: (name, cb) => { if (handlers.get(name) === cb) handlers.delete(name); } },
  require(name) {
    if (name === 'react') return react;
    if (name === 'react/jsx-runtime') return require(root + '/node_modules/react/jsx-runtime');
    if (name === 'cs2/ui') return ui;
    if (name === 'cs2/api') return { bindValue: (_, name, value) => ({name, value}), useValue: b => bindings[b.name] ?? b.value,
      trigger: (_, name, ...args) => { events.push([name, ...args]); if (name === 'setPanelOpen') bindings.panelOpen = args[0]; if(name === 'setRoadLevel') {bindings.roadLevel=args[0];bindings.ready=false;} if(name === 'setImportRail') {bindings.importRail=args[0];bindings.ready=false;} if(name === 'setImportForests') {bindings.importForests=args[0];bindings.ready=false;} if(acknowledgeActions&&(name==='prepare'||name==='confirm')) bindings.actionRevision=String(Number(bindings.actionRevision??0)+1); } };
    if (name.endsWith('.scss')) return { __esModule: true, default: new Proxy({}, { get: (_, p) => p }) };
    if (name === './location-picker') return { LocationPicker: 'LocationPicker' };
    throw Error(name);
  }
});
function render() {
  let tree;
  for(let pass=0;pass<5;pass++){cleanup=[];hookIndex=0;dirty=false;tree=output.exports.ImportEarthLocationContent();cleanup.forEach(fn=>fn());if(!dirty)break;}
  const nodes = [];
  function visit(node) { if (!node || typeof node !== 'object') return; if (Array.isArray(node)) { node.forEach(visit); return; }
    nodes.push(node); visit(node.props?.children); visit(node.props?.content); }
  visit(tree); return nodes;
}
function button(nodes, focusKey) { const node = nodes.find(n => n.props?.focusKey === focusKey); assert(node, focusKey); return node; }
function containsConfirmation(node) {
  if (!node || typeof node !== 'object') return false;
  if (Array.isArray(node)) return node.some(containsConfirmation);
  return node.props?.className === 'confirmBackdrop' || containsConfirmation(node.props?.children);
}
function assertViewportConfirmation(nodes) {
  const backdrop = nodes.find(n => n.props?.className === 'confirmBackdrop');
  assert(backdrop, 'Confirmation backdrop missing');
  assert(!containsConfirmation(nodes.find(n => n.type === 'Panel')), 'Confirmation must not be clipped inside native Panel');
  assert(!containsConfirmation(nodes.find(n => n.props?.className === 'host')), 'Confirmation must not inherit launcher host hit-testing');
  assert(nodes.find(n => n.type === 'Portal').props.children.includes(backdrop), 'Confirmation must be a viewport Portal sibling');
  assert(nodes.some(n => n.props?.role === 'dialog' && n.props['aria-modal'] === 'true'), 'Replacement needs a visible accessible dialog');
}
const isLogViewport = node => node.type === 'Scrollable' && node.props.className === 'logViewport';
let nodes = render();
assert(!nodes.some(n => n.props?.focusKey === 'import-earth-location-launcher'), 'Launcher must stay hidden outside editor');
assert(events.some(e => e[0] === 'frontendState' && JSON.parse(e[1]).isEditor === false), 'Hidden frontend must still acknowledge its mount');
bindings.isEditor = true;
nodes = render();
assert(events.some(e => e[0] === 'frontendState' && JSON.parse(e[1]).isEditor === true), 'Editor transition must acknowledge standalone visibility');
const acknowledgements = events.filter(e => e[0] === 'frontendState').length;
render();
assert.equal(events.filter(e => e[0] === 'frontendState').length, acknowledgements, 'Unchanged render repeated mount acknowledgement');
assert(!nodes.some(n => n.type === 'Panel'));
button(nodes, 'import-earth-location-launcher').props.onSelect();
nodes = render(); assert(nodes.some(n => n.type === 'Panel'));
assert(!nodes.some(n => n.type === 'select' || n.type === 'option'), 'Gameface-incompatible select must never render');
assert.equal(nodes.filter(n => n.type === 'DropdownItem').length, 1);
bindings.mapExtDetected = true;
nodes = render(); assert.equal(nodes.filter(n => n.type === 'DropdownItem').length, 3);
bindings.mapSize = 'mapext57';
nodes = render(); assert.equal(nodes.find(n => n.type === 'DropdownToggle').props.children, '57.344 km (MapExt)');
nodes.find(n => n.type === 'DropdownItem' && n.props.value === 'mapext28').props.onChange('mapext28');
assert.deepEqual(events.at(-1), ['setMapSize', 'mapext28']);
assert(nodes.find(n => n.type === 'Dropdown').props.theme.dropdownPopup, 'Popup must have explicit foreground theme');
assert(!nodes.some(n => n.type === 'input' && n.props.type === 'checkbox'), 'Unsupported browser checkbox must not render');
assert.equal(nodes.filter(n=>n.props?.role==='radio').length,4);
for (const radio of nodes.filter(n=>n.props?.role==='radio')) {
 const indicator=radio.props.children[0];assert(indicator.props.className.includes('radioIndicator'),'Road choice needs a circular radio indicator');
 assert(!indicator.props.className.includes('checkbox'),'Road radio must not inherit the square checkbox box');
 assert.equal(indicator.props['aria-hidden'],'true');
 assert.equal(indicator.props.className.includes('radioSelected'),radio.props['aria-checked']);
}
assert.equal(button(nodes,'iel-roads-primary').props['aria-checked'],true);assert.equal(button(nodes,'iel-rail').props['aria-checked'],true);
assert.equal(button(nodes,'iel-forests').props['aria-checked'],true);
button(nodes,'iel-forests').props.onSelect();nodes=render();assert.equal(button(nodes,'iel-forests').props['aria-checked'],false);assert.equal(button(nodes,'iel-rail').props['aria-checked'],true);
for(const tier of ['none','highways','primary','secondary']) {
 button(nodes,`iel-roads-${tier}`).props.onSelect();nodes=render();assert.equal(nodes.filter(n=>n.props?.role==='radio'&&n.props['aria-checked']).length,1);assert.equal(button(nodes,'iel-rail').props['aria-checked'],true);
 button(nodes,'iel-rail').props.onSelect();nodes=render();assert.equal(button(nodes,`iel-roads-${tier}`).props['aria-checked'],true);assert.equal(button(nodes,'iel-rail').props['aria-checked'],false);
 button(nodes,'iel-rail').props.onSelect();nodes=render();
}
const prepareButton=button(nodes,'iel-prepare');const beforePrepare=events.filter(e=>e[0]==='prepare').length;prepareButton.props.onSelect();prepareButton.props.onSelect();assert.equal(events.filter(e=>e[0]==='prepare').length,beforePrepare+1,'Rapid duplicate prepare accepted');assert.equal(events.at(-1)[0],'prepare');
nodes = render(); assert(!nodes.some(isLogViewport), 'Prepare must not automatically open logs');
button(nodes, 'iel-log').props.onSelect(); nodes = render(); assert(nodes.some(isLogViewport), 'View log must still open logs');
assert.equal(nodes.find(isLogViewport).props.trackVisibility, 'always');
assert.equal(nodes.find(isLogViewport).props.controller,undefined,'Rollback must not install a custom scroll controller');
bindings.log='new latest line';nodes=render();assert(nodes.some(n=>n.type==='pre'&&n.props.children==='new latest line'),'Scrollable log stopped updating');
bindings.busy = true; nodes = render(); assert(nodes.some(isLogViewport));
bindings.busy = false; bindings.stage = 0; bindings.error = 'All providers exhausted';
nodes = render(); assert(nodes.some(isLogViewport), 'Failure must preserve the logs');
button(nodes, 'iel-prepare').props.onSelect();
nodes = render(); assert(nodes.some(isLogViewport), 'Preparing again must not toggle logs off');
button(nodes, 'iel-log').props.onSelect(); nodes = render(); assert(!nodes.some(isLogViewport));
button(nodes, 'iel-prepare').props.onSelect(); nodes = render(); assert(!nodes.some(isLogViewport), 'Preparing must preserve a manually hidden log');
button(nodes, 'iel-log').props.onSelect(); nodes = render(); assert(nodes.some(isLogViewport));
bindings.ready = true; nodes = render(); button(nodes, 'iel-replace').props.onSelect(); nodes = render();
assert(nodes.some(n => n.props?.focusKey === 'iel-confirm-replace'));
assert(nodes.some(n=>n.type==='p'&&typeof n.props.children==='string'&&n.props.children.includes('up to 5 minutes')), 'Confirmation missing freeze warning');
assertViewportConfirmation(nodes);
button(nodes, 'iel-confirm-cancel').props.onSelect(); nodes = render();
assert(!nodes.some(n => n.props?.focusKey === 'iel-confirm-replace'), 'Cancel must release the overlay');
assert(bindings.panelOpen); assert(!events.some(e => e[0] === 'confirm'), 'Cancel must never replace the map');
assert(nodes.some(isLogViewport), 'Cancelling confirmation must preserve the logs');
button(nodes, 'iel-replace').props.onSelect(); nodes = render();
nodes.find(n => n.type === 'Panel').props.onClose(); nodes = render();
assert(!nodes.some(n => n.props?.className === 'confirmBackdrop'), 'Closing the panel must release its confirmation overlay');
assert(!events.some(e => e[0] === 'confirm'), 'Closing confirmation must not mutate the map');
bindings.panelOpen = true; nodes = render();
assert(!nodes.some(n => n.props?.className === 'confirmBackdrop'), 'Reopening must not restore a stale overlay');
button(nodes, 'iel-replace').props.onSelect(); nodes = render(); assertViewportConfirmation(nodes);
handlers.get('keydown')({key: 'Escape', preventDefault(){}, stopPropagation(){}});
nodes = render(); assert(!nodes.some(n => n.props?.focusKey === 'iel-confirm-replace'));
assert(bindings.panelOpen); assert(!events.some(e => e[0] === 'confirm'));
handlers.get('keydown')({key: 'Escape', preventDefault(){}, stopPropagation(){}});
assert(!bindings.panelOpen);
bindings.panelOpen = true; bindings.ready = true; nodes = render(); button(nodes, 'iel-replace').props.onSelect(); nodes = render();
assertViewportConfirmation(nodes);
assert(nodes.some(n=>n.type==='Scrollable'&&n.props.className==='confirmBody'&&n.props.horizontal===false),'Confirmation needs scrollable body');
assert(button(nodes,'iel-confirm-replace').props.className.includes('danger'));assert(button(nodes,'iel-confirm-cancel').props.className.includes('utility'));
const replaceButton=button(nodes,'iel-confirm-replace');const beforeReplace=events.filter(e=>e[0]==='confirm').length;replaceButton.props.onSelect();replaceButton.props.onSelect();assert.equal(events.filter(e=>e[0]==='confirm').length,beforeReplace+1,'Rapid duplicate replacement accepted');assert.equal(events.at(-1)[0],'confirm');
// Frequent status ticks cannot unlock a pending action before backend acknowledgement.
nodes=render();acknowledgeActions=false;
button(nodes,'iel-replace').props.onSelect();nodes=render();button(nodes,'iel-confirm-replace').props.onSelect();
nodes=render();bindings.status='Smoothing slopes… 12 seconds';nodes=render();
assert.equal(button(nodes,'iel-replace').props.disabled,true,'Status tick unlocked pending replacement');
acknowledgeActions=true;
bindings.busy = true; bindings.stage = 6; bindings.ready = false;
nodes = render(); assert(!nodes.some(n => n.props?.focusKey === 'iel-cancel'));
nodes.find(n => n.type === 'Panel').props.onClose(); assert(!bindings.panelOpen);
bindings.panelOpen=true;bindings.busy=false;bindings.ready=true;nodes=render();button(nodes,'iel-replace').props.onSelect();nodes=render();assert(nodes.some(n=>n.props?.focusKey==='iel-confirm-replace'));
const selectedRoads=bindings.roadLevel,selectedRail=bindings.importRail;bindings.mapSession='map-B';bindings.ready=false;bindings.status='Map session reset';nodes=render();
assert(!nodes.some(n=>n.props?.focusKey==='iel-confirm-replace'),'Map change retained stale confirmation');assert(!nodes.some(isLogViewport),'Map change retained previous map log visibility');assert.equal(bindings.roadLevel,selectedRoads);assert.equal(bindings.importRail,selectedRail);
assert.equal(button(nodes,'iel-prepare').props.disabled,false,'Map reset left prepare disabled');
bindings.isEditor = false; assert.equal(render().length, 0);
console.log('PASS: launcher, native map-size dropdown, preparation, confirmation, Escape/close, application cancellation guard, editor-only visibility');
