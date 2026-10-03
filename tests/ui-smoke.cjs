const fs = require('fs');
const vm = require('vm');
const assert = require('assert');
const root = require('path').resolve(__dirname, '..', 'ImportEarthLocation.UI');
const ts = require(root + '/node_modules/typescript');
const React = require(root + '/node_modules/react');
const bindings = { isEditor: true, panelOpen: false, busy: false, ready: false, stage: 0, mapExtDetected: false };
const events = [];
const hooks = [];
let hookIndex = 0;
let cleanup = [];
let dirty = false;
const handlers = new Map();
const react = { ...React,
  useState(initial) { const i = hookIndex++; if (!(i in hooks)) hooks[i] = initial; return [hooks[i], value => { const next = typeof value === 'function' ? value(hooks[i]) : value; if (next !== hooks[i]) {hooks[i] = next; dirty = true;} }]; },
  useRef(initial) { const i = hookIndex++; if (!(i in hooks)) hooks[i] = {current:initial}; return hooks[i]; },
  useEffect(effect, deps) { const i = hookIndex++; const old = hooks[i]; if (!old || deps.some((v,j) => v !== old.deps[j])) cleanup.push(() => { old?.cleanup?.(); hooks[i] = {deps, cleanup: effect()}; }); }
};
const ui = Object.fromEntries(['Button','Dropdown','DropdownItem','DropdownToggle','Panel','Portal','Scrollable','Tooltip'].map(name => [name, name]));
const output = { exports: {} };
const code = ts.transpileModule(fs.readFileSync(root + '/src/mods/import-earth-location.tsx', 'utf8'), { compilerOptions: { jsx: ts.JsxEmit.ReactJSX, module: ts.ModuleKind.CommonJS, target: ts.ScriptTarget.ES2020, esModuleInterop: true } }).outputText;
vm.runInNewContext(code, { exports: output.exports, console,
  window: { addEventListener: (name, cb) => handlers.set(name, cb), removeEventListener: (name, cb) => { if (handlers.get(name) === cb) handlers.delete(name); } },
  require(name) {
    if (name === 'react') return react;
    if (name === 'react/jsx-runtime') return require(root + '/node_modules/react/jsx-runtime');
    if (name === 'cs2/ui') return ui;
    if (name === 'cs2/api') return { bindValue: (_, name, value) => ({name, value}), useValue: b => bindings[b.name] ?? b.value,
      trigger: (_, name, ...args) => { events.push([name, ...args]); if (name === 'setPanelOpen') bindings.panelOpen = args[0]; if(name === 'setRoadLevel') {bindings.roadLevel=args[0];bindings.ready=false;} if(name === 'setImportRail') {bindings.importRail=args[0];bindings.ready=false;} if(name==='prepare'||name==='confirm') bindings.actionRevision=String(Number(bindings.actionRevision??0)+1); } };
    if (name.endsWith('.scss')) return { __esModule: true, default: new Proxy({}, { get: (_, p) => p }) };
    if (name === './location-picker') return { LocationPicker: 'LocationPicker' };
    throw Error(name);
  }
});
function render() {
  let tree;
  for(let pass=0;pass<5;pass++){cleanup=[];hookIndex=0;dirty=false;tree=output.exports.ImportEarthLocationContent();cleanup.forEach(fn=>fn());if(!dirty)break;}
  const nodes = [];
  function visit(node) { if (!node || typeof node !== 'object') return; if (Array.isArray(node)) { node.forEach(visit); return; } nodes.push(node); visit(node.props?.children); visit(node.props?.content); }
  visit(tree); return nodes;
}
function button(nodes, focusKey) { const node = nodes.find(n => n.props?.focusKey === focusKey); assert(node, focusKey); return node; }
const isLogViewport = node => node.type === 'Scrollable' && node.props.className === 'logViewport';
let nodes = render();
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
assert.equal(button(nodes,'iel-roads-primary').props['aria-checked'],true);assert.equal(button(nodes,'iel-rail').props['aria-checked'],true);
for(const tier of ['none','highways','primary','secondary']) {
 button(nodes,`iel-roads-${tier}`).props.onSelect();nodes=render();assert.equal(nodes.filter(n=>n.props?.role==='radio'&&n.props['aria-checked']).length,1);assert.equal(button(nodes,'iel-rail').props['aria-checked'],true);
 button(nodes,'iel-rail').props.onSelect();nodes=render();assert.equal(button(nodes,`iel-roads-${tier}`).props['aria-checked'],true);assert.equal(button(nodes,'iel-rail').props['aria-checked'],false);
 button(nodes,'iel-rail').props.onSelect();nodes=render();
}
const prepareButton=button(nodes,'iel-prepare');const beforePrepare=events.filter(e=>e[0]==='prepare').length;prepareButton.props.onSelect();prepareButton.props.onSelect();assert.equal(events.filter(e=>e[0]==='prepare').length,beforePrepare+1,'Rapid duplicate prepare accepted');assert.equal(events.at(-1)[0],'prepare');
nodes = render(); assert(nodes.some(isLogViewport), 'Prepare must open the logs');
assert.equal(nodes.find(isLogViewport).props.trackVisibility, 'always');
bindings.busy = true; nodes = render(); assert(nodes.some(isLogViewport));
bindings.busy = false; bindings.stage = 0; bindings.error = 'All providers exhausted';
nodes = render(); assert(nodes.some(isLogViewport), 'Failure must preserve the logs');
button(nodes, 'iel-prepare').props.onSelect();
nodes = render(); assert(nodes.some(isLogViewport), 'Preparing again must not toggle logs off');
button(nodes, 'iel-log').props.onSelect(); nodes = render(); assert(!nodes.some(isLogViewport));
button(nodes, 'iel-prepare').props.onSelect(); nodes = render(); assert(nodes.some(isLogViewport));
bindings.ready = true; nodes = render(); button(nodes, 'iel-replace').props.onSelect(); nodes = render();
assert(nodes.some(n => n.props?.focusKey === 'iel-confirm-replace'));
handlers.get('keydown')({key: 'Escape', preventDefault(){}, stopPropagation(){}});
nodes = render(); assert(!nodes.some(n => n.props?.focusKey === 'iel-confirm-replace'));
assert(bindings.panelOpen); assert(!events.some(e => e[0] === 'confirm'));
handlers.get('keydown')({key: 'Escape', preventDefault(){}, stopPropagation(){}});
assert(!bindings.panelOpen);
bindings.panelOpen = true; bindings.ready = true; nodes = render(); button(nodes, 'iel-replace').props.onSelect(); nodes = render();
assert(nodes.some(n=>n.type==='Scrollable'&&n.props.className==='confirmBody'&&n.props.horizontal===false),'Confirmation needs scrollable body');
assert(button(nodes,'iel-confirm-replace').props.className.includes('danger'));assert(button(nodes,'iel-confirm-cancel').props.className.includes('utility'));
const replaceButton=button(nodes,'iel-confirm-replace');const beforeReplace=events.filter(e=>e[0]==='confirm').length;replaceButton.props.onSelect();replaceButton.props.onSelect();assert.equal(events.filter(e=>e[0]==='confirm').length,beforeReplace+1,'Rapid duplicate replacement accepted');assert.equal(events.at(-1)[0],'confirm');
bindings.busy = true; bindings.stage = 6; bindings.ready = false;
nodes = render(); assert(!nodes.some(n => n.props?.focusKey === 'iel-cancel'));
nodes.find(n => n.type === 'Panel').props.onClose(); assert(!bindings.panelOpen);
bindings.panelOpen=true;bindings.busy=false;bindings.ready=true;nodes=render();button(nodes,'iel-replace').props.onSelect();nodes=render();assert(nodes.some(n=>n.props?.focusKey==='iel-confirm-replace'));
const selectedRoads=bindings.roadLevel,selectedRail=bindings.importRail;bindings.mapSession='map-B';bindings.ready=false;bindings.status='Map session reset';nodes=render();
assert(!nodes.some(n=>n.props?.focusKey==='iel-confirm-replace'),'Map change retained stale confirmation');assert(!nodes.some(isLogViewport),'Map change retained previous map log visibility');assert.equal(bindings.roadLevel,selectedRoads);assert.equal(bindings.importRail,selectedRail);
assert.equal(button(nodes,'iel-prepare').props.disabled,false,'Map reset left prepare disabled');
bindings.isEditor = false; assert.equal(render().length, 0);
console.log('PASS: launcher, native map-size dropdown, preparation, confirmation, Escape/close, application cancellation guard, editor-only visibility');
