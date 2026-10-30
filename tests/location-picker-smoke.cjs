const fs = require('fs');
const path = require('path');
const vm = require('vm');
const assert = require('assert');
const root = path.resolve(__dirname, '../ImportEarthLocation.UI');
const ts = require(root + '/node_modules/typescript');
const React = require(root + '/node_modules/react');
const hooks = [], timers = new Map(), bindings = {};
let index = 0, effects = [], dirty = false, timerId = 0, committed;
const place = { label: 'Marseille', latitude: 43.296482, longitude: 5.36978 };
const react = { ...React,
  useState(initial) { const i = index++; if (!(i in hooks)) hooks[i] = initial;
    return [hooks[i], value => { const next = typeof value === 'function' ? value(hooks[i]) : value;
      if (next !== hooks[i]) { hooks[i] = next; dirty = true; } }]; },
  useRef(initial) { const i = index++; if (!(i in hooks)) hooks[i] = { current: initial }; return hooks[i]; },
  useEffect(effect, deps) { const i = index++, old = hooks[i];
    if (!old || deps.some((v, j) => v !== old.deps[j])) effects.push(() => {
      old?.cleanup?.(); hooks[i] = { deps, cleanup: effect() }; }); }
};
const output = { exports: {} };
const code = ts.transpileModule(fs.readFileSync(root + '/src/mods/location-picker.tsx', 'utf8'), {
  compilerOptions: { jsx: ts.JsxEmit.ReactJSX, module: ts.ModuleKind.CommonJS, target: ts.ScriptTarget.ES2020, esModuleInterop: true }
}).outputText;
vm.runInNewContext(code, { exports: output.exports, console,
  window: { setTimeout(fn) { timers.set(++timerId, fn); return timerId; }, clearTimeout(id) { timers.delete(id); } },
  require(name) {
    if (name === 'react') return react;
    if (name === 'react/jsx-runtime') return require(root + '/node_modules/react/jsx-runtime');
    if (name === 'cs2/ui') return { Button: 'Button', Panel: 'Panel', Scrollable: 'Scrollable' };
    if (name === 'cs2/api') return { bindValue: (_, name, fallback) => ({ name, fallback }), useValue: b => bindings[b.name] ?? b.fallback,
      trigger: (_, name, payload) => { if (name === 'searchPlace') {
        bindings.placeResults = JSON.stringify({ revision: JSON.parse(payload).revision, results: [place] });
      } } };
    if (name.endsWith('.scss')) return { __esModule: true, default: new Proxy({}, { get: (_, key) => key }) };
    throw Error(name);
  }
});
function render() {
  let tree;
  for (let pass = 0; pass < 10; pass++) {
    index = 0; effects = []; dirty = false;
    tree = output.exports.LocationPicker({ initial: { latitude: 0, longitude: 0, mapSize: 'vanilla' }, mapExt: true,
      onCommit: value => { committed = value; }, onClose() {} });
    effects.forEach(fn => fn()); if (!dirty) break;
  }
  const nodes = [];
  function visit(node) { if (!node || typeof node !== 'object') return;
    if (Array.isArray(node)) return node.forEach(visit);
    nodes.push(node); visit(node.props?.children); }
  visit(tree); return nodes;
}
let nodes = render();
nodes.find(n => n.props?.['aria-label'] === 'City or place').props.onChange({ target: { value: 'Marseille' } });
nodes = render(); nodes.find(n => n.type === 'Button' && n.props.children === 'Search').props.onSelect();
nodes = render();
assert(nodes.some(n => n.props?.className === 'resultButton'), 'Search response did not open results');
nodes.find(n => n.props?.className === 'section').props.onClickCapture({ target: {
  closest(selector) { return selector === 'input, button' ? {} : null; }
} });
nodes = render();
assert(!nodes.some(n => n.props?.className === 'resultButton'), 'Another input control did not dismiss results');
nodes.find(n => n.type === 'Button' && n.props.children === 'Search').props.onSelect(); nodes = render();
// Simulate capture before the native result button's selection callback. A
// rerender here reproduces the premature unmount, not just onSelect in isolation.
nodes.find(n => n.props?.className === 'section').props.onClickCapture({ target: {
  closest(selector) { return selector === '[data-location-results]' ? {} : selector === 'input, button' ? {} : null; }
} });
nodes = render();
const result = nodes.find(n => n.props?.className === 'resultButton');
assert(result, 'Capture dismissed the result before selection could run');
result.props.onSelect(); nodes = render();
const coordinates = nodes.filter(n => n.type === 'input' && !n.props['aria-label']);
assert.equal(coordinates[0].props.value, String(place.latitude));
assert.equal(coordinates[1].props.value, String(place.longitude));
assert(!nodes.some(n => n.props?.className === 'resultButton'), 'Selected result remained open');
// The chosen place's import footprint fills the 400×260 preview, not a speck on the world view.
const footprint = nodes.find(n => n.props?.className === 'footprint').props.style;
const footprintWidth = parseFloat(footprint.width), footprintHeight = parseFloat(footprint.height);
assert(footprintWidth <= 400 * 0.85 + 1e-6 && footprintHeight <= 260 * 0.85 + 1e-6 && (footprintWidth > 400 * 0.4 || footprintHeight > 260 * 0.4),
  `Selecting a place did not zoom to its footprint: ${footprintWidth}×${footprintHeight}`);
nodes.find(n => n.type === 'Button' && n.props.children === 'Use This Location').props.onSelect();
assert.equal(committed.latitude, place.latitude); assert.equal(committed.longitude, place.longitude);
console.log('PASS: search response, capture-before-selection, populated coordinates, result dismissal and committed location. Native Gameface event dispatch/layout still needs in-game verification.');
