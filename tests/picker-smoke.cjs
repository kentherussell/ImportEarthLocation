const fs=require('fs'),vm=require('vm'),assert=require('assert'),path=require('path');
const root=path.resolve(__dirname,'..','ImportEarthLocation.UI'), ts=require(root+'/node_modules/typescript');
let hooks=[],cursor=0,pending=[],timers=new Map(),timerId=0;
const bindings={placeResults:'{}',displayTiles:'{}'}, events=[],commits=[];
const react={...require(root+'/node_modules/react'),
 useState(initial){const i=cursor++; if(!(i in hooks)) hooks[i]=initial; return[hooks[i],v=>hooks[i]=typeof v==='function'?v(hooks[i]):v];},
 useRef(initial){const i=cursor++; if(!(i in hooks)) hooks[i]={current:initial}; return hooks[i];},
 useEffect(effect,deps){const i=cursor++,old=hooks[i]; if(!old||deps.some((v,j)=>v!==old.deps[j])) pending.push(()=>{old?.cleanup?.();hooks[i]={deps,cleanup:effect()};});}
};
const output={exports:{}};
vm.runInNewContext(ts.transpileModule(fs.readFileSync(root+'/src/mods/location-picker.tsx','utf8'),{compilerOptions:{jsx:ts.JsxEmit.ReactJSX,module:ts.ModuleKind.CommonJS,target:ts.ScriptTarget.ES2020,esModuleInterop:true}}).outputText,{
 exports:output.exports, window:{setTimeout(fn){const id=++timerId;timers.set(id,fn);return id;},clearTimeout(id){timers.delete(id);}},
 require(name){if(name==='react')return react;if(name==='react/jsx-runtime')return require(root+'/node_modules/react/jsx-runtime');if(name==='cs2/ui')return{Button:'Button',Panel:'Panel',Scrollable:'Scrollable'};if(name==='cs2/api')return{bindValue:(_,name,value)=>({name,value}),useValue:b=>bindings[b.name]??b.value,trigger:(_,name,...args)=>events.push([name,...args])};if(name.endsWith('.scss'))return{__esModule:true,default:new Proxy({},{get:(_,p)=>p})};throw Error(name);}
});
const api=output.exports;
assert.strictEqual(api.parseCoordinate('-',90),null);assert.strictEqual(api.parseCoordinate('',90),null);assert.strictEqual(api.parseCoordinate('NaN',90),null);assert.strictEqual(api.parseCoordinate('91',90),null);assert.strictEqual(api.parseCoordinate('-.5',90),-.5);assert.strictEqual(api.parseCoordinate('180',180),180);
for(const [lat,lon] of [[0,0],[43.72,15.82],[-80,179]]){const p=api.project(lat,lon,9),q=api.unproject(p.x,p.y,9);assert(Math.abs(q.latitude-lat)<1e-6&&Math.abs(q.longitude-lon)<1e-6);}
let props={initial:{latitude:43.72,longitude:15.82,mapSize:'mapext57'},mapExt:false,onCommit:value=>commits.push(value),onClose:()=>events.push(['closed'])};
function render(){cursor=0;pending=[];const tree=api.LocationPicker(props),nodes=[];function visit(n){if(!n||typeof n!=='object')return;if(Array.isArray(n)){n.forEach(visit);return;}nodes.push(n);visit(n.props?.children);}visit(tree);pending.forEach(fn=>fn());return nodes;}
function button(nodes,label){const n=nodes.find(n=>n.type==='Button'&&n.props.children===label);assert(n,label);return n;}
let nodes=render();assert(!nodes.some(n=>n.type==='select'||n.type==='option'));assert(nodes.some(n=>n.type==='Button'&&n.props.children===14.336)||nodes.some(n=>n.type==='Button'&&Array.isArray(n.props.children)&&n.props.children[0]===14.336));
assert(!nodes.some(n=>n.type==='Button'&&Array.isArray(n.props.children)&&n.props.children[0]===57.344));
let inputs=nodes.filter(n=>n.type==='input');inputs[1].props.onChange({target:{value:'-'}});nodes=render();button(nodes,'Use This Location').props.onSelect();assert.equal(commits.length,0);assert.equal(nodes.filter(n=>n.type==='input')[1].props.value,'-');
inputs=nodes.filter(n=>n.type==='input');inputs[1].props.onChange({target:{value:'44'}});inputs[2].props.onChange({target:{value:'16'}});nodes=render();button(nodes,'Use This Location').props.onSelect();assert.equal(commits.length,1);assert.deepStrictEqual(JSON.parse(JSON.stringify(commits[0])),{latitude:44,longitude:16,mapSize:'vanilla'});assert(!events.some(e=>e[0]==='prepare'||e[0]==='confirm'),'Picker must not start imports');
nodes=render();nodes.find(n=>n.type==='input').props.onChange({target:{value:'Zadar'}});nodes=render();button(nodes,'Search').props.onSelect();nodes=render();const oldRevision=JSON.parse(events.filter(e=>e[0]==='searchPlace').at(-1)[1]).revision;
nodes.find(n=>n.type==='input').props.onChange({target:{value:'Split'}});nodes=render();bindings.placeResults=JSON.stringify({revision:oldRevision,results:[{label:'stale city',latitude:1,longitude:2}]});nodes=render();assert(!nodes.some(n=>n.props?.children==='stale city'),'Stale search result shown');
button(nodes,'Search').props.onSelect();nodes=render();const freshRevision=JSON.parse(events.filter(e=>e[0]==='searchPlace').at(-1)[1]).revision;assert(freshRevision>oldRevision);
bindings.placeResults=JSON.stringify({revision:freshRevision,results:[{label:'fresh city',latitude:45,longitude:17}]});nodes=render();button(nodes,'fresh city').props.onSelect();nodes=render();assert.equal(nodes.filter(n=>n.type==='input')[1].props.value,'45.000000');assert.equal(commits.length,1,'Search must remain draft-only');
for(const fn of [...timers.values()])fn();timers.clear();nodes=render();const tileRevision=JSON.parse(events.filter(e=>e[0]==='previewTiles').at(-1)[1]).revision;bindings.displayTiles=JSON.stringify({revision:tileRevision,tiles:{},error:true});nodes=render();assert(nodes.some(n=>n.props?.children==='Map preview unavailable. Enter coordinates manually.'));
button(nodes,'Cancel').props.onSelect();assert.equal(commits.length,1);for(const hook of hooks)hook?.cleanup?.();assert(events.some(e=>e[0]==='cancelLocationServices'));
hooks=[];nodes=render();nodes.find(n=>n.type==='input').props.onChange({target:{value:'Zadar'}});nodes=render();button(nodes,'Search').props.onSelect();const reopened=JSON.parse(events.filter(e=>e[0]==='searchPlace').at(-1)[1]).revision;assert(reopened>freshRevision,'Reopened picker reused stale revision');
console.log('PASS: coordinate validity/partial edits, projection, vanilla fallback, draft-only commit/cancel, no import actions, stale/reopened search protection, failed preview fallback, request cancellation.');
