const fs=require('fs'),vm=require('vm'),assert=require('assert'),path=require('path');
const root=path.resolve(__dirname,'..','ImportEarthLocation.UI'), ts=require(root+'/node_modules/typescript');
let hooks=[],cursor=0,pending=[],timers=new Map(),timerId=0,dirty=false;
const bindings={placeResults:'{}',displayTiles:'{}'}, events=[],commits=[];
const react={...require(root+'/node_modules/react'),
 useState(initial){const i=cursor++; if(!(i in hooks)) hooks[i]=initial; return[hooks[i],v=>{const next=typeof v==='function'?v(hooks[i]):v;if(next!==hooks[i]){hooks[i]=next;dirty=true;}}];},
 useRef(initial){const i=cursor++; if(!(i in hooks)) hooks[i]={current:initial}; return hooks[i];},
 useEffect(effect,deps){const i=cursor++,old=hooks[i]; if(!old||deps.some((v,j)=>v!==old.deps[j])) pending.push(()=>{old?.cleanup?.();hooks[i]={deps,cleanup:effect()};});}
};
const output={exports:{}};
vm.runInNewContext(ts.transpileModule(fs.readFileSync(root+'/src/mods/location-picker.tsx','utf8'),{compilerOptions:{jsx:ts.JsxEmit.ReactJSX,module:ts.ModuleKind.CommonJS,target:ts.ScriptTarget.ES2020,esModuleInterop:true}}).outputText,{
 exports:output.exports, window:{setTimeout(fn,delay){const id=++timerId;timers.set(id,{fn,delay});return id;},clearTimeout(id){timers.delete(id);}},
 require(name){if(name==='react')return react;if(name==='react/jsx-runtime')return require(root+'/node_modules/react/jsx-runtime');if(name==='cs2/ui')return{Button:'Button',Panel:'Panel',Scrollable:'Scrollable'};if(name==='cs2/api')return{bindValue:(_,name,value)=>({name,value}),useValue:b=>bindings[b.name]??b.value,trigger:(_,name,...args)=>events.push([name,...args])};if(name.endsWith('.scss'))return{__esModule:true,default:new Proxy({},{get:(_,p)=>p})};throw Error(name);}
});
const api=output.exports;
assert.strictEqual(api.parseCoordinate('-',90),null);assert.strictEqual(api.parseCoordinate('',90),null);assert.strictEqual(api.parseCoordinate('NaN',90),null);assert.strictEqual(api.parseCoordinate('91',90),null);assert.strictEqual(api.parseCoordinate('-.5',90),-.5);assert.strictEqual(api.parseCoordinate('180',180),180);
for(const [lat,lon] of [[0,0],[43.72,15.82],[-80,179]]){const p=api.project(lat,lon,9),q=api.unproject(p.x,p.y,9);assert(Math.abs(q.latitude-lat)<1e-6&&Math.abs(q.longitude-lon)<1e-6);}
let props={initial:{latitude:43.72,longitude:15.82,mapSize:'mapext57'},mapExt:false,onCommit:value=>commits.push(value),onClose:()=>events.push(['closed'])};
function render(){let tree;for(let pass=0;pass<5;pass++){cursor=0;pending=[];dirty=false;tree=api.LocationPicker(props);pending.forEach(fn=>fn());if(!dirty)break;}const nodes=[];function visit(n){if(!n||typeof n!=='object')return;if(Array.isArray(n)){n.forEach(visit);return;}nodes.push(n);visit(n.props?.children);}visit(tree);return nodes;}
function fireTimers(delay){for(const [id,timer] of [...timers])if(timer.delay===delay){timers.delete(id);timer.fn();}}
function button(nodes,label){const n=nodes.find(n=>n.type==='Button'&&n.props.children===label);assert(n,label);return n;}
let nodes=render();assert(!nodes.some(n=>n.type==='select'||n.type==='option'));assert.equal(button(nodes,'14km').props.disabled,false);
assert.equal(button(nodes,'28km').props.disabled,true);assert.equal(button(nodes,'57km').props.disabled,true);
assert.equal(nodes.find(n=>n.type==='Panel').props.header,'Select Location');
assert.equal(nodes.find(n=>n.props?.className==='sizeChoices').props.children.length,3);
let inputs=nodes.filter(n=>n.type==='input');inputs[1].props.onChange({target:{value:'-'}});nodes=render();button(nodes,'Use This Location').props.onSelect();assert.equal(commits.length,0);assert.equal(nodes.filter(n=>n.type==='input')[1].props.value,'-');
inputs=nodes.filter(n=>n.type==='input');inputs[1].props.onChange({target:{value:'44'}});inputs[2].props.onChange({target:{value:'16'}});nodes=render();button(nodes,'Use This Location').props.onSelect();assert.equal(commits.length,1);assert.deepStrictEqual(JSON.parse(JSON.stringify(commits[0])),{latitude:44,longitude:16,mapSize:'vanilla'});assert(!events.some(e=>e[0]==='prepare'||e[0]==='confirm'),'Picker must not start imports');
nodes=render();nodes.find(n=>n.type==='input').props.onChange({target:{value:'Zadar'}});nodes=render();button(nodes,'Search').props.onSelect();nodes=render();const oldRevision=JSON.parse(events.filter(e=>e[0]==='searchPlace').at(-1)[1]).revision;
nodes.find(n=>n.type==='input').props.onChange({target:{value:'Split'}});nodes=render();bindings.placeResults=JSON.stringify({revision:oldRevision,results:[{label:'stale city',latitude:1,longitude:2}]});nodes=render();assert(!nodes.some(n=>n.props?.children==='stale city'),'Stale search result shown');
let prevented=false;const searchInput=nodes.find(n=>n.type==='input');
const searchesBefore=events.filter(e=>e[0]==='searchPlace').length;
searchInput.props.onKeyDown({key:'Enter',repeat:false,nativeEvent:{isComposing:true},preventDefault(){},stopPropagation(){}});
assert.equal(events.filter(e=>e[0]==='searchPlace').length,searchesBefore,'IME Enter triggered a search');
searchInput.props.onKeyDown({key:'Enter',repeat:false,nativeEvent:{isComposing:false},preventDefault(){prevented=true;},stopPropagation(){}});
assert(prevented,'Enter search must suppress default key handling');
nodes=render();const freshRevision=JSON.parse(events.filter(e=>e[0]==='searchPlace').at(-1)[1]).revision;assert(freshRevision>oldRevision);
nodes.find(n=>n.type==='input').props.onKeyDown({key:'Enter',repeat:false,nativeEvent:{isComposing:false},preventDefault(){},stopPropagation(){}});
assert.equal(events.filter(e=>e[0]==='searchPlace').length,searchesBefore+1,'Enter repeated an in-flight search');
bindings.placeResults=JSON.stringify({revision:freshRevision,message:'Select a result.',results:[{label:'fresh city',latitude:45.123456789,longitude:17.123456789}]});nodes=render();
assert(!nodes.some(n=>n.props?.className==='searchSpinner'),'Completed search kept loading indicator');
assert(!nodes.some(n=>n.props?.children==='Select a result.'),'Redundant result instruction remains');
button(nodes,'fresh city').props.onSelect();nodes=render();assert.equal(nodes.filter(n=>n.type==='input')[1].props.value,'45.123456789');assert.equal(commits.length,1,'Search must remain draft-only');
assert(!nodes.some(n=>n.props?.className==='results'),'Selecting a result must dismiss results');
// Typing enters loading immediately, but only one search fires after 3.5s idle.
nodes.find(n=>n.type==='input').props.onChange({target:{value:'Marseille'}});nodes=render();
assert(nodes.some(n=>n.props?.className==='searchSpinner'));
assert.equal([...timers.values()].filter(t=>t.delay===3500).length,1);
nodes.find(n=>n.type==='input').props.onChange({target:{value:'Washington'}});nodes=render();
assert.equal([...timers.values()].filter(t=>t.delay===3500).length,1,'Typing must replace the debounce timer');
const beforeAuto=events.filter(e=>e[0]==='searchPlace').length;fireTimers(3500);nodes=render();
assert.equal(events.filter(e=>e[0]==='searchPlace').length,beforeAuto+1);
assert.equal(JSON.parse(events.filter(e=>e[0]==='searchPlace').at(-1)[1]).query,'Washington');
const autoRevision=JSON.parse(events.filter(e=>e[0]==='searchPlace').at(-1)[1]).revision;
bindings.placeResults=JSON.stringify({revision:autoRevision,results:[{label:'DC',latitude:38.9,longitude:-77}]});nodes=render();
assert(nodes.some(n=>n.props?.className==='results'));assert(!nodes.some(n=>n.props?.className==='searchSpinner'));
nodes.find(n=>n.props?.onClickCapture).props.onClickCapture({target:{closest:()=>null}});nodes=render();assert(nodes.some(n=>n.props?.className==='results'),'Scrolling results must not dismiss them');
nodes.find(n=>n.props?.onClickCapture).props.onClickCapture({target:{closest:selector=>selector==='[data-location-results]'?{}:null}});nodes=render();assert(nodes.some(n=>n.props?.className==='results'),'Result interaction must preserve results until selection');
nodes.find(n=>n.props?.onClickCapture).props.onClickCapture({target:{closest:selector=>selector==='input, button'?{}:null}});nodes=render();assert(!nodes.some(n=>n.props?.className==='results'),'Control interaction must dismiss results');
nodes.find(n=>n.type==='input').props.onChange({target:{value:''}});nodes=render();
assert(!nodes.some(n=>n.props?.className==='searchSpinner'));assert(![...timers.values()].some(t=>t.delay===3500));
props={...props,mapExt:true};nodes=render();assert.equal(button(nodes,'57km').props.disabled,false);button(nodes,'57km').props.onSelect();nodes=render();assert.equal(button(nodes,'57km').props.selected,true);
const coordinate=nodes.filter(n=>n.type==='input')[1];coordinate.props.onChange({target:{value:'44.123456789'}});nodes=render();nodes.filter(n=>n.type==='input')[1].props.onBlur();nodes=render();
fireTimers(300);nodes=render();const tileRevision=JSON.parse(events.filter(e=>e[0]==='previewTiles').at(-1)[1]).revision;bindings.displayTiles=JSON.stringify({revision:tileRevision,tiles:{},error:true});nodes=render();assert(nodes.some(n=>n.props?.children==='Map preview unavailable. Enter coordinates manually.'));
button(nodes,'Cancel').props.onSelect();assert.equal(commits.length,1);for(const hook of hooks)hook?.cleanup?.();assert(events.some(e=>e[0]==='cancelLocationServices'));
hooks=[];nodes=render();assert.equal(nodes.find(n=>n.type==='input').props.value,'');assert(!nodes.some(n=>n.props?.className==='results'||n.props?.className==='searchSpinner'),'Reopening must clear search state');nodes.find(n=>n.type==='input').props.onChange({target:{value:'Zadar'}});nodes=render();button(nodes,'Search').props.onSelect();const reopened=JSON.parse(events.filter(e=>e[0]==='searchPlace').at(-1)[1]).revision;assert(reopened>autoRevision,'Reopened picker reused stale revision');
console.log('PASS: coordinate precision/blur, projection, MapExt size selection, draft-only commit/cancel, debounced/manual search, loading/results lifecycle, stale/reopened response protection, failed preview fallback, request cancellation.');
