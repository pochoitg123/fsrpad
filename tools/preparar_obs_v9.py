"""Genera FSRpad_N8_OBS_v9.zip desde el proyecto v8. Python 3, sin dependencias."""
from pathlib import Path
from zipfile import ZipFile, ZIP_DEFLATED
import sys, re
src = Path(sys.argv[1]) if len(sys.argv)>1 else Path(__file__).with_name("FSRpad_N8_Pad_v8.zip")
out = src.with_name("FSRpad_N8_OBS_v9.zip")
if src.resolve()==out.resolve(): raise SystemExit("Selecciona el ZIP de la versión 8.")
with ZipFile(src) as z: files={n:z.read(n) for n in z.namelist()}
def member(t):
    found=[n for n in files if n.endswith("/"+t)]
    if len(found)!=1: raise ValueError("Falta "+t)
    return found[0]
web,cfg=member("web_pages.h"),member("ota_config.h")
s=files[web].decode("utf-8")
start=s.index('R"HTMLEOF(')+len('R"HTMLEOF(')
end=s.index(')HTMLEOF"',start)
p=s[start:end]
def once(old,new):
    global p
    if p.count(old)!=1: raise ValueError("El archivo no coincide con la v8: "+old[:60])
    p=p.replace(old,new,1)
p=re.sub(r"\.dot\.ok\{[^}]*\}",".dot.ok{background:#06b864;box-shadow:none}",p)
once("</style></head>", """
.obs-copy{margin:8px 0 0;background:#1b2932;color:#d6e9ef;border:1px solid #45606e;border-radius:7px;padding:9px 14px;cursor:pointer}
html.obs,html.obs body{margin:0!important;padding:0!important;width:100%!important;height:100%!important;background:transparent!important;overflow:hidden}
html.obs body>*:not(#tab-test-c),html.obs #tab-test-c>*:not(#layout-panel),html.obs #layout-panel>*:not(#test-pad){display:none!important}
html.obs #tab-test-c,html.obs #layout-panel,html.obs #test-pad{display:grid!important;place-items:center;width:100vw!important;height:100vh!important;max-width:none!important;max-height:none!important;margin:0!important;padding:0!important;border:0!important;background:transparent!important;box-shadow:none!important;pointer-events:none}
html.obs #test-pad svg.pad-svg{width:100%!important;height:100%!important;max-width:none!important;max-height:none!important}
</style><script>if(new URLSearchParams(location.search).get('obs')==='1')document.documentElement.classList.add('obs')</script></head>""")
once('<div class="test-pad-wrap" id="test-pad">','''<button class="obs-copy" type="button" onclick="copyObsLink()">Copiar enlace para OBS</button>
  <input id="obs-manual" aria-label="Enlace para OBS" readonly hidden onclick="this.select()">
  <div class="test-pad-wrap" id="test-pad">''')
once("let initialPiu=false;", r"""
const obsQuery=new URLSearchParams(location.search), isObs=document.documentElement.classList.contains('obs');
async function copyObsLink(){
  let u=new URL('/panel',location.href);
  try{const s=await (await fetch('/wifi/state',{cache:'no-store'})).json();if(s.ip&&s.ip!=='0.0.0.0')u.hostname=s.ip;else if(s.viaAP)u.hostname='192.168.4.1';}catch(e){}
  u.searchParams.set('obs','1');u.searchParams.set('layout',piuEnabled?'piu':'ddr');
  u.searchParams.set('skin',currentPadSkin());
  const colors=Array.from({length:piuEnabled?5:4},(_,i)=>String(sensorActivationColor(i,currentSkinConfig())).replace('#',''));
  u.searchParams.set('colors',colors.join(','));
  const field=document.getElementById('obs-manual');field.value=u.href;
  try{await navigator.clipboard.writeText(u.href);field.hidden=true;toast('Enlace de OBS copiado');return;}catch(e){}
  field.hidden=false;field.focus();field.select();
  try{if(document.execCommand('copy')){field.hidden=true;toast('Enlace de OBS copiado');return;}}catch(e){}
  toast('Copia la dirección seleccionada');
}
let initialPiu=false;""")
once("applyPiu(initialPiu);connect();", """
if(isObs){
  initialPiu=obsQuery.get('layout')==='piu';
  const id=obsQuery.get('skin');
  if(initialPiu)padSkin5=['piu','sl'].includes(id)?id:'piu';
  else padSkin4=['ddr','itg','sl'].includes(id)?id:'ddr';
  const colors=(obsQuery.get('colors')||'').split(',');
  if(colors.length===(initialPiu?5:4)&&colors.every(v=>/^[0-9a-f]{6}$/i.test(v)))
    sensorColors[(initialPiu?'5:':'4:')+(initialPiu?padSkin5:padSkin4)]=colors.map(v=>'#'+v);
}
applyPiu(initialPiu);connect();""")
once("localStorage.setItem('fsrpad.padView'", "if(!isObs)localStorage.setItem('fsrpad.padView'")
once("function sendJSON(o){if(ws", "function sendJSON(o){if(!isObs&&ws")
s=s[:start]+p+s[end:]
# Reutilizar el PNG oficial ya embebido y servido desde flash.
s=s.replace('</title>','</title><link rel="icon" type="image/png" href="/assets/logo.png?v=obs9">')
files[web]=s.encode()
c,n=re.subn(rb'(?m)^#define PAD_BUILD 8\b',b'#define PAD_BUILD 9',files[cfg])
if n!=1: raise ValueError("Se necesita la versión 8")
files[cfg]=c
with ZipFile(out,"w",ZIP_DEFLATED) as z:
    for name,data in files.items(): z.writestr(name,data)
with ZipFile(out) as z: assert z.testzip() is None
print(out.resolve())
