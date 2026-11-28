from pathlib import Path
import hashlib,re,struct,subprocess,sys,tempfile,zipfile
from zlib import crc32
b=Path('ota/firmware-8.bin').read_bytes()
assert hashlib.sha256(b).hexdigest()=='980fdf7ecf41a2a34c0e1ef1badf36f4f58f82fb8b31e722f5564c21eddd3235'
pages={}
for m in re.finditer(b'<!doctype html>',b,re.I):
    n=b.find(b'\0',m.start())
    if n<0: continue
    data=b[m.start():n]
    if b'</html>' not in data: continue
    for tag,var,mark in [('SPLASH','SPLASH_HTML',b'keep-network'),('OTA','UPDATE_PAGE',b'@BUILD@'),('HTMLEOF','CONFIG_HTML',b'id="layout-ddr"'),('WIFISETUP','WIFI_SETUP_HTML',b'id="host-preview"')]:
        if mark in data:
            assert var not in pages
            pages[var]='static const char '+var+'[] PROGMEM = R"'+tag+'('+data.decode()+')'+tag+'";\n'
assert len(pages)==4,pages.keys()
dims={(1360,164):'BRAND_ARROWS',(384,256):'PAD_LOGO',(768,768):'PAD_DDR',(737,768):'PAD_ITG',(260,260):'PAD_SL',(718,768):'PAD_PIU'}
imgs={}
for m in re.finditer(re.escape(b'\x89PNG\r\n\x1a\n'),b):
    i=m.start();p=i+8;wh=struct.unpack_from('>II',b,p+8)
    if wh not in dims: continue
    while p+12<=len(b):
        n,typ=struct.unpack_from('>I4s',b,p)
        assert p+n+12<=len(b) and crc32(b[p+4:p+8+n])==int.from_bytes(b[p+8+n:p+12+n],'big')
        p+=12+n
        if typ==b'IEND':break
    assert typ==b'IEND'
    name=dims[wh]+'_PNG';raw=b[i:p]
    imgs[name]='static const unsigned char '+name+'[] PROGMEM={\n'+''.join(','.join('0x%02x'%v for v in raw[j:j+24])+',\n' for j in range(0,len(raw),24))+'};\n'
assert len(imgs)==6,imgs.keys()
with tempfile.TemporaryDirectory() as td:
    tmp=Path(td);src=tmp/'src.zip'
    with zipfile.ZipFile(src,'w') as z:
        z.writestr('FSRpad_N8/web_pages.h','#pragma once\n'+''.join(pages.values())+''.join(imgs.values()))
        z.writestr('FSRpad_N8/ota_config.h','#define PAD_BUILD 8\n')
    subprocess.run([sys.executable,'tools/preparar_obs_v9.py',str(src)],check=True)
    with zipfile.ZipFile(tmp/'FSRpad_N8_OBS_v9.zip') as z:
        h=z.read('FSRpad_N8/web_pages.h')
    assert b'Copiar enlace para OBS' in h and b'background:#06b864' in h
    for i,js in enumerate(re.findall(rb'<script>(.*?)</script>',h,re.S)):
        f=tmp/f'script{i}.js';f.write_bytes(js);subprocess.run(['node','--check',str(f)],check=True)
    Path('web_pages.h').write_bytes(h)
