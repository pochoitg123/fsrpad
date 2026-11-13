"""Prepara los dos archivos para un servidor HTTPS estático."""
import argparse, hashlib, json, pathlib
p=argparse.ArgumentParser()
p.add_argument('bin',type=pathlib.Path)
p.add_argument('build',type=int)
p.add_argument('--out',type=pathlib.Path,default=pathlib.Path('publicar'))
a=p.parse_args()
model='FSRpad:ESP32-WROOM-32E-N8:v1'
b=a.bin.read_bytes()
if not (0<a.build<2**32 and 24<len(b)<=0x330000 and b[0]==0xe9 and b[12:14]==bytes(2) and model.encode() in b):
    p.error('Debe ser la aplicación .ino.bin de FSRpad N8, dentro de 3,1875 MiB.')
a.out.mkdir(exist_ok=True,parents=True)
(a.out/f'firmware-{a.build}.bin').write_bytes(b)
(a.out/'manifest.json').write_text(json.dumps(dict(model=model,build=a.build,size=len(b),sha256=hashlib.sha256(b).hexdigest()),indent=2))
