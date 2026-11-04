from pathlib import Path
Import('env')
root = Path(env['PROJECT_DIR'])
out = root / 'include' / 'web_assets.h'
out.parent.mkdir(exist_ok=True)
entries = [('index.html','text/html; charset=utf-8'),('style.css','text/css'),('app.js','application/javascript'),('transport.mjs','application/javascript')]
body = '#pragma once\n#include <Arduino.h>\nstruct WebAsset {const char* path;const char* mime;const char* data;size_t size;};\n'
assets=[]
for i,(name,mime) in enumerate(entries):
    path=root/'web'/name
    data=path.read_bytes() if path.exists() else b'Web interface not built yet'
    body += 'static const char asset_%d[] PROGMEM = {' % i + ','.join(str(c) for c in data) + ',0};\n'
    assets.append('{"/%s","%s",asset_%d,%d}' % (name,mime,i,len(data)))
body += 'static const WebAsset webAssets[] = {' + ','.join(assets) + '};\n'
out.write_text(body,encoding='utf-8')
