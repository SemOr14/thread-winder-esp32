from pathlib import Path
import shutil
root = Path(__file__).resolve().parent.parent
for name in ('index.html','app.js','style.css','transport.mjs'):
    shutil.copy2(root/'web'/name, root/'docs'/name)
(root/'docs'/'.nojekyll').touch()
print('GitHub Pages files refreshed in docs/')
