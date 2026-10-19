#!/usr/bin/env python3
"""Check the exact Git-visible source set and optionally resolve simulation URIs."""
import argparse
import ast
from pathlib import Path
import re
import subprocess
import xml.etree.ElementTree as ET

root = Path(__file__).resolve().parents[1]
parser = argparse.ArgumentParser(description=__doc__)
parser.add_argument('--with-assets', action='store_true', help='also resolve all model:// URIs locally')
args = parser.parse_args()
data = subprocess.check_output(['git', 'ls-files', '--cached', '--others', '--exclude-standard', '-z'], cwd=str(root))
names = sorted(set(filter(None, data.decode().split('\0'))))
failures = []
total = 0
for name in names:
    p = root / name
    if p.is_symlink():
        failures.append('Source symlink: ' + name)
        continue
    if not p.is_file():
        failures.append('Missing source file: ' + name)
        continue
    total += p.stat().st_size
    if name.split('/')[0] in ('build', 'devel', 'install', 'logs', 'external', 'dist'):
        failures.append('Generated/external file visible to Git: ' + name)
    if p.suffix in ('.pt', '.so', '.pyc', '.pyo', '.bag', '.ulg'):
        failures.append('Binary runtime artifact visible to Git: ' + name)
    if p.stat().st_size >= 50 * 1024**2:
        failures.append('Large source file: ' + name)
    if p.suffix in ('.py', '.sh', '.launch', '.xml', '.cpp', '.hpp', '.cc', '.hh', '.world', '.sdf', '.json', '.md'):
        text = p.read_text(errors='replace')
        if re.search(r'/home/(?:xhy|robin)/', text):
            failures.append('Original machine path: ' + name)
        if re.search(r'-----BEGIN (?:RSA |OPENSSH |EC )?PRIVATE KEY-----', text):
            failures.append('Private key material: ' + name)
        if p.suffix == '.py':
            try:
                ast.parse(text, filename=name)
            except SyntaxError as error:
                failures.append('Python syntax: %s: %s' % (name, error))
    if p.name == 'package.xml':
        manifest = ET.parse(str(p)).getroot()
        licenses = [n.text for n in manifest.findall('license')]
        if not licenses or any(n in (None, '', 'TODO') for n in licenses):
            failures.append('Missing package license: ' + name)
    if p.suffix in ('.launch', '.xml', '.world', '.sdf', '.config'):
        try:
            ET.parse(str(p))
        except ET.ParseError as error:
            failures.append('XML syntax: %s: %s' % (name, error))

if args.with_assets:
    model_roots = [root / 'src/swarm_simulation/models', root / 'assets/models']
    documents = list((root / 'src/swarm_simulation').rglob('*.world'))
    documents += list((root / 'src/swarm_simulation/models').rglob('*.sdf'))
    documents += list((root / 'assets/models').rglob('*.sdf'))
    for p in documents:
        # XML parsing omits commented-out sample material/model references.
        doc = ET.parse(str(p)).getroot()
        for text in doc.itertext():
            for path in re.findall(r'model://([^\s<]+)', text):
                if not any((base / path).exists() for base in model_roots):
                    failures.append('Unresolved model URI in %s: model://%s' % (p.relative_to(root), path))
if failures:
    raise SystemExit('\n'.join(failures))
print('Source audit passed: %d files, %.1f MiB, no source symlinks or bundled runtime binaries.' % (len(names), total / 1024**2))
if args.with_assets:
    print('Simulation model URIs resolved using packaged resources.')
