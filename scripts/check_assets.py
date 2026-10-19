#!/usr/bin/env python3
"""Verify a release payload using the versioned SHA-256 manifest."""
import argparse
import hashlib
import json
from pathlib import Path

root = Path(__file__).resolve().parents[1]
parser = argparse.ArgumentParser(description=__doc__)
parser.add_argument('--models-only', action='store_true')
parser.add_argument('--weights-only', action='store_true')
args = parser.parse_args()
manifest = json.loads((root / 'config/assets.json').read_text())
failures = []
checked = 0
for rel, info in manifest['files'].items():
    if args.models_only and not rel.startswith('assets/models/'):
        continue
    if args.weights_only and not rel.startswith('assets/weights/'):
        continue
    p = root / rel
    if not p.is_file():
        failures.append('missing: ' + rel)
        continue
    digest = hashlib.sha256()
    with p.open('rb') as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b''):
            digest.update(block)
    if p.stat().st_size != info['bytes'] or digest.hexdigest() != info['sha256']:
        failures.append('checksum mismatch: ' + rel)
    checked += 1
if failures:
    raise SystemExit('\n'.join(failures) + '\nInstall the matching release assets; see docs/installation.md.')
print('Verified %d asset files.' % checked)
