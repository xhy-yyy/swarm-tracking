#!/usr/bin/env python3
"""Create a clean source archive and two separate GitHub Release payloads."""
import gzip
import hashlib
import json
from pathlib import Path
import subprocess
import tarfile

root = Path(__file__).resolve().parents[1]
dist = root / 'dist'
dist.mkdir(exist_ok=True)

def pack(name, files, prefix=''):
    target = dist / name
    with target.open('wb') as raw:
        with gzip.GzipFile(filename='', mode='wb', fileobj=raw, mtime=0) as zipped:
            with tarfile.open(fileobj=zipped, mode='w') as archive:
                for p in sorted(files):
                    info = archive.gettarinfo(str(p), arcname=prefix + str(p.relative_to(root)))
                    info.uid = info.gid = info.mtime = 0
                    info.uname = info.gname = ''
                    with p.open('rb') as stream:
                        archive.addfile(info, stream)
    print('%s (%.1f MiB)' % (target.name, target.stat().st_size / 1024**2))
    return hashlib.sha256(target.read_bytes()).hexdigest()

checksums = {}
for group in ['models', 'weights']:
    files = [p for p in (root / 'assets' / group).rglob('*') if p.is_file() and p.name != '.gitkeep']
    if not files:
        raise SystemExit('Missing local assets for ' + group)
    name = 'swarm-tracking-%s-v0.1.0.tar.gz' % group
    checksums[name] = pack(name, files)
(root / 'config/release_assets.json').write_text(json.dumps(checksums, indent=2) + '\n')

output = subprocess.check_output(['git', 'ls-files', '--cached', '--others', '--exclude-standard', '-z'], cwd=str(root))
files = sorted({root / rel for rel in output.decode().split('\0') if rel})
files = [p for p in files if p.is_file() and not p.is_symlink()]
checksums['swarm-tracking-source-v0.1.0.tar.gz'] = pack(
    'swarm-tracking-source-v0.1.0.tar.gz', files, 'swarm-tracking/')
(dist / 'SHA256SUMS').write_text(''.join('%s  %s\n' % (sha, name) for name, sha in sorted(checksums.items())))
