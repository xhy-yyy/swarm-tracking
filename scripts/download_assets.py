#!/usr/bin/env python3
"""Download release payloads from a supplied GitHub Release URL or local directory."""
import argparse
import hashlib
import json
from pathlib import Path
import shutil
import tarfile
import tempfile
from urllib.request import urlopen

root = Path(__file__).resolve().parents[1]
parser = argparse.ArgumentParser(description=__doc__)
parser.add_argument('source', help='Release download URL or directory containing the two asset archives')
args = parser.parse_args()
checksums = json.loads((root / 'config/release_assets.json').read_text())
with tempfile.TemporaryDirectory(prefix='swarm-assets-') as directory:
    for name, expected in checksums.items():
        target = Path(directory) / name
        local = Path(args.source) / name
        if local.is_file():
            shutil.copyfile(str(local), str(target))
        else:
            with urlopen(args.source.rstrip('/') + '/' + name, timeout=120) as response, target.open('wb') as stream:
                shutil.copyfileobj(response, stream)
        if hashlib.sha256(target.read_bytes()).hexdigest() != expected:
            raise SystemExit('Archive checksum mismatch: ' + name)
        with tarfile.open(str(target), 'r:gz') as archive:
            members = archive.getmembers()
            for member in members:
                relative = Path(member.name)
                if relative.is_absolute() or '..' in relative.parts or member.issym() or member.islnk():
                    raise SystemExit('Unsafe archive entry: ' + member.name)
                if not (member.isfile() or member.isdir()):
                    raise SystemExit('Unsupported archive entry: ' + member.name)
                if relative.parts[:2] not in (('assets', 'weights'), ('assets', 'models')):
                    raise SystemExit('Unexpected asset entry: ' + member.name)
            archive.extractall(str(root), members=members)
        print('Installed ' + name)
