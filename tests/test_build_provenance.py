#!/usr/bin/env python3
"""Exercise the build-time generator, including archives and dirty rebuilds."""
import json
import shutil
import subprocess
import sys
import tempfile
from pathlib import Path
cmake, script = sys.argv[1:]
with tempfile.TemporaryDirectory(prefix='lns-provenance-') as tmp:
    root=Path(tmp)/'archive';root.mkdir()
    (root/'src').mkdir();(root/'src'/'unit.cpp').write_text('first source\n')
    (root/'CMakeLists.txt').write_text('example\n');(root/'Makefile').write_text('example\n')
    settings=Path(tmp)/'settings.json';settings.write_text('"compiler":"test compiler","platform":"test platform","configuration":"test flags"')
    header=Path(tmp)/'BuildProvenance.h'
    def generate():
        subprocess.run([cmake,f'-DROOT={root}',f'-DSETTINGS={settings}',f'-DOUT={header}','-P',script],check=True,capture_output=True,timeout=10)
        return json.loads(header.read_text().split('R"lns(',1)[1].split(')lns"')[0])
    archive=generate();assert archive['revision'] is None and archive['dirty'] is None
    assert len(archive['source_sha256'])==64
    timestamp=header.stat().st_mtime_ns;assert generate()==archive and header.stat().st_mtime_ns==timestamp
    if shutil.which('git'):
        def git(*args):
            return subprocess.run(['git','-c','commit.gpgsign=false','-c','core.hooksPath=/dev/null','-C',str(root),*args],check=True,capture_output=True,text=True,timeout=10).stdout.strip()
        git('init');git('add','.');git('-c','user.name=Test','-c','user.email=test@example.invalid','commit','-m','fixture')
        clean=generate();assert clean['dirty'] is False and clean['revision']==git('rev-parse','HEAD')
        (root/'src'/'unit.cpp').write_text('second source\n')
        dirty=generate();assert dirty['dirty'] is True and dirty['revision']==clean['revision']
        assert dirty['source_sha256']!=clean['source_sha256']
        git('add','.');git('-c','user.name=Test','-c','user.email=test@example.invalid','commit','-m','fixture update')
        committed=generate();assert committed['revision']!=clean['revision'] and committed['dirty'] is False
    print('PASS: archive fallback, build-time revision and dirty state, source identity, unchanged-header stability')
