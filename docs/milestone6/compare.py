#!/usr/bin/env python3
"""Usage: compare.py OLD_PROBE NEW_PROBE SOURCE_ROOT OUTPUT_JSON.
Compile comparison.cpp with each revision's headers and core/backend archives first.
"""
from pathlib import Path
import hashlib
import json
import subprocess
import sys
import tempfile
old,new,source,output=map(lambda p:Path(p).resolve(),sys.argv[1:])
cases=[]
for name in ['empty-8-8','random-32-32-10','maze-32-32-2']:
    for seed in [42,7]: cases.append((name,8,0,seed,'lns-sat'))
for variant in ['initial-radius-2','fixed-step-2','increasing-step']:
    for seed in [42,7]: cases.append(('empty-8-8',8,0,seed,variant))
for seed in [42,7]: cases.append(('empty-8-8',4,1,seed,'lns-sat'))
cases += [('disconnected',1,0,42,'lns-sat'),('empty-8-8',33,0,42,'lns-sat')]
results=[]
with tempfile.TemporaryDirectory(prefix='lns-compare-') as directory:
    tmp=Path(directory)
    (tmp/'disconnected.map').write_text('type octile\nheight 1\nwidth 3\nmap\n.@.\n')
    (tmp/'disconnected.scen').write_text('version 1\n0\tdisconnected.map\t3\t1\t0\t0\t2\t0\t2\n')
    for name,n,index,seed,variant in cases:
        map_path=source/'mapf-map'/f'{name}.map'
        scenario=source/'mapf-scen-even/scen-even'/f'{name}-even-1.scen'
        if name=='disconnected': map_path,scenario=tmp/'disconnected.map',tmp/'disconnected.scen'
        args=list(map(str,[map_path,scenario,n,index,seed,variant]))
        pair=[]
        for executable in [old,new]:
            p=subprocess.run([str(executable),*args],cwd=tmp,text=True,capture_output=True,timeout=15,check=True)
            pair.append('\n'.join(line for line in p.stdout.splitlines() if line.startswith(('CONTRACT ','PATH ')))+'\n')
        assert pair[0]==pair[1],(args,pair)
        results.append({'map':name,'agents':n,'index':index,'seed':seed,'variant':variant,
                        'outcome':pair[1].splitlines()[0],
                        'contract_sha256':hashlib.sha256(pair[1].encode()).hexdigest(),'exact_match':True})
output.write_text(json.dumps(results,indent=2)+'\n')
print(f'PASS: {len(results)} before/after pairs match statuses, validity, makespans and every returned path')
