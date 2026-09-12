#!/usr/bin/env python3
"""Black-box process contracts. Every solve/verify has a 10-second external guard."""
import copy
import hashlib
import json
import os
from pathlib import Path
import resource
import shutil
import subprocess
import sys
import tempfile
import time

exe, fake, fixtures = map(lambda s: Path(s).resolve(), sys.argv[1:])
checks = 0

def run(args, code=0, binary=exe, env=None, preexec_fn=None):
    global checks
    p = subprocess.run([str(binary), *map(str,args)], text=True, capture_output=True,
                       timeout=10, env={**os.environ, **(env or {})}, preexec_fn=preexec_fn)
    assert p.returncode == code, (args, p.returncode, code, p.stdout, p.stderr)
    checks += 1
    return p

with tempfile.TemporaryDirectory(prefix='lns-public-cli-') as tmp:
    os.chdir(tmp)
    map_path, scenario = Path('map é " \\.map'), Path('scenario.scen')
    shutil.copyfile(fixtures/'empty-8-8.map',map_path)
    shutil.copyfile(fixtures/'empty-8-8-even-1.scen',scenario)
    inputs = ['--map',map_path,'--scenario',scenario]
    solve = ['solve',*inputs,'--agents','4']
    output = Path('result.json')
    verify = ['verify',*inputs,'--solution',output]
    assert 'Usage:' in run(['--help']).stdout
    version = run(['--version']).stdout
    assert 'schema=1' in version
    for option in ['--help','--version']:
        with open('/dev/full','w') as full:
            p=subprocess.run([str(exe),option],stdout=full,stderr=subprocess.PIPE,timeout=10)
            assert p.returncode==5
    copied=Path('relocated-solver');shutil.copy2(exe,copied)
    assert run(['--version'],binary=copied.resolve()).stdout==version
    build = json.loads(version.split('schema=1 ',1)[1])
    assert len(build['source_sha256']) == 64 and len(build['minisat']['source_sha256']) == 64
    assert build['compiler'] and build['configuration'] and build['platform']
    for args in [[],['--help','extra'],['solve'],['verify'],['--wat'],solve+['--solver','minisat'],solve+['--format','csv'],solve+['--map','again'],solve+['--agents'],solve+['--wat','1'],solve+['--log-level','trace'],solve+['--variant','bogus'],solve+['--variant','l!n!s'],solve+['--output',''],solve+['--seed','--output','x']]:
        run(args,2)
    for option, values in {
        '--agents':['0','-1','1.5','1x','2147483648','+4'],
        '--scenario-index':['-1','1e1','2147483648'],
        '--seed':['1.5','nan','2147483648','-2147483649','+1'],
        '--makespan-increment':['0','-1','1x'],
        '--makespan-increase-limit':['-1','1.5'],
        '--makespan-bound':['-1','inf','2147483648'],
        '--wall-clock-limit-ms':['0','-1','1.2','1e300'],
        '--lazy-iteration-limit':['0','-1','1e2'],
        '--full-map-fallback-threshold':['0','-0.1','1.1','nan','inf','1e999','0x1p-1','+0.5',' 0.5','0.5x'],
    }.items():
        for value in values:
            args = ['solve',*inputs] if option == '--agents' else solve
            run([*args,option,value],2)
    config = Path('settings.ini')
    required = f'map={map_path}\nscenario={scenario}\nnum_agents=4\n'
    for suffix in ['unknown=1\n','seed=1\nseed=2\n','seed=nan\n','seed=\n','variant=wrong\n','makespan_bound=-1\n','solver=minisat\n','this is not key=value\n']:
        config.write_text(required+suffix)
        run(['solve','--config',config],2)
    config.write_text(required+'seed=7\nscenario_index=1\nlog_level=quiet\n')
    run(['solve','--seed','42','--config',config,'--output',output])
    doc = json.loads(output.read_text())
    assert doc['configuration']['seed'] == 42
    assert doc['instance']['first_row'] == 4 and doc['instance']['last_row_exclusive'] == 8
    run(verify)
    run([*solve,'--scenario-index','8','--output',output],2)
    assert json.loads(output.read_text())['solution'] is None
    run([*solve,'--scenario-index','2147483647'],2)
    # Quiet and ordinary solve must not create undeclared files.
    before = set(Path('.').iterdir())
    p = run([*solve,'--log-level','quiet'])
    assert not p.stdout and not p.stderr
    assert set(Path('.').iterdir()) == before
    info = run(solve)
    debug = run([*solve,'--log-level','debug'])
    assert not info.stdout and not debug.stdout
    assert 'Loaded map' in info.stderr and '[LNS] Map:' not in info.stderr
    assert '[LNS] Map:' in debug.stderr and 'Agent 0' in debug.stderr
    # Explicit legacy diagnostics are opt-in and no longer capture stdout.
    config.write_text(required+'log=diagnostic.log\nlog_level=debug\n')
    p = run(['--config',config])
    assert not p.stdout and '[LNS] Map:' in Path('diagnostic.log').read_text()
    assert not Path('logs').exists()
    # All four variants preserve a verified saved result.
    for variant in ['lns-sat','initial-radius-2','fixed-step-2','increasing-step']:
        run([*solve,'--variant',variant,'--output',output,'--log-level','quiet'])
        run(verify)
    run([*solve,'--makespan-bound','7','--output',output,'--log-level','quiet'],1)
    run([*solve,'--makespan-bound','8','--makespan-increment','3','--makespan-increase-limit','0','--output',output,'--log-level','quiet'])
    run([*solve,'--output',output,'--log-level','quiet'])
    doc = json.loads(output.read_text())
    assert doc['status']=='solved' and doc['verification']=='passed' and doc['makespan']==8
    assert [a['id'] for a in doc['solution']['agents']]==list(range(4))
    assert doc['instance']['map_sha256']==hashlib.sha256(map_path.read_bytes()).hexdigest()
    assert doc['instance']['scenario_sha256']==hashlib.sha256(scenario.read_bytes()).hexdigest()
    assert doc['runtime']['total_ms'] >= doc['runtime']['solver_ms'] >= 0
    assert doc['configuration']['wall_clock_limit_ms'] is None
    run(verify,binary=fake)  # would throw if verification called the solve function
    def mutate(fn,code=2):
        d=copy.deepcopy(doc); fn(d); output.write_text(json.dumps(d)); return run(verify,code)
    mutate(lambda d:d.update(schema_version=2))
    mutate(lambda d:d.update(schema_version=1.0))
    for key in doc:
        mutate(lambda d,k=key:d.pop(k))
    mutate(lambda d:d.update(unknown=1))
    mutate(lambda d:d['solution']['agents'].pop())
    mutate(lambda d:d['solution']['agents'][1].update(id=0))
    mutate(lambda d:d['solution']['agents'][0]['positions'].pop())
    mutate(lambda d:d['solution'].update(horizon=5))
    mutate(lambda d:d['solution']['agents'][0]['positions'][0].__setitem__(0,0.0))
    mutate(lambda d:d['solution']['agents'][0]['positions'][0].__setitem__(0,True))
    mutate(lambda d:d['solution']['agents'][0]['positions'][0].__setitem__(0,'0'))
    mutate(lambda d:d['solution']['agents'][0]['positions'][0].__setitem__(0,2147483648))
    mutate(lambda d:d['solution']['agents'][0].update(id=99),4)
    mutate(lambda d:d['solution']['agents'][0]['positions'][0].__setitem__(0,-2147483648),4)
    mutate(lambda d:d['solution']['agents'][0]['positions'].__setitem__(1,[7,7]),4)
    mutate(lambda d:[a.update(positions=[xy[::-1] for xy in a['positions']]) for a in d['solution']['agents']],4)
    mutate(lambda d:d.update(makespan=7),4)
    mutate(lambda d:d['instance'].update(scenario_index=1),4)
    mutate(lambda d:d['instance'].update(first_row=4,last_row_exclusive=8,scenario_index=1),4)
    mutate(lambda d:d['instance'].update(map_sha256='0'*64),4)
    mutate(lambda d:d['instance'].update(scenario_sha256='0'*64),4)
    mutate(lambda d:d['solution']['agents'][1].update(positions=copy.deepcopy(d['solution']['agents'][0]['positions'])),4)
    mutate(lambda d:d.update(verification='failed'),0)  # stored flags are not trusted
    mutate(lambda d:d['solution']['agents'].reverse(),0) # reader permits any ordering
    for text in ['{',json.dumps(doc)[:-3],json.dumps(doc)+'junk',json.dumps(doc).replace('"schema_version": 1','"schema_version": 1, "schema_version": 1'),json.dumps(doc).replace('"schema_version": 1','"schema_version": NaN')]:
        output.write_text(text);run(verify,2)
    # Independent vertex and edge-swap conflict diagnostics on a 2x2 fixture.
    Path('tiny.map').write_text('type octile\nheight 2\nwidth 2\nmap\n..\n..\n')
    Path('tiny.scen').write_text('version 1\n0\ttiny.map\t2\t2\t0\t0\t1\t0\t1\n0\ttiny.map\t2\t2\t1\t0\t0\t0\t1\n')
    run(['solve','--map','tiny.map','--scenario','tiny.scen','--agents','2','--output',output,'--log-level','quiet'])
    tiny=json.loads(output.read_text()); tiny['makespan']=1;tiny['solution']['horizon']=2
    tiny['solution']['agents'][0]['positions']=[[0,0],[0,1]]
    tiny['solution']['agents'][1]['positions']=[[0,1],[0,0]]
    output.write_text(json.dumps(tiny))
    p=run(['verify','--map','tiny.map','--scenario','tiny.scen','--solution',output],4)
    assert 'edge_conflict' in p.stderr
    tiny['solution']['agents'][1]['positions']=[[0,0],[0,1]];output.write_text(json.dumps(tiny))
    p=run(['verify','--map','tiny.map','--scenario','tiny.scen','--solution',output],4)
    assert 'vertex_conflict' in p.stderr
    # Stale success is replaced for exhausted/invalid/internal outcomes.
    for args,code in [([*solve,'--makespan-bound','0'],1),(['solve','--map','missing.map','--scenario',scenario,'--agents','4'],2)]:
        output.write_text(json.dumps(doc)); run([*args,'--output',output],code)
        failed=json.loads(output.read_text());assert failed['solution'] is None and failed['makespan'] is None
        assert failed['verification']=='not_performed'
        if code==2: assert failed['instance']['map_sha256'] is None and failed['instance']['scenario_sha256']
        assert failed['runtime']['total_ms']>=0
    run([*solve,'--output',output],3,binary=fake)
    failure=json.loads(output.read_text());assert failure['status']=='invalid_state' and failure['solution'] is None
    run([*solve,'--output',output],3,binary=fake,env={'LNS_TEST_MODE':'corrupt'})
    assert json.loads(output.read_text())['verification']=='failed'
    t=time.monotonic()
    run([*solve,'--wall-clock-limit-ms','1','--output',output],1,binary=fake,env={'LNS_TEST_MODE':'late'})
    timed=json.loads(output.read_text());assert timed['runtime']['solver_ms']>=10 and timed['runtime']['total_ms']>=10
    assert time.monotonic()-t >= .01 and timed['termination_reason']=='wall_clock_limit'
    # Output protection and failure cleanup, including kernel write failure.
    original=map_path.read_bytes()
    for destination in [map_path,scenario,'.','missing-parent/result.json','/dev/full']:
        run([*solve,'--output',destination],5)
    Path('map-symlink').symlink_to(map_path)
    os.link(map_path,'map-hardlink')
    for destination in ['map-symlink','map-hardlink']:
        run([*solve,'--output',destination],5)
    assert map_path.read_bytes()==original
    config.write_text(required+'log=both.json\n')
    run(['solve','--config',config,'--output','both.json'],5)
    run(['solve','--config',config,'--output',config],5)
    output.write_text(json.dumps(doc))
    def limit_output():
        import signal
        signal.signal(signal.SIGXFSZ,signal.SIG_IGN)
        resource.setrlimit(resource.RLIMIT_FSIZE,(16,16))
    run([*solve,'--output',output],5,preexec_fn=limit_output)
    assert not output.exists() and not list(Path('.').glob('*.tmp.*'))
    # Hashes bind the snapshot actually used even when the source file changes during solve.
    run([*solve,'--output',output],binary=fake,env={'LNS_TEST_MODE':'snapshot'})
    snap=json.loads(output.read_text());assert snap['instance']['map_sha256']==hashlib.sha256(original).hexdigest()
    run(verify,4)
    map_path.write_bytes(original);run(verify)
    assert not Path('logs').exists()
print(f'PASS: {checks} public CLI subprocess contracts')
