"""Compile an isolated experiment using existing build flags and current core source."""
from pathlib import Path
import shlex
import subprocess
import hashlib
import json
import os

here=Path(__file__).resolve().parent
ws=here.parents[1]
build=ws/'build/arena_multi_hunav_core'
flags=(build/'CMakeFiles/multi_sfm_benchmark.dir/flags.make').read_text()
includes=shlex.split(next(x.split('=',1)[1] for x in flags.splitlines() if x.startswith('CXX_INCLUDES =')))
link=shlex.split((build/'CMakeFiles/multi_sfm_benchmark.dir/link.txt').read_text())
libs=link[link.index('libmulti_sfm.a')+1:]
vendor=ws/'src/arena_multi_hunav_core/vendor/lightsfm/include'
core=ws/'src/arena_multi_hunav_core/src/core.cpp'
subprocess.run([link[0],'-std=c++17','-O2',*includes,'-I'+str(vendor),str(here/'compare.cpp'),str(core),'-o',str(here/'compare'),*libs],check=True,cwd=build)
env=os.environ.copy()
env['LD_LIBRARY_PATH']=':'.join(sorted({str(Path(p).parent) for p in libs if p.startswith('/')})+[env.get('LD_LIBRARY_PATH','')])
for scenario in ['symmetric','staggered','wide']:
    output=here/scenario
    output.mkdir(exist_ok=True)
    subprocess.run([str(here/'compare'),str(output)]+([scenario] if scenario!='symmetric' else []),check=True,env=env)
files=[core,here/'compare.cpp',vendor/'sfm.hpp',Path('/home/lpc/workspace/arena5_ws/src/deps/hunav/lightsfm/include/sfm.hpp')]
(here/'manifest.json').write_text(json.dumps({str(p):hashlib.sha256(p.read_bytes()).hexdigest() for p in files},indent=2)+'\n')
print('Saved symmetric, staggered, wide trajectories in',here)
