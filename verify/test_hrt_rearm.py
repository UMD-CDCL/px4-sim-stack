"""Exercise the checked-out POSIX timer code with deterministic rearm ordering.

Only the clock, mutex and underlying wakeup are stubbed. The actual HRT
insertion/invocation routines and PX4 queue functions run in the harness.
Rearming inside the callback reproduces the ordering of a worker that rearms
before the timer thread reacquires its lock, without a probabilistic race.
"""
from pathlib import Path
import shutil
import subprocess

import pytest

ROOT = Path(__file__).resolve().parents[1]
PX4 = ROOT / 'src/PX4-Autopilot'


@pytest.fixture(scope='module')
def harness(tmp_path_factory):
    if not shutil.which('g++') or not (PX4 / 'Makefile').exists():
        pytest.skip('requires g++ and the PX4 checkout from px4sim setup px4')
    source = (PX4 / 'platforms/posix/src/px4/common/drv_hrt.cpp').read_text()
    functions = []
    for start, stop in [
        ('static void\nhrt_call_enter(', '/**\n * Timer interrupt'),
        ('static void\nhrt_call_internal(', '/*\n * Call callout(arg) after'),
        ('static void\nhrt_call_invoke()', 'int px4_clock_gettime'),
    ]:
        begin = source.index(start)
        functions.append(source[begin:source.index(stop, begin)])
    directory = tmp_path_factory.mktemp('hrt-rearm')
    generated = directory / 'timer.inc'
    generated.write_text('\n'.join(functions))
    binary = directory / 'hrt-rearm'
    subprocess.run([
        'g++', '-std=c++17', '-Wall', '-Wextra',
        '-I', str(PX4 / 'platforms/posix/include'), '-I', str(directory),
        str(ROOT / 'verify/fixtures/hrt_rearm.cpp'),
        *(str(PX4 / 'platforms/common/work_queue' / name)
          for name in ['queue.c', 'sq_addafter.c', 'sq_addlast.c']),
        '-o', str(binary),
    ], check=True, capture_output=True, text=True)
    return binary


@pytest.mark.parametrize('case', ['normal', 'periodic_rearm', 'one_shot_rearm', 'cancel', 'delay'])
def test_rearming_preserves_queue_and_requested_timing(harness, case):
    result = subprocess.run([str(harness), case], capture_output=True, text=True, timeout=5)
    assert result.returncode == 0, result.stdout + result.stderr
