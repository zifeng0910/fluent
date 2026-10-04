"""Isolated contact campaign evidence. Does not write Benchmark C files."""
from pathlib import Path
from benchmark_C_recovery_v2_common import atomic, read, sha, stamp, native

ROOT = Path(__file__).resolve().parents[1]
EVID = ROOT / 'evidence/benchmark_D_contact_baseline'
OUT = ROOT / 'live_cases/benchmark_D_contact_baseline'
CAMPAIGN = 'BENCHMARK_D_FRICTIONLESS_CONTACT_BASELINE'
MASS = 8.904428864007322e-6
# Fluent SDOF products of inertia; off-diagonal tensor entries are -products.
INERTIA_DIAG = [7.257810693523445e-13, 3.929384741151496e-12, 3.9293847411514897e-12]
INERTIA_PRODUCTS = [-2.782693607479792e-28, 1.9327591150313555e-29, 2.4813784672159598e-29]

def initialize():
    EVID.mkdir(parents=True, exist_ok=True)
    OUT.mkdir(parents=True, exist_ok=True)

def state(status, **items):
    initialize()
    rec = read(EVID/'state.json')
    rec.update(campaign=CAMPAIGN, timestamp=stamp(), status=status, **items)
    atomic(EVID/'state.json', rec)
    return rec

def event(name, **items):
    import json
    initialize()
    with (EVID/'events.jsonl').open('a', encoding='utf-8') as fp:
        fp.write(json.dumps(dict(timestamp=stamp(), event=name, **items), allow_nan=False)+'\n')
