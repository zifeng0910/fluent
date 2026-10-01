"""Recovery gates only. Tests never launch Fluent or edit native physics."""
import copy,json,tempfile,unittest
from pathlib import Path
from unittest.mock import patch
import numpy as np
from scipy.spatial.transform import Rotation
import benchmark_C_recovery_v2_supervisor as sup
import benchmark_C_recovery_v2_restart as restart
from benchmark_C_recovery_v2_common import archive_history
from benchmark_C_recovery_v2_common import mpi_node_associations

class RecoverySafety(unittest.TestCase):
    def test_calibrated_guard_never_undercuts_observed_solve(self):
        p=sup.resource_policy(14.,19.48876190185547)
        self.assertGreaterEqual(p['required_guard_gib'],23.386514282226562)
        self.assertGreaterEqual(p['safety_reserve_gib'],3.)
    def test_quaternion_mapping_against_native_DT_Q_history(self):
        for r in archive_history():
            q=Rotation.from_rotvec([float(r[f'theta_{a}_rad']) for a in 'xyz']).as_quat()[[3,0,1,2]]
            expected=np.array([float(r[f'q{i}']) for i in range(4)])
            self.assertLess(min(max(abs(q-expected)),max(abs(q+expected))),1e-12)
    def test_velocity_mismatch_blocks_continuation(self):
        expected=restart.state_vector(restart.archived(32));a=copy.deepcopy(expected);a['omega'][2]+=.01
        self.assertEqual(restart.compare(a,expected)['omega']['status'],'FAIL')
    def test_step33_wrong_trajectory_rejected(self):
        r=copy.deepcopy(restart.archived(33));r['com_x_m']=str(float(r['com_x_m'])+.00001)
        with tempfile.TemporaryDirectory() as d,patch.object(restart,'OUT',Path(d)):
            with self.assertRaises(RuntimeError):restart.compare_step33(r)
            self.assertEqual(json.loads((Path(d)/'step33_recomputed_validation.json').read_text())['status'],'FAIL')
    def test_restart_deadline_persists_and_duplicate_lock_rejected(self):
        with tempfile.TemporaryDirectory() as d:
            root=Path(d);(root/'evidence/benchmark_C_longrun').mkdir(parents=True);out=root/'v2'
            with patch.object(sup,'ROOT',root),patch.object(sup,'OUT',out),patch.object(sup,'STATE',out/'state.json'):
                a=sup.Recovery();deadline=a.s['recovery_v2_deadline']
                with self.assertRaises(SystemExit):sup.Recovery()
                a.lock.close();b=sup.Recovery();self.assertEqual(b.s['recovery_v2_deadline'],deadline);b.lock.close()
    def test_solve_load_peak_higher_than_history_is_respected(self):
        self.assertAlmostEqual(sup.resource_policy(25.,19.5)['required_guard_gib'],30.)
    def test_service_spawned_MPI_node_requires_host_port_match(self):
        from types import SimpleNamespace
        class Host:
            pid=100
            def net_connections(self,kind):return [SimpleNamespace(laddr=SimpleNamespace(port=56236))]
        class Node:
            pid=200
            def __init__(self,port):self.port=port
            def cmdline(self):return ['fl_mpi2610.exe','-mport',f'127.0.0.1:127.0.0.1:{self.port}:0','node']
            def create_time(self):return 123.
        self.assertEqual(mpi_node_associations([Host()],[Node(56236)])[0]['registered_host_pid'],100)
        self.assertEqual(mpi_node_associations([Host()],[Node(12345)]),[])
if __name__=='__main__':unittest.main(verbosity=2)
