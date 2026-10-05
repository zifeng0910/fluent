"""Ownership and deadline regressions; never starts Fluent or scheduled tasks."""
import contextlib,json,sys,tempfile,time,unittest
from pathlib import Path
from unittest.mock import patch
import benchmark_D_native_contact_overnight_supervisor as s

class HaltLoop(Exception):pass

class ControlTests(unittest.TestCase):
    def test_job_process_name_does_not_collide_with_event_name(self):
        with tempfile.TemporaryDirectory() as directory:
            root=Path(directory);evid=root/'evidence';evid.mkdir()
            def event(name,**data):s.atomic(evid/'event.json',dict(event=name,**data))
            with patch.object(s,'ROOT',root),patch.object(s,'EVID',evid),patch.object(s,'checked_code'),\
                 patch.object(s,'native',return_value=[]),patch.object(s,'event',side_effect=event),\
                 patch.object(s.subprocess,'Popen') as popen,patch.object(s.psutil,'Process') as process:
                popen.return_value.pid=999999;process.return_value.create_time.return_value=10
                process.return_value.name.return_value='pythonw.exe'
                s.launch('worker',s.WORKER,['--branch','micro25'],'micro25')
            self.assertEqual(s.read(evid/'event.json')['process_identity']['name'],'pythonw.exe')

    def test_control_can_record_supervisor_closed(self):
        with tempfile.TemporaryDirectory() as directory:
            evid=Path(directory)
            with patch.object(s,'EVID',evid):s.control('COMPLETE',supervisor_alive=False)
            self.assertFalse(s.read(evid/'supervisor_state.json')['supervisor_alive'])

    def test_second_native_solver_is_refused_before_popen(self):
        with patch.object(s,'checked_code'),patch.object(s,'native',return_value=[object()]),patch.object(s.subprocess,'Popen') as popen:
            with self.assertRaisesRegex(RuntimeError,'second launch'):
                s.launch('worker',s.WORKER,['--branch','micro25'],'micro25')
            popen.assert_not_called()

    def test_fallback_controller_cannot_launch_with_existing_engine(self):
        with patch.object(s,'checked_code'),patch.object(s,'native',return_value=[object()]),patch.object(s.subprocess,'Popen') as popen:
            with self.assertRaisesRegex(RuntimeError,'second launch'):
                s.launch('fallback',s.FALLBACK,['--run'])
            popen.assert_not_called()

    def test_completion_preserves_measured_load_route(self):
        gates={key:True for key in ['selected_contact_dt_PASS','full_selected_branch_PASS','actual_native_2ms',
            'controlled_load_response_PASS','native_2ms_checkpoint_PASS','owned_engine_closed']}
        report=dict(route='SDOF_LOAD_PATH',status='NUMERICAL_PASS',numerical_gates=gates,
            checkpoint_1p9ms={'status':'PASS','time_s':.0019},checkpoint_2ms={'status':'PASS','time_s':.002})
        with patch.object(s,'read',return_value={'status':'PASS'}):
            self.assertEqual(s.verified_completion_result(report),'BENCHMARK_D_SDOF_LOAD_CONTACT_2MS_PASS')
        report['route']='COMPLIANT_LOAD_PATH'
        with patch.object(s,'read',return_value={'status':'PASS'}):
            self.assertEqual(s.verified_completion_result(report),'BENCHMARK_D_COMPLIANT_LOAD_CONTACT_2MS_PASS')

    def test_completion_rejects_unmeasured_load_response(self):
        gates={key:True for key in ['selected_contact_dt_PASS','full_selected_branch_PASS','actual_native_2ms',
            'controlled_load_response_PASS','native_2ms_checkpoint_PASS','owned_engine_closed']}
        report=dict(route='SDOF_LOAD_PATH',status='NUMERICAL_PASS',numerical_gates=gates,
            checkpoint_1p9ms={'status':'PASS','time_s':.0019},checkpoint_2ms={'status':'PASS','time_s':.002})
        with patch.object(s,'read',return_value={'status':'NOT_REVIEWED'}):
            with self.assertRaisesRegex(RuntimeError,'Measured controlled'):s.verified_completion_result(report)

    def test_dead_fallback_helper_adopts_resource_waiting_child(self):
        from unittest.mock import Mock
        with tempfile.TemporaryDirectory() as directory:
            root=Path(directory);evid=root/'evidence';out=root/'live';b=evid/'branches/load_micro25'
            b.mkdir(parents=True);out.mkdir()
            s.atomic(evid/'window.json',{'hard_deadline_epoch':time.time()+600})
            s.atomic(evid/'state.json',{'status':'FALLBACK_LOAD_VALIDATION','route':'SDOF_LOAD_PATH'})
            s.atomic(evid/'active_job.json',{'kind':'fallback','key':'dead-helper','pid':123,'created':12.5,'name':'pythonw.exe'})
            s.atomic(b/'worker_identity.json',{'pid':456,'created':20,'name':'pythonw.exe'})
            s.atomic(b/'configuration.json',{'route':'SDOF_LOAD_PATH'})
            s.atomic(b/'worker_state.json',{'status':'RESOURCE_WAIT','solver_alive':False,'worker_alive':True})
            worker=Mock();worker.cmdline.return_value=['pythonw.exe',s.WORKER]
            def identity(rec):return worker if rec.get('pid')==456 else None
            def state(status,**items):
                record=s.read(evid/'state.json');record.update(status=status,**items);s.atomic(evid/'state.json',record)
            with patch.object(s,'EVID',evid),patch.object(s,'OUT',out),patch.object(s,'identity',side_effect=identity),\
                 patch.object(s,'native',return_value=[]),patch.object(s,'event'),patch.object(s,'state',side_effect=state),\
                 patch.object(s.time,'sleep',side_effect=HaltLoop),patch.object(s.subprocess,'Popen') as popen,\
                 patch('sys.stdout',sys.stdout),patch('sys.stderr',sys.stderr):
                with self.assertRaises(HaltLoop):s.main()
                popen.assert_not_called();sys.stdout.close();sys.stderr.close()
            active=s.read(evid/'active_job.json')
            self.assertEqual(active['kind'],'fallback_worker');self.assertEqual(active['pid'],456)
            self.assertEqual(s.read(evid/'supervisor_state.json')['status'],'RESOURCE_WAIT')

    def deadline_running_job(self,kind):
        with tempfile.TemporaryDirectory() as directory:
            root=Path(directory);evid=root/'evidence';out=root/'live';evid.mkdir();out.mkdir()
            s.atomic(evid/'window.json',{'hard_deadline_epoch':time.time()-1})
            s.atomic(evid/'state.json',{'status':'NATIVE_MICRORUN_25US'})
            s.atomic(evid/'active_job.json',{'kind':kind,'branch':'micro25' if kind in {'worker','fallback_worker'} else None,
                'key':'owned-job','pid':123,'created':12.5,'name':'pythonw.exe'})
            def state(status,**items):
                record=s.read(evid/'state.json');record.update(status=status,**items);s.atomic(evid/'state.json',record)
            with patch.object(s,'EVID',evid),patch.object(s,'OUT',out),patch.object(s,'state',side_effect=state),\
                 patch.object(s,'event'),patch.object(s,'identity',return_value=object()),\
                 patch.object(s.time,'sleep',side_effect=HaltLoop),patch.object(s.subprocess,'Popen') as popen,\
                 patch('sys.stdout',sys.stdout),patch('sys.stderr',sys.stderr):
                with self.assertRaises(HaltLoop):s.main()
                popen.assert_not_called()
                sys.stdout.close();sys.stderr.close()
            self.assertEqual(s.read(out/'stop_request.json')['reason'],'TIME_BUDGET_EXHAUSTED')
            self.assertEqual(s.read(evid/'state.json')['status'],'DEADLINE_GRACEFUL_CHECKPOINT_WAIT')

    def test_deadline_only_requests_owned_worker_checkpoint(self):self.deadline_running_job('worker')
    def test_deadline_also_stops_owned_fallback_without_new_solve(self):self.deadline_running_job('fallback')
    def test_deadline_stops_detached_fallback_worker(self):self.deadline_running_job('fallback_worker')

    def test_resource_failure_does_not_disprove_native_contact(self):
        with tempfile.TemporaryDirectory() as directory:
            evid=Path(directory)
            with patch.object(s,'EVID',evid),patch.object(s,'fail') as fail,patch.object(s,'launch') as launch:
                result=s.native_failure('micro25',{'failure_class':'RESOURCE_HARD_STOP','status':'FAIL'})
                self.assertIsNone(result);launch.assert_not_called();fail.assert_called_once()

    def test_lifecycle_uses_latest_verified_native_boundary(self):
        with tempfile.TemporaryDirectory() as directory:
            root=Path(directory);evid=root/'evidence';b=evid/'branches/micro25';b.mkdir(parents=True)
            case=root/'native.cas.h5';data=root/'native.dat.h5';case.write_bytes(b'CASE');data.write_bytes(b'DATA')
            meta={'status':'NATIVE_CHECKPOINT_NUMERICAL_GATES_PASS','time_s':.00185,
                'case':str(case),'data':str(data),'case_sha256':s.sha(case),'data_sha256':s.sha(data)}
            s.atomic(b/'native_0074_checkpoint.json',meta)
            s.atomic(b/'latest_verified_checkpoint.json',{'metadata':'evidence/branches/micro25/native_0074_checkpoint.json'})
            s.atomic(b/'worker_identity.json',{'pid':999999,'created':10,'name':'pythonw.exe'})
            s.atomic(b/'owned_engine.json',{'processes':[{'pid':999998,'created':9,'name':'fl2610.exe'}]})
            s.atomic(b/'memory_summary.json',{'abort':None,'samples':5})
            s.atomic(evid/'configuration.json',{'max_lifecycle_restarts':2})
            spec={'branch':'micro25','dt_s':25e-6,'start_time_s':.00175,'end_time_s':.0019,'stage':'NATIVE_MICRORUN_25US'}
            with patch.object(s,'ROOT',root),patch.object(s,'EVID',evid),patch.object(s,'native',return_value=[]),\
                 patch.object(s,'specification',return_value=spec),patch.object(s,'event'):
                result=s.lifecycle_recovery('micro25')
            self.assertEqual(result,('worker','micro25_recovery1'))
            new=s.read(evid/'branch_specs/micro25_recovery1.json')
            self.assertEqual(new['start_time_s'],.00185)
            self.assertNotEqual(new['start_time_s'],.00175)
            self.assertEqual(new['history_prefix_branches'],['micro25'])

    def test_unverified_checkpoint_never_resumes(self):
        with tempfile.TemporaryDirectory() as directory:
            root=Path(directory);evid=root/'evidence';b=evid/'branches/micro25';b.mkdir(parents=True)
            s.atomic(b/'latest_verified_checkpoint.json',{'metadata':'evidence/branches/micro25/failure_checkpoint.json'})
            s.atomic(b/'failure_checkpoint.json',{'status':'SAVED_HASHED_STOP_PENDING_REVIEW','time_s':.00185})
            s.atomic(evid/'configuration.json',{'max_lifecycle_restarts':2})
            with patch.object(s,'ROOT',root),patch.object(s,'EVID',evid),patch.object(s,'native',return_value=[]),\
                 patch.object(s,'specification',return_value={'branch':'micro25','start_time_s':.00175,'end_time_s':.0019}):
                self.assertIsNone(s.lifecycle_recovery('micro25'))

if __name__=='__main__':unittest.main()
