"""Safety tests use temporary evidence and mocked processes; never launch Fluent."""
import argparse, importlib.util, json, tempfile, unittest
from pathlib import Path
from unittest.mock import patch, Mock

spec=importlib.util.spec_from_file_location('watchdog',Path(__file__).with_name('benchmark_C_longrun_supervisor.py'))
w=importlib.util.module_from_spec(spec);spec.loader.exec_module(w)

class SafetyTests(unittest.TestCase):
    def setUp(self):
        self.tmp=tempfile.TemporaryDirectory();self.root=Path(self.tmp.name)
        self.patches=[]
        for name,path in {'ROOT':self.root,'E':self.root/'evidence','LOG':self.root/'evidence/longrun',
                          'STATE':self.root/'evidence/state.json','QUEUE':self.root/'queue.json',
                          'SESSION':self.root/'evidence/session.json','FREE':self.root/'evidence/free.json',
                          'HIST':self.root/'evidence/history.csv'}.items():
            p=patch.object(w,name,path);p.start();self.patches.append(p)
        self.sup=w.Supervisor(argparse.Namespace(interval=180,minimum_ram_gib=22,once=True))
    def tearDown(self):
        self.sup.lock.close()
        for p in self.patches:p.stop()
        self.tmp.cleanup()
    def test_duplicate_supervisor_refused(self):
        with self.assertRaises(SystemExit):w.Supervisor(self.sup.args)
    def test_resources_wait_does_not_truncate_or_launch(self):
        w.HIST.write_text('original history',encoding='utf-8')
        self.sup.s['interrupted_run_archive']='already archived'
        with patch.object(self.sup,'resources_ready',return_value=False),patch.object(self.sup,'launch') as launch:
            self.sup.recovery({})
            self.assertEqual(w.HIST.read_text(),'original history');launch.assert_not_called()
    def test_existing_native_prevents_launch(self):
        with patch.object(w,'native',return_value=[{'pid':1}]),patch.object(w.subprocess,'Popen') as spawn:
            with self.assertRaises(RuntimeError):self.sup.launch({})
            spawn.assert_not_called()
    def test_repeated_repair_configuration_rejected(self):
        job=self.root/'scripts/repair.py';job.parent.mkdir();job.write_text('pass')
        evidence=self.root/'failure.json';evidence.write_text('{}')
        self.sup.s['last_failure']={'class':'DYNAMIC_GATE','timestamp':'failure1'}
        self.sup.s['repairs_attempted']=[{'failure_class':'DYNAMIC_GATE','configuration_sha256':'same'}]
        w.atomic(w.LOG/'repair_plan.json',{'failure_timestamp':'failure1','decision':'REPAIR','job':'scripts/repair.py',
                  'job_sha256':w.sha(job),'root_cause':'tested','evidence_files':['failure.json'],
                  'concrete_change':'tested','configuration_sha256':'same'})
        with patch.object(self.sup,'launch') as launch:
            self.sup.reviewed_action();launch.assert_not_called()
        self.assertEqual(self.sup.s['campaign_status'],'HARD_BLOCKER_REQUIRES_REVIEW')
    def test_stop_queues_exit_without_killing_worker(self):
        process=Mock(pid=10);process.create_time.return_value=1;process.children.return_value=[]
        with patch.object(w,'worker',return_value=process):self.sup.controlled_stop('12-hour deadline')
        process.terminate.assert_not_called()
        self.assertEqual(w.read(w.QUEUE),{'action':'exit'})
        self.assertTrue((w.LOG/'stop_request.json').exists())
    def test_deadline_not_reset_when_supervisor_restarts(self):
        start=self.sup.s['campaign_start_time'];self.sup.save();self.sup.lock.close()
        self.sup=w.Supervisor(self.sup.args)
        self.assertEqual(self.sup.s['campaign_start_time'],start)

if __name__=='__main__':unittest.main()
