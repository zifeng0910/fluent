"""Offline interface regressions; synthetic fixtures do not establish CFD PASS."""
import copy,json,tempfile,unittest
from pathlib import Path
from unittest.mock import patch
import benchmark_D_native_contact_overnight_fallback as f


class FallbackTests(unittest.TestCase):
    def test_recovery_event_count_excludes_unsaved_tail(self):
        with tempfile.TemporaryDirectory() as directory:
            evid=Path(directory);a=evid/'branches/a';b=evid/'branches/b';a.mkdir(parents=True);b.mkdir()
            f.atomic(a/'configuration.json',{'dt_s':25e-6,'history_prefix_branches':[]})
            f.atomic(b/'configuration.json',{'dt_s':25e-6,'start_time_s':.00185,'history_prefix_branches':['a']})
            old=[dict(CURRENT_TIME=t,physical_impulse_applied=True) for t in [.001825,.00185]]
            new=[dict(CURRENT_TIME=.00185,physical_impulse_applied=True)]
            (a/'native_contact_callback_trace_node.jsonl').write_text(''.join(json.dumps(e)+'\n' for e in old))
            (b/'native_contact_callback_trace_node.jsonl').write_text(''.join(json.dumps(e)+'\n' for e in new))
            with patch.object(f,'EVID',evid):events=f.retained_applied_events('b',.001875)
            self.assertEqual([e['_evidence_branch'] for e in events],['a','b'])
            self.assertEqual(len(events),2)

    def test_resource_failure_has_no_contact_instability_claim(self):
        with tempfile.TemporaryDirectory() as directory:
            root=Path(directory);evid=root/'evidence';b=evid/'branches/load25';b.mkdir(parents=True)
            f.atomic(b/'failure.json',{'failure_class':'RESOURCE_HARD_STOP'})
            with patch.object(f,'ROOT',root),patch.object(f,'EVID',evid),patch.object(f,'control'):
                with self.assertRaises(f.FallbackResourceBlocker):f.resource_failure('load25',{'failure_class':'RESOURCE_HARD_STOP'})
            report=f.read(evid/'fallback_resource_blocker.json')
            self.assertEqual(report['status'],'STOPPED_RESOURCE_BLOCKER')
            self.assertFalse(report['contact_instability_claim']);self.assertFalse(report['compliant_eligible'])

    def test_load_audit_satisfies_shared_reviewer_schema_without_setter_claim(self):
        import benchmark_D_native_contact_overnight_review as core
        with tempfile.TemporaryDirectory() as directory:
            evid=Path(directory);b=evid/'branches/load25';b.mkdir(parents=True)
            dt=25e-6;fn=.1000001;J=fn*dt
            event=dict(callback_index=1,CURRENT_TIME=.001875,timestep_index=75,moving_body=True,
                dynamic_zone_id=2,analytic_contact_point_count=47677,physical_impulse_applied=True,
                contact_point=[0,.0009,0],contact_normal=[0,-1,0],DT_CG=[0,0,0],
                impulse_N_s=J,Fcontact_N=[0,-fn,0],Tcontact_COM_Nm=[0,0,0],load_contact_model=1,
                stiffness_N_m=1,damping_N_s_m=1,gap_predicted_m=-1e-7,normal_velocity_before=-.1,
                requested_velocity=[0,0,0],requested_omega=[0,0,0],overwrite_called=False)
            cached={**event,'callback_index':2,'physical_impulse_applied':False}
            for role in ['node','host']:
                (b/f'native_contact_callback_trace_{role}.jsonl').write_text(json.dumps(event)+'\n'+json.dumps(cached)+'\n')
            ns=dict(com=[0,0,0],q=[1,0,0,0],velocity=[0,0,0],omega=[0,0,0])
            post=dict(time_s=.0019,native_state=ns,Fmag_N=[0,0,0],Tmag_Nm=[0,0,0],physical_gap_m=0,
                signed_gap_m=0,connectivity=dict(total=4699301,orphan=0,invalid_donors=0,unidentified=0,
                    minimum_volume_m3=1e-17,nonpositive_volume=0,nonfinite_volume=0),hard_failures=[])
            f.atomic(b/'final_next_start.json',post)
            f.atomic(evid/'fallback_controlled_validation.json',{'status':'PASS','controlled_source_sha256':f.sha(f.ROOT/f.SOURCE)})
            with patch.object(f,'EVID',evid):audit,_=f.load_event_audit(b,[post],post,dt)
            self.assertEqual(audit['status'],'PASS')
            checked=audit['response_checks'][0]
            for key in ['independent_impulse_math_PASS','completed_response_PASS','next_step_inherited_PASS',
                        'inheritance_evidence_available','terminal_zero_step_boundary_only']:
                self.assertTrue(checked[key])
            self.assertTrue(audit['no_instantaneous_load_path_setter_claim'])
            self.assertFalse(checked['actual_next_properties_entry_PASS'])
            # Re-evaluating a cached load is allowed; changing it is not.
            cached['Fcontact_N']=[0,-2*fn,0]
            (b/'native_contact_callback_trace_node.jsonl').write_text(json.dumps(event)+'\n'+json.dumps(cached)+'\n')
            with patch.object(f,'EVID',evid):invalid,_=f.load_event_audit(b,[post],post,dt)
            self.assertEqual(invalid['status'],'FAIL')


if __name__=='__main__':unittest.main()
