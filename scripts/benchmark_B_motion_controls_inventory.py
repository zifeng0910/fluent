import json
def run(solver, context):
 c=solver.settings.solution.run_calculation
 result={"children":c.child_names,"state":c.get_state()}
 (context['root']/'evidence/benchmark_B_motion_controls.json').write_text(json.dumps(result,indent=2,default=str))
