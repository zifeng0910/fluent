"""Inspect standard export on saved diagnostic state without unsafe helpers."""
import json
def run(solver,context):
    root=context['root'];s=solver.settings
    s.file.read_case(file_name=str(root/'live_cases/benchmark_C_candidates/benchmark_C_candidate_D_diagnostic.cas.h5'))
    s.solution.initialization.standard_initialize()
    r={'ascii_surface_choices':s.file.export.ascii.surface_name_list.allowed_values(),'ascii_fields':s.file.export.ascii.quantities.allowed_values()}
    (root/'evidence/benchmark_C_standard_field_probe.json').write_text(json.dumps(r,indent=2,default=str))
    s.file.export.ascii(file_name=str(root/'live_cases/benchmark_C_fineA/official_fields_probe.csv'),surface_name_list=[],delimiter='comma',quantities=['overset-cell-type','overset-donor-size-ratio','cell-volume'],location='cell-center')
