"""Read installed declarations and identify documented facts without assuming theta conventions."""
from pathlib import Path
from benchmark_D_overwrite_common import *

def snippets(path,terms,radius=1):
    lines=path.read_text(errors='replace').splitlines();out=[]
    for i,line in enumerate(lines):
        if any(term in line for term in terms):
            lo=max(0,i-radius);hi=min(len(lines),i+radius+1)
            out.append(dict(line=i+1,excerpt=[dict(line=j+1,text=lines[j]) for j in range(lo,hi)]))
    return dict(path=path.as_posix(),sha256=sha(path),matches=out)

def main():
    base=Path(r'H:/Program Files/ANSYS Inc/v261/fluent/fluent26.1.0/src')
    files=[(base/'udf/udf.h',['#define DEFINE_CONTACT']),
        (base/'mesh/six_dof.h',['SDOF_Get_Motion','SDOF_Overwrite_Motion','SDOF_Fill_RB_State','SDOF_Threads_Compute_Motion','SDOF_LOAD_F_','SDOF_LOAD_M_','SDOFP_LOAD']),
        (base/'mesh/dynamesh_tools.h',['real theta[3]','Euler angles for 6-dof','quaternion q','real cg[3]','#define DT_THETA','#define DT_Q','#define DT_CG','tmp_cg','tmp_theta','tmp_state;','current_state;','contact_active_p']),
        (base/'util/quaternion.h',['typedef real quaternion','Theta_From_Q','Q_From_Theta','Euler_From_Q','Theta_From_Euler','Omega_From_Theta_Angles','Omega_From_Euler_Angles'])]
    doc='https://ansyshelp.ansys.com/public/Views/Secured/corp/v261/en/flu_udf/flu_udf_DynamicMeshDEFINE.html'
    audit=dict(timestamp=stamp(),release='Installedv261 / Fluent26.1.0',status='PARTIAL',
        installed_definitions=[snippets(p,t) for p,t in files],
        local_official_example=dict(status='NOT_FOUND',search_roots=[r'H:/Program Files/ANSYS Inc/v261/fluent',r'H:/Program Files/ANSYS Inc/v261/commonfiles/help'],
            searched_extensions=['.h','.c','.cpp','.html','.htm','.txt','.scm','.js','.xml','.json','.pdf'],
            installed_help_contents='GUI helpXML and graphics; standalone Fluent UDF contact example not located',
            original_local_example_file=None,original_local_example_lines=None),
        official_same_release_online_example=dict(classification='DOCUMENTED',url=doc,version='v261',section='2.6.7.3 Example1',
            get_line=891,get_call='SDOF_Get_Motion (dt, vel0, omega0, theta0);',
            line_numbers_basis='Web tool extracted document text; not installed-file source lines',
            overwrite_line=944,overwrite_call='SDOF_Overwrite_Motion (dt, vel0, omega0, theta0);',
            same_returned_omega_and_theta_passed_back=True,velocity_changed_by_reflection=True,
            exact_visible_value_noop_trajectory_guarantee_documented=False,overwrite_inside_DEFINE_CONTACT_only=True,
            storage='Two call excerpts only; online documentation is not mislabeled as an installed original'),
        statements=[
            dict(classification='DOCUMENTED',statement='Installed rigid-body theta field comment labels global orientation angles / Euler angles for6DOF. This comment does not specify Get_Motion third-array output.'),
            dict(classification='INFERRED_FROM_HEADER',statement='DT_Q,DT_CG,DT_THETA alias separate fields of current_state. tmp_state and tmp_cg/tmp_theta are distinct storage.'),
            dict(classification='INFERRED_FROM_HEADER',statement='Theta_From_Q and Euler_From_Q are different declared native functions. Their declaration alone proves neither a conversion sequence nor the Get/Overwrite theta contract.'),
            dict(classification='EMPIRICALLY_MEASURED',statement='Previous actual6DOF callbacks returned nonzero native v/omega but caller-zero theta while DT_Q was non-identity. Same-value overwrite changed pose, not motion arrays.'),
            dict(classification='UNKNOWN',statement='Get/Overwrite implementation bodies, Euler sequence, branch flags and predictor/corrector routing were not provided by the installed headers.'),
            dict(classification='DOCUMENTED',statement='Official v261 example passes theta returned by Get to Overwrite; it does not promise no-op equivalence to omitting Overwrite.')],
        conversions_in_diagnostics='Native helpers on copied values only. No converted theta is sent to Overwrite or assigned to Dynamic_Thread.',
        binary_disassembly_used=False,official_installation_modified=False)
    atomic(EVID/'local_api_audit.json',audit);print('Local API audit PARTIAL; exact official same-release usage verified online')

if __name__=='__main__':main()
