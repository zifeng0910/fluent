"""Compile the independent static-connectivity audit without magnetic callbacks."""
def run(solver,context):
    root=context['root'];user=solver.settings.setup.user_defined
    user.compiled_udf(library_name=str(root/'fluent_udf/libbenchmark_C_pose'),source_files=[str(root/'fluent_udf/benchmark_C_pose_statistics.c')],header_files=[],use_built_in_compiler=True)
    user.load(udf_library_name=str(root/'fluent_udf/libbenchmark_C_pose'))
