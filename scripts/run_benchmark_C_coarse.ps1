param([switch]$RunPipeline,[int]$GeometryPid,[double]$GeometryCreated)
$ErrorActionPreference='Stop'
$taskRoot='H:\fluent'
$taskName='Fluent-Benchmark-C-Coarse-Screening'
$coarseLogs=Join-Path $taskRoot 'evidence\benchmark_C_coarse_A'
New-Item -ItemType Directory -Path $coarseLogs -Force | Out-Null
if ($RunPipeline) {
    Set-Location -LiteralPath $taskRoot
    $workerArgs=@('-u',"$taskRoot\scripts\benchmark_C_coarse_pipeline.py",'--geometry-pid',"$GeometryPid",'--geometry-created',"$GeometryCreated")
    $worker=Start-Process -FilePath "$taskRoot\.venv\Scripts\python.exe" -ArgumentList $workerArgs -WorkingDirectory $taskRoot -WindowStyle Hidden -PassThru -Wait -RedirectStandardOutput "$coarseLogs\scheduler_stdout.log" -RedirectStandardError "$coarseLogs\scheduler_stderr.log"
    exit $worker.ExitCode
}
$existing=Get-ScheduledTask -TaskName $taskName -ErrorAction SilentlyContinue
if ($existing -and $existing.State -eq 'Running') { Write-Output 'Coarse pipeline already running; duplicate refused.';exit 0 }
$knownState=Join-Path $coarseLogs 'state.json'
if (Test-Path -LiteralPath $knownState) {
    $coarseState=Get-Content -LiteralPath $knownState -Raw | ConvertFrom-Json
    if ($coarseState.status -in @('COMPLETE','HARD_BLOCKER','RESOURCE_BLOCKER')) { throw 'Terminal coarse evidence exists; inspect before reviewed continuation.' }
}
$coarseArgs="-NoProfile -WindowStyle Hidden -ExecutionPolicy Bypass -File H:\fluent\scripts\run_benchmark_C_coarse.ps1 -RunPipeline -GeometryPid $GeometryPid -GeometryCreated $GeometryCreated"
$action=New-ScheduledTaskAction -Execute "$env:SystemRoot\System32\WindowsPowerShell\v1.0\powershell.exe" -Argument $coarseArgs -WorkingDirectory $taskRoot
$principal=New-ScheduledTaskPrincipal -UserId ([System.Security.Principal.WindowsIdentity]::GetCurrent().Name) -LogonType Interactive -RunLevel Limited
# This is a finite 40-step workflow, no recurring heartbeat or new hourly campaign budget.
$settings=New-ScheduledTaskSettingsSet -MultipleInstances IgnoreNew -ExecutionTimeLimit ([TimeSpan]::Zero) -AllowStartIfOnBatteries -DontStopIfGoingOnBatteries
Register-ScheduledTask -TaskName $taskName -Action $action -Principal $principal -Settings $settings -Description 'Finite coarse C: unchanged component, 4 static poses then gated 10/20/40 steps, no LLM dependency.' -Force | Out-Null
Start-ScheduledTask -TaskName $taskName
Write-Output "Started $taskName; state=$knownState"
