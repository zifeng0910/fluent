param([switch]$RunSupervisor)
$ErrorActionPreference='Stop'
$taskRoot='H:\fluent'
$taskName='Fluent-Benchmark-C-Recovery-V2'
$v2Logs=Join-Path $taskRoot 'evidence\benchmark_C_recovery_v2'
New-Item -ItemType Directory -Path $v2Logs -Force | Out-Null
if ($RunSupervisor) {
    Set-Location -LiteralPath $taskRoot
    & "$taskRoot\.venv\Scripts\python.exe" -u "$taskRoot\scripts\benchmark_C_recovery_v2_supervisor.py" 1>> "$v2Logs\scheduler_stdout.log" 2>> "$v2Logs\scheduler_stderr.log"
    exit $LASTEXITCODE
}
$existing=Get-ScheduledTask -TaskName $taskName -ErrorAction SilentlyContinue
if ($existing -and $existing.State -eq 'Running') { Write-Output 'Existing V2 task running; duplicate refused.';exit 0 }
$stateFile=Join-Path $v2Logs 'state.json'
if (Test-Path -LiteralPath $stateFile) {
    $v2State=Get-Content -LiteralPath $stateFile -Raw -Encoding UTF8 | ConvertFrom-Json
    if ($v2State.state -in @('COMPLETE','HARD_BLOCKER','RESOURCE_BLOCKER')) { Write-Output "V2 terminal state: $($v2State.state). Deadline preserved; inspect evidence.";exit 0 }
    if ($v2State.supervisor_pid) {
        $known=Get-CimInstance Win32_Process -Filter "ProcessId = $($v2State.supervisor_pid)" -ErrorAction SilentlyContinue
        if ($known -and $known.CommandLine -like '*benchmark_C_recovery_v2_supervisor.py*') { Write-Output 'V2 supervisor exists; duplicate refused.';exit 0 }
    }
}
$oldTask=Get-ScheduledTask -TaskName 'Fluent-Benchmark-C-Longrun' -ErrorAction SilentlyContinue
if ($oldTask -and $oldTask.State -eq 'Running') { throw 'Old local supervisor must be safely handed over first.' }
$action=New-ScheduledTaskAction -Execute "$env:SystemRoot\System32\WindowsPowerShell\v1.0\powershell.exe" -Argument '-NoProfile -WindowStyle Hidden -ExecutionPolicy Bypass -File H:\fluent\scripts\run_benchmark_C_recovery_v2.ps1 -RunSupervisor' -WorkingDirectory $taskRoot
$principal=New-ScheduledTaskPrincipal -UserId ([System.Security.Principal.WindowsIdentity]::GetCurrent().Name) -LogonType Interactive -RunLevel Limited
$settings=New-ScheduledTaskSettingsSet -MultipleInstances IgnoreNew -ExecutionTimeLimit (New-TimeSpan -Hours 10 -Minutes 15) -AllowStartIfOnBatteries -DontStopIfGoingOnBatteries -StartWhenAvailable -RestartCount 2 -RestartInterval (New-TimeSpan -Minutes 1)
# Restarting this wrapper retains state.json's original V2 deadline. No recurring LLM invocation.
Register-ScheduledTask -TaskName $taskName -Action $action -Principal $principal -Settings $settings -Description 'Local-only C Recovery V2; fixed 10h deadline, one-core checkpoint probe/restart, no LLM.' -Force | Out-Null
Start-ScheduledTask -TaskName $taskName
Write-Output "Started $taskName; launching shell can exit. State: $stateFile"
