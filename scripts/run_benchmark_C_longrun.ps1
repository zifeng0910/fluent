param([switch]$RunSupervisor)
$ErrorActionPreference = 'Stop'
$taskRoot = 'H:\fluent'
$taskName = 'Fluent-Benchmark-C-Longrun'
$logRoot = Join-Path $taskRoot 'evidence\benchmark_C_longrun'
New-Item -ItemType Directory -Path $logRoot -Force | Out-Null
if ($RunSupervisor) {
    Set-Location -LiteralPath $taskRoot
    & "$taskRoot\.venv\Scripts\python.exe" -u "$taskRoot\scripts\benchmark_C_longrun_supervisor.py" --interval 180 --minimum-ram-gib 22 1>> "$logRoot\scheduler_stdout.log" 2>> "$logRoot\scheduler_stderr.log"
    exit $LASTEXITCODE
}
$existingTask = Get-ScheduledTask -TaskName $taskName -ErrorAction SilentlyContinue
if ($existingTask -and $existingTask.State -eq 'Running') {
    Write-Output 'Campaign supervisor already running; duplicate start refused.'
    exit 0
}
$statePath = Join-Path $taskRoot 'evidence\benchmark_C_longrun_state.json'
if (Test-Path -LiteralPath $statePath) {
    $campaignState = Get-Content -LiteralPath $statePath -Raw -Encoding UTF8 | ConvertFrom-Json
    if ($campaignState.campaign_status -in @('COMPLETE','HARD_BLOCKER_REQUIRES_REVIEW','TIME_BUDGET_EXHAUSTED','STOPPED_RESOURCE_BLOCKER')) {
        Write-Output "Campaign stopped: $($campaignState.campaign_status). Read state before authorizing a new budget or repair."
        exit 0
    }
    if ($campaignState.supervisor_pid) {
        $ownedSupervisor = Get-CimInstance Win32_Process -Filter "ProcessId = $($campaignState.supervisor_pid)" -ErrorAction SilentlyContinue
        if ($ownedSupervisor -and $ownedSupervisor.CommandLine -like '*benchmark_C_longrun_supervisor.py*') {
            Write-Output 'Existing supervisor detected; duplicate start refused.'
            exit 0
        }
    }
}
$action = New-ScheduledTaskAction -Execute "$env:SystemRoot\System32\WindowsPowerShell\v1.0\powershell.exe" -Argument '-NoProfile -WindowStyle Hidden -ExecutionPolicy Bypass -File H:\fluent\scripts\run_benchmark_C_longrun.ps1 -RunSupervisor' -WorkingDirectory $taskRoot
$principal = New-ScheduledTaskPrincipal -UserId ([System.Security.Principal.WindowsIdentity]::GetCurrent().Name) -LogonType Interactive -RunLevel Limited
$settings = New-ScheduledTaskSettingsSet -MultipleInstances IgnoreNew -ExecutionTimeLimit (New-TimeSpan -Hours 12 -Minutes 30) -AllowStartIfOnBatteries -DontStopIfGoingOnBatteries -StartWhenAvailable
# The Python deadline is persisted and never reset by re-invoking this command.
# Scheduler ceiling includes the final safe atomic checkpoint and shutdown grace.
Register-ScheduledTask -TaskName $taskName -Action $action -Principal $principal -Settings $settings -Description 'Single-session Benchmark C continuation; persisted 12-hour deadline; 3-minute monitoring.' -Force | Out-Null
Start-ScheduledTask -TaskName $taskName
Write-Output "Started $taskName. State: $statePath"
