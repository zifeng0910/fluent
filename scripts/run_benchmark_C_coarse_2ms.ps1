$ErrorActionPreference = 'Stop'
$taskRoot = 'H:\fluent'
$taskName = 'Fluent-Benchmark-C-Coarse-2ms'
$taskStatePath = Join-Path $taskRoot 'evidence\benchmark_C_coarse_2ms\state.json'
$campaignState = Get-Content -LiteralPath $taskStatePath -Raw -Encoding UTF8 | ConvertFrom-Json
if ($campaignState.status -notin @('PREPARED','RESOURCE_WAIT')) {
    throw "Campaign is $($campaignState.status); inspect evidence before resuming."
}
$existingTask = Get-ScheduledTask -TaskName $taskName -ErrorAction SilentlyContinue
if ($existingTask -and $existingTask.State -eq 'Running') { throw 'Supervisor already running.' }
$action = New-ScheduledTaskAction -Execute "$taskRoot\.venv\Scripts\pythonw.exe" -Argument '-u H:\fluent\scripts\benchmark_C_coarse_2ms_supervisor.py' -WorkingDirectory $taskRoot
$principal = New-ScheduledTaskPrincipal -UserId ([System.Security.Principal.WindowsIdentity]::GetCurrent().Name) -LogonType Interactive -RunLevel Limited
$settings = New-ScheduledTaskSettingsSet -MultipleInstances IgnoreNew -ExecutionTimeLimit ([TimeSpan]::Zero) -AllowStartIfOnBatteries -DontStopIfGoingOnBatteries -StartWhenAvailable
# Python owns the inherited absolute deadline and safe checkpoint grace.
Register-ScheduledTask -TaskName $taskName -Action $action -Principal $principal -Settings $settings -Description 'One-rank coarse native step40 to step80; zero-step continuity, hard gates, actual FieldData.' -Force | Out-Null
Start-ScheduledTask -TaskName $taskName
Write-Output "Started $taskName."
