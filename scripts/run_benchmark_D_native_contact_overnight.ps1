$ErrorActionPreference = 'Stop'
$taskRoot = 'H:\fluent'
$taskName = 'Fluent-Benchmark-D-Native-Contact-Overnight'
$taskEvidence = Join-Path $taskRoot 'evidence\benchmark_D_native_contact_overnight'
$campaignWindow = Get-Content -LiteralPath (Join-Path $taskEvidence 'window.json') -Raw -Encoding UTF8 | ConvertFrom-Json
$campaignState = Get-Content -LiteralPath (Join-Path $taskEvidence 'state.json') -Raw -Encoding UTF8 | ConvertFrom-Json
if ([DateTimeOffset]::UtcNow.ToUnixTimeSeconds() -ge $campaignWindow.hard_deadline_epoch) { throw 'Immutable hard deadline expired.' }
if ($campaignState.status -in @('COMPLETE','HARD_BLOCKER','TIME_BUDGET_EXHAUSTED')) { throw "Terminal state $($campaignState.status) requires evidence review." }
$existingTask = Get-ScheduledTask -TaskName $taskName -ErrorAction SilentlyContinue
if ($existingTask -and $existingTask.State -eq 'Running') { throw 'Supervisor already running.' }
$action = New-ScheduledTaskAction -Execute "$taskRoot\.venv\Scripts\pythonw.exe" -Argument '-u H:\fluent\scripts\benchmark_D_native_contact_overnight_supervisor.py' -WorkingDirectory $taskRoot
$principal = New-ScheduledTaskPrincipal -UserId ([System.Security.Principal.WindowsIdentity]::GetCurrent().Name) -LogonType Interactive -RunLevel Limited
$settings = New-ScheduledTaskSettingsSet -MultipleInstances IgnoreNew -ExecutionTimeLimit ([TimeSpan]::Zero) -AllowStartIfOnBatteries -DontStopIfGoingOnBatteries -StartWhenAvailable
Register-ScheduledTask -TaskName $taskName -Action $action -Principal $principal -Settings $settings -Description 'Independent single-solver frictionless contact FSM; immutable10h target/12h hard limit.' -Force | Out-Null
Start-ScheduledTask -TaskName $taskName
Write-Output "Started $taskName."
