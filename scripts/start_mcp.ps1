$ErrorActionPreference = 'Stop'
$taskRoot = 'H:\fluent'
$taskPort = 8765
if (Get-NetTCPConnection -LocalPort $taskPort -State Listen -ErrorAction SilentlyContinue) {
    throw "Port $taskPort is already in use. Inspect the owner before starting another server."
}
New-Item -ItemType Directory -Force -Path "$taskRoot\logs" | Out-Null
$env:AWP_ROOT261 = 'H:\Program Files\ANSYS Inc\v261'
$env:PYTHONUTF8 = '1'
$taskServer = Start-Process -FilePath "$taskRoot\.venv\Scripts\ansys-fluent-mcp.exe" -ArgumentList '--transport','http','--host','127.0.0.1','--port',"$taskPort" -WorkingDirectory $taskRoot -WindowStyle Hidden -RedirectStandardOutput "$taskRoot\logs\mcp-server.stdout.log" -RedirectStandardError "$taskRoot\logs\mcp-server.stderr.log" -PassThru
$taskServer.Id | Set-Content "$taskRoot\logs\mcp-server.pid"
Write-Output "Local MCP PID: $($taskServer.Id)"
