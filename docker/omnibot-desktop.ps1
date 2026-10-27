# Copyright 2026 OhhO Robotics
# SPDX-License-Identifier: Apache-2.0
<#
.SYNOPSIS
  OmniBot browser desktop launcher (Windows). Needs only Docker Desktop.

.EXAMPLE
  .\docker\omnibot-desktop.ps1                    # start, then open the browser
  .\docker\omnibot-desktop.ps1 -Port 6081         # use another host port
  .\docker\omnibot-desktop.ps1 -World apartment
  .\docker\omnibot-desktop.ps1 -Password          # require a random VNC password
  .\docker\omnibot-desktop.ps1 -Build             # build the image locally
  .\docker\omnibot-desktop.ps1 stop               # also: restart, logs, shell, status, update, down

  Wraps `docker compose --profile desktop` on the repository's
  docker-compose.yml, so `docker compose up -d desktop` is equivalent.
#>
[CmdletBinding()]
param(
    [Parameter(Position = 0)]
    [ValidateSet('start', 'stop', 'restart', 'logs', 'shell', 'status', 'update', 'down')]
    [string]$Command = 'start',
    [int]$Port = 0,
    [string]$World = '',
    [switch]$Password,
    [switch]$Build,
    [switch]$NoBrowser
)

$ErrorActionPreference = 'Stop'
$Repo = Split-Path -Parent $PSScriptRoot
$ComposeFile = Join-Path $Repo 'docker-compose.yml'
$Service = 'desktop'
$Container = 'omnibot_desktop'

function Fail([string]$Message) { Write-Host "omnibot-desktop: $Message" -ForegroundColor Red; exit 1 }

if ($Port -gt 0) { $env:OMNIBOT_PORT = "$Port" }
if (-not $env:OMNIBOT_PORT) { $env:OMNIBOT_PORT = '6080' }
if ($World) { $env:OMNIBOT_WORLD = $World }
if ($Password -and -not $env:OMNIBOT_VNC_PASSWORD) {
    $chars = [char[]]'ABCDEFGHJKLMNPQRSTUVWXYZabcdefghijkmnpqrstuvwxyz23456789'
    $env:OMNIBOT_VNC_PASSWORD = -join (1..8 | ForEach-Object { $chars | Get-Random })
}
$PortNum = $env:OMNIBOT_PORT

if (-not (Get-Command docker -ErrorAction SilentlyContinue)) { Fail 'Docker is not installed (https://docs.docker.com/desktop/).' }
docker info *> $null
if ($LASTEXITCODE -ne 0) { Fail 'Docker is not running. Start Docker Desktop and try again.' }

function Invoke-Compose { docker compose -f $ComposeFile --profile desktop @args; if ($LASTEXITCODE -ne 0) { Fail "docker compose $args failed" } }

function Get-Url {
    if ($env:OMNIBOT_VNC_PASSWORD) { "http://localhost:$PortNum/vnc.html?password=$($env:OMNIBOT_VNC_PASSWORD)" }
    else { "http://localhost:$PortNum/" }
}

function Wait-Desktop {
    Write-Host -NoNewline 'Waiting for the desktop'
    for ($i = 0; $i -lt 180; $i++) {
        try {
            $r = Invoke-WebRequest -UseBasicParsing -TimeoutSec 3 -Uri "http://127.0.0.1:$PortNum/vnc.html"
            if ($r.StatusCode -eq 200) { Write-Host ' ready.'; return }
        } catch { }
        $running = docker inspect -f '{{.State.Running}}' $Container 2>$null
        if ($running -eq 'false') {
            Write-Host ''
            docker logs --tail 40 $Container
            Fail 'the container stopped while starting (see above).'
        }
        Write-Host -NoNewline '.'
        Start-Sleep -Seconds 1
    }
    Write-Host ''
    Write-Host "The desktop is taking longer than usual; try $(Get-Url) in a moment."
}

function Start-Desktop {
    $shared = if ($env:OMNIBOT_SHARED_DIR) { $env:OMNIBOT_SHARED_DIR } else { Join-Path $Repo 'omnibot-shared' }
    New-Item -ItemType Directory -Force -Path $shared | Out-Null
    if ($Build) { Invoke-Compose up -d --build $Service } else { Invoke-Compose up -d $Service }
    Wait-Desktop
    Write-Host ''
    Write-Host "OmniBot desktop:  $(Get-Url)" -ForegroundColor Green
    Write-Host 'Gazebo and RViz open on the desktop (first start can take a minute).'
    Write-Host "Stop it with:     $PSCommandPath stop"
    if (-not $NoBrowser) { Start-Process (Get-Url) }
}

switch ($Command) {
    'start'   { Start-Desktop }
    'stop'    { Invoke-Compose stop $Service }
    'restart' { Invoke-Compose stop $Service; Start-Desktop }
    'logs'    { docker compose -f $ComposeFile --profile desktop logs -f --tail 200 $Service }
    'shell'   { docker exec -it $Container bash -l }
    'status'  {
        docker ps -a --filter "name=^/$Container$" --format 'table {{.Names}}\t{{.Status}}\t{{.Ports}}'
        $s = docker exec $Container omnibot-desktop-sim status 2>$null
        if ($s) { Write-Host "sim: $s" }
    }
    'update'  { Invoke-Compose pull $Service; Start-Desktop }
    'down'    { Invoke-Compose rm -sf $Service; Write-Host 'Removed the container (the omnibot-desktop-home volume is kept).' }
}
