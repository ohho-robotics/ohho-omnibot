# Copyright 2026 OhhO Robotics
# SPDX-License-Identifier: Apache-2.0
# Pester 3.4 tests. They dot-source vr-sim-network.ps1 and call planning
# functions only. They do not launch the script entry point.

$ErrorActionPreference = 'Stop'

$here = $PSScriptRoot
if ([string]::IsNullOrWhiteSpace($here)) {
    $here = Split-Path -Parent $MyInvocation.MyCommand.Path
}
$script:VrSimNetworkScript = (Resolve-Path (Join-Path $here '..\vr-sim-network.ps1')).Path

$script:VrSimParseErrors = $null
$script:VrSimTokens = $null
$script:VrSimAst = [System.Management.Automation.Language.Parser]::ParseFile(
    $script:VrSimNetworkScript,
    [ref]$script:VrSimTokens,
    [ref]$script:VrSimParseErrors)

. $script:VrSimNetworkScript

function Get-OhhoAstFunctionName {
    param($Node)
    $current = $Node
    while ($null -ne $current) {
        if ($current -is [System.Management.Automation.Language.FunctionDefinitionAst]) {
            return $current.Name
        }
        $current = $current.Parent
    }
    return ''
}

function Invoke-VrSimSafe {
    param(
        [string]$Mode,
        [switch]$WhatIf,
        [switch]$DryRun,
        [switch]$RestartWsl,
        [switch]$ForcePortProxy,
        [string]$LanAddress = '192.168.1.23',
        [AllowEmptyString()][string]$WslAddress = '172.22.1.2',
        [object[]]$ExistingFirewall = @(),
        [object[]]$ExistingPortProxy = @(),
        [AllowEmptyString()][string]$WslConfigText = '',
        [bool]$IsAdmin = $true,
        [int]$OsBuild = 26200,
        [int]$WebRtcUdpStart = 50000,
        [int]$WebRtcUdpEnd = 50100,
        [string]$ListenAddress = '0.0.0.0',
        [string]$CollectorError = '',
        [scriptblock]$CommandRunner
    )
    if (-not $CommandRunner) {
        $CommandRunner = { param($Action) throw "unexpected mutation $($Action.Kind): $($Action.Detail)" }
    }
    $params = @{
        LanAddress        = $LanAddress
        WslAddress        = $WslAddress
        ExistingFirewall  = $ExistingFirewall
        ExistingPortProxy = $ExistingPortProxy
        WslConfigText     = $WslConfigText
        IsAdmin           = $IsAdmin
        OsBuild           = $OsBuild
        WebRtcUdpStart    = $WebRtcUdpStart
        WebRtcUdpEnd      = $WebRtcUdpEnd
        ListenAddress     = $ListenAddress
        WslConfigPath     = 'C:\ohh205-test-not-real\.wslconfig'
        CollectorError    = $CollectorError
        CommandRunner     = $CommandRunner
    }
    if (-not [string]::IsNullOrWhiteSpace($Mode)) { $params.Mode = $Mode }
    if ($WhatIf) { $params.WhatIf = $true }
    if ($DryRun) { $params.DryRun = $true }
    if ($RestartWsl) { $params.RestartWsl = $true }
    if ($ForcePortProxy) { $params.ForcePortProxy = $true }
    $previousMutations = $env:OHHO_VR_SIM_NETWORK_DISABLE_MUTATIONS
    $env:OHHO_VR_SIM_NETWORK_DISABLE_MUTATIONS = '1'
    try {
        return Invoke-VrSimNetworkCore @params
    }
    finally {
        if ($null -eq $previousMutations) {
            Remove-Item Env:\OHHO_VR_SIM_NETWORK_DISABLE_MUTATIONS -ErrorAction SilentlyContinue
        }
        else {
            $env:OHHO_VR_SIM_NETWORK_DISABLE_MUTATIONS = $previousMutations
        }
    }
}

Describe 'vr-sim-network parse and safety boundary' {
    It 'parses with no errors' {
        $n = 0
        if ($null -ne $script:VrSimParseErrors) { $n = @($script:VrSimParseErrors).Count }
        $n | Should Be 0
    }

    It 'keeps mutations inside Invoke-VrSimNetworkAction' {
        $commands = $script:VrSimAst.FindAll({
                param($node)
                $node -is [System.Management.Automation.Language.CommandAst]
            }, $true)
        $exits = @($script:VrSimAst.FindAll({
                    param($node)
                    $node -is [System.Management.Automation.Language.ExitStatementAst]
                }, $true))
        $exits.Count | Should Be 1
        (Get-OhhoAstFunctionName $exits[0]) | Should Be ''
        foreach ($command in $commands) {
            $name = $command.GetCommandName()
            $fn = Get-OhhoAstFunctionName $command
            $text = $command.Extent.Text
            if ($name -in @('New-NetFirewallRule', 'Set-NetFirewallRule', 'Remove-NetFirewallRule', 'Set-NetFirewallPortFilter')) {
                $fn | Should Be 'Invoke-VrSimNetworkAction'
            }
            if ($name -in @('netsh', 'netsh.exe')) {
                if ($text -match '\b(add|delete)\b') {
                    $fn | Should Be 'Invoke-VrSimNetworkAction'
                }
                elseif ($text -match '\bshow\b') {
                    $fn | Should Be 'Get-VrSimLivePortProxy'
                }
                else {
                    throw "unexpected netsh command in ${fn}: $text"
                }
            }
            if ($name -in @('wsl', 'wsl.exe')) {
                if ($text -match '--shutdown') {
                    $fn | Should Be 'Invoke-VrSimNetworkAction'
                }
                else {
                    $fn | Should Be 'Get-VrSimLiveWslAddress'
                }
            }
        }

        $members = $script:VrSimAst.FindAll({
                param($node)
                $node -is [System.Management.Automation.Language.InvokeMemberExpressionAst]
            }, $true)
        $writes = 0
        foreach ($member in $members) {
            $memberName = [string]$member.Member.Value
            if ($memberName -eq 'WriteAllText') {
                $writes++
                (Get-OhhoAstFunctionName $member) | Should Be 'Invoke-VrSimNetworkAction'
            }
        }
        $writes | Should Be 1

        $raw = [System.IO.File]::ReadAllText($script:VrSimNetworkScript)
        ($raw.Contains('Set-NetConnectionProfile')) | Should Be $false
        ($raw.Contains('SupportsShouldProcess')) | Should Be $true
    }

    It 'does not expose an IsAdmin bypass on the public entry function' {
        $fn = $script:VrSimAst.FindAll({
                param($node)
                $node -is [System.Management.Automation.Language.FunctionDefinitionAst] -and $node.Name -eq 'Invoke-VrSimNetwork'
            }, $true) | Select-Object -First 1
        $names = @($fn.Body.ParamBlock.Parameters | ForEach-Object { $_.Name.VariablePath.UserPath })
        ($names -contains 'Mode') | Should Be $true
        ($names -contains 'DryRun') | Should Be $true
        ($names -contains 'IsAdmin') | Should Be $false
        ($names -contains 'CommandRunner') | Should Be $false
    }
}

Describe 'pairing code and LAN selection' {
    It 'matches the CRC-16/CCITT-FALSE check value' {
        (Get-VrSimCrc16Ccitt -Text '123456789') | Should Be 0x29B1
    }

    It 'builds a stable pairing code for the sample LAN address' {
        (Get-VrSimPairingCode -Ipv4 '192.168.1.23') | Should Be 'OHHO-E2A4'
        (Get-VrSimVideoUrl -Ipv4 '192.168.1.23') | Should Be 'http://192.168.1.23:8080'
        (Get-VrSimRosbridgeUrl -Ipv4 '192.168.1.23') | Should Be 'ws://192.168.1.23:9090'
    }

    It 'prefers Wi-Fi over Ethernet, virtual adapters, and non-private addresses' {
        $picked = Select-VrSimLanAddress -Adapters @(
            @{ Address = '172.22.1.1'; Alias = 'vEthernet (WSL)' },
            @{ Address = '169.254.1.1'; Alias = 'Ethernet' },
            @{ Address = '10.0.0.5'; Alias = 'Ethernet' },
            @{ Address = '192.168.1.23'; Alias = 'Wi-Fi' },
            @{ Address = '127.0.0.1'; Alias = 'Loopback' }
        )
        $picked | Should Be '192.168.1.23'
    }

    It 'uses Ethernet when Wi-Fi is absent' {
        $picked = Select-VrSimLanAddress -Adapters @(
            @{ Address = '192.168.65.1'; Alias = 'vEthernet (DockerNAT)' },
            @{ Address = '10.1.2.3'; Alias = 'Ethernet 2' }
        )
        $picked | Should Be '10.1.2.3'
    }
}

Describe 'wslconfig edit' {
    It 'creates a mirrored config and then stays unchanged' {
        $first = Get-VrSimWslConfigEdit -Text ''
        $first.Changed | Should Be $true
        ($first.Text.Contains("networkingMode=mirrored")) | Should Be $true
        ($first.Text.Contains('[wsl2]')) | Should Be $true
        $second = Get-VrSimWslConfigEdit -Text $first.Text
        $second.Changed | Should Be $false
        (Test-VrSimWslMirrored -Text $first.Text) | Should Be $true
    }

    It 'replaces nat and preserves another section' {
        $text = "[boot]`r`nsystemd=true`r`n`r`n[wsl2]`r`nnetworkingMode=nat`r`nmemory=8GB`r`n"
        $edit = Get-VrSimWslConfigEdit -Text $text
        $edit.Changed | Should Be $true
        ($edit.Text.Contains('systemd=true')) | Should Be $true
        ($edit.Text.Contains('memory=8GB')) | Should Be $true
        ($edit.Text.Contains('networkingMode=mirrored')) | Should Be $true
        ($edit.Text.Contains('networkingMode=nat')) | Should Be $false
        $again = Get-VrSimWslConfigEdit -Text $edit.Text
        $again.Changed | Should Be $false
    }
}

Describe 'portproxy parser and firewall specs' {
    It 'parses netsh v4tov4 rows and ignores the header' {
        $sample = @"
Listen on ipv4:             Connect to ipv4:

Address         Port        Address         Port
--------------- ----------  --------------- ----------
0.0.0.0         9090        172.22.1.2      9090
192.168.1.23    8080        172.22.1.2      8080

"@
        $rows = @(ConvertFrom-VrSimPortProxy -Text $sample)
        $rows.Count | Should Be 2
        $rows[0].ListenPort | Should Be 9090
        $rows[0].ConnectAddress | Should Be '172.22.1.2'
        $rows[1].ListenAddress | Should Be '192.168.1.23'
    }

    It 'opens only TCP 9090, TCP 8080, and the WebRTC UDP range on Private' {
        $specs = @(Get-VrSimFirewallSpecs)
        $specs.Count | Should Be 3
        @($specs | Where-Object { $_.LocalPort -eq '9090' -and $_.Protocol -eq 'TCP' }).Count | Should Be 1
        @($specs | Where-Object { $_.LocalPort -eq '8080' -and $_.Protocol -eq 'TCP' }).Count | Should Be 1
        @($specs | Where-Object { $_.LocalPort -eq '50000-50100' -and $_.Protocol -eq 'UDP' }).Count | Should Be 1
        foreach ($spec in $specs) {
            $spec.Profile | Should Be 'Private'
            $spec.RemoteAddress | Should Be 'LocalSubnet'
            $spec.Direction | Should Be 'Inbound'
            ($spec.LocalPort -eq '8765') | Should Be $false
        }
    }
}

Describe 'network plan' {
    It 'plans portproxy and three firewall rules, and a second pass is empty' {
        $plan = Get-VrSimNetworkPlan -Mode PortProxy -LanAddress '192.168.1.23' -WslAddress '172.22.1.2' -ExistingFirewall @() -ExistingPortProxy @() -WslConfigText '' -OsBuild 26200
        $details = @($plan.Actions | ForEach-Object { $_.Detail })
        $details.Count | Should Be 5
        ($details -join "`n") | Should Match 'portproxy add 0\.0\.0\.0:9090 -> 172\.22\.1\.2:9090'
        ($details -join "`n") | Should Match 'portproxy add 0\.0\.0\.0:8080 -> 172\.22\.1\.2:8080'
        ($details -join "`n") | Should Match 'TCP 9090'
        ($details -join "`n") | Should Match 'TCP 8080'
        ($details -join "`n") | Should Match 'UDP 50000-50100'
        ($details -join "`n") | Should Not Match '8765'
        ($details -join "`n") | Should Not Match 'wsl --shutdown'
        @($plan.Actions | Where-Object { $_.Kind -eq 'portproxy-add' -and $_.Proxy.ConnectPort -notin 9090, 8080 }).Count | Should Be 0
        $state = Merge-VrSimNetworkState -State @{ Firewall = @(); PortProxy = @(); WslConfigText = '' } -Actions $plan.Actions
        $again = Get-VrSimNetworkPlan -Mode PortProxy -LanAddress '192.168.1.23' -WslAddress '172.22.1.2' -ExistingFirewall $state.Firewall -ExistingPortProxy $state.PortProxy -WslConfigText $state.WslConfigText -OsBuild 26200
        @($again.Actions).Count | Should Be 0
    }

    It 'replaces a stale WSL target and leaves an unrelated proxy' {
        $existingProxy = @(
            [pscustomobject]@{ ListenAddress = '0.0.0.0'; ListenPort = 9090; ConnectAddress = '172.22.1.2'; ConnectPort = 9090 },
            [pscustomobject]@{ ListenAddress = '0.0.0.0'; ListenPort = 8080; ConnectAddress = '172.22.1.2'; ConnectPort = 8080 },
            [pscustomobject]@{ ListenAddress = '0.0.0.0'; ListenPort = 8765; ConnectAddress = '172.22.1.2'; ConnectPort = 8765 }
        )
        $plan = Get-VrSimNetworkPlan -Mode PortProxy -WslAddress '172.22.1.9' -ExistingFirewall @(Get-VrSimFirewallSpecs) -ExistingPortProxy $existingProxy -WslConfigText '' -OsBuild 26200
        $details = @($plan.Actions | ForEach-Object { $_.Detail }) -join "`n"
        $details | Should Match 'portproxy delete 0\.0\.0\.0:9090 -> 172\.22\.1\.2:9090'
        $details | Should Match 'portproxy add 0\.0\.0\.0:9090 -> 172\.22\.1\.9:9090'
        $details | Should Not Match '8765'
        $state = Merge-VrSimNetworkState -State @{ Firewall = @(Get-VrSimFirewallSpecs); PortProxy = $existingProxy; WslConfigText = '' } -Actions $plan.Actions
        @($state.PortProxy | Where-Object { $_.ListenPort -eq 8765 }).Count | Should Be 1
        $again = Get-VrSimNetworkPlan -Mode PortProxy -WslAddress '172.22.1.9' -ExistingFirewall $state.Firewall -ExistingPortProxy $state.PortProxy -WslConfigText '' -OsBuild 26200
        @($again.Actions).Count | Should Be 0
    }

    It 'plans mirrored networking, removes owned proxies, and shuts WSL down only when asked' {
        $proxy = @([pscustomobject]@{ ListenAddress = '0.0.0.0'; ListenPort = 9090; ConnectAddress = '172.22.1.2'; ConnectPort = 9090 })
        $plan = Get-VrSimNetworkPlan -Mode Mirrored -WslAddress '172.22.1.2' -ExistingFirewall @() -ExistingPortProxy $proxy -WslConfigText '' -OsBuild 26200 -RestartWsl
        $details = @($plan.Actions | ForEach-Object { $_.Detail }) -join "`n"
        $details | Should Match 'wslconfig set networkingMode=mirrored'
        $details | Should Match 'wsl --shutdown'
        $details | Should Match 'portproxy delete 0\.0\.0\.0:9090'
        $details | Should Not Match 'portproxy add'
        $state = Merge-VrSimNetworkState -State @{ Firewall = @(); PortProxy = $proxy; WslConfigText = '' } -Actions $plan.Actions
        (Test-VrSimWslMirrored -Text $state.WslConfigText) | Should Be $true
        $again = Get-VrSimNetworkPlan -Mode Mirrored -WslAddress '172.22.1.2' -ExistingFirewall $state.Firewall -ExistingPortProxy $state.PortProxy -WslConfigText $state.WslConfigText -OsBuild 26200 -RestartWsl
        @($again.Actions).Count | Should Be 0
    }

    It 'omits wsl --shutdown when RestartWsl is off' {
        $plan = Get-VrSimNetworkPlan -Mode Mirrored -WslConfigText '' -OsBuild 26200 -ExistingFirewall @() -ExistingPortProxy @()
        $kinds = @($plan.Actions | ForEach-Object { $_.Kind })
        ($kinds -contains 'wsl-shutdown') | Should Be $false
        ($kinds -contains 'wslconfig-write') | Should Be $true
    }

    It 'refuses mirrored mode on an older build and still plans portproxy' {
        $mirrored = Get-VrSimNetworkPlan -Mode Mirrored -OsBuild 19041 -WslConfigText '' -ExistingFirewall @() -ExistingPortProxy @()
        @($mirrored.Actions).Count | Should Be 0
        (@($mirrored.Errors) -join "`n") | Should Match '22621'
        $proxy = Get-VrSimNetworkPlan -Mode PortProxy -OsBuild 19041 -WslAddress '172.22.1.2' -ExistingFirewall @() -ExistingPortProxy @() -WslConfigText ''
        @($proxy.Errors).Count | Should Be 0
        @($proxy.Actions).Count | Should Be 5
    }

    It 'errors when portproxy has no WSL address and does not invent a connect address' {
        $plan = Get-VrSimNetworkPlan -Mode PortProxy -WslAddress '' -ExistingFirewall @() -ExistingPortProxy @() -WslConfigText '' -OsBuild 26200
        (@($plan.Errors) -join "`n") | Should Match 'WSL IPv4 is unknown'
        @($plan.Actions | Where-Object { $_.Kind -like 'portproxy-*' }).Count | Should Be 0
    }

    It 'leaves portproxy alone when mirrored is already configured' {
        $config = Get-VrSimWslConfigEdit -Text ''
        $plan = Get-VrSimNetworkPlan -Mode PortProxy -WslAddress '172.22.1.2' -WslConfigText $config.Text -ExistingFirewall @() -ExistingPortProxy @() -OsBuild 26200
        @($plan.Actions | Where-Object { $_.Kind -like 'portproxy-*' }).Count | Should Be 0
        (@($plan.Warnings) -join "`n") | Should Match 'left unchanged'
        $forced = Get-VrSimNetworkPlan -Mode PortProxy -WslAddress '172.22.1.2' -WslConfigText $config.Text -ExistingFirewall @() -ExistingPortProxy @() -OsBuild 26200 -ForcePortProxy
        @($forced.Actions | Where-Object { $_.Kind -eq 'portproxy-add' }).Count | Should Be 2
    }

    It 'updates a drifted firewall rule and retires an old WebRTC range' {
        $old = [pscustomobject]@{
            Name = 'OhhoVrSim-WebRTC-UDP-60000-60010'
            DisplayName = 'OhhO VR sim WebRTC (UDP 60000-60010)'
            Protocol = 'UDP'
            LocalPort = '60000-60010'
            Profile = 'Private'
            Direction = 'Inbound'
            Action = 'Allow'
            Enabled = $true
            RemoteAddress = 'LocalSubnet'
        }
        $publicRos = [pscustomobject]@{
            Name = 'OhhoVrSim-Rosbridge-TCP-9090'
            DisplayName = 'OhhO VR sim rosbridge (TCP 9090)'
            Protocol = 'TCP'
            LocalPort = '9090'
            Profile = 'Public'
            Direction = 'Inbound'
            Action = 'Allow'
            Enabled = $true
            RemoteAddress = 'LocalSubnet'
        }
        $plan = Get-VrSimNetworkPlan -Mode Mirrored -OsBuild 26200 -WslConfigText (Get-VrSimWslConfigEdit -Text '').Text -ExistingFirewall @($old, $publicRos) -ExistingPortProxy @()
        $kinds = @($plan.Actions | ForEach-Object { $_.Kind })
        ($kinds -contains 'firewall-remove') | Should Be $true
        ($kinds -contains 'firewall-update') | Should Be $true
        ($kinds -contains 'wsl-shutdown') | Should Be $false
    }
}

Describe 'dry run and apply gate' {
    It 'WhatIf prints the LAN IP, QR URL, and pairing code and applies nothing' {
        $result = Invoke-VrSimSafe -Mode PortProxy -WhatIf
        $result.ExitCode | Should Be 0
        $result.Dry | Should Be $true
        @($result.Applied).Count | Should Be 0
        ($result.Report.Contains('192.168.1.23')) | Should Be $true
        ($result.Report.Contains('http://192.168.1.23:8080')) | Should Be $true
        ($result.Report.Contains('ws://192.168.1.23:9090')) | Should Be $true
        ($result.Report.Contains('OHHO-E2A4')) | Should Be $true
        ($result.Report.Contains('50000-50100')) | Should Be $true
        ($result.Report.Contains('Status: dry run, nothing changed.')) | Should Be $true
        ($result.Report.Contains([char]27)) | Should Be $true
    }

    It 'DryRun matches WhatIf for a non-admin session' {
        $result = Invoke-VrSimSafe -Mode Mirrored -DryRun -IsAdmin $false -RestartWsl
        $result.ExitCode | Should Be 0
        ($result.Report.Contains('wsl --shutdown')) | Should Be $true
        ($result.Report.Contains('Status: dry run, nothing changed.')) | Should Be $true
    }

    It 'asks Varun to choose a mode and changes nothing' {
        $result = Invoke-VrSimSafe -WhatIf
        $result.ExitCode | Should Be 2
        ($result.Report.Contains('Varun TODO')) | Should Be $true
        ($result.Report.Contains('Plan Mirrored:')) | Should Be $true
        ($result.Report.Contains('Plan PortProxy:')) | Should Be $true
        ($result.Report.Contains('Status: no mode selected, nothing changed.')) | Should Be $true
    }

    It 'still asks for a mode when the WSL address is unknown' {
        $result = Invoke-VrSimSafe -WslAddress '' -WhatIf
        $result.ExitCode | Should Be 2
        ($result.Report.Contains('WSL IPv4 is unknown')) | Should Be $true
        ($result.Report.Contains('Varun TODO')) | Should Be $true
    }

    It 'refuses to apply without elevation' {
        $result = Invoke-VrSimSafe -Mode PortProxy -IsAdmin $false
        $result.ExitCode | Should Be 1
        @($result.Applied).Count | Should Be 0
        ($result.Report.Contains('administrator PowerShell')) | Should Be $true
    }

    It 'records planned actions on an elevated apply through the injected runner' {
        $bag = @{ Calls = New-Object System.Collections.Generic.List[object] }
        $runner = {
            param($Action)
            [void]$bag.Calls.Add($Action)
        }.GetNewClosure()
        $result = Invoke-VrSimSafe -Mode PortProxy -IsAdmin $true -CommandRunner $runner
        $result.ExitCode | Should Be 0
        $bag.Calls.Count | Should Be 5
        ($result.Report.Contains('Status: applied 5 actions.')) | Should Be $true
        @($bag.Calls | Where-Object { $_.Kind -eq 'wsl-shutdown' }).Count | Should Be 0
        @($bag.Calls | Where-Object { $_.Detail -match '8765' }).Count | Should Be 0
    }

    It 'reports an already-matching elevated apply as up to date' {
        $first = Get-VrSimNetworkPlan -Mode PortProxy -WslAddress '172.22.1.2' -ExistingFirewall @() -ExistingPortProxy @() -WslConfigText '' -OsBuild 26200
        $state = Merge-VrSimNetworkState -State @{ Firewall = @(); PortProxy = @(); WslConfigText = '' } -Actions $first.Actions
        $bag = @{ Calls = 0 }
        $runner = { param($Action) $bag.Calls++ }.GetNewClosure()
        $result = Invoke-VrSimSafe -Mode PortProxy -IsAdmin $true -ExistingFirewall $state.Firewall -ExistingPortProxy $state.PortProxy -WslConfigText $state.WslConfigText -CommandRunner $runner
        $result.ExitCode | Should Be 0
        $bag.Calls | Should Be 0
        ($result.Report.Contains('already up to date')) | Should Be $true
    }
}

Describe 'QR code' {
    It 'builds format bits for ECC M' {
        (Get-QrFormatBits -Mask 0) | Should Be 0x5412
        (Get-QrFormatBits -Mask 1) | Should Be 0x5125
        (Get-QrFormatBits -Mask 2) | Should Be 0x5E7C
        (Get-QrFormatBits -Mask 7) | Should Be 0x4AA0
    }

    It 'matches the Reed-Solomon remainder vector' {
        $gf = Get-VrSimQrGf
        $rem = Get-QrRsRemainder -Gf $gf -Data ([int[]](64, 24, 109, 111)) -Degree 10
        @($rem).Count | Should Be 10
        $rem[0] | Should Be 65
        $rem[1] | Should Be 206
        $rem[9] | Should Be 101
    }

    It 'draws finder patterns and a stable matrix for the video URL' {
        $qr = Get-QrMatrix -Text 'http://192.168.1.23:8080'
        $qr.Version | Should Be 2
        $qr.Mask | Should Be 5
        $qr.Size | Should Be 25
        $qr.Rows[0].Substring(0, 7) | Should Be '1111111'
        $qr.Rows[0].Substring($qr.Size - 7, 7) | Should Be '1111111'
        $qr.Rows[7][0] | Should Be '0'
        $again = Get-QrMatrix -Text 'http://192.168.1.23:8080'
        ($qr.Rows -join '') | Should Be ($again.Rows -join '')
        $sha = [System.Security.Cryptography.SHA256]::Create()
        $hashBytes = $sha.ComputeHash([System.Text.Encoding]::ASCII.GetBytes(($qr.Rows -join '')))
        $hash = ([System.BitConverter]::ToString($hashBytes)).Replace('-', '')
        # Mask 5 symbol for this URL. Placement matches segno's placer for that mask.
        $hash | Should Be 'D87C98E9E66F9F62F12F59F6D87E422D000B948CDB24DB4BC9B1CA5A696F5744'
        $ansi = Format-QrAnsi -Matrix $qr
        ($ansi.Contains([string][char]27 + '[30;47m')) | Should Be $true
    }
}
