# Copyright 2026 OhhO Robotics
# SPDX-License-Identifier: Apache-2.0
<#
.SYNOPSIS
    One-shot Windows setup so a Quest on the LAN can reach the WSL2 sim.

.DESCRIPTION
    Prints the PC LAN IPv4, the Quest video URL (http://<ip>:8080), the
    rosbridge URL (ws://<ip>:9090), a pairing code, and a QR code of the
    video URL.

    Does not choose mirrored networking vs portproxy. Pass -Mode Mirrored
    or -Mode PortProxy. With no -Mode, the script prints both plans and
    exits 2.

    -WhatIf and -DryRun print the plan and change nothing. Applying a mode
    requires an elevated PowerShell. Firewall rules are inbound Allow on the
    Private profile, remote address LocalSubnet, and only these ports:
    TCP 9090, TCP 8080, and UDP 50000-50100 (override with -WebRtcUdpStart
    and -WebRtcUdpEnd).

    netsh interface portproxy forwards TCP. The UDP range is a firewall rule
    for mirrored mode, where WSL shares the host interfaces.

    See docs/vr-sim-network-windows.md.

.PARAMETER Mode
    Mirrored or PortProxy. Required to apply. Omit to print both plans.

.PARAMETER DryRun
    Print the plan and change nothing. Same result as -WhatIf.

.PARAMETER RestartWsl
    With -Mode Mirrored, run wsl --shutdown after .wslconfig actually changes.
    That stops every WSL distro. Ignored with -WhatIf and -DryRun.

.PARAMETER ForcePortProxy
    Add TCP proxies even when .wslconfig already sets networkingMode=mirrored.

.PARAMETER WslDistro
    Distro passed to wsl.exe -d when reading the IPv4 address. Default navdev.

.PARAMETER ListenAddress
    IPv4 address portproxy listens on. Default 0.0.0.0.

.PARAMETER WebRtcUdpStart
    First UDP port of the WebRTC firewall range. Default 50000.

.PARAMETER WebRtcUdpEnd
    Last UDP port of the WebRTC firewall range. Default 50100.

.PARAMETER LanAddress
    Override the detected Wi-Fi or Ethernet IPv4.

.PARAMETER WslAddress
    Override the detected WSL IPv4 used as the portproxy connect address.

.EXAMPLE
    powershell -NoProfile -File .\scripts\windows\vr-sim-network.ps1 -WhatIf

.EXAMPLE
    powershell -NoProfile -File .\scripts\windows\vr-sim-network.ps1 -Mode Mirrored -WhatIf

.EXAMPLE
    powershell -NoProfile -File .\scripts\windows\vr-sim-network.ps1 -Mode PortProxy

.NOTES
    Exit 0: dry run of a chosen mode, or apply finished.
    Exit 1: the plan has errors, or apply was requested without elevation.
    Exit 2: no -Mode, so nothing was changed.
    OHHO_VR_SIM_NETWORK_DISABLE_MUTATIONS=1 makes the apply step throw
    instead of calling netsh, the firewall cmdlets, or wsl --shutdown.
#>

$script:VrSimRosbridgePort = 9090
$script:VrSimVideoPort = 8080
$script:VrSimWebRtcUdpStart = 50000
$script:VrSimWebRtcUdpEnd = 50100
$script:VrSimMirroredMinBuild = 22621

function Get-VrSimQrGf {
    $existing = Get-Variable -Name VrSimQrGf -Scope Script -ErrorAction SilentlyContinue
    if ($existing) { return $existing.Value }
    $exp = New-Object int[] 512
    $log = New-Object int[] 256
    $x = 1
    for ($i = 0; $i -lt 255; $i++) {
        $exp[$i] = $x
        $log[$x] = $i
        $x = $x * 2
        if ($x -ge 256) { $x = $x -bxor 0x11D }
    }
    for ($i = 255; $i -lt 512; $i++) {
        $exp[$i] = $exp[$i - 255]
    }
    $script:VrSimQrGf = @{ Exp = $exp; Log = $log }
    return $script:VrSimQrGf
}

function Get-QrGfMul {
    param($Gf, [int]$A, [int]$B)
    if ($A -eq 0 -or $B -eq 0) { return 0 }
    return [int]$Gf.Exp[([int]$Gf.Log[$A] + [int]$Gf.Log[$B])]
}

function Get-QrRsDivisor {
    param($Gf, [int]$Degree)
    $result = New-Object int[] $Degree
    $result[$Degree - 1] = 1
    $root = 1
    for ($i = 0; $i -lt $Degree; $i++) {
        for ($j = 0; $j -lt $result.Length; $j++) {
            $result[$j] = Get-QrGfMul -Gf $Gf -A $result[$j] -B $root
            if (($j + 1) -lt $result.Length) {
                $result[$j] = $result[$j] -bxor $result[$j + 1]
            }
        }
        $root = Get-QrGfMul -Gf $Gf -A $root -B 2
    }
    return ,$result
}

function Get-QrRsRemainder {
    param($Gf, [int[]]$Data, [int]$Degree)
    $divisor = Get-QrRsDivisor -Gf $Gf -Degree $Degree
    $result = New-Object int[] $Degree
    foreach ($b in $Data) {
        $factor = ([int]$b) -bxor $result[0]
        for ($k = 0; $k -lt ($Degree - 1); $k++) {
            $result[$k] = $result[$k + 1]
        }
        $result[$Degree - 1] = 0
        for ($i = 0; $i -lt $Degree; $i++) {
            $result[$i] = $result[$i] -bxor (Get-QrGfMul -Gf $Gf -A $divisor[$i] -B $factor)
        }
    }
    return ,$result
}

function Add-QrBits {
    param($List, [int]$Value, [int]$Length)
    for ($i = $Length - 1; $i -ge 0; $i--) {
        [void]$List.Add([int](($Value -shr $i) -band 1))
    }
}

function Get-QrVersionSpec {
    param([int]$Version)
    # ECC M block layout. Data is the data-codeword count of each block.
    switch ($Version) {
        1 { return @{ Version = 1; Ec = 10; Data = @(16); Align = @(); Remainder = 0; RawModules = 208 } }
        2 { return @{ Version = 2; Ec = 16; Data = @(28); Align = @(6, 18); Remainder = 7; RawModules = 359 } }
        3 { return @{ Version = 3; Ec = 26; Data = @(44); Align = @(6, 22); Remainder = 7; RawModules = 567 } }
        4 { return @{ Version = 4; Ec = 18; Data = @(32, 32); Align = @(6, 26); Remainder = 7; RawModules = 807 } }
        5 { return @{ Version = 5; Ec = 24; Data = @(43, 43); Align = @(6, 30); Remainder = 7; RawModules = 1079 } }
        6 { return @{ Version = 6; Ec = 16; Data = @(27, 27, 27, 27); Align = @(6, 34); Remainder = 7; RawModules = 1383 } }
        default { throw "QR version $Version is outside 1-6" }
    }
}

function Get-QrVersionForByteCount {
    param([int]$Count)
    for ($version = 1; $version -le 6; $version++) {
        $spec = Get-QrVersionSpec -Version $version
        $capacity = 0
        foreach ($n in $spec.Data) { $capacity += [int]$n }
        $neededBits = 4 + 8 + (8 * $Count)
        if ($neededBits -le ($capacity * 8)) { return $version }
    }
    throw "QR payload is $Count bytes, past version 6-M"
}

function Get-QrFormatBits {
    param([int]$Mask)
    # ECC level M is format bits 00, so the 5-bit data value is the mask id.
    $data = $Mask
    $rem = $data
    for ($i = 0; $i -lt 10; $i++) {
        $hi = ($rem -shr 9) -band 1
        $rem = $rem -shl 1
        if ($hi -eq 1) { $rem = $rem -bxor 0x537 }
    }
    return (((($data -shl 10) -bor $rem) -bxor 0x5412) -band 0x7FFF)
}

function New-QrGrid {
    param([int]$Size)
    $module = New-Object 'int[][]' $Size
    $func = New-Object 'bool[][]' $Size
    for ($y = 0; $y -lt $Size; $y++) {
        $module[$y] = New-Object int[] $Size
        $func[$y] = New-Object bool[] $Size
    }
    return @{ Size = $Size; Module = $module; Func = $func }
}

function Set-QrModule {
    param($Grid, [int]$X, [int]$Y, [bool]$Dark, [bool]$Function)
    $Grid.Module[$Y][$X] = [int]([bool]$Dark)
    if ($Function) { $Grid.Func[$Y][$X] = $true }
}

function Set-QrFinder {
    param($Grid, [int]$X, [int]$Y)
    for ($dy = -1; $dy -le 7; $dy++) {
        for ($dx = -1; $dx -le 7; $dx++) {
            $xx = $X + $dx
            $yy = $Y + $dy
            if ($xx -lt 0 -or $yy -lt 0 -or $xx -ge $Grid.Size -or $yy -ge $Grid.Size) { continue }
            $dark = $false
            if ($dx -ge 0 -and $dx -le 6 -and $dy -ge 0 -and $dy -le 6) {
                $edge = ($dx -eq 0 -or $dx -eq 6 -or $dy -eq 0 -or $dy -eq 6)
                $core = ($dx -ge 2 -and $dx -le 4 -and $dy -ge 2 -and $dy -le 4)
                $dark = $edge -or $core
            }
            Set-QrModule -Grid $Grid -X $xx -Y $yy -Dark $dark -Function $true
        }
    }
}

function Set-QrAlignment {
    param($Grid, [int]$Cx, [int]$Cy)
    for ($dy = -2; $dy -le 2; $dy++) {
        for ($dx = -2; $dx -le 2; $dx++) {
            $dist = [Math]::Max([Math]::Abs($dx), [Math]::Abs($dy))
            $dark = ($dist -ne 1)
            Set-QrModule -Grid $Grid -X ($Cx + $dx) -Y ($Cy + $dy) -Dark $dark -Function $true
        }
    }
}

function Reserve-QrFormat {
    param($Grid)
    $size = $Grid.Size
    for ($i = 0; $i -le 5; $i++) { Set-QrModule -Grid $Grid -X 8 -Y $i -Dark $false -Function $true }
    Set-QrModule -Grid $Grid -X 8 -Y 7 -Dark $false -Function $true
    Set-QrModule -Grid $Grid -X 8 -Y 8 -Dark $false -Function $true
    Set-QrModule -Grid $Grid -X 7 -Y 8 -Dark $false -Function $true
    for ($i = 9; $i -lt 15; $i++) { Set-QrModule -Grid $Grid -X (14 - $i) -Y 8 -Dark $false -Function $true }
    for ($i = 0; $i -lt 8; $i++) { Set-QrModule -Grid $Grid -X ($size - 1 - $i) -Y 8 -Dark $false -Function $true }
    for ($i = 8; $i -lt 15; $i++) { Set-QrModule -Grid $Grid -X 8 -Y ($size - 15 + $i) -Dark $false -Function $true }
    Set-QrModule -Grid $Grid -X 8 -Y ($size - 8) -Dark $true -Function $true
}

function Set-QrFormatOnMatrix {
    param($Matrix, [int]$Bits)
    $size = $Matrix.Length
    $pairs = New-Object System.Collections.Generic.List[object]
    for ($i = 0; $i -le 5; $i++) { [void]$pairs.Add(@(8, $i, $i)) }
    [void]$pairs.Add(@(8, 7, 6))
    [void]$pairs.Add(@(8, 8, 7))
    [void]$pairs.Add(@(7, 8, 8))
    for ($i = 9; $i -lt 15; $i++) { [void]$pairs.Add(@((14 - $i), 8, $i)) }
    for ($i = 0; $i -lt 8; $i++) { [void]$pairs.Add(@(($size - 1 - $i), 8, $i)) }
    for ($i = 8; $i -lt 15; $i++) { [void]$pairs.Add(@(8, ($size - 15 + $i), $i)) }
    foreach ($pair in $pairs) {
        $bit = ($Bits -shr $pair[2]) -band 1
        $Matrix[$pair[1]][$pair[0]] = $bit
    }
    $Matrix[($size - 8)][8] = 1
}

function Test-QrMask {
    param([int]$Mask, [int]$X, [int]$Y)
    switch ($Mask) {
        0 { return ((($X + $Y) % 2) -eq 0) }
        1 { return (($Y % 2) -eq 0) }
        2 { return (($X % 3) -eq 0) }
        3 { return ((($X + $Y) % 3) -eq 0) }
        4 {
            $y2 = [int]($Y / 2)
            $x3 = [int]($X / 3)
            return ((($y2 + $x3) % 2) -eq 0)
        }
        5 { return (((($X * $Y) % 2) + (($X * $Y) % 3)) -eq 0) }
        6 { return ((((($X * $Y) % 2) + (($X * $Y) % 3)) % 2) -eq 0) }
        7 { return ((((($X + $Y) % 2) + (($X * $Y) % 3)) % 2) -eq 0) }
        default { throw "bad QR mask $Mask" }
    }
}

function Add-QrFinderHistory {
    param([int]$RunLength, [int[]]$Hist, [int]$Size)
    if ($Hist[0] -eq 0) { $RunLength += $Size }
    for ($i = $Hist.Length - 1; $i -ge 1; $i--) { $Hist[$i] = $Hist[$i - 1] }
    $Hist[0] = $RunLength
}

function Get-QrFinderPatternCount {
    param([int[]]$Hist)
    $n = $Hist[1]
    $core = ($n -gt 0 -and $Hist[2] -eq $n -and $Hist[3] -eq ($n * 3) -and $Hist[4] -eq $n -and $Hist[5] -eq $n)
    $count = 0
    if ($core -and $Hist[0] -ge ($n * 4) -and $Hist[6] -ge $n) { $count += 1 }
    if ($core -and $Hist[6] -ge ($n * 4) -and $Hist[0] -ge $n) { $count += 1 }
    return $count
}

function Complete-QrFinderPenalty {
    param([int]$RunColor, [int]$RunLen, [int[]]$Hist, [int]$Size)
    if ($RunColor -eq 1) {
        Add-QrFinderHistory -RunLength $RunLen -Hist $Hist -Size $Size
        $RunLen = 0
    }
    $RunLen += $Size
    Add-QrFinderHistory -RunLength $RunLen -Hist $Hist -Size $Size
    return (Get-QrFinderPatternCount -Hist $Hist)
}

function Get-QrLinePenalty {
    param($Modules, [int]$Fixed, [bool]$Column)
    $size = $Modules.Length
    $result = 0
    $runColor = 0
    $runLen = 0
    $hist = New-Object int[] 7
    for ($i = 0; $i -lt $size; $i++) {
        $color = if ($Column) { $Modules[$i][$Fixed] } else { $Modules[$Fixed][$i] }
        if ($color -eq $runColor) {
            $runLen++
            if ($runLen -eq 5) { $result += 3 }
            elseif ($runLen -gt 5) { $result += 1 }
        }
        else {
            Add-QrFinderHistory -RunLength $runLen -Hist $hist -Size $size
            if ($runColor -eq 0) { $result += (Get-QrFinderPatternCount -Hist $hist) * 40 }
            $runColor = $color
            $runLen = 1
        }
    }
    $result += (Complete-QrFinderPenalty -RunColor $runColor -RunLen $runLen -Hist $hist -Size $size) * 40
    return $result
}

function Get-QrPenalty {
    param($Modules)
    $size = $Modules.Length
    $result = 0
    for ($i = 0; $i -lt $size; $i++) {
        $result += Get-QrLinePenalty -Modules $Modules -Fixed $i -Column $false
        $result += Get-QrLinePenalty -Modules $Modules -Fixed $i -Column $true
    }
    for ($y = 0; $y -lt ($size - 1); $y++) {
        for ($x = 0; $x -lt ($size - 1); $x++) {
            $c = $Modules[$y][$x]
            if ($c -eq $Modules[$y][$x + 1] -and $c -eq $Modules[$y + 1][$x] -and $c -eq $Modules[$y + 1][$x + 1]) {
                $result += 3
            }
        }
    }
    $dark = 0
    for ($y = 0; $y -lt $size; $y++) {
        for ($x = 0; $x -lt $size; $x++) {
            if ($Modules[$y][$x] -eq 1) { $dark++ }
        }
    }
    $total = $size * $size
    $k = [int][Math]::Floor((([Math]::Abs(($dark * 20) - ($total * 10)) + $total - 1) / $total))
    $result += ($k * 10)
    return $result
}

function Copy-QrModules {
    param($Grid)
    $size = $Grid.Size
    $copy = New-Object 'int[][]' $size
    for ($y = 0; $y -lt $size; $y++) {
        $copy[$y] = New-Object int[] $size
        [Array]::Copy($Grid.Module[$y], $copy[$y], $size)
    }
    return $copy
}

function Get-QrMatrix {
    param([Parameter(Mandatory = $true)][string]$Text)
    $payload = [System.Text.Encoding]::UTF8.GetBytes($Text)
    $version = Get-QrVersionForByteCount -Count $payload.Length
    $spec = Get-QrVersionSpec -Version $version
    $capacity = 0
    foreach ($n in $spec.Data) { $capacity += [int]$n }
    $bits = New-Object System.Collections.Generic.List[int]
    Add-QrBits -List $bits -Value 0x4 -Length 4
    Add-QrBits -List $bits -Value $payload.Length -Length 8
    foreach ($b in $payload) { Add-QrBits -List $bits -Value ([int]$b) -Length 8 }
    $capacityBits = $capacity * 8
    $term = [Math]::Min(4, $capacityBits - $bits.Count)
    if ($term -gt 0) { Add-QrBits -List $bits -Value 0 -Length $term }
    while (($bits.Count % 8) -ne 0) { [void]$bits.Add(0) }
    $pad = 0xEC
    while ($bits.Count -lt $capacityBits) {
        Add-QrBits -List $bits -Value $pad -Length 8
        if ($pad -eq 0xEC) { $pad = 0x11 } else { $pad = 0xEC }
    }
    $dataBytes = New-Object int[] $capacity
    for ($i = 0; $i -lt $capacity; $i++) {
        $v = 0
        for ($j = 0; $j -lt 8; $j++) { $v = ($v -shl 1) -bor $bits[($i * 8) + $j] }
        $dataBytes[$i] = $v
    }
    $gf = Get-VrSimQrGf
    $blocks = @()
    $offset = 0
    foreach ($dlen in $spec.Data) {
        $slice = New-Object int[] $dlen
        [Array]::Copy($dataBytes, $offset, $slice, 0, $dlen)
        $offset += $dlen
        $ec = Get-QrRsRemainder -Gf $gf -Data $slice -Degree $spec.Ec
        $blocks += ,[pscustomobject]@{ Data = $slice; Ec = $ec }
    }
    $interleaved = New-Object System.Collections.Generic.List[int]
    $maxData = 0
    foreach ($n in $spec.Data) { if ($n -gt $maxData) { $maxData = $n } }
    for ($i = 0; $i -lt $maxData; $i++) {
        foreach ($block in $blocks) {
            if ($i -lt $block.Data.Length) { [void]$interleaved.Add([int]$block.Data[$i]) }
        }
    }
    for ($i = 0; $i -lt $spec.Ec; $i++) {
        foreach ($block in $blocks) { [void]$interleaved.Add([int]$block.Ec[$i]) }
    }
    $allBits = New-Object System.Collections.Generic.List[int]
    foreach ($byte in $interleaved) { Add-QrBits -List $allBits -Value ([int]$byte) -Length 8 }
    for ($i = 0; $i -lt $spec.Remainder; $i++) { [void]$allBits.Add(0) }

    $size = 21 + (4 * ($version - 1))
    $grid = New-QrGrid -Size $size
    Set-QrFinder -Grid $grid -X 0 -Y 0
    Set-QrFinder -Grid $grid -X ($size - 7) -Y 0
    Set-QrFinder -Grid $grid -X 0 -Y ($size - 7)
    foreach ($ry in $spec.Align) {
        foreach ($cx in $spec.Align) {
            if ($grid.Func[$ry][$cx]) { continue }
            Set-QrAlignment -Grid $grid -Cx $cx -Cy $ry
        }
    }
    for ($i = 0; $i -lt $size; $i++) {
        $dark = (($i % 2) -eq 0)
        if (-not $grid.Func[6][$i]) { Set-QrModule -Grid $grid -X $i -Y 6 -Dark $dark -Function $true }
        if (-not $grid.Func[$i][6]) { Set-QrModule -Grid $grid -X 6 -Y $i -Dark $dark -Function $true }
    }
    Reserve-QrFormat -Grid $grid
    $funcCount = 0
    for ($y = 0; $y -lt $size; $y++) {
        for ($x = 0; $x -lt $size; $x++) {
            if ($grid.Func[$y][$x]) { $funcCount++ }
        }
    }
    $dataModules = ($size * $size) - $funcCount
    if ($dataModules -ne $spec.RawModules) {
        throw "QR v$version function modules leave $dataModules data modules, expected $($spec.RawModules)"
    }
    if ($allBits.Count -ne $dataModules) {
        throw "QR v$version has $($allBits.Count) bits for $dataModules modules"
    }
    $bitIndex = 0
    for ($right = $size - 1; $right -ge 1; $right -= 2) {
        if ($right -eq 6) { $right = 5 }
        for ($vert = 0; $vert -lt $size; $vert++) {
            for ($j = 0; $j -le 1; $j++) {
                $x = $right - $j
                $upward = ((($right + 1) -band 2) -eq 0)
                if ($upward) { $y = $size - 1 - $vert } else { $y = $vert }
                if (-not $grid.Func[$y][$x]) {
                    $grid.Module[$y][$x] = $allBits[$bitIndex]
                    $bitIndex++
                }
            }
        }
    }
    if ($bitIndex -ne $allBits.Count) {
        throw "QR placement wrote $bitIndex of $($allBits.Count) bits"
    }

    $bestMask = 0
    $bestScore = [int]::MaxValue
    $best = $null
    for ($mask = 0; $mask -le 7; $mask++) {
        $copy = Copy-QrModules -Grid $grid
        for ($y = 0; $y -lt $size; $y++) {
            for ($x = 0; $x -lt $size; $x++) {
                if (-not $grid.Func[$y][$x] -and (Test-QrMask -Mask $mask -X $x -Y $y)) {
                    $copy[$y][$x] = 1 - $copy[$y][$x]
                }
            }
        }
        Set-QrFormatOnMatrix -Matrix $copy -Bits (Get-QrFormatBits -Mask $mask)
        $score = Get-QrPenalty -Modules $copy
        if ($score -lt $bestScore) {
            $bestScore = $score
            $bestMask = $mask
            $best = $copy
        }
    }
    $rows = New-Object System.Collections.Generic.List[string]
    for ($y = 0; $y -lt $size; $y++) {
        $chars = New-Object char[] $size
        for ($x = 0; $x -lt $size; $x++) {
            if ($best[$y][$x] -eq 1) { $chars[$x] = '1' } else { $chars[$x] = '0' }
        }
        [void]$rows.Add((-join $chars))
    }
    return [pscustomobject]@{
        Version = $version
        Size    = $size
        Mask    = $bestMask
        Rows    = $rows.ToArray()
    }
}

function Format-QrAnsi {
    param($Matrix)
    $quiet = 4
    $n = @($Matrix.Rows).Count
    $size = $n + (2 * $quiet)
    $esc = [char]27
    $sb = New-Object System.Text.StringBuilder
    [void]$sb.Append($esc)
    [void]$sb.Append('[30;47m')
    for ($y = 0; $y -lt $size; $y += 2) {
        for ($x = 0; $x -lt $size; $x++) {
            $top = Test-QrAnsiDark -Rows $Matrix.Rows -Quiet $quiet -X $x -Y $y
            $bot = $false
            if (($y + 1) -lt $size) {
                $bot = Test-QrAnsiDark -Rows $Matrix.Rows -Quiet $quiet -X $x -Y ($y + 1)
            }
            if ($top -and $bot) { [void]$sb.Append([char]0x2588) }
            elseif ($top) { [void]$sb.Append([char]0x2580) }
            elseif ($bot) { [void]$sb.Append([char]0x2584) }
            else { [void]$sb.Append(' ') }
        }
        [void]$sb.AppendLine()
    }
    [void]$sb.Append($esc)
    [void]$sb.Append('[0m')
    return $sb.ToString()
}

function Test-QrAnsiDark {
    param($Rows, [int]$Quiet, [int]$X, [int]$Y)
    $n = @($Rows).Count
    $xx = $X - $Quiet
    $yy = $Y - $Quiet
    if ($xx -lt 0 -or $yy -lt 0 -or $xx -ge $n -or $yy -ge $n) { return $false }
    return ($Rows[$yy][$xx] -eq '1')
}

function Get-VrSimCrc16Ccitt {
    param([Parameter(Mandatory = $true)][string]$Text)
    $crc = 0xFFFF
    $bytes = [System.Text.Encoding]::ASCII.GetBytes($Text)
    foreach ($b in $bytes) {
        $crc = $crc -bxor (([int]$b) -shl 8)
        for ($i = 0; $i -lt 8; $i++) {
            if (($crc -band 0x8000) -ne 0) {
                $crc = (($crc -shl 1) -bxor 0x1021) -band 0xFFFF
            }
            else {
                $crc = ($crc -shl 1) -band 0xFFFF
            }
        }
    }
    return [int]$crc
}

function Get-VrSimPairingCode {
    param([Parameter(Mandatory = $true)][string]$Ipv4)
    $crc = Get-VrSimCrc16Ccitt -Text ("ohho|{0}|{1}|{2}" -f $Ipv4, $script:VrSimRosbridgePort, $script:VrSimVideoPort)
    return ('OHHO-{0:X4}' -f $crc)
}

function Get-VrSimVideoUrl {
    param([Parameter(Mandatory = $true)][string]$Ipv4)
    return ("http://{0}:{1}" -f $Ipv4, $script:VrSimVideoPort)
}

function Get-VrSimRosbridgeUrl {
    param([Parameter(Mandatory = $true)][string]$Ipv4)
    return ("ws://{0}:{1}" -f $Ipv4, $script:VrSimRosbridgePort)
}

function Test-VrSimIPv4 {
    param([AllowEmptyString()][string]$Ip)
    if ([string]::IsNullOrWhiteSpace($Ip)) { return $false }
    if ($Ip -notmatch '^(\d{1,3})\.(\d{1,3})\.(\d{1,3})\.(\d{1,3})$') { return $false }
    foreach ($part in @($Matches[1], $Matches[2], $Matches[3], $Matches[4])) {
        $n = [int]$part
        if ($n -lt 0 -or $n -gt 255) { return $false }
        if ($part.Length -gt 1 -and $part.StartsWith('0')) { return $false }
    }
    return $true
}

function Test-VrSimPrivateIPv4 {
    param([string]$Ip)
    if (-not (Test-VrSimIPv4 $Ip)) { return $false }
    $oct = $Ip.Split('.')
    $a = [int]$oct[0]
    $b = [int]$oct[1]
    if ($a -eq 10) { return $true }
    if ($a -eq 192 -and $b -eq 168) { return $true }
    if ($a -eq 172 -and $b -ge 16 -and $b -le 31) { return $true }
    return $false
}

function Select-VrSimLanAddress {
    param([object[]]$Adapters)
    $virtual = '(?i)(vEthernet|WSL|Hyper-V|Default Switch|VirtualBox|VMware|Loopback|Bluetooth|Npcap|\bTAP\b|\bTUN\b|Docker|VPN|Tailscale|ZeroTier|WireGuard|Wintun|Virtual)'
    $candidates = @()
    foreach ($ad in @($Adapters)) {
        if ($null -eq $ad) { continue }
        $address = [string]$ad.Address
        $alias = [string]$ad.Alias
        if (-not (Test-VrSimPrivateIPv4 $address)) { continue }
        if ($alias -match $virtual) { continue }
        $rank = 2
        if ($alias -match '(?i)wi-?fi|wlan|wireless') { $rank = 0 }
        elseif ($alias -match '^(?i)Ethernet( \d+)?$') { $rank = 1 }
        $candidates += [pscustomobject]@{ Address = $address; Rank = $rank; Alias = $alias }
    }
    $best = $candidates | Sort-Object Rank, Alias, Address | Select-Object -First 1
    if ($null -eq $best) { return $null }
    return [string]$best.Address
}

function Get-VrSimWslConfigEdit {
    param([AllowEmptyString()][string]$Text)
    $newline = "`n"
    if ($Text -match "`r`n") { $newline = "`r`n" }
    if ([string]::IsNullOrEmpty($Text)) {
        $created = "[wsl2]${newline}networkingMode=mirrored${newline}"
        return [pscustomobject]@{ Text = $created; Changed = $true }
    }
    $lines = [regex]::Split($Text, "`r`n|`n|`r")
    if ($lines.Length -gt 0 -and $lines[$lines.Length - 1] -eq '' -and $Text.EndsWith("`n")) {
        if ($lines.Length -eq 1) { $lines = @() }
        else { $lines = $lines[0..($lines.Length - 2)] }
    }
    $inWsl2 = $false
    $seenWsl2 = $false
    $keyWritten = $false
    $changed = $false
    $out = New-Object System.Collections.Generic.List[string]
    foreach ($line in $lines) {
        if ($line -match '^\s*\[(.+?)\]\s*$') {
            if ($inWsl2 -and -not $keyWritten) {
                [void]$out.Add('networkingMode=mirrored')
                $keyWritten = $true
                $changed = $true
            }
            $inWsl2 = ($Matches[1] -eq 'wsl2')
            if ($inWsl2) { $seenWsl2 = $true }
            [void]$out.Add($line)
            continue
        }
        if ($inWsl2 -and $line -match '^\s*networkingMode\s*=\s*(.*?)\s*$') {
            $value = $Matches[1].Trim().Trim('"').Trim("'")
            if ($value -eq 'mirrored') { [void]$out.Add($line) }
            else {
                [void]$out.Add('networkingMode=mirrored')
                $changed = $true
            }
            $keyWritten = $true
            continue
        }
        [void]$out.Add($line)
    }
    if ($seenWsl2 -and -not $keyWritten) {
        [void]$out.Add('networkingMode=mirrored')
        $changed = $true
    }
    if (-not $seenWsl2) {
        if ($out.Count -gt 0 -and $out[$out.Count - 1] -ne '') { [void]$out.Add('') }
        [void]$out.Add('[wsl2]')
        [void]$out.Add('networkingMode=mirrored')
        $changed = $true
    }
    if (-not $changed) {
        return [pscustomobject]@{ Text = $Text; Changed = $false }
    }
    $newText = (($out -join $newline) + $newline)
    return [pscustomobject]@{ Text = $newText; Changed = $true }
}

function Test-VrSimWslMirrored {
    param([AllowEmptyString()][string]$Text)
    $edit = Get-VrSimWslConfigEdit -Text $Text
    return (-not $edit.Changed)
}

function Get-VrSimFirewallSpecs {
    param(
        [int]$WebRtcUdpStart = 50000,
        [int]$WebRtcUdpEnd = 50100
    )
    $udp = '{0}-{1}' -f $WebRtcUdpStart, $WebRtcUdpEnd
    $specs = @(
        [pscustomobject]@{
            Name          = 'OhhoVrSim-Rosbridge-TCP-9090'
            DisplayName   = 'OhhO VR sim rosbridge (TCP 9090)'
            Protocol      = 'TCP'
            LocalPort     = '9090'
            Profile       = 'Private'
            Direction     = 'Inbound'
            Action        = 'Allow'
            Enabled       = $true
            RemoteAddress = 'LocalSubnet'
        },
        [pscustomobject]@{
            Name          = 'OhhoVrSim-Video-TCP-8080'
            DisplayName   = 'OhhO VR sim video (TCP 8080)'
            Protocol      = 'TCP'
            LocalPort     = '8080'
            Profile       = 'Private'
            Direction     = 'Inbound'
            Action        = 'Allow'
            Enabled       = $true
            RemoteAddress = 'LocalSubnet'
        },
        [pscustomobject]@{
            Name          = ("OhhoVrSim-WebRTC-UDP-{0}" -f $udp)
            DisplayName   = ("OhhO VR sim WebRTC (UDP {0})" -f $udp)
            Protocol      = 'UDP'
            LocalPort     = $udp
            Profile       = 'Private'
            Direction     = 'Inbound'
            Action        = 'Allow'
            Enabled       = $true
            RemoteAddress = 'LocalSubnet'
        }
    )
    return $specs
}

function Test-VrSimEnabled {
    param($Value)
    return ($Value -eq $true -or [string]$Value -eq 'True')
}

function Test-VrSimFirewallMatch {
    param($Existing, $Spec)
    $proto = [string]$Existing.Protocol
    if ($proto -eq '6') { $proto = 'TCP' }
    if ($proto -eq '17') { $proto = 'UDP' }
    if ($proto -ne [string]$Spec.Protocol) { return $false }
    $port = (([string]$Existing.LocalPort) -replace '\s', '')
    $want = (([string]$Spec.LocalPort) -replace '\s', '')
    if ($port -ne $want) { return $false }
    if ([string]$Existing.Profile -ne 'Private') { return $false }
    if ([string]$Existing.Direction -ne 'Inbound') { return $false }
    if ([string]$Existing.Action -ne 'Allow') { return $false }
    if (-not (Test-VrSimEnabled $Existing.Enabled)) { return $false }
    $remote = (([string]$Existing.RemoteAddress) -replace '\s', '')
    $ok = @('LocalSubnet', 'LocalSubnet4', 'LocalSubnet6', 'LocalSubnet4,LocalSubnet6', 'LocalSubnet6,LocalSubnet4')
    if ($ok -notcontains $remote) { return $false }
    return $true
}

function ConvertFrom-VrSimPortProxy {
    param([AllowEmptyString()][string]$Text)
    $list = New-Object System.Collections.Generic.List[object]
    foreach ($line in [regex]::Split([string]$Text, "`r`n|`n|`r")) {
        if ($line -match '^\s*(\d{1,3}(?:\.\d{1,3}){3})\s+(\d+)\s+(\d{1,3}(?:\.\d{1,3}){3})\s+(\d+)\s*$') {
            [void]$list.Add([pscustomobject]@{
                    ListenAddress  = $Matches[1]
                    ListenPort     = [int]$Matches[2]
                    ConnectAddress = $Matches[3]
                    ConnectPort    = [int]$Matches[4]
                })
        }
    }
    return $list.ToArray()
}

function New-VrSimAction {
    param(
        [string]$Kind,
        [string]$Operation,
        [string]$Target,
        [string]$Detail,
        $Spec,
        $Proxy,
        [string]$Name,
        [string]$Text
    )
    return [pscustomobject]@{
        Kind      = $Kind
        Operation = $Operation
        Target    = $Target
        Detail    = $Detail
        Spec      = $Spec
        Proxy     = $Proxy
        Name      = $Name
        Text      = $Text
    }
}

function Get-VrSimActionRank {
    param([string]$Kind)
    switch ($Kind) {
        'portproxy-delete' { return 10 }
        'portproxy-add' { return 20 }
        'firewall-remove' { return 30 }
        'firewall-update' { return 40 }
        'firewall-add' { return 50 }
        'wslconfig-write' { return 60 }
        'wsl-shutdown' { return 70 }
        default { return 100 }
    }
}

function Get-VrSimNetworkPlan {
    param(
        [Parameter(Mandatory = $true)]
        [ValidateSet('Mirrored', 'PortProxy')]
        [string]$Mode,
        [AllowEmptyString()][string]$LanAddress,
        [AllowEmptyString()][string]$WslAddress,
        [string]$ListenAddress = '0.0.0.0',
        [int]$WebRtcUdpStart = 50000,
        [int]$WebRtcUdpEnd = 50100,
        [object[]]$ExistingFirewall = @(),
        [object[]]$ExistingPortProxy = @(),
        [AllowEmptyString()][string]$WslConfigText = '',
        [int]$OsBuild = 22621,
        [switch]$RestartWsl,
        [switch]$ForcePortProxy
    )
    $actions = New-Object System.Collections.Generic.List[object]
    $warnings = New-Object System.Collections.Generic.List[string]
    $errors = New-Object System.Collections.Generic.List[string]

    if ($WebRtcUdpStart -lt 1 -or $WebRtcUdpEnd -gt 65535 -or $WebRtcUdpStart -gt $WebRtcUdpEnd) {
        [void]$errors.Add('WebRTC UDP range must be inside 1-65535 with start <= end.')
    }
    if (-not (Test-VrSimIPv4 $ListenAddress)) {
        [void]$errors.Add("Listen address '$ListenAddress' is not an IPv4 address.")
    }
    $span = 0
    if ($errors.Count -eq 0) { $span = $WebRtcUdpEnd - $WebRtcUdpStart + 1 }
    if ($span -gt 200) {
        [void]$warnings.Add("WebRTC UDP range is $span ports. The default is 101 ports (50000-50100).")
    }
    if ($Mode -eq 'Mirrored' -and $OsBuild -lt $script:VrSimMirroredMinBuild) {
        [void]$errors.Add("Mirrored networking needs Windows 11 22H2 (build $($script:VrSimMirroredMinBuild)) or newer. This build is $OsBuild. Use -Mode PortProxy.")
    }

    $canFirewall = ($errors.Count -eq 0)
    if ($canFirewall) {
        $specs = @(Get-VrSimFirewallSpecs -WebRtcUdpStart $WebRtcUdpStart -WebRtcUdpEnd $WebRtcUdpEnd)
        $owned = @($ExistingFirewall | Where-Object {
                $null -ne $_ -and (
                    [string]$_.Name -like 'OhhoVrSim-*' -or
                    [string]$_.DisplayName -like 'OhhO VR sim *'
                )
            })
        foreach ($spec in $specs) {
            $existing = $owned | Where-Object {
                $_.Name -eq $spec.Name -or $_.DisplayName -eq $spec.DisplayName
            } | Select-Object -First 1
            if ($null -eq $existing) {
                [void]$actions.Add((New-VrSimAction -Kind 'firewall-add' -Operation 'Add firewall rule' -Target $spec.DisplayName -Detail ("firewall add {0} profile Private remote LocalSubnet" -f $spec.DisplayName) -Spec $spec))
            }
            elseif (-not (Test-VrSimFirewallMatch -Existing $existing -Spec $spec)) {
                $update = [pscustomobject]@{
                    Name          = [string]$existing.Name
                    DisplayName   = $spec.DisplayName
                    Protocol      = $spec.Protocol
                    LocalPort     = $spec.LocalPort
                    Profile       = 'Private'
                    Direction     = 'Inbound'
                    Action        = 'Allow'
                    Enabled       = $true
                    RemoteAddress = 'LocalSubnet'
                }
                [void]$actions.Add((New-VrSimAction -Kind 'firewall-update' -Operation 'Update firewall rule' -Target $spec.DisplayName -Detail ("firewall update {0} profile Private remote LocalSubnet" -f $spec.DisplayName) -Spec $update))
            }
        }
        $webrtc = $specs | Where-Object { $_.Protocol -eq 'UDP' } | Select-Object -First 1
        foreach ($old in $owned) {
            $oldName = [string]$old.Name
            if ($oldName -like 'OhhoVrSim-WebRTC-UDP-*' -and $oldName -ne $webrtc.Name) {
                [void]$actions.Add((New-VrSimAction -Kind 'firewall-remove' -Operation 'Remove firewall rule' -Target $oldName -Detail ("firewall remove {0}" -f $oldName) -Name $oldName))
            }
        }
    }

    $mirroredOn = Test-VrSimWslMirrored -Text $WslConfigText
    $skipProxy = ($Mode -eq 'PortProxy' -and $mirroredOn -and -not $ForcePortProxy)
    if ($skipProxy) {
        [void]$warnings.Add('networkingMode=mirrored is already set. Portproxy was left unchanged. Pass -ForcePortProxy to add TCP proxies anyway, or use -Mode Mirrored.')
    }

    $wslOk = (Test-VrSimIPv4 $WslAddress) -and ($WslAddress -ne '0.0.0.0') -and (-not $WslAddress.StartsWith('127.'))
    $touchProxy = $false
    if ($Mode -eq 'Mirrored' -and $canFirewall) { $touchProxy = $true }
    if ($Mode -eq 'PortProxy' -and -not $skipProxy -and $canFirewall) { $touchProxy = $true }
    if ($touchProxy -and $Mode -eq 'PortProxy' -and -not $wslOk) {
        [void]$errors.Add('WSL IPv4 is unknown. Start the distro and re-run so portproxy can target its eth0 address. Pass -WslAddress when auto-detect is wrong, or -WslDistro when navdev is the container name rather than the distro.')
        $touchProxy = $false
    }
    if ($touchProxy) {
        foreach ($port in @($script:VrSimRosbridgePort, $script:VrSimVideoPort)) {
            $existingForPort = @($ExistingPortProxy | Where-Object { $null -ne $_ -and [int]$_.ListenPort -eq $port })
            if ($Mode -eq 'Mirrored') {
                foreach ($row in $existingForPort) {
                    $detail = "portproxy delete $($row.ListenAddress):$($row.ListenPort) -> $($row.ConnectAddress):$($row.ConnectPort)"
                    [void]$actions.Add((New-VrSimAction -Kind 'portproxy-delete' -Operation 'Delete portproxy' -Target "$($row.ListenAddress):$port" -Detail $detail -Proxy $row))
                }
            }
            else {
                $exact = @($existingForPort | Where-Object {
                        [string]$_.ListenAddress -eq $ListenAddress -and
                        [string]$_.ConnectAddress -eq $WslAddress -and
                        [int]$_.ConnectPort -eq $port
                    })
                $extras = @($existingForPort | Where-Object {
                        -not (
                            [string]$_.ListenAddress -eq $ListenAddress -and
                            [string]$_.ConnectAddress -eq $WslAddress -and
                            [int]$_.ConnectPort -eq $port
                        )
                    })
                foreach ($row in $extras) {
                    $detail = "portproxy delete $($row.ListenAddress):$($row.ListenPort) -> $($row.ConnectAddress):$($row.ConnectPort)"
                    [void]$actions.Add((New-VrSimAction -Kind 'portproxy-delete' -Operation 'Delete portproxy' -Target "$($row.ListenAddress):$port" -Detail $detail -Proxy $row))
                }
                if ($exact.Count -gt 1) {
                    for ($i = 1; $i -lt $exact.Count; $i++) {
                        $row = $exact[$i]
                        $detail = "portproxy delete $($row.ListenAddress):$($row.ListenPort) -> $($row.ConnectAddress):$($row.ConnectPort)"
                        [void]$actions.Add((New-VrSimAction -Kind 'portproxy-delete' -Operation 'Delete portproxy' -Target "$($row.ListenAddress):$port" -Detail $detail -Proxy $row))
                    }
                }
                if ($exact.Count -eq 0) {
                    $proxy = [pscustomobject]@{
                        ListenAddress  = $ListenAddress
                        ListenPort     = $port
                        ConnectAddress = $WslAddress
                        ConnectPort    = $port
                    }
                    $detail = "portproxy add $($ListenAddress):$port -> $($WslAddress):$port"
                    [void]$actions.Add((New-VrSimAction -Kind 'portproxy-add' -Operation 'Add portproxy' -Target "${ListenAddress}:$port" -Detail $detail -Proxy $proxy))
                }
            }
        }
    }

    if ($Mode -eq 'Mirrored' -and $canFirewall) {
        $edit = Get-VrSimWslConfigEdit -Text $WslConfigText
        if ($edit.Changed) {
            [void]$actions.Add((New-VrSimAction -Kind 'wslconfig-write' -Operation 'Write .wslconfig' -Target 'networkingMode=mirrored' -Detail 'wslconfig set networkingMode=mirrored' -Text $edit.Text))
            if ($RestartWsl) {
                [void]$actions.Add((New-VrSimAction -Kind 'wsl-shutdown' -Operation 'Shut down WSL' -Target 'wsl --shutdown' -Detail 'wsl --shutdown (stops every WSL distro)'))
            }
            else {
                [void]$warnings.Add('After networkingMode=mirrored is written, WSL keeps the old network until wsl --shutdown. Pass -RestartWsl to have this script run that, or run it yourself. It stops every distro.')
            }
        }
    }

    if ($Mode -eq 'PortProxy' -and $canFirewall) {
        [void]$warnings.Add('Portproxy forwards TCP 9090 and 8080. WebRTC media is UDP, so the UDP firewall rule is for mirrored mode.')
    }

    $ranked = New-Object System.Collections.Generic.List[object]
    $seq = 0
    foreach ($action in $actions) {
        $ranked.Add([pscustomobject]@{ Rank = (Get-VrSimActionRank $action.Kind); Seq = $seq; Action = $action }) | Out-Null
        $seq++
    }
    $ordered = @($ranked | Sort-Object Rank, Seq | ForEach-Object { $_.Action })
    return [pscustomobject]@{
        Actions  = $ordered
        Warnings = $warnings.ToArray()
        Errors   = $errors.ToArray()
    }
}

function Merge-VrSimNetworkState {
    param($State, $Actions)
    $firewall = New-Object System.Collections.Generic.List[object]
    foreach ($rule in @($State.Firewall)) {
        if ($null -ne $rule) { [void]$firewall.Add($rule) }
    }
    $proxy = New-Object System.Collections.Generic.List[object]
    foreach ($row in @($State.PortProxy)) {
        if ($null -ne $row) { [void]$proxy.Add($row) }
    }
    $config = [string]$State.WslConfigText
    foreach ($action in @($Actions)) {
        if ($null -eq $action) { continue }
        switch ($action.Kind) {
            'firewall-add' {
                [void]$firewall.Add($action.Spec)
            }
            'firewall-update' {
                $keep = New-Object System.Collections.Generic.List[object]
                foreach ($rule in $firewall) {
                    if ([string]$rule.Name -ne [string]$action.Spec.Name) { [void]$keep.Add($rule) }
                }
                [void]$keep.Add($action.Spec)
                $firewall = $keep
            }
            'firewall-remove' {
                $keep = New-Object System.Collections.Generic.List[object]
                foreach ($rule in $firewall) {
                    if ([string]$rule.Name -ne [string]$action.Name) { [void]$keep.Add($rule) }
                }
                $firewall = $keep
            }
            'portproxy-delete' {
                $row = $action.Proxy
                $keep = New-Object System.Collections.Generic.List[object]
                foreach ($item in $proxy) {
                    $same = (
                        [string]$item.ListenAddress -eq [string]$row.ListenAddress -and
                        [int]$item.ListenPort -eq [int]$row.ListenPort -and
                        [string]$item.ConnectAddress -eq [string]$row.ConnectAddress -and
                        [int]$item.ConnectPort -eq [int]$row.ConnectPort
                    )
                    if (-not $same) { [void]$keep.Add($item) }
                }
                $proxy = $keep
            }
            'portproxy-add' {
                [void]$proxy.Add($action.Proxy)
            }
            'wslconfig-write' {
                $config = [string]$action.Text
            }
            'wsl-shutdown' { }
        }
    }
    return [pscustomobject]@{
        Firewall      = $firewall.ToArray()
        PortProxy     = $proxy.ToArray()
        WslConfigText = $config
    }
}

function Format-VrSimNetworkReport {
    param(
        [AllowEmptyString()][string]$Mode,
        [AllowEmptyString()][string]$LanAddress,
        [AllowEmptyString()][string]$WslAddress,
        $Plans,
        [string[]]$ExtraWarnings,
        [string[]]$ExtraErrors,
        [string]$Status,
        [int]$ExitCode,
        [int]$WebRtcUdpStart = 50000,
        [int]$WebRtcUdpEnd = 50100
    )
    $sb = New-Object System.Text.StringBuilder
    [void]$sb.AppendLine('OhhO VR sim network (OHH-205)')
    [void]$sb.AppendLine('Rules: Private profile, remote LocalSubnet, TCP 9090, TCP 8080, UDP WebRTC range.')
    [void]$sb.AppendLine('The adapter network category stays as it is. Set Wi-Fi or Ethernet to Private for these rules to match.')
    if ([string]::IsNullOrWhiteSpace($LanAddress)) {
        [void]$sb.AppendLine('LAN IP:         (none found)')
        [void]$sb.AppendLine('Quest browser:  (needs a LAN IPv4)')
        [void]$sb.AppendLine('Quest VR app:   (needs a LAN IPv4)')
        [void]$sb.AppendLine('Pairing code:   (needs a LAN IPv4)')
    }
    else {
        $video = Get-VrSimVideoUrl -Ipv4 $LanAddress
        $ros = Get-VrSimRosbridgeUrl -Ipv4 $LanAddress
        $code = Get-VrSimPairingCode -Ipv4 $LanAddress
        [void]$sb.AppendLine("LAN IP:         $LanAddress")
        [void]$sb.AppendLine("Quest browser:  $video")
        [void]$sb.AppendLine("Quest VR app:   $ros")
        [void]$sb.AppendLine("Pairing code:   $code")
        [void]$sb.AppendLine("QR encodes:     $video")
        $qr = Get-QrMatrix -Text $video
        [void]$sb.AppendLine((Format-QrAnsi -Matrix $qr))
    }
    if (-not [string]::IsNullOrWhiteSpace($WslAddress)) {
        [void]$sb.AppendLine("WSL IP:         $WslAddress")
    }
    [void]$sb.AppendLine(("WebRTC UDP:     {0}-{1}" -f $WebRtcUdpStart, $WebRtcUdpEnd))
    if ([string]::IsNullOrWhiteSpace($Mode)) {
        [void]$sb.AppendLine('Mode:           not selected')
        [void]$sb.AppendLine('Varun TODO: choose -Mode Mirrored or -Mode PortProxy after trying both on this PC.')
    }
    else {
        [void]$sb.AppendLine("Mode:           $Mode")
    }
    foreach ($plan in @($Plans)) {
        if ($null -eq $plan) { continue }
        if (-not [string]::IsNullOrWhiteSpace($plan.Title)) {
            [void]$sb.AppendLine($plan.Title)
        }
        foreach ($action in @($plan.Plan.Actions)) {
            [void]$sb.AppendLine("- $($action.Detail)")
        }
        foreach ($warning in @($plan.Plan.Warnings)) {
            [void]$sb.AppendLine("Warning: $warning")
        }
        foreach ($err in @($plan.Plan.Errors)) {
            [void]$sb.AppendLine("Error: $err")
        }
    }
    foreach ($warning in @($ExtraWarnings)) {
        if (-not [string]::IsNullOrWhiteSpace($warning)) { [void]$sb.AppendLine("Warning: $warning") }
    }
    foreach ($err in @($ExtraErrors)) {
        if (-not [string]::IsNullOrWhiteSpace($err)) { [void]$sb.AppendLine("Error: $err") }
    }
    [void]$sb.AppendLine($Status)
    [void]$sb.AppendLine("Exit code: $ExitCode")
    return $sb.ToString()
}

function Test-VrSimCurrentUserAdmin {
    $identity = [Security.Principal.WindowsIdentity]::GetCurrent()
    $principal = New-Object Security.Principal.WindowsPrincipal($identity)
    return $principal.IsInRole([Security.Principal.WindowsBuiltInRole]::Administrator)
}

function Get-VrSimLiveLanAddress {
    $rows = @()
    $addrs = @(Get-NetIPAddress -AddressFamily IPv4 -ErrorAction Stop | Where-Object {
            $_.AddressState -eq 'Preferred' -or [string]$_.AddressState -eq 'Preferred'
        })
    foreach ($a in $addrs) {
        $rows += [pscustomobject]@{ Address = [string]$a.IPAddress; Alias = [string]$a.InterfaceAlias }
    }
    return (Select-VrSimLanAddress -Adapters $rows)
}

function Get-VrSimLiveWslAddress {
    param([string]$Distro)
    $cmd = Get-Command wsl.exe -ErrorAction SilentlyContinue
    if ($null -eq $cmd) { return $null }
    $output = & wsl.exe -d $Distro -e hostname -I 2>&1
    if ($LASTEXITCODE -ne 0) { return $null }
    $text = ($output | Out-String)
    foreach ($token in ($text -split '\s+')) {
        if ((Test-VrSimIPv4 $token) -and -not $token.StartsWith('127.')) { return $token }
    }
    return $null
}

function Get-VrSimLiveWslConfig {
    param([string]$Path)
    if ([string]::IsNullOrWhiteSpace($Path)) { return '' }
    if (Test-Path -LiteralPath $Path) {
        return [System.IO.File]::ReadAllText($Path)
    }
    return ''
}

function Get-VrSimLivePortProxy {
    $output = & netsh.exe interface portproxy show v4tov4 2>&1 | Out-String
    if ($LASTEXITCODE -ne 0) {
        return [pscustomobject]@{
            Ok    = $false
            Error = "netsh interface portproxy show v4tov4 exited $LASTEXITCODE"
            Rows  = @()
        }
    }
    return [pscustomobject]@{
        Ok    = $true
        Error = ''
        Rows  = @(ConvertFrom-VrSimPortProxy -Text $output)
    }
}

function ConvertFrom-VrSimLiveFirewallRule {
    param($Rule)
    $port = $Rule | Get-NetFirewallPortFilter
    $addr = $Rule | Get-NetFirewallAddressFilter
    $local = ((@($port.LocalPort) | ForEach-Object { "$_" }) -join ',') -replace '\s', ''
    $remote = ((@($addr.RemoteAddress) | ForEach-Object { "$_" }) -join ',')
    $profile = [string]$Rule.Profile
    if ($profile -match '^\d+$') {
        $n = [int]$profile
        if ($n -eq 2147483647) { $profile = 'Any' }
        else {
            $names = @()
            if ($n -band 1) { $names += 'Domain' }
            if ($n -band 2) { $names += 'Private' }
            if ($n -band 4) { $names += 'Public' }
            $profile = ($names -join ',')
        }
    }
    $proto = [string]$port.Protocol
    if ($proto -eq '6') { $proto = 'TCP' }
    if ($proto -eq '17') { $proto = 'UDP' }
    return [pscustomobject]@{
        Name          = [string]$Rule.Name
        DisplayName   = [string]$Rule.DisplayName
        Protocol      = $proto
        LocalPort     = $local
        Profile       = $profile
        Direction     = [string]$Rule.Direction
        Action        = [string]$Rule.Action
        Enabled       = ($Rule.Enabled -eq $true -or [string]$Rule.Enabled -eq 'True')
        RemoteAddress = $remote
    }
}

function Get-VrSimLiveFirewall {
    $found = @()
    $byDisplay = @(Get-NetFirewallRule -DisplayName 'OhhO VR sim*' -ErrorAction SilentlyContinue)
    $byName = @(Get-NetFirewallRule -Name 'OhhoVrSim-*' -ErrorAction SilentlyContinue)
    $seen = @{}
    foreach ($rule in @($byDisplay + $byName)) {
        if ($null -eq $rule) { continue }
        $key = [string]$rule.Name
        if ($seen.ContainsKey($key)) { continue }
        $seen[$key] = $true
        $found += (ConvertFrom-VrSimLiveFirewallRule -Rule $rule)
    }
    return $found
}

function Invoke-VrSimNetworkAction {
    param([Parameter(Mandatory = $true)]$Action)
    if ($env:OHHO_VR_SIM_NETWORK_DISABLE_MUTATIONS -eq '1') {
        throw "OHHO_VR_SIM_NETWORK_DISABLE_MUTATIONS blocked $($Action.Kind)"
    }
    switch ($Action.Kind) {
        'firewall-add' {
            $spec = $Action.Spec
            New-NetFirewallRule -Name $spec.Name -DisplayName $spec.DisplayName -Group 'OhhO VR sim' -Direction Inbound -Action Allow -Protocol $spec.Protocol -LocalPort $spec.LocalPort -Profile Private -RemoteAddress LocalSubnet -EdgeTraversalPolicy Block -Enabled True -Description 'OhhO local sim. Private profile and local subnet only.' -ErrorAction Stop | Out-Null
        }
        'firewall-update' {
            $spec = $Action.Spec
            Set-NetFirewallRule -Name $spec.Name -Direction Inbound -Action Allow -Profile Private -RemoteAddress LocalSubnet -EdgeTraversalPolicy Block -Enabled True -ErrorAction Stop | Out-Null
            Get-NetFirewallRule -Name $spec.Name -ErrorAction Stop | Get-NetFirewallPortFilter | Set-NetFirewallPortFilter -Protocol $spec.Protocol -LocalPort $spec.LocalPort -ErrorAction Stop | Out-Null
        }
        'firewall-remove' {
            Remove-NetFirewallRule -Name $Action.Name -ErrorAction Stop
        }
        'portproxy-delete' {
            $row = $Action.Proxy
            if (-not (Test-VrSimIPv4 $row.ListenAddress)) { throw 'portproxy listen address is not IPv4' }
            & netsh.exe interface portproxy delete v4tov4 ("listenaddress=$($row.ListenAddress)") ("listenport=$([int]$row.ListenPort)")
            if ($LASTEXITCODE -ne 0) { throw "netsh portproxy delete exited $LASTEXITCODE" }
        }
        'portproxy-add' {
            $row = $Action.Proxy
            if (-not (Test-VrSimIPv4 $row.ListenAddress)) { throw 'portproxy listen address is not IPv4' }
            if (-not (Test-VrSimIPv4 $row.ConnectAddress)) { throw 'portproxy connect address is not IPv4' }
            & netsh.exe interface portproxy add v4tov4 ("listenaddress=$($row.ListenAddress)") ("listenport=$([int]$row.ListenPort)") ("connectaddress=$($row.ConnectAddress)") ("connectport=$([int]$row.ConnectPort)")
            if ($LASTEXITCODE -ne 0) { throw "netsh portproxy add exited $LASTEXITCODE" }
        }
        'wslconfig-write' {
            if ([string]::IsNullOrWhiteSpace($Action.Path)) { throw 'wslconfig path is empty' }
            $utf8 = New-Object System.Text.UTF8Encoding $false
            [System.IO.File]::WriteAllText($Action.Path, [string]$Action.Text, $utf8)
        }
        'wsl-shutdown' {
            & wsl.exe --shutdown
            if ($LASTEXITCODE -ne 0) { throw "wsl --shutdown exited $LASTEXITCODE" }
        }
        default { throw "unknown action $($Action.Kind)" }
    }
}

function Invoke-VrSimNetworkCore {
    [CmdletBinding(SupportsShouldProcess = $true)]
    param(
        [ValidateSet('Mirrored', 'PortProxy')]
        [string]$Mode,
        [switch]$DryRun,
        [switch]$RestartWsl,
        [switch]$ForcePortProxy,
        [string]$WslDistro = 'navdev',
        [string]$ListenAddress = '0.0.0.0',
        [int]$WebRtcUdpStart = 50000,
        [int]$WebRtcUdpEnd = 50100,
        [AllowEmptyString()][string]$LanAddress,
        [AllowEmptyString()][string]$WslAddress,
        [object[]]$ExistingFirewall = @(),
        [object[]]$ExistingPortProxy = @(),
        [AllowEmptyString()][string]$WslConfigText = '',
        [string]$WslConfigPath = '',
        [bool]$IsAdmin = $false,
        [int]$OsBuild = 0,
        [AllowEmptyString()][string]$CollectorError = '',
        [scriptblock]$CommandRunner
    )
    if (-not $PSBoundParameters.ContainsKey('CommandRunner') -or $null -eq $CommandRunner) {
        $CommandRunner = { param($Action) Invoke-VrSimNetworkAction -Action $Action }
    }
    $dry = $DryRun.IsPresent -or [bool]$WhatIfPreference
    $extraErrors = New-Object System.Collections.Generic.List[string]
    $extraWarnings = New-Object System.Collections.Generic.List[string]
    if (-not [string]::IsNullOrWhiteSpace($CollectorError)) { [void]$extraErrors.Add($CollectorError) }
    if (-not [string]::IsNullOrWhiteSpace($Mode) -and [string]::IsNullOrWhiteSpace($LanAddress)) {
        [void]$extraErrors.Add('No LAN IPv4 found on Wi-Fi or Ethernet.')
    }
    elseif ([string]::IsNullOrWhiteSpace($LanAddress)) {
        [void]$extraWarnings.Add('No LAN IPv4 found on Wi-Fi or Ethernet. The QR code needs that address.')
    }

    $planViews = @()
    $blockingErrors = New-Object System.Collections.Generic.List[string]
    foreach ($err in $extraErrors) { [void]$blockingErrors.Add($err) }

    if ([string]::IsNullOrWhiteSpace($Mode)) {
        foreach ($name in @('Mirrored', 'PortProxy')) {
            $one = Get-VrSimNetworkPlan -Mode $name -LanAddress $LanAddress -WslAddress $WslAddress -ListenAddress $ListenAddress -WebRtcUdpStart $WebRtcUdpStart -WebRtcUdpEnd $WebRtcUdpEnd -ExistingFirewall $ExistingFirewall -ExistingPortProxy $ExistingPortProxy -WslConfigText $WslConfigText -OsBuild $OsBuild -RestartWsl:$RestartWsl -ForcePortProxy:$ForcePortProxy
            $planViews += [pscustomobject]@{ Title = "Plan ${name}:"; Plan = $one }
            foreach ($err in @($one.Errors)) { [void]$blockingErrors.Add([string]$err) }
        }
    }
    else {
        $one = Get-VrSimNetworkPlan -Mode $Mode -LanAddress $LanAddress -WslAddress $WslAddress -ListenAddress $ListenAddress -WebRtcUdpStart $WebRtcUdpStart -WebRtcUdpEnd $WebRtcUdpEnd -ExistingFirewall $ExistingFirewall -ExistingPortProxy $ExistingPortProxy -WslConfigText $WslConfigText -OsBuild $OsBuild -RestartWsl:$RestartWsl -ForcePortProxy:$ForcePortProxy
        $planViews += [pscustomobject]@{ Title = ''; Plan = $one }
        foreach ($err in @($one.Errors)) { [void]$blockingErrors.Add([string]$err) }
    }

    $applied = New-Object System.Collections.Generic.List[object]
    $exit = 0
    $status = ''
    $modeChosen = -not [string]::IsNullOrWhiteSpace($Mode)
    $hasErrors = ($blockingErrors.Count -gt 0)

    if (-not $modeChosen) {
        # A missing WSL address is expected until the distro is up. It stays in
        # the portproxy section of the report and does not hide the mode choice.
        $serious = @($blockingErrors | Where-Object { $_ -notlike 'WSL IPv4 is unknown*' })
        if (@($serious).Count -gt 0) {
            $exit = 1
            $status = 'Status: no mode selected, nothing changed. The preview has errors.'
        }
        else {
            $exit = 2
            $status = 'Status: no mode selected, nothing changed.'
        }
    }
    elseif ($dry) {
        if ($hasErrors) {
            $exit = 1
            $status = 'Status: dry run, nothing changed. The plan has errors.'
        }
        else {
            $exit = 0
            $status = 'Status: dry run, nothing changed.'
        }
        if ($WhatIfPreference) {
            foreach ($view in $planViews) {
                foreach ($action in @($view.Plan.Actions)) {
                    [void]$PSCmdlet.ShouldProcess($action.Target, $action.Operation)
                }
            }
        }
    }
    elseif (-not $IsAdmin) {
        $exit = 1
        $status = 'Status: not applied. An administrator PowerShell is required to change firewall or portproxy settings.'
    }
    elseif ($hasErrors) {
        $exit = 1
        $status = 'Status: not applied. The plan has errors.'
    }
    else {
        $planned = @($planViews[0].Plan.Actions)
        if ($planned.Count -eq 0) {
            $exit = 0
            $status = 'Status: already up to date, nothing to change.'
        }
        else {
            foreach ($action in $planned) {
                if (-not $PSCmdlet.ShouldProcess($action.Target, $action.Operation)) { continue }
                $payload = $action
                if ($action.Kind -eq 'wslconfig-write') {
                    $payload = [pscustomobject]@{
                        Kind      = $action.Kind
                        Operation = $action.Operation
                        Target    = $action.Target
                        Detail    = $action.Detail
                        Text      = $action.Text
                        Path      = $WslConfigPath
                        Spec      = $action.Spec
                        Proxy     = $action.Proxy
                        Name      = $action.Name
                    }
                }
                & $CommandRunner $payload
                [void]$applied.Add($payload)
            }
            $exit = 0
            $status = "Status: applied $($applied.Count) actions."
        }
    }

    $report = Format-VrSimNetworkReport -Mode $Mode -LanAddress $LanAddress -WslAddress $WslAddress -Plans $planViews -ExtraWarnings $extraWarnings.ToArray() -ExtraErrors $extraErrors.ToArray() -Status $status -ExitCode $exit -WebRtcUdpStart $WebRtcUdpStart -WebRtcUdpEnd $WebRtcUdpEnd
    return [pscustomobject]@{
        ExitCode = $exit
        Report   = $report
        Applied  = $applied.ToArray()
        Dry      = $dry
    }
}

function Invoke-VrSimNetwork {
    [CmdletBinding(SupportsShouldProcess = $true)]
    param(
        [ValidateSet('Mirrored', 'PortProxy')]
        [string]$Mode,
        [switch]$DryRun,
        [switch]$RestartWsl,
        [switch]$ForcePortProxy,
        [ValidatePattern('^[A-Za-z0-9][A-Za-z0-9._-]{0,63}$')]
        [string]$WslDistro = 'navdev',
        [string]$ListenAddress = '0.0.0.0',
        [int]$WebRtcUdpStart = 50000,
        [int]$WebRtcUdpEnd = 50100,
        [string]$LanAddress,
        [string]$WslAddress
    )
    $osBuild = [int](Get-ItemProperty -Path 'HKLM:\SOFTWARE\Microsoft\Windows NT\CurrentVersion' -Name CurrentBuild).CurrentBuild
    $isAdmin = Test-VrSimCurrentUserAdmin
    $collectorErrors = New-Object System.Collections.Generic.List[string]
    if (-not $PSBoundParameters.ContainsKey('LanAddress')) {
        try { $LanAddress = Get-VrSimLiveLanAddress }
        catch { [void]$collectorErrors.Add("LAN address lookup failed: $($_.Exception.Message)") }
    }
    if (-not $PSBoundParameters.ContainsKey('WslAddress')) {
        try { $WslAddress = [string](Get-VrSimLiveWslAddress -Distro $WslDistro) }
        catch { [void]$collectorErrors.Add("WSL address lookup failed: $($_.Exception.Message)") }
    }
    $wslPath = Join-Path $env:USERPROFILE '.wslconfig'
    try { $wslText = Get-VrSimLiveWslConfig -Path $wslPath }
    catch {
        $wslText = ''
        [void]$collectorErrors.Add("Could not read .wslconfig: $($_.Exception.Message)")
    }
    $firewall = @()
    try { $firewall = @(Get-VrSimLiveFirewall) }
    catch { [void]$collectorErrors.Add("Firewall lookup failed: $($_.Exception.Message)") }
    $proxyRows = @()
    try {
        $proxyResult = Get-VrSimLivePortProxy
        if (-not $proxyResult.Ok) { [void]$collectorErrors.Add([string]$proxyResult.Error) }
        else { $proxyRows = @($proxyResult.Rows) }
    }
    catch { [void]$collectorErrors.Add("Portproxy lookup failed: $($_.Exception.Message)") }

    $core = @{
        DryRun             = [bool]$DryRun
        RestartWsl         = [bool]$RestartWsl
        ForcePortProxy     = [bool]$ForcePortProxy
        WslDistro          = $WslDistro
        ListenAddress      = $ListenAddress
        WebRtcUdpStart     = $WebRtcUdpStart
        WebRtcUdpEnd       = $WebRtcUdpEnd
        LanAddress         = [string]$LanAddress
        WslAddress         = [string]$WslAddress
        ExistingFirewall   = @($firewall)
        ExistingPortProxy  = @($proxyRows)
        WslConfigText      = [string]$wslText
        WslConfigPath      = $wslPath
        IsAdmin            = [bool]$isAdmin
        OsBuild            = $osBuild
        CollectorError     = ($collectorErrors -join ' ')
        WhatIf             = [bool]$WhatIfPreference
    }
    if ($PSBoundParameters.ContainsKey('Mode')) { $core.Mode = $Mode }
    return Invoke-VrSimNetworkCore @core
}

if ($MyInvocation.InvocationName -ne '.') {
    try {
        $utf8 = New-Object System.Text.UTF8Encoding $false
        [Console]::OutputEncoding = $utf8
    }
    catch { }
    $ohhoVrSimResult = Invoke-VrSimNetwork @args
    Write-Host $ohhoVrSimResult.Report
    exit $ohhoVrSimResult.ExitCode
}
