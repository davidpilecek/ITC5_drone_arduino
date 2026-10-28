param([switch]$HostOnly, [int[]]$Modes = @(0,1,2,3))
$ErrorActionPreference = 'Stop'
$projectRoot = (Resolve-Path (Join-Path $PSScriptRoot '../..')).Path
$diagnostics = Join-Path $projectRoot 'diagnostics'
$testRoot = Join-Path $projectRoot 'firmware/tests/modular'
$gpp = 'C:/msys64/ucrt64/bin/g++.exe'

function Run-DesktopCheck([string]$Name, [string[]]$Sources) {
    $executable = Join-Path $diagnostics "$Name.exe"
    $log = Join-Path $diagnostics "$Name.log"
    & $gpp -std=c++17 -O2 -Wall -Wextra -static @Sources -o $executable *> $log
    if ($LASTEXITCODE -ne 0) { Get-Content -LiteralPath $log; throw "$Name did not compile." }
    & $executable >> $log
    if ($LASTEXITCODE -ne 0) { Get-Content -LiteralPath $log; throw "$Name failed." }
    Get-Content -LiteralPath $log
}

Run-DesktopCheck 'modular_control_tests' @(
    (Join-Path $testRoot 'control_tests.cpp'),
    (Join-Path $PSScriptRoot 'Control.cpp'),
    (Join-Path $testRoot 'safety_checks.cpp'))
Run-DesktopCheck 'modular_height_tests' @((Join-Path $testRoot 'height_tests.cpp'))
Run-DesktopCheck 'modular_package_tests' @(
    (Join-Path $testRoot 'package_tests.cpp'),
    (Join-Path $projectRoot 'firmware/examples/LearningFilter/src/LearningFilter.cpp'))
Run-DesktopCheck 'readability_tests' @(
    (Join-Path $projectRoot 'firmware/tests/readability/behavior_tests.cpp'))

if ($HostOnly) { return }
$compiler = 'C:/Program Files/Arduino IDE/resources/app/lib/backend/resources/arduino-cli.exe'
$cliConfig = Join-Path $env:USERPROFILE '.arduinoIDE/arduino-cli.yaml'
foreach ($mode in $Modes) {
    if ($mode -lt 0 -or $mode -gt 3) { throw "Unsupported mode $mode" }
    $log = Join-Path $diagnostics "modular_mode$mode.log"
    & $compiler compile --config-file $cliConfig --fqbn teensy:avr:teensy41 `
        --build-property "build.flags.defs=-D__IMXRT1062__ -DTEENSYDUINO=160 -DITC_OUTPUT_MODE=$mode" `
        --build-path (Join-Path $diagnostics 'build_modular') $PSScriptRoot *> $log
    if ($LASTEXITCODE -ne 0) { Get-Content -LiteralPath $log -Tail 60; throw "Mode $mode failed." }
    Write-Output "Modular firmware mode $mode passed."
}
# This script compiles only. It does not upload, install libraries, or operate motors.
