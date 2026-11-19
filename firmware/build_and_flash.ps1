param(
    [ValidateSet("Pkt", "CW", "Off")]
    [string]$Stage = "Pkt",
    [switch]$BuildOnly,
    [switch]$EraseOnly,
    [switch]$Clean
)

$ErrorActionPreference = "Stop"

$FirmwareDir = $PSScriptRoot
$Gmake = "C:\ti\ccs2101\ccs\utils\bin\gmake.exe"
$XdsDfu = "C:\ti\ccs2101\ccs\ccs_base\common\uscif\xds110\xdsdfu.exe"
$DSLite = "C:\ti\ccs2101\ccs\ccs_base\DebugServer\bin\DSLite.exe"
$Ccxml = Join-Path $FirmwareDir "CC2652R1F_XDS110.ccxml"

Push-Location $FirmwareDir
try {
    if ($Clean) {
        Write-Host "[1/3] Cleaning firmware build outputs..." -ForegroundColor Cyan
        & $Gmake clean
        if ($LASTEXITCODE -ne 0) { throw "gmake clean failed with exit code $LASTEXITCODE" }
    }

    if (-not $EraseOnly) {
        Write-Host "[1/3] Building PoC CC2652R1F firmware (1.0 kbps Pkt + 200 kHz CW)..." -ForegroundColor Cyan
        & $Gmake all
        if ($LASTEXITCODE -ne 0) { throw "Firmware build failed with exit code $LASTEXITCODE" }
        if ($Stage -eq "Off") {
            & $Gmake backscatter_poc_off.out
            if ($LASTEXITCODE -ne 0) { throw "Off firmware build failed with exit code $LASTEXITCODE" }
        }
        Write-Host "[OK] Built backscatter_poc_tag.out and backscatter_poc_cw.out" -ForegroundColor Green
    }

    if ($BuildOnly) {
        exit 0
    }

    Write-Host "[2/3] Checking XDS110 USB Debug Probe..." -ForegroundColor Cyan
    $dfuOut = & $XdsDfu -e 2>&1 | Out-String
    Write-Host $dfuOut.Trim()
    if ($dfuOut -match "Found 0 devices") {
        Write-Host ""
        Write-Host "[ERROR] No XDS110 USB Debug Probe is currently enumerated on Windows." -ForegroundColor Red
        Write-Host "Diagnostic checklist:" -ForegroundColor Yellow
        Write-Host "  1. Check the Micro-USB cable on the LAUNCHXL-CC26X2R1 (some cables are power-only, with no D+/D- data wires)." -ForegroundColor Yellow
        Write-Host "  2. Check that the green XDS110 power LED near the USB connector is lit and J101 jumper block (TMS/TCK/RXD/TXD/3V3) is populated." -ForegroundColor Yellow
        Write-Host "  3. Try plugging directly into a PC USB port instead of an unpowered hub." -ForegroundColor Yellow
        exit 2
    }

    if ($EraseOnly) {
        Write-Host "[3/3] Erasing CC2652R1F Flash via DSLite..." -ForegroundColor Cyan
        & $DSLite flash --config="$Ccxml" -b Erase -e
        if ($LASTEXITCODE -ne 0) { throw "DSLite flash erase failed with exit code $LASTEXITCODE" }
        Write-Host "[OK] CC2652R1F Flash erased cleanly." -ForegroundColor Green
        exit 0
    }

    $OutName = switch ($Stage) {
        "CW"  { "backscatter_poc_cw.out" }
        "Off" { "backscatter_poc_off.out" }
        default { "backscatter_poc_tag.out" }
    }
    $OutPath = Join-Path $FirmwareDir $OutName
    if (-not (Test-Path $OutPath)) {
        throw "Firmware binary not found: $OutPath"
    }

    Write-Host "[3/3] Erasing old firmware, flashing $OutName, verifying, and starting MCU..." -ForegroundColor Cyan
    & $DSLite flash --config="$Ccxml" -b Erase -f -v -u -e "$OutPath"
    if ($LASTEXITCODE -ne 0) { throw "DSLite flash/verify failed with exit code $LASTEXITCODE" }

    Write-Host "[OK] Flashed and started $OutName on LAUNCHXL-CC26X2R1!" -ForegroundColor Green
}
finally {
    Pop-Location
}
