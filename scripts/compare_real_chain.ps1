param(
    [string]$ReleaseDir = "d:\vscode\light\CamCom-Binary\build-vs26\bin\Release",
    [string]$Reference = "01.bin",
    [int[]]$Payloads = @(6200, 6500, 6800),
    [string]$VideoPattern = "test{0}.mp4"
)

$ErrorActionPreference = "Stop"

if (-not (Test-Path -LiteralPath $ReleaseDir)) {
    throw "ReleaseDir not found: $ReleaseDir"
}

$decodeExe = Join-Path $ReleaseDir "decode.exe"
if (-not (Test-Path -LiteralPath $decodeExe)) {
    throw "decode.exe not found: $decodeExe"
}

$referencePath = Join-Path $ReleaseDir $Reference
if (-not (Test-Path -LiteralPath $referencePath)) {
    throw "reference file not found: $referencePath"
}

Set-Location -LiteralPath $ReleaseDir

$results = @()

foreach ($payload in $Payloads) {
    $videoName = [string]::Format($VideoPattern, $payload)
    $videoPath = Join-Path $ReleaseDir $videoName
    if (-not (Test-Path -LiteralPath $videoPath)) {
        Write-Host "skip payload=$payload, missing video: $videoName"
        continue
    }

    $outName = "cmp_${payload}_out.bin"
    $voteName = "cmp_${payload}_vote.bin"
    $raw = & $decodeExe $videoName $outName $voteName $Reference 2>&1

    $decodedFrames = 0
    $totalFrames = 0
    $comparedBytes = 0
    $recoveredBytes = 0
    $acc = ""
    $full = ""

    foreach ($line in $raw) {
        if ($line -match "^decoded frames\s*:\s*(\d+)") { $decodedFrames = [int]$matches[1] }
        elseif ($line -match "^total frames\s*:\s*(\d+)") { $totalFrames = [int]$matches[1] }
        elseif ($line -match "^compared bytes\s*:\s*(\d+)") { $comparedBytes = [int]$matches[1] }
        elseif ($line -match "^recovered bytes\s*:\s*(\d+)") { $recoveredBytes = [int]$matches[1] }
        elseif ($line -match "^accuracy\s*:\s*([0-9]+\.[0-9]+%)") { $acc = $matches[1] }
        elseif ($line -match "^full accuracy\s*:\s*([0-9]+\.[0-9]+%)") { $full = $matches[1] }
    }

    $isAcc100 = $acc -eq "100.00%"
    $results += [PSCustomObject]@{
        payload = $payload
        video = $videoName
        decoded_frames = $decodedFrames
        total_frames = $totalFrames
        recovered_bytes = $recoveredBytes
        compared_bytes = $comparedBytes
        accuracy = $acc
        full_accuracy = $full
        acc100 = $isAcc100
    }
}

if ($results.Count -eq 0) {
    Write-Host "no valid test videos found."
    Write-Host "expected pattern: $VideoPattern in $ReleaseDir"
    exit 1
}

Write-Host ""
Write-Host "=== Real Chain Comparison ==="
$results |
    Sort-Object -Property payload |
    Format-Table payload, video, decoded_frames, total_frames, recovered_bytes, compared_bytes, accuracy, full_accuracy -AutoSize

$candidates = $results | Where-Object { $_.acc100 -eq $true -and $_.decoded_frames -gt 0 }
if ($candidates.Count -eq 0) {
    Write-Host ""
    Write-Host "no profile reached 100% byte accuracy on recovered data."
    exit 2
}

$best = $candidates |
    Sort-Object -Property @{Expression = "compared_bytes"; Descending = $true}, @{Expression = "decoded_frames"; Descending = $true}, @{Expression = "payload"; Descending = $false} |
    Select-Object -First 1

Write-Host ""
Write-Host ("recommended payload: {0}" -f $best.payload)
Write-Host ("reason: highest compared_bytes ({0}) with 100% accuracy." -f $best.compared_bytes)
