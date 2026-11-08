# Pull committed server changes without overwriting laptop edits.
$ErrorActionPreference = 'Stop'
$repo = Split-Path -Parent $PSScriptRoot
$gitCommand = Get-Command git -ErrorAction SilentlyContinue
$gitExe = if ($gitCommand) { $gitCommand.Source } else { 'C:\Program Files\Git\cmd\git.exe' }
$branch = & $gitExe -C $repo branch --show-current
if ($LASTEXITCODE -ne 0 -or $branch -ne 'isaac') {
    throw 'Switch to isaac before syncing.'
}
$changes = & $gitExe -C $repo status --porcelain
if ($LASTEXITCODE -ne 0 -or $changes) {
    throw 'Laptop has uncommitted changes. Save and reconcile them before syncing.'
}
& $gitExe -C $repo fetch workstation
if ($LASTEXITCODE -ne 0) { throw 'Could not fetch workstation.' }
& $gitExe -C $repo merge --ff-only workstation/isaac
if ($LASTEXITCODE -ne 0) { throw 'Branches have diverged. Reconcile without force.' }
Write-Host 'Laptop source copy matches the committed server branch.'
