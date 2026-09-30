# Run on Windows. Keep this window open while using Foxglove.
param(
    [string]$ServerAddress = "10.16.205.165",
    [string]$UserName = "lpc",
    [int]$RemoteFoxglovePort = 8765
)
Write-Host "Foxglove: ws://localhost:8765"
Write-Host "WebRTC: Server=$ServerAddress  Signal=49100  Stream=47998"
& ssh -N -o ExitOnForwardFailure=yes -o ServerAliveInterval=30 -o ServerAliveCountMax=3 -L "8765:127.0.0.1:$RemoteFoxglovePort" "$UserName@$ServerAddress"
exit $LASTEXITCODE
