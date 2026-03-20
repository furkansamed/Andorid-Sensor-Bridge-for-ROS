param(
    [string]$Distro = "Ubuntu-22.04",
    [string]$ListenHost = "0.0.0.0",
    [int]$TcpPort = 5000,
    [int]$UdpPort = 5001
)

$scriptDir = Split-Path -Parent $MyInvocation.MyCommand.Path
Set-Location $scriptDir

python .\wsl_nat_relay.py --distro $Distro --listen-host $ListenHost --tcp-port $TcpPort --udp-port $UdpPort
