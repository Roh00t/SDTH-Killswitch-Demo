# Killswitch - open Windows Firewall for TAK CoT multicast and the phone dashboard.
# RUN AS ADMINISTRATOR on the WinTAK machine:
#   powershell -ExecutionPolicy Bypass -File .\win_firewall_cot.ps1
#
# One-liner equivalent (paste into an elevated prompt):
#   New-NetFirewallRule -DisplayName "TAK CoT UDP In" -Direction Inbound -Protocol UDP -LocalPort 6969 -Action Allow -Profile Domain,Private,Public; New-NetFirewallRule -DisplayName "TAK CoT UDP Out" -Direction Outbound -Protocol UDP -LocalPort 6969 -Action Allow -Profile Domain,Private,Public

#Requires -RunAsAdministrator
$ErrorActionPreference = "Stop"
$Port = 6969
$Group = "239.2.3.1"

Write-Host "=== Killswitch CoT firewall setup ===" -ForegroundColor Cyan

# Remove stale rules so re-running is idempotent.
Get-NetFirewallRule -DisplayName "TAK CoT UDP*" -ErrorAction SilentlyContinue |
    Remove-NetFirewallRule -ErrorAction SilentlyContinue

New-NetFirewallRule -DisplayName "TAK CoT UDP In" -Direction Inbound `
    -Protocol UDP -LocalPort $Port -Action Allow `
    -Profile Domain,Private,Public | Out-Null
Write-Host "[+] Inbound  UDP $Port allowed (Domain, Private, Public)" -ForegroundColor Green

New-NetFirewallRule -DisplayName "TAK CoT UDP Out" -Direction Outbound `
    -Protocol UDP -LocalPort $Port -Action Allow `
    -Profile Domain,Private,Public | Out-Null
Write-Host "[+] Outbound UDP $Port allowed (Domain, Private, Public)" -ForegroundColor Green

# Live dashboard for a phone on the same network (iPhone Safari): the page is
# served by `python -m http.server 8000` and its feed by the bridge on 8765.
Get-NetFirewallRule -DisplayName "Killswitch Dashboard*" -ErrorAction SilentlyContinue |
    Remove-NetFirewallRule -ErrorAction SilentlyContinue

New-NetFirewallRule -DisplayName "Killswitch Dashboard TCP In" -Direction Inbound `
    -Protocol TCP -LocalPort 8000,8765 -Action Allow `
    -Profile Domain,Private,Public | Out-Null
Write-Host "[+] Inbound  TCP 8000, 8765 allowed (dashboard for phones on this network)" -ForegroundColor Green

# A firewall rule alone is not sufficient. Two further things bite on demo day.
Write-Host "`n--- Checks the firewall rule does NOT cover ---" -ForegroundColor Yellow

# 1. Network profile. Windows silently blocks most inbound traffic on Public.
Get-NetConnectionProfile | ForEach-Object {
    $colour = if ($_.NetworkCategory -eq "Public") { "Red" } else { "Green" }
    Write-Host ("    {0,-28} profile={1}" -f $_.InterfaceAlias, $_.NetworkCategory) -ForegroundColor $colour
}
Write-Host "    If any adapter shows Public, set it Private:" -ForegroundColor Yellow
Write-Host '    Set-NetConnectionProfile -InterfaceAlias "<name>" -NetworkCategory Private' -ForegroundColor Gray

# 2. Multiple adapters. The sender must bind IP_MULTICAST_IF to the right one.
Write-Host "`n--- Adapter IPv4 addresses (for --multicast-if on the sender) ---" -ForegroundColor Yellow
Get-NetIPAddress -AddressFamily IPv4 |
    Where-Object { $_.IPAddress -ne "127.0.0.1" } |
    ForEach-Object { Write-Host ("    {0,-28} {1}" -f $_.InterfaceAlias, $_.IPAddress) }

Write-Host "`nWinTAK: Settings > Network Preferences > add UDP input $Group : $Port" -ForegroundColor Cyan
Write-Host "Verify with:  netstat -an | findstr $Port" -ForegroundColor Gray
