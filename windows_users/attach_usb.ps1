#Requires -RunAsAdministrator
Start-Transcript -Path "$env:USERPROFILE\usb_debug_log.txt" -Append

# Configurazione
$TARGET_PATTERNS = @("03e7:2485", "03e7:f63b")  # Luxonis/Movidius
$POLL_INTERVAL = 1  # Secondi
$WSL_DISTRIBUTION = "Ubuntu-22.04"

function Get-DeviceStatus {
    # 1. Acquisisci output RAW
    $rawOutput = usbipd list
    Write-Output "====== OUTPUT ======"
    Write-Output $rawOutput
    Write-Output "===================="

    $connectedSection = ($rawOutput -split "`n") | Select-String -Pattern "^Connected:" -Context 0,100
    $deviceLines = $connectedSection.Context.PostContext | Where-Object { $_ -match "^\d+-\d+" }

    $devices = @()
    foreach ($line in $deviceLines) {
    	foreach ($pattern in $TARGET_PATTERNS){
	        if ($line -Match $pattern -and $line -Match "Shared") {
        	        Write-Output "=== FOUND CAMERA ==="
                	Write-Output $line
             	   	Write-Output "===================="
			$busid = $line.Split(" ")[0]
			Write-Output $primaParte
			if ($line -Match "Not shared"){
				usbipd bind --busid $busid
			}
			usbipd attach --wsl $WSL_DISTRIBUTION --busid $busid
        	}
	}
    }

}

Write-Output "[$(Get-Date)] Avvio monitoraggio con debug..."
while ($true) {
    Get-DeviceStatus
    Start-Sleep -Seconds $POLL_INTERVAL
}