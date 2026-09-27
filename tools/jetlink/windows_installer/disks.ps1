function Test-EraseConfirmation([string]$Value) {
  return (@('설치','INSTALL') -ccontains $Value.Trim())
}

function Test-InstallDisk($Disk, [long]$ImageBytes, [int]$SourceDisk) {
  return ($null -ne $Disk -and $Disk.BusType -eq 'USB' -and -not $Disk.IsBoot -and
    -not $Disk.IsSystem -and -not $Disk.IsReadOnly -and -not $Disk.IsOffline -and
    $Disk.Size -ge $ImageBytes -and $Disk.Number -ne $SourceDisk -and
    -not [string]::IsNullOrWhiteSpace([string]$Disk.UniqueId))
}

function Assert-SameDisk($Current, $Selected, [long]$ImageBytes, [int]$SourceDisk) {
  if (-not (Test-InstallDisk $Current $ImageBytes $SourceDisk) -or
      $Current.Number -ne $Selected.Number -or $Current.Size -ne $Selected.Size -or
      $Current.UniqueId -ne $Selected.UniqueId -or
      ([string]$Current.SerialNumber).Trim() -ne ([string]$Selected.SerialNumber).Trim()) {
    throw "SD카드 연결 정보가 바뀌었습니다. 02를 다시 실행하세요.`nCard identity changed. Check the card and retry 02."
  }
}
