# Jetson SSH 접속 안내

*Connect to Carrot Jetson over SSH*

**일반 사용에는 필요 없는 선택 사항입니다.** PC에서 Jetson의 상태를 확인하거나 진단할 때 사용합니다. 이 안내는 Carrot R2 보호 이미지 기준이며, 계정은 **`jetlink`**, 접속 방식은 **SSH 공개키 인증**입니다. 기본 비밀번호는 없습니다.

> **Optional for diagnostics; not required for normal use.** This guide covers the Carrot R2 protected image. Log in as **`jetlink`** using an **SSH key**. There is no default password.

설치 배치 파일과 콤마의 Wi-Fi 전달 기능은 SSH 키를 자동 등록하지 않습니다. 처음 한 번 아래 절차로 등록하면 됩니다. **이미지를 다시 기록할 필요는 없습니다.**

> The installer and comma Wi-Fi provisioning do not enroll SSH keys. Register your key once using the steps below. **No image rewrite is needed.**

## 1. PC에서 접속용 키 만들기 · 약 1분

*Create a key on your PC · About 1 minute*

Windows **PowerShell**을 열고 아래 명령을 실행하세요. 같은 이름의 키가 있으면 그대로 사용하며 덮어쓰지 않습니다.

> Open Windows **PowerShell** and run these commands. An existing key with this name is reused, not overwritten.

```powershell
$jetsonKey = Join-Path $env:USERPROFILE '.ssh\carrot_jetson'
New-Item -ItemType Directory -Force (Split-Path $jetsonKey) | Out-Null
if (!(Test-Path $jetsonKey) -and !(Test-Path "$jetsonKey.pub")) {
  ssh-keygen -t ed25519 -f $jetsonKey
}
```

`Enter passphrase`가 나오면 **키를 보호할 암호를 정해 두 번 입력**하세요. 암호 없이 쓰려면 두 번 모두 Enter를 누릅니다. 입력 중 글자가 안 보이는 것은 정상입니다. 이 암호는 Jetson 계정 비밀번호가 아닙니다.

> At `Enter passphrase`, enter a passphrase twice to protect your key, or press Enter twice for no passphrase. Invisible typing is normal. This is a key passphrase, not a Jetson account password.

`ssh` 또는 `ssh-keygen`을 찾을 수 없다고 나오면 Windows의 **선택적 기능**에서 **OpenSSH 클라이언트**를 설치한 뒤 PowerShell을 다시 여세요.

> If `ssh` or `ssh-keygen` is unavailable, install **OpenSSH Client** through Windows Optional Features, then reopen PowerShell.

## 2. 카드 또는 SSD에 공개키 등록 · 약 2분

*Enroll the public key on the card or SSD · About 2 minutes*

**Jetson을 정상 종료하고 전원을 완전히 분리한 뒤** microSD를 PC 리더기에, 또는 NVMe SSD를 USB NVMe 케이스에 연결하세요. 설치 직후 아직 PC에 연결된 매체라면 그대로 진행합니다. Windows가 포맷을 요청하면 **취소**하세요.

> **Shut down Jetson and disconnect power completely** before moving its microSD to a PC reader or its NVMe SSD to a USB NVMe enclosure. Media still connected after installation can be used directly. **Cancel** any Windows format prompt.

탐색기에 **CARROTSETUP** 드라이브가 보이는지 확인하세요. 보이지 않으면 디스크 관리에서 해당 매체의 기존 작은 **CARROTSETUP FAT 파티션**에 드라이브 문자만 할당합니다. 파티션을 만들거나 포맷하지 마세요. 다른 설치 매체는 분리하세요.

> Locate **CARROTSETUP** in File Explorer. If necessary, use Disk Management to assign a drive letter to the medium's existing small **CARROTSETUP FAT partition**. Do not create or format partitions. Disconnect other installation media.

같은 PowerShell 창에서 아래를 실행하세요. CARROTSETUP을 자동으로 찾고, 공개키를 `setup.json`에 저장합니다. **기존 setup.json이 있거나 대상이 여러 개이면 중단**합니다.

> Run the following in the same PowerShell window. It finds CARROTSETUP and writes your public key to `setup.json`. **It stops if a setup file already exists or multiple targets are found.**

```powershell
$jetsonKey = Join-Path $env:USERPROFILE '.ssh\carrot_jetson'
if (!(Test-Path $jetsonKey) -or !(Test-Path "$jetsonKey.pub")) {
  throw 'SSH key pair missing. Complete step 1 first.'
}
$jetsonVolumes = @(Get-Volume | Where-Object FileSystemLabel -eq 'CARROTSETUP')
if ($jetsonVolumes.Count -ne 1 -or !$jetsonVolumes[0].DriveLetter) {
  throw 'Connect exactly one CARROTSETUP volume with a drive letter.'
}
$jetsonSetup = '{0}:\setup.json' -f $jetsonVolumes[0].DriveLetter
if (Test-Path $jetsonSetup) {
  throw 'setup.json already exists. Preserve its settings before editing.'
}
$jetsonPublicKey = (Get-Content -Raw "$jetsonKey.pub").Trim()
$jetsonJson = @{ ssh_public_keys = @($jetsonPublicKey) } | ConvertTo-Json
[System.IO.File]::WriteAllText($jetsonSetup, $jetsonJson, [System.Text.UTF8Encoding]::new($false))
Write-Host "Saved public key: $jetsonSetup"
```

**`.pub`가 붙은 공개키만 전달합니다.** 확장자 없는 `carrot_jetson` 파일은 개인키이므로 PC에 보관하고 타인에게 보내지 마세요. 이미 다른 사람의 키가 등록되어 있다면 이번 `ssh_public_keys` 목록에 유지할 공개키를 모두 넣어야 합니다. 새 목록은 기존 접속 키 목록을 교체합니다. 기존 setup.json을 편집할 때는 다른 설정을 보존하고 **UTF-8 BOM 없이** 저장하세요.

> Transfer **only the `.pub` public key**. Keep the private `carrot_jetson` file on your PC. The new `ssh_public_keys` list **replaces existing authorized keys**, so include every public key you want to retain. When editing an existing setup file, preserve its other settings and save as **UTF-8 without BOM**.

PC에서 안전하게 제거하고, **전원이 분리된 Jetson**에 매체를 장착한 다음 켜세요. 정상 부팅하면서 키를 등록합니다. 성공하면 setup.json은 삭제되고 CARROTSETUP의 `SETUP-RESULT.json`에 `ssh_keys_installed` 개수가 기록됩니다. 이미 사용 중인 R2 이미지도 다음 부팅 때 등록할 수 있습니다.

> Safely eject the medium, install it in the **unpowered Jetson**, then power on. Successful provisioning removes setup.json and records `ssh_keys_installed` in CARROTSETUP's `SETUP-RESULT.json`. An already used R2 image can enroll keys on a later boot too.

## 3. IP 확인 후 접속 · 이후에는 이것만

*Find the IP and connect · Repeat only this step for later connections*

PC와 Jetson을 서로 통신 가능한 같은 Wi-Fi/LAN에 연결하세요. **Jetson USB 진단 화면 또는 캐럿웹의 Jetson/eGPU 상태**에서 Jetson IP를 확인합니다. 콤마 IP가 아닌 **Jetson IP**를 사용하세요. 공유기나 핫스팟의 기기 간 통신 차단이 켜져 있으면 같은 Wi-Fi에서도 접속할 수 없습니다.

> Connect both devices to a reachable Wi-Fi/LAN. Read the **Jetson IP** from its USB diagnostic display or Carrot Web's Jetson/eGPU status. Do not use comma's IP. Wi-Fi client isolation can block access even on the same network.

아래 예시 주소 `192.168.0.123`을 **실제 Jetson IP로 바꿔서** 실행하세요.

> Replace the example `192.168.0.123` with **your Jetson's actual IP**.

```powershell
ssh -i "$env:USERPROFILE\.ssh\carrot_jetson" jetlink@192.168.0.123
```

처음에는 장치 지문과 연결 확인 질문이 나옵니다. 주소와 대상 장치가 맞는지 확인하고, 가능하면 장치 관리자가 제공한 SSH 지문과 비교한 뒤 `yes`를 입력하세요. 키 암호를 설정했다면 이어서 입력합니다. 접속을 끝낼 때는 `exit`를 입력하세요.

> The first connection shows a host fingerprint. Confirm the address and device, compare the fingerprint with one supplied by the device administrator where available, then enter `yes`. Enter your key passphrase if set. Type `exit` to disconnect.

## 연결이 안 될 때

*Troubleshooting*

- **`Permission denied (publickey)`**: 계정 `jetlink`, 선택한 개인키, 2단계 공개키 등록을 확인하세요. 비밀번호를 찾아 입력하는 방식이 아닙니다.
  > Check the `jetlink` username, selected private key and enrollment in step 2. Password login is disabled.
- **`Connection timed out` / `No route to host`**: Jetson 부팅, 현재 IP, Wi-Fi 연결과 기기 간 통신 허용 여부를 확인하세요. 부팅 자체가 실패하면 SSH로 접속할 수 없습니다.
  > Check boot, current IP, network connectivity and client isolation. SSH cannot diagnose a device that never reaches a working network/SSH service.
- **`Connection refused`**: 주소가 맞는지 확인하고 부팅 완료를 기다리세요. 그 주소에서 SSH 서비스가 아직 연결을 받고 있지 않습니다.
  > Verify the IP and wait for boot to finish. No SSH service is accepting the connection at that address.
- **`REMOTE HOST IDENTIFICATION HAS CHANGED`**: 기존에 알던 장치와 SSH 키가 다릅니다. 이미지 재설치나 IP 재할당 여부를 먼저 확인하세요. 확인 없이 경고를 무시하거나 기존 키를 삭제하지 마세요.
  > The host identity differs from the saved one. Verify whether the device was reimaged or its IP reassigned before replacing a saved host key.

[설치 안내로 돌아가기](INSTALL-WINDOWS-KO.md)

*Back to the installation guide*
