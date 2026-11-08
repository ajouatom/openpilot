param(
  [Parameter(Mandatory=$true)][string]$Serial,
  [Parameter(Mandatory=$true)][string]$Apk,
  [Parameter(Mandatory=$true)][string]$DebugKeystore,
  [string]$Sdk = "$env:LOCALAPPDATA\Android\Sdk",
  [string]$Jdk = 'C:/Program Files/Java/jdk-23'
)
$ErrorActionPreference='Stop'
if ($Serial -notmatch '^emulator-\d+$') { throw 'This destructive test-storage fixture only runs on an emulator.' }
$build = Join-Path $PSScriptRoot ('build/recovery-' + [Guid]::NewGuid().ToString('N'))
New-Item -ItemType Directory -Force "$build/classes","$build/dex" | Out-Null
$bin=Join-Path $Sdk 'build-tools/35.0.0'
$platform=Join-Path $Sdk 'platforms/android-36/android.jar'
$adb=Join-Path $Sdk 'platform-tools/adb.exe'
if ((& $adb -s $Serial shell getprop sys.boot_completed).Trim() -ne '1') { throw 'Wait for the emulator to finish booting before running instrumentation.' }
function Check([string]$step) { if ($LASTEXITCODE -ne 0) { throw "$step failed" } }
$manifest=@'
<manifest xmlns:android="http://schemas.android.com/apk/res/android" package="me.shind0.carrotweb.recoverytest">
<uses-sdk android:minSdkVersion="26" android:targetSdkVersion="36" />
<application android:label="CarrotAssist recovery test" />
<instrumentation android:name="me.shind0.carrotweb.RecoveryInstrumentation" android:targetPackage="me.shind0.carrotweb" />
</manifest>
'@
[IO.File]::WriteAllText("$build/AndroidManifest.xml",$manifest)
$sources=@(Get-ChildItem -LiteralPath (Join-Path $PSScriptRoot 'src') -Filter '*.java' -Recurse | ForEach-Object FullName)
& "$Jdk/bin/javac.exe" -source 8 -target 8 -encoding UTF-8 -bootclasspath "$platform;$bin/core-lambda-stubs.jar" -d "$build/classes" @sources "$PSScriptRoot/tests/RecoveryInstrumentation.java"
Check 'Compile instrumentation'
& "$Jdk/bin/jar.exe" cf "$build/classes.jar" -C "$build/classes" .
Check 'Package test classes'
& "$bin/d8.bat" --lib $platform --min-api 26 --output "$build/dex" "$build/classes.jar"
Check 'Dex tests'
& "$bin/aapt2.exe" link -I $platform --manifest "$build/AndroidManifest.xml" -o "$build/unsigned.apk"
Check 'Test APK'
& "$Jdk/bin/jar.exe" uf "$build/unsigned.apk" -C "$build/dex" classes.dex
Check 'Add tests'
& "$bin/zipalign.exe" -f -p 4 "$build/unsigned.apk" "$build/aligned.apk"
Check 'Align tests'
& "$bin/apksigner.bat" sign --ks $DebugKeystore --ks-key-alias androiddebugkey --ks-pass pass:android --out "$build/test.apk" "$build/aligned.apk"
Check 'Sign tests'
& $adb -s $Serial install -r $Apk
Check 'Install target'
& $adb -s $Serial install -r "$build/test.apk"
Check 'Install instrumentation'
$result=& $adb -s $Serial shell am instrument -w me.shind0.carrotweb.recoverytest/me.shind0.carrotweb.RecoveryInstrumentation
$result
if ($LASTEXITCODE -ne 0 -or "$result" -notmatch 'Recovery WebView checks passed:') { throw 'WebView recovery test failed' }
