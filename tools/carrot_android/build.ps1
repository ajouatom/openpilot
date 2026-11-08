[CmdletBinding(DefaultParameterSetName='Debug')]
param(
  [string]$Sdk = "$env:LOCALAPPDATA\Android\Sdk",
  [string]$BuildTools = "35.0.0",
  [string]$Platform = "android-36",
  [string]$Jdk = $env:JAVA_HOME,
  [Parameter(Mandatory=$true)][string]$OutputApk,
  [Parameter(Mandatory=$true,ParameterSetName='Debug')][string]$DebugKeystore,
  [Parameter(Mandatory=$true,ParameterSetName='Release')][string]$SigningKeystore,
  [Parameter(ParameterSetName='Release')][string]$SigningAlias = 'carrotassist',
  [Parameter(ParameterSetName='Release')][string]$SigningPasswordEnv = 'CARROT_ASSIST_SIGNING_PASSWORD'
)
$ErrorActionPreference = 'Stop'
if (!$Jdk) {
  $javaProperty = (& java -XshowSettings:properties -version 2>&1 | ForEach-Object { "$_" } | Select-String '^\s*java.home = ').ToString()
  $Jdk = ($javaProperty -replace '^\s*java.home = ','').Trim()
}
$javaExe = Join-Path $Jdk 'bin\java.exe'
$javacExe = Join-Path $Jdk 'bin\javac.exe'
$jarExe = Join-Path $Jdk 'bin\jar.exe'
$keytoolExe = Join-Path $Jdk 'bin\keytool.exe'
$sourceRoot = $PSScriptRoot
$buildRoot = Join-Path $sourceRoot ('build\' + [Guid]::NewGuid().ToString('N'))
$classes = Join-Path $buildRoot 'classes'
$dex = Join-Path $buildRoot 'dex'
$testClasses = Join-Path $buildRoot 'tests'
$platformJar = Join-Path $Sdk "platforms\$Platform\android.jar"
$bin = Join-Path $Sdk "build-tools\$BuildTools"
New-Item -ItemType Directory -Force $classes,$dex,$testClasses | Out-Null
function Assert-Exit([string]$step) { if ($LASTEXITCODE -ne 0) { throw "$step failed: $LASTEXITCODE" } }
$coreSources = @((Join-Path $sourceRoot 'src\me\shind0\carrotweb\DiscoveryPolicy.java'), (Join-Path $sourceRoot 'src\me\shind0\carrotweb\DeviceProbe.java'))
& $javacExe -encoding UTF-8 -d $testClasses @coreSources (Join-Path $sourceRoot 'tests\DiscoveryTest.java')
Assert-Exit 'Compile discovery checks'
& $javaExe -cp $testClasses DiscoveryTest
Assert-Exit 'Discovery checks'
& $javacExe -encoding UTF-8 -d $testClasses (Join-Path $sourceRoot 'src\me\shind0\carrotweb\UpdatePolicy.java') (Join-Path $sourceRoot 'tests\UpdatePolicyTest.java')
Assert-Exit 'Compile update policy checks'
& $javaExe -cp $testClasses UpdatePolicyTest
Assert-Exit 'Update policy checks'
& $javacExe -encoding UTF-8 -cp $testClasses -d $testClasses (Join-Path $sourceRoot 'src\me\shind0\carrotweb\ConnectionPolicy.java') (Join-Path $sourceRoot 'tests\ConnectionPolicyTest.java')
Assert-Exit 'Compile connection policy checks'
& $javaExe -cp $testClasses ConnectionPolicyTest
Assert-Exit 'Connection policy checks'
& $javacExe -encoding UTF-8 -cp $testClasses -d $testClasses (Join-Path $sourceRoot 'src\me\shind0\carrotweb\LegacyOriginHints.java') (Join-Path $sourceRoot 'tests\LegacyOriginHintsTest.java')
Assert-Exit 'Compile legacy origin checks'
& $javaExe -cp $testClasses LegacyOriginHintsTest
Assert-Exit 'Legacy origin checks'
& $javacExe -encoding UTF-8 -d $testClasses (Join-Path $sourceRoot 'src/me/shind0/carrotweb/LocalCommand.java') (Join-Path $sourceRoot 'src/me/shind0/carrotweb/SpokenSetting.java') (Join-Path $sourceRoot 'tests/LocalCommandTest.java')
Assert-Exit 'Compile local voice checks'
& $javaExe -cp $testClasses me.shind0.carrotweb.LocalCommandTest
Assert-Exit 'Local voice checks'

$sources = @(Get-ChildItem -LiteralPath (Join-Path $sourceRoot 'src') -Filter '*.java' -Recurse | ForEach-Object FullName)
& $javacExe -source 8 -target 8 -encoding UTF-8 -bootclasspath "$platformJar;$(Join-Path $bin 'core-lambda-stubs.jar')" -d $classes @sources
Assert-Exit 'Compile Android app'
$resources = Join-Path $buildRoot 'resources.zip'
& (Join-Path $bin 'aapt2.exe') compile --dir (Join-Path $sourceRoot 'res') -o $resources
Assert-Exit 'Compile resources'
$unsigned = Join-Path $buildRoot 'unsigned.apk'
& (Join-Path $bin 'aapt2.exe') link -I $platformJar --manifest (Join-Path $sourceRoot 'AndroidManifest.xml') --min-sdk-version 26 --target-sdk-version 36 -o $unsigned $resources
Assert-Exit 'Link APK'
$classJar = Join-Path $buildRoot 'classes.jar'
& $jarExe cf $classJar -C $classes .
Assert-Exit 'Package classes'
& (Join-Path $bin 'd8.bat') --lib $platformJar --min-api 26 --output $dex $classJar
Assert-Exit 'Dex app'
& $jarExe uf $unsigned -C $dex classes.dex
Assert-Exit 'Add dex'
$aligned = Join-Path $buildRoot 'aligned.apk'
& (Join-Path $bin 'zipalign.exe') -f -p 4 $unsigned $aligned
Assert-Exit 'Align APK'
if ($PSCmdlet.ParameterSetName -eq 'Debug') {
  $DebugKeystore = [IO.Path]::GetFullPath($DebugKeystore)
  if (!(Test-Path -LiteralPath $DebugKeystore)) {
  New-Item -ItemType Directory -Force ([IO.Path]::GetDirectoryName($DebugKeystore)) | Out-Null
  & $keytoolExe -genkeypair -keystore $DebugKeystore -storepass android -keypass android -alias androiddebugkey -dname 'CN=Android Debug,O=Android,C=US' -keyalg RSA -keysize 2048 -validity 3650
  Assert-Exit 'Create local test signing key'
  }
  $SigningKeystore = $DebugKeystore
  $SigningAlias = 'androiddebugkey'
  $signingPassArg = 'pass:android'
} else {
  $SigningKeystore = [IO.Path]::GetFullPath($SigningKeystore)
  if (!(Test-Path -LiteralPath $SigningKeystore) -or ![Environment]::GetEnvironmentVariable($SigningPasswordEnv)) { throw 'Release signing key and password environment variable are required' }
  $signingPassArg = "env:$SigningPasswordEnv"
}
$OutputApk = [IO.Path]::GetFullPath($OutputApk)
New-Item -ItemType Directory -Force ([IO.Path]::GetDirectoryName($OutputApk)) | Out-Null
& (Join-Path $bin 'apksigner.bat') sign --ks $SigningKeystore --ks-key-alias $SigningAlias --ks-pass $signingPassArg --key-pass $signingPassArg --out $OutputApk $aligned
Assert-Exit 'Sign APK'
& (Join-Path $bin 'apksigner.bat') verify --verbose $OutputApk
Assert-Exit 'Verify signature'
Get-FileHash -LiteralPath $OutputApk -Algorithm SHA256
