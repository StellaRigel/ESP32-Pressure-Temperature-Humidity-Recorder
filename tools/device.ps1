<#
  ============================================================================
   tools/device.ps1 —— 设备交互小工具（PowerShell，Windows 上直接用）
  ----------------------------------------------------------------------------
   用法：
     .\device.ps1 probe  [ip]                    查看 /status + /weblog 末尾
     .\device.ps1 ota    [ip] [-Bin <bin>]       OTA 上传 bin，然后等设备重新上线
     .\device.ps1 flash  [port] [-Bin <bin>]     USB 烧录（使用下方固定 FQBN）
     .\device.ps1 log    [port] [-Seconds 22] [-Trigger <url>]   抓串口日志
     .\device.ps1 find                           扫局域网找设备（80 端口 + /status 判定）
     .\device.ps1 dump   [ip] [-Out <file>]      抓显存 ASCII（= /ui-dump）
     .\device.ps1 toast  [ip] [-Dur 20000]       让屏幕上弹一条横幅

   参数默认值按本工程习惯：ip=192.168.42.99（2026-09-17 起；设备会随所在网络换网段，
   网络一变请显式传 ip），bin=%TEMP%\pht_build2\PHT_2_0.ino.bin（arduino-cli --build-path 的输出）。

   ⚠️ 2026-09-17 踩坑备忘：
     ① `arduino-cli` 实际在 **E:\TOOLS\Arduino\Arduino IDE\resources\app\lib\backend\resources\**（IDE 2.x 内置），
        原先写死的 D 盘路径已失效 → 现已改为 Find-ArduinoCli 自动搜索。
     ② 工程在 OneDrive 且路径含 `C++`/空格时，`PartitionScheme=custom` 的 COPY 配方会
        `The syntax of the command is incorrect` → **先把 .ino/.h/.csv 复制到纯 ASCII 短路径**（如 C:\phtc\PHT_2_0）再编译。
     ③ 串口默认 COM4（= USBSER000，板载 USB-Serial-JTAG；本机 COM1 是别的东西）。
  ============================================================================
#>
param(
  [Parameter(Position = 0)][string]$Cmd = 'probe',
  [Parameter(Position = 1)][string]$Ip = '192.168.42.99',
  [string]$Port = 'COM4',
  [string]$Bin = "$env:TEMP\pht_build2\PHT_2_0.ino.bin",
  [int]$Seconds = 22,
  [int]$Dur = 20000,
  [string]$Out = '',
  [string]$Trigger = ''
)
$ErrorActionPreference = 'Continue'
$FQBN = 'esp32:esp32:esp32s3:FlashSize=16M,PSRAM=opi,PartitionScheme=custom,FlashMode=qio,CPUFreq=240,CDCOnBoot=cdc'
$CFG  = "$env:USERPROFILE\.arduinoIDE\arduino-cli.yaml"

# ---- 找 arduino-cli（2026-09-17：原先写死的 D:\TOOLS\... 已失效，改为按候选+搜索）----
function Find-ArduinoCli {
  $cand = @(
    'E:\TOOLS\Arduino\Arduino IDE\resources\app\lib\backend\resources\arduino-cli.exe',  # IDE 2.x 内置（本机实测）
    'D:\TOOLS\Arduino\Arduino CLI\arduino-cli.exe',                                      # 旧路径（已失效，留作兜底）
    'E:\TOOLS\Arduino\Arduino CLI\arduino-cli.exe',
    "$env:LOCALAPPDATA\Programs\Arduino IDE\resources\app\lib\backend\resources\arduino-cli.exe",
    'C:\Program Files\Arduino IDE\resources\app\lib\backend\resources\arduino-cli.exe'
  )
  foreach ($c in $cand) { if (Test-Path $c) { return $c } }
  $g = Get-Command arduino-cli.exe -ErrorAction SilentlyContinue      # PATH 里有没有
  if ($g) { return $g.Source }
  foreach ($root in @('E:\TOOLS', 'D:\TOOLS', "$env:LOCALAPPDATA\Programs", 'C:\Program Files')) {  # 最后搜一轮（限深度）
    if (-not (Test-Path $root)) { continue }
    $hit = Get-ChildItem $root -Recurse -Depth 6 -Filter 'arduino-cli.exe' -ErrorAction SilentlyContinue |
           Select-Object -First 1
    if ($hit) { return $hit.FullName }
  }
  return $null
}
$CLI = Find-ArduinoCli
if (-not (Test-Path $CFG)) { $CFG = '' }      # 配置不存在就别硬传 --config-file

function Invoke-ArduinoCli([string[]]$CliArgs) {
  if (-not $CLI) { Write-Output '❌ 找不到 arduino-cli（请改本脚本里的候选路径，或装 arduino-cli 并加入 PATH）'; return 1 }
  $a = @(); if ($CFG) { $a += @('--config-file', $CFG) } ; $a += $CliArgs
  & $CLI @a
  return $LASTEXITCODE
}

function Wait-Online([string]$ip, [int]$Sec = 60) {
  $t0 = Get-Date
  while (((Get-Date) - $t0).TotalSeconds -lt $Sec) {
    try { $null = Invoke-WebRequest "http://$ip/status" -TimeoutSec 4 -UseBasicParsing; return $true } catch { Start-Sleep -Seconds 2 }
  }
  return $false
}

switch ($Cmd) {
  'probe' {
    try {
      $s = (Invoke-WebRequest "http://$Ip/status" -TimeoutSec 6 -UseBasicParsing).Content | ConvertFrom-Json
      Write-Output "[$Ip] 在线  batt=$($s.battVolt)V  chg=$(if($s.charging){'充'}else{'放'})  wifi=$($s.wifi)  count=$($s.count)  mode=$(if($s.deviceMode -eq 1){'MOV'}else{'FIX'})"
    } catch { Write-Output "[$Ip] ❌ /status 无响应: $_" }
    try {
      $lg = (Invoke-WebRequest "http://$Ip/weblog" -TimeoutSec 6 -UseBasicParsing).Content | ConvertFrom-Json
      Write-Output "--- /weblog 共 $(@($lg.lines).Count) 条，末尾 10 条 ---"
      @($lg.lines) | Select-Object -Last 10 | ForEach-Object { Write-Output "  | $_" }
    } catch { Write-Output "❌ /weblog 无响应" }
  }
  'ota' {
    if (-not (Test-Path $Bin)) { Write-Output "❌ 找不到 bin: $Bin"; break }
    Write-Output "POST http://$Ip/update  ($([math]::Round((Get-Item $Bin).Length/1KB))KB)"
    $r = & curl.exe -s -X POST -F "firmware=@$Bin" "http://$Ip/update" --max-time 180
    if ([string]::IsNullOrWhiteSpace($r)) { $rShow = '（空 —— 设备重启时回包偶尔会丢，以重新上线为准）' } else { $rShow = $r }
    Write-Output ("回包: " + $rShow)
    if (Wait-Online $Ip 90) { Write-Output "✅ 已重新上线" } else { Write-Output "❌ 90 秒内未上线，去抓串口日志看看" }
  }
  'flash' {
    Get-Process serial-monitor -ErrorAction SilentlyContinue | Stop-Process -Force
    Start-Sleep -Seconds 2
    $rc = Invoke-ArduinoCli @('upload', '-p', $Port, '--fqbn', $FQBN, '--input-dir', (Split-Path $Bin), 'PHT_2_0')
    if ($rc -eq 0) { Write-Output "✅ 烧录命令已执行（退出码 0）" } else { Write-Output "❌ 烧录失败（退出码 $rc）" }
    if (Wait-Online $Ip 60) { Write-Output "✅ 已上线" } else { Write-Output "❌ 未上线（若刚刷完，可先看串口日志：.\device.ps1 log $Port）" }
  }
  'log' {
    $sp = New-Object System.IO.Ports.SerialPort $Port, 115200, 'None', 8, 'One'
    $sp.Encoding = [System.Text.Encoding]::UTF8
    $sp.DtrEnable = $false; $sp.RtsEnable = $false
    try { $sp.Open() } catch { Write-Output "串口打开失败: $_"; break }
    $sb = New-Object System.Text.StringBuilder
    [void]$sb.Append($sp.ReadExisting()); Start-Sleep -Seconds 2
    if ($Trigger) { try { $null = Invoke-WebRequest $Trigger -TimeoutSec 5 -UseBasicParsing; [void]$sb.Append("[[trigger OK $Trigger]]`n") } catch { [void]$sb.Append("[[trigger ERR $_]]`n") } }
    $dl = (Get-Date).AddSeconds($Seconds)
    while ((Get-Date) -lt $dl) { Start-Sleep -Milliseconds 200; try { [void]$sb.Append($sp.ReadExisting()) } catch { break } }
    $sp.Close()
    $txt = $sb.ToString()
    if ($Out) { Set-Content -Path $Out -Value $txt -Encoding UTF8; Write-Output "已写入 $Out" } else { Write-Output $txt }
  }
  'find' {
    $pfx = @()
    Get-NetIPAddress -AddressFamily IPv4 | Where-Object { $_.IPAddress -notlike '127.*' -and $_.IPAddress -notlike '169.254.*' -and $_.PrefixLength -ge 24 } |
      ForEach-Object { $pfx += (($_.IPAddress -split '\.')[0..2] -join '.') }
    $pfx = $pfx | Sort-Object -Unique
    Write-Output "扫描网段: $($pfx -join ', ')"
    foreach ($p in $pfx) {
      $cli = @()
      foreach ($n in 1..254) { $c = New-Object System.Net.Sockets.TcpClient; $null = $c.BeginConnect("$p.$n", 80, $null, $null); $cli += [pscustomobject]@{ H = "$p.$n"; C = $c } }
      Start-Sleep -Milliseconds 1800
      foreach ($t in $cli) {
        if ($t.C.Connected) {
          try { $s = (Invoke-WebRequest "http://$($t.H)/status" -TimeoutSec 3 -UseBasicParsing).Content; if ($s -match 'bmpOK') { Write-Output ">>> 命中气象站: $($t.H)  $($s.Substring(0, [Math]::Min(110, $s.Length)))" } } catch { }
        }
        $t.C.Close()
      }
    }
    Write-Output '--- done ---'
  }
  'dump' {
    $txt = (Invoke-WebRequest "http://$Ip/ui-dump" -TimeoutSec 20 -UseBasicParsing).Content
    if ($Out) { Set-Content -Path $Out -Value $txt -Encoding UTF8; Write-Output "已写入 $Out（$((Get-Content $Out).Count) 行）" } else { Write-Output $txt }
  }
  'toast' {
    Write-Output (& curl.exe -s "http://$Ip/toast?d=$Dur")
  }
  default { Write-Output "未知子命令 '$Cmd'。可用: probe | ota | flash | log | find | dump | toast" }
}
