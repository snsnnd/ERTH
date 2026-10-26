<#
  STM32F103C8T6 小车固件 一键构建脚本（Windows）
  流程：CubeMX 无头生成 MDK-ARM 工程 -> 挂载 mycode -> 注入 main.c 入口 -> UV4 编译

  用法（或直接双击 build.cmd）：
    powershell -NoProfile -ExecutionPolicy Bypass -File build.ps1
    powershell -ExecutionPolicy Bypass -File build.ps1 -NoGenerate      # 只重新注入+编译
    powershell -ExecutionPolicy Bypass -File build.ps1 -MxToolchain "MDK-ARM V5"   # 改用 AC5

  退出码与 UV4 语义一致：0=无错无警告  1=仅警告  2=有错误  3=致命错误/环境问题
#>
[CmdletBinding()]
param(
    [string]$CubemxRoot = 'D:\stm32\STM32CUBEMX',        # STM32CubeMX 安装目录
    [string]$Uv4Path    = 'D:\stm32\Keil\UV4\UV4.exe',   # Keil uVision4/5 命令行程序
    [string]$WorkRoot   = 'D:\tmp_my',                   # 工程落地目录
    [string]$MxToolchain = 'MDK-ARM V6',                 # MDK-ARM V6=AC6(默认，现代 MDK 自带) / MDK-ARM V5=AC5
    [switch]$NoGenerate                                  # 跳过 CubeMX，直接重新注入+编译
)

$ErrorActionPreference = 'Stop'

# ---------------- 常量 ----------------
$ProjName   = 'STM32F103C8_Car'
$FlashLimit = 65536   # STM32F103C8T6 Flash
$RamLimit   = 20480   # STM32F103C8T6 SRAM

$Here    = Split-Path -Parent $MyInvocation.MyCommand.Path   # .../NEW_Borad/firmware
$Repo    = Split-Path -Parent $Here                          # .../NEW_Borad
$SrcRoot = Join-Path $Repo 'mycode'
$IoCSrc  = Join-Path $Here "$ProjName.ioc"
$Work    = Join-Path $WorkRoot $ProjName

function Write-Step([string]$m) { Write-Host "`n==> $m" -ForegroundColor Cyan }
function Fail([string]$m, [int]$code = 3) {
    Write-Host "`n[失败] $m" -ForegroundColor Red
    exit $code
}

# =========================================================
# 1. 环境检查
# =========================================================
Write-Step "1/7 检查工具链"

if (-not (Test-Path $IoCSrc)) { Fail "找不到工程描述文件：$IoCSrc" }
if (-not (Test-Path $SrcRoot)) { Fail "找不到源码目录：$SrcRoot" }

$CubemxExe = Join-Path $CubemxRoot 'STM32CubeMX.exe'
if (-not (Test-Path $CubemxExe)) {
    Fail "CubeMX 不存在：$CubemxExe`n可用 -CubemxRoot 指定安装目录（例如 -CubemxRoot 'E:\STM32CUBEMX'）"
}
$JavaExe = Join-Path $CubemxRoot 'jre\bin\java.exe'
Write-Host "  CubeMX : $CubemxExe"

if (-not (Test-Path $Uv4Path)) { Fail "UV4 不存在：$Uv4Path`n可用 -Uv4Path 指定（例如 -Uv4Path 'C:\Keil\UV4\UV4.exe'）" }
Write-Host "  UV4    : $Uv4Path"

# =========================================================
# 2. 准备干净的工作目录
# =========================================================
if (-not $NoGenerate) {
    Write-Step "2/7 准备工作目录 $Work"
    if (Test-Path $Work) { Remove-Item $Work -Recurse -Force }
    New-Item -ItemType Directory -Path $Work -Force | Out-Null
    Copy-Item $IoCSrc (Join-Path $Work "$ProjName.ioc") -Force
} else {
    Write-Step "2/7 跳过目录清理（-NoGenerate）"
    if (-not (Test-Path (Join-Path $Work "$ProjName.ioc"))) {
        Fail "-NoGenerate 需要已存在的工程：$Work`n首次请不带该参数运行"
    }
}

# =========================================================
# 3. CubeMX 无头生成 MDK-ARM 工程
# =========================================================
$Uvprojx = $null
if (-not $NoGenerate) {
    Write-Step "3/7 CubeMX 生成 MDK-ARM 工程（toolchain = $MxToolchain）"

    # 脚本放在 work 目录之外，避免生成过程清理掉它
    $MxScript = Join-Path $WorkRoot '_cubemx_gen.txt'
    $IoCInWork = Join-Path $Work "$ProjName.ioc"
    $lines = @(
        "config load $IoCInWork",
        "project toolchain $MxToolchain",
        "project generate",
        "exit"
    )
    # toolchain 名带空格，CubeMX 脚本要求加引号
    $lines[1] = 'project toolchain "' + $MxToolchain + '"'
    [IO.File]::WriteAllText($MxScript, ($lines -join "`r`n") + "`r`n", (New-Object Text.UTF8Encoding($false)))

    Write-Host "  脚本: $MxScript"

    Push-Location $Work
    try {
        # 首选：直接调用 CubeMX 启动器
        $p = Start-Process -FilePath $CubemxExe -ArgumentList @('-q', ('"{0}"' -f $MxScript)) -Wait -PassThru -WindowStyle Hidden
        $mxRc = $p.ExitCode
        $found = Get-ChildItem -Path $Work -Recurse -Filter '*.uvprojx' -ErrorAction SilentlyContinue | Select-Object -First 1

        # 回退：用 CubeMX 自带 JRE 直接跑 jar（个别安装的 exe 启动器不转发参数）
        if ((-not $found) -and (Test-Path $JavaExe)) {
            Write-Host "  exe 启动未产出工程，回退 java -jar ..." -ForegroundColor Yellow
            & $JavaExe -jar ('"{0}"' -f $CubemxExe) -q ('"{0}"' -f $MxScript)
            $found = Get-ChildItem -Path $Work -Recurse -Filter '*.uvprojx' -ErrorAction SilentlyContinue | Select-Object -First 1
        }

        if (-not $found) {
            $hint = "CubeMX 未生成 .uvprojx（退出码 $mxRc）。常见原因：`n" +
                    "  1) .ioc 字段与本机 CubeMX 版本不匹配（本 .ioc 按 6.8.1 保存，换 CubeMX 版本试试）`n" +
                    "  2) 本机未安装 MDK-ARM 工具链组件`n" +
                    "  3) 工具链名不被接受，可改用 -MxToolchain `"MDK-ARM V5`"`n" +
                    "  4) 看 CubeMX 窗口/日志（生成后应有 Core\ 与 MDK-ARM\ 目录）"
            Fail $hint
        }
        $Uvprojx = $found.FullName
        Write-Host "  已生成: $Uvprojx"
    }
    finally { Pop-Location }
} else {
    Write-Step "3/7 跳过 CubeMX（-NoGenerate）"
    $Uvprojx = (Get-ChildItem -Path $Work -Recurse -Filter '*.uvprojx' -ErrorAction SilentlyContinue | Select-Object -First 1).FullName
    if (-not $Uvprojx) { Fail "工作目录里没有 .uvprojx：$Work" }
}

# =========================================================
# 4. 挂载 mycode
# =========================================================
Write-Step "4/7 挂载 mycode 源码"

$Dst = Join-Path $Work 'mycode'
if (Test-Path $Dst) { Remove-Item $Dst -Recurse -Force }
Copy-Item $SrcRoot $Dst -Recurse -Force

$CSources = @(Get-ChildItem -Path $Dst -Recurse -Filter '*.c' | Sort-Object FullName)
if ($CSources.Count -eq 0) { Fail "mycode 下没有 .c 文件" }

# Keil 用源文件名生成 .obj，同名会冲突
$dup = $CSources | Group-Object Name | Where-Object { $_.Count -gt 1 }
if ($dup) { Fail ("源文件名重复，Keil 无法编译：" + (($dup | ForEach-Object { $_.Name }) -join ', ')) }
Write-Host "  $($CSources.Count) 个 C 源文件"

# =========================================================
# 5. 注入 .uvprojx：源文件组 + 头文件搜索路径
# =========================================================
Write-Step "5/7 注入源文件与头文件路径到 .uvprojx"

# mycode 相对 .uvprojx 的根路径（Keil 支持相对路径，也支持绝对路径兜底）
# 相对形式 = 由 .uvprojx 相对 work 目录的深度算出的 ../../.. 前缀
$CodeRoot = $null
if ($Uvprojx.StartsWith($Work, [StringComparison]::OrdinalIgnoreCase)) {
    $uvRel   = $Uvprojx.Substring($Work.Length).TrimStart('\', '/')
    $depth   = ($uvRel -split '[\\/]').Count - 1
    $CodeRoot = ('..\' * $depth) + 'mycode'
}
if (-not $CodeRoot) {
    $CodeRoot = $Dst    # 无法算相对路径时退回绝对路径（Keil 同样接受）
    Write-Host "  [提示] .uvprojx 不在工作目录内，改用绝对路径引用 mycode" -ForegroundColor Yellow
}

# mycode 下参与编译的头文件搜索路径
$IncSub = @('', 'top', 'driver\basic', 'driver\community\iic', 'driver\mpu', 'driver\tool')
$IncPath = ($IncSub | ForEach-Object {
        if ($_ -eq '') { $CodeRoot } else { $CodeRoot + '\' + $_ }
    }) -join ';'

$uvText = [IO.File]::ReadAllText($Uvprojx)

# 5.1 头文件搜索路径（Cads/Aads 各可能一份，全部追加；每次都是新生成的工程，无需去重）
$uvText = [regex]::Replace($uvText, '(<IncludePath>)([^<]*)(</IncludePath>)', ('$1$2;' + $IncPath + '$3'))

# 5.2 mycode 源文件组
$filesXml = ($CSources | ForEach-Object {
        $rel = $CodeRoot + ($_.FullName.Substring($Dst.Length) -replace '/', '\')
        "        <File>`r`n" +
        "          <FileName>$($_.Name)</FileName>`r`n" +
        "          <FileType>1</FileType>`r`n" +
        "          <FilePath>$rel</FilePath>`r`n" +
        "        </File>"
    }) -join "`r`n"

$groupXml = "<Group>`r`n" +
            "  <GroupName>mycode</GroupName>`r`n" +
            "  <Files>`r`n" + $filesXml + "`r`n" +
            "  </Files>`r`n" +
            "</Group>`r`n" +
            "      </Groups>"

$before = $uvText.Length
$uvText = $uvText.Replace('</Groups>', $groupXml)
if ($uvText.Length -eq $before) { Fail ".uvprojx 中没有 </Groups>，工程结构异常：$Uvprojx" }

# 保留原编码（Keil 的 .uvprojx 常带 UTF-8 BOM）
$enc = New-Object Text.UTF8Encoding($false)
$head3 = [IO.File]::ReadAllBytes($Uvprojx)
if ($head3.Length -ge 3 -and $head3[0] -eq 239 -and $head3[1] -eq 187 -and $head3[2] -eq 191) {
    $enc = New-Object Text.UTF8Encoding($true)
}
[IO.File]::WriteAllText($Uvprojx, $uvText, $enc)
Write-Host "  头文件路径 += $IncPath"

# =========================================================
# 6. 注入固件入口（main.c）与 JTAG 引脚释放（msp）
# =========================================================
Write-Step "6/7 注入 main.c 入口"

function Add-AfterMarker {
    param([string]$Path, [string[]]$Markers, [string]$Line, [string]$Pad)
    $t = [IO.File]::ReadAllText($Path)
    if ($t -match [regex]::Escape($Line)) { return $false }   # 幂等
    foreach ($mk in $Markers) {
        $i = $t.IndexOf($mk)
        if ($i -ge 0) {
            $e = $t.IndexOf('*/', $i)
            $head = $t.Substring($i, $e + 2 - $i)
            $t = $t.Substring(0, $i) + $head + "`r`n" + $Pad + $Line + $t.Substring($e + 2)
            [IO.File]::WriteAllText($Path, $t)
            return $true
        }
    }
    return $false
}

$MainC = Join-Path $Work 'Core\Src\main.c'
if (-not (Test-Path $MainC)) { Fail "找不到 $MainC（CubeMX 生成结果不完整）" }

$ok = $true
$ok = (Add-AfterMarker -Path $MainC -Markers @('/* USER CODE BEGIN Includes */', '/* USER CODE BEGIN 0 */') -Line '#include "FSM.h" // init()/loop()/global_robot' -Pad '') -and $ok
$ok = (Add-AfterMarker -Path $MainC -Markers @('/* USER CODE BEGIN 2 */') -Line 'init(&global_robot); // 固件初始化（含 TIM1 10ms 基准）' -Pad '  ') -and $ok
$ok = (Add-AfterMarker -Path $MainC -Markers @('/* USER CODE BEGIN 3 */') -Line 'loop(&global_robot); // 主循环：状态机 + 定位' -Pad '    ') -and $ok
if (-not $ok) { Fail "main.c 缺少 USER CODE 注入点，CubeMX 模板版本异常：$MainC" }
Write-Host "  main.c: init() + loop() 已注入"

# PA15/PB3/PB4 = LED2~LED4，占用了 JTAG 引脚：必须关 JTAG 留 SWD，
# 否则这三个灯不会亮（不影响编译，属于运行期保险）
$Msp = Join-Path $Work 'Core\Src\stm32f1xx_hal_msp.c'
if (Test-Path $Msp) {
    $mspText = [IO.File]::ReadAllText($Msp)
    if ($mspText -notmatch 'REMAP_SWJ_NOJTAG') {
        $inject = "  __HAL_RCC_AFIO_CLK_ENABLE();`r`n" +
                  "  __HAL_AFIO_REMAP_SWJ_NOJTAG(); // 释放 PA15/PB3/PB4(JTAG) 给 LED2~4，保留 SWD"
        if ($mspText -match '__HAL_RCC_AFIO_CLK_ENABLE\(\)\s*;') {
            $mspText = $mspText -replace '(__HAL_RCC_AFIO_CLK_ENABLE\(\)\s*;)', $inject
            [IO.File]::WriteAllText($Msp, $mspText)
            Write-Host "  msp: 已释放 JTAG，保留 SWD"
        }
        elseif ($mspText -match 'void\s+HAL_MspInit\s*\(\s*void\s*\)\s*\{') {
            $mspText = $mspText -replace '(void\s+HAL_MspInit\s*\(\s*void\s*\)\s*\{)', "`$1`r`n$inject"
            [IO.File]::WriteAllText($Msp, $mspText)
            Write-Host "  msp: 已释放 JTAG，保留 SWD"
        }
        else {
            Write-Host "  [提示] 未找到 HAL_MspInit 注入点，跳过 JTAG 释放（不影响编译）" -ForegroundColor Yellow
        }
    }
    else {
        Write-Host "  msp: JTAG 释放已存在，跳过"
    }
}

# =========================================================
# 7. UV4 编译
# =========================================================
Write-Step "7/7 UV4 编译"

$UvDir   = Split-Path -Parent $Uvprojx
$LogName = 'uv4_build.log'
$LogFile = Join-Path $UvDir $LogName

# -b 增量构建，-j0 不弹对话框，-o 输出日志（相对 .uvprojx 所在目录）
$uvArgs = @('-b', '"' + $Uvprojx + '"', '-j0', '-o', '"' + $LogName + '"')
Write-Host "  UV4 $($uvArgs -join ' ')"

$uv = Start-Process -FilePath $Uv4Path -ArgumentList $uvArgs -Wait -PassThru -WindowStyle Hidden
$uvCode = $uv.ExitCode

# 日志可能落在工程目录或工作目录，逐一尝试
$foundLog = @($LogFile, (Join-Path $Work $LogName), (Join-Path $WorkRoot $LogName)) |
    Where-Object { Test-Path $_ } | Select-Object -First 1
$uvTextLog = if ($foundLog) { [IO.File]::ReadAllText($foundLog) } else { '' }
if ($foundLog) { Write-Host "  日志: $foundLog" }

# ---- Program Size / 资源预算 ----
$flashUsed = $null; $ramUsed = $null
if ($uvTextLog -match 'Program Size:\s*Code=(\d+).*?RO-data=(\d+)') { $flashUsed = [int]$Matches[1] + [int]$Matches[2] }
if ($uvTextLog -match 'RW-data=(\d+)\s+ZI-data=(\d+)')               { $ramUsed   = [int]$Matches[1] + [int]$Matches[2] }

Write-Host "`n---------------- 结果 ----------------" -ForegroundColor Cyan
switch ($uvCode) {
    0 { Write-Host "编译通过：无错误、无警告" -ForegroundColor Green }
    1 { Write-Host "编译通过：有警告（$uvCode）" -ForegroundColor Yellow }
    2 { Write-Host "编译失败：有错误（$uvCode）" -ForegroundColor Red }
    3 { Write-Host "编译失败：致命错误（$uvCode）" -ForegroundColor Red }
    default { Write-Host "UV4 返回异常退出码 $uvCode（常见原因：许可证未激活 / 工程文件损坏）" -ForegroundColor Red }
}

if ($flashUsed) {
    $pct = [math]::Round($flashUsed * 100.0 / $FlashLimit, 1)
    $c = if ($flashUsed -gt $FlashLimit) { 'Red' } elseif ($pct -gt 85) { 'Yellow' } else { 'Green' }
    Write-Host ("Flash: {0} / {1} 字节 ({2}%)  [Code+RO]" -f $flashUsed, $FlashLimit, $pct) -ForegroundColor $c
    if ($flashUsed -gt $FlashLimit) { Write-Host "  ! 超出 F103C8 的 64KB Flash，需要裁剪（如关 DEBUG_PRINT、删 oledfont）" -ForegroundColor Red }
}
if ($ramUsed) {
    $pct = [math]::Round($ramUsed * 100.0 / $RamLimit, 1)
    $c = if ($ramUsed -gt $RamLimit) { 'Red' } elseif ($pct -gt 85) { 'Yellow' } else { 'Green' }
    Write-Host ("RAM  : {0} / {1} 字节 ({2}%)  [RW+ZI]" -f $ramUsed, $RamLimit, $pct) -ForegroundColor $c
    if ($ramUsed -gt $RamLimit) { Write-Host "  ! 超出 F103C8 的 20KB RAM" -ForegroundColor Red }
}

if ($uvCode -ge 2 -and $foundLog) {
    Write-Host "`n----- 错误摘录（前 40 条 error/warning）-----" -ForegroundColor Red
    $uvTextLog -split "`r?`n" | Where-Object { $_ -match '(error|warning)\s*#?\d*' } |
        Select-Object -First 40 | ForEach-Object { Write-Host $_ }
}
Write-Host "----------------------------------------`n" -ForegroundColor Cyan

if ($uvCode -gt 3) { $uvCode = 3 }
exit $uvCode
