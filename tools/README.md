# tools/ —— 屏幕自检与设备交互工具

这套工具解决一个问题：**改屏幕/改版面时，不靠人盯着 2.13" 小屏看，而是把真机显存抓回来逐像素比对。**
本工程用它抓出过两个真 bug（`SPI.transfer` 回读污染显存、`/toast?k=fail` 误显示 IP）。

## 环境

* **Node ≥ 18**（`node -v`），**零第三方依赖**（只用内置 `http/fs/zlib/url`）。
* `device.ps1` 需要 PowerShell + `curl.exe`（Win10+ 自带）；烧录需要 arduino-cli 路径（脚本里写死了本机路径）。
* 设备侧依赖固件里的 **`GET /ui-dump`** 端点（见下）。

## 设备接口约定

| 接口 | 用途 |
|---|---|
| `GET /ui-dump` | 把显存读回成 ASCII：每行 `yyy\|##..#`（`#`=墨迹，`.`=空白），**逻辑坐标 250×122** |
| `GET /ui-dump?x0=&y0=&x1=&y1=` | 同上，只取一块区域（省流量，例如 V1 只取 `x0=0&y0=20&x1=62&y1=121`） |
| `GET /toast[?d=ms][&k=ok\|fail\|ap][&q=1]` | 在屏幕上弹横幅（`?d` 停留时长，`?k` 强制类型，`?q=1` 再排一条热点横幅） |
| `GET /wireless[?force=auto\|wifi\|ble]` | 无线互斥联调口：强制某种无线（**3 分钟自愈**）；不传参只回报当前状态 |
| `GET /status` `GET /weblog` | 设备状态 / 环形日志（JSON） |
| `POST /update` | OTA 升级（multipart 字段名 `firmware`） |

> 逻辑坐标系由 `ui_display.h` 的 `readLogical()` 定义（横屏 250×122，物理面板 122×250，`UI_ROT` 切 90/270）。
> `/ui-dump` 和所有绘制都走同一套映射，所以抓回来的图能直接和离线渲染逐像素比。

## 脚本

| 脚本 | 干嘛 | 典型命令 |
|---|---|---|
| `lib.mjs` | **共用库**：字模解析、画布、图标点阵、Toast 版式、`/ui-dump` 解析、PNG 写盘、设备 HTTP | 被下面各脚本 import |
| `screen-check.mjs` | **平时版面自检**：dump 完整性、中文标签 = 1261 点、留白、数值区，并打印顶栏/数据区 ASCII | `node tools/screen-check.mjs 192.168.31.99` |
| `toast-verify.mjs` | **横幅全套自检**：三种版面逐像素、失败→排队→热点页时序、到期还原、超长 SSID 截断 | `node tools/toast-verify.mjs 192.168.31.99` |
| `anomaly-check.mjs` | **气压距平自检**：基线健全性（日均不再被空槽稀释）、固定/移动门控、**移动样本不得进基线**；会临时切模式并自动恢复 | `node tools/anomaly-check.mjs 192.168.31.99 --wait 35000` |
| `ble-web.html` | **手机蓝牙测试页**（只读）：Android Chrome 打开 → 连接 `PHT_2_X` → 读 `/status`。设备需处于蓝牙模式（移动+电池，或 `/wireless?force=ble`，3 分钟自愈） | 手机 Chrome 打开本文件 |
| `dump-page.mjs` | **把设备整页存成手机本地文件**：蓝牙只读页要求 `file://`（安全上下文）才能用 Web Bluetooth | `node tools/dump-page.mjs 192.168.31.99 bt-page.html` |
| `wifi-check.mjs` | **多组 WiFi 自检（20 项）**：列表/扫描/添加不打断连接/自动选最强/删除；外加**保存后当前组刷新**（日志证据）、密码留空不清空、IP 不被改动；`--reboot` 还会验网页重启（会临时加一条假网络，结束自动删） | `node tools/wifi-check.mjs 192.168.31.99 --reboot` |
| `capture-screen.mjs` | 抓真机屏幕拼成 PNG（"实拍图"，可发给别人验收） | `node tools/capture-screen.mjs 192.168.31.99 --out shot.png` |
| `preview.mjs` | **离线**预览版面 → PNG（不接硬件，用来跟你确认设计） | `node tools/preview.mjs --panel "ap\|AP MODE\|SSID \"X\"\|192.168.5.1"` |
| `icons-gen.mjs` | 生成自绘图标点阵：`--kind side\|above --radii 5,9 --span 45` → 可直接粘贴的 C++ 代码 | `node tools/icons-gen.mjs --radii 5,9 --span 45` |
| `extract-cjk.mjs` | 从中文字模来源（`backups/ui_landscape.h`）反解生成 `ui_cjk.h`，带往返比对 | `node tools/extract-cjk.mjs` |
| `device.ps1` | 设备交互：`probe` / `ota` / `flash` / `log`(串口) / `find`(扫网段) / `dump` / `toast` | `.\tools\device.ps1 ota 192.168.31.97` |
| `check-md.mjs` | **文档体检**：① 表格里**未转义的裸竖线**（哪怕在行内代码里也会撑破表格）；② **未配对的代码围栏**（多一个会把后面整篇吞进代码块）；退出码 1=有异常 | `node tools/check-md.mjs`（默认扫全工程 *.md） |

## ⚠️ 最重要的约定

**① `tools/lib.mjs` 里的版式与点阵必须和固件同步改。**

**② 改完文档跑一下 `node tools/check-md.mjs` 体检**（退出码 0 才算干净）。它查两类「整段渲染不出来」的坑：

* **表格里的裸竖线**：GFM 里哪怕写在行内代码里也会被当列分隔符 → 一律转义成 `\|`（渲染出来仍是竖线本身，语义不变）。
* **未配对的代码围栏（三反引号）**：多留一个会开启一个永不关闭的代码块，**把后面整篇内容吞进去**
  —— 症状就是「前面正常、后面全坏」。实测事故：设计文档取消蓝牙提示时删了内容但漏删围栏，
  留了个孤儿三反引号，把后面 200 多行全吞了。

具体位置：`ICON.tick/cross/bcast`（↔ `ui_display.h::drawTick/drawCross/drawBcast`，含位序）、
`drawToast()`（↔ `toastLine1()` 的每类图标槽宽 `iw`/间距 `gap`、三行 y 坐标 30/63/87）。
改了固件不同步改这里，自检会报"差异 N px"——那不是固件错，是期望值旧了。

字体改了就换 `ui_font_oldsans.h`（`loadFont()` 自己解析 adv 表 + 字模表，不需要改代码）。

## 典型工作流

```powershell
# ① 改完屏幕相关代码，编译 + 上机
arduino-cli compile --fqbn <FQBN> --build-path %TEMP%\pht_build2 PHT_2_X
powershell -ExecutionPolicy Bypass -File tools\device.ps1 flash COM9     # 直接 .\device.ps1 会被执行策略拦住
powershell -ExecutionPolicy Bypass -File tools\device.ps1 ota 192.168.31.97

# ② 自动验收（不用人看屏）
node tools\screen-check.mjs 192.168.31.99      # 平时版面
node tools\toast-verify.mjs 192.168.31.99      # 横幅 + 时序   → exit code 非 0 即为失败
node tools\anomaly-check.mjs 192.168.31.99    # 距平：基线 + 固定/移动门控（临时切模式，结束自动恢复）
node tools\wifi-check.mjs 192.168.31.99       # 多组 WiFi：增删/扫描/自动选最强（临时加一条假网络，结束自动删）

# ③ 需要给人看效果时出图
node tools\capture-screen.mjs 192.168.31.99 --out shot.png     # 真机实拍
node tools\preview.mjs --panel "ok|WiFi OK|SSID \"x\"|1.2.3.4" --out design.png   # 离线设计稿

# ④ 改图标 / 加中文
node tools\icons-gen.mjs --kind side --radii 5,9 --span 45     # 挑点阵，粘进 ui_display.h
node tools\extract-cjk.mjs                                     # 重新生成 ui_cjk.h（会覆盖！）
```

## 注意事项

* `extract-cjk.mjs` **会覆盖 `ui_cjk.h`**，跑之前确认底图没动过。
* `device.ps1` 直接执行会被 PowerShell 执行策略拦住，用 `powershell -ExecutionPolicy Bypass -File tools\device.ps1 ...`。
* `wifi-check.mjs` 会在设备的 WiFi 配置里临时加一条假网络（并在结束时删掉）；
  它也会触发一次 `/wifi-reconnect`——正常一两秒就重连回来，若真换了网络记得把电脑也切过去。
* `capture-screen.mjs` / `toast-verify.mjs` 会在你的屏幕上弹真横幅（几秒～十几秒后自动还原），
  测完请确认设备回到平时版面（`screen-check.mjs` 一看便知）。
* 电池供电时采样/刷屏是省电节流的，但横幅本身不受影响（`showAll()` 在横幅期间只刷顶栏）。
* 脚本都接受 `[ip]` 参数；不传默认 `192.168.31.99`。多台设备可用 `.97/.98/.99` 分别跑。
