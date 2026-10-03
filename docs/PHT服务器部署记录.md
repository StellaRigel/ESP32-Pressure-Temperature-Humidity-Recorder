# PHT 服务器部署记录（KICKPI K3B）

> 2026-10-03 完成部署并验证。归档环境信息与关键坑，便于重建。

## 一、硬件与系统

| 项 | 值 |
|---|---|
| 板子 | KICKPI K3B（RK3562 / 2GB RAM / 16GB eMMC） |
| 系统 | **Armbian 26.8.6 trixie**（Debian 13.7） |
| 内核 | `7.2.5-edge-rockchip64`（aarch64） |
| 根设备 | `/dev/mmcblk1p1`（eMMC，14.7G） |
| 主机名 | `pht-server` |
| 静态 IP | **`192.168.1.100/24`**，网关 `192.168.1.1`，DNS `223.5.5.5` |
| 时区 | `Asia/Shanghai` |
| 内存 | 1.9G + 984M zram swap |
| 串口 | `COM3`（CH343，**1500000 波特率**） |

**⚠️ 地址规划（重要）**：
```
192.168.1.99   = 【PHT 气象站 ESP32】—— 绝不能让服务器占用！
192.168.1.100  = 【K3B 服务器】
192.168.1.42   = PC
```

**为什么选 `.100` 而不是 `.20`**：
`.20` 落在**家用路由器 DHCP 池的常见区间**（很多默认从 `.2` 或 `.10` 起分配几十上百个），
存在被路由器分配出去造成冲突的风险。`.100` 位置更高、更不容易撞上。
实测该网段在用地址为 `.1`（网关）`.2`（DNS）`.33` `.44` 等，`.100` 确认空闲。

## 二、eMMC 安装

**关键：`armbian-install` 有非交互 API**（不必碰 TUI）：

```bash
armbian-install --api detect                                   # 机器可读设备清单
armbian-install --api plan --target /dev/mmcblk1 --boot emmc --fs ext4
armbian-install --target /dev/mmcblk1 --boot emmc --fs ext4 --yes
```

- 从 SD 卡启动，安装到 eMMC（`detect` 只列出 eMMC = `mmcblk1`，明确无歧义）
- 43 秒完成；单 ext4 分区占满全盘；引导加载器写入 eMMC

## 三、部署步骤（已执行）

```bash
# 1) 用户与目录
useradd -r -s /usr/sbin/nologin -d /opt/pht pht
mkdir -p /opt/pht/server
chown -R pht:pht /opt/pht

# 2) 上传代码（scp）
scp app.py config.py db.py ingest.py mqtt_ingest.py requirements.txt \
    README.md analyze_battcal.py root@192.168.1.100:/opt/pht/server/
scp -r static deploy root@192.168.1.100:/opt/pht/server/

# 3) 依赖（需先装 python3.13-venv）
apt-get install -y python3.13-venv python3-pip
su -s /bin/bash pht -c 'cd /opt/pht/server && python3 -m venv .venv'
su -s /bin/bash pht -c 'cd /opt/pht/server && \
  .venv/bin/pip install -r requirements.txt -i https://mirrors.aliyun.com/pypi/simple/'

# 4) 服务
install -m 644 deploy/pht-server.service /etc/systemd/system/
mkdir -p /opt/pht/server/data && chown pht:pht /opt/pht/server/data
systemctl daemon-reload
systemctl enable --now pht-server
```

**已装版本**：fastapi 0.142.2 / uvicorn 0.54.0 / pydantic 2.13.5 / Python 3.13.5

## 四、验证结果（2026-10-03）

```
GET  /api/v1/health  → {"ok":true,...,"totalSamples":0}
GET  /               → HTTP 200, 5547 B, text/html
POST /api/v1/samples → {"ok":true,"accepted":1,"duplicated":0,"rejected":0}
GET  /api/v1/devices → {"device_id":"test-pc",...,"n":1}
```

服务内存占用 **34.3 MB**（上限 512M），系统内存占用 11.6%。

## 五、踩过的坑（务必记住）

### 1. ⛔ 这板子**不能用 `systemctl reboot` / `reboot`**

```
✅ 硬上电（断电再上电）→ 正常启动
❌ 软重启             → 关机后不再起来，必须手动断电
```

实测两次：`systemd-shutdown[1]: All filesystems unmounted.` 之后**再无任何串口输出**。
**一律用硬断电重启。**

### 2. 静态 IP 被 netplan 的通配配置抢走

systemd 257 的日志揭示了原因：

```
eth0: Found matching .network file, based on potentially unpredictable interface name:
      /run/systemd/network/10-netplan-all-eth-interfaces.network
```

Armbian 默认的 `10-dhcp-all-interfaces.yaml` 生成 `[Match] Name=e*`（通配），
**systemd 会选用它而不是更精确的 `Name=eth0`**。

**解法**：把该 netplan 文件移走，只留自己的静态配置：
```bash
mv /etc/netplan/10-dhcp-all-interfaces.yaml /root/10-dhcp-all-interfaces.yaml.bak
netplan generate && systemctl restart systemd-networkd
```

### 3. Mali GPU（panfrost）导致内核 MCE

```
Internal error: synchronous external abort  ← panfrost 初始化时触发
  rockchip_pd_power_off / regmap_mmio_read32le
```

设备树有 Mali-G52 节点，`panfrost` 自动绑定但在电源域操作时 abort。**非致命**（系统照常启动）。
服务器不需要 GPU，**拉黑即可**：

```bash
echo "blacklist panfrost" > /etc/modprobe.d/blacklist-panfrost.conf
update-initramfs -u
```

**⚠️ `update-initramfs -u` 之后必须确认 `/boot/uInitrd` 完好**（U-Boot 依赖它）：
```bash
ls -la /boot/uInitrd /boot/uInitrd-$(uname -r)
```
若损坏：插回 SD 卡启动，挂载 eMMC，从 SD 的 `/boot` 复制一份好的 uInitrd。

### 4. 无 WiFi（板载模组不可用）

天线在、`sdio-pwrseq` 与 `wireless-wlan` 设备树节点都在，但：
```
/sys/bus/sdio/devices/  → 空
dmesg: mmc0 SDIO 控制器 400k→300k→200k→100k 后放弃（设备未应答）
内核无线模块: rtl8189es/fs, rtl8723ds, rtl8192eu, uwe5622 ... 无 AIC8800/VS6621
```
**结论：只能走有线网**。若需无线，建议用 USB 网卡（RTL8188EU 等 mainline 驱动现成）。

### 5. 串口操作要点

- **波特率 1500000**（不是 115200）
- **串口独占**：MobaXterm 占着时其他程序打不开
- 复杂命令建议 **base64 传输**（`echo <b64> | base64 -d | bash`），彻底避开引号/分号/管道的转义地狱

## 六、待办

- [ ] **API Token 加固**（穿透前必须；`config.py` 尚无 `PHT_API_TOKEN`）
- [ ] 按 IP 限速（防暴力试探）
- [ ] 内网穿透选型（倾向 Cloudflare Tunnel —— 免费版 HTTP 够用）
- [ ] ESP32 侧推送功能（POST `/api/v1/samples` + `X-PHT-Token` + 断网补传）
- [ ] SD 卡还给 ESP32（需重新烧录气象站固件用的归档卡）

## 七、内网穿透（方案已定，暂不安装）

**选定服务**：**Sakura Frp**（樱花frp）。

**决策时间**：2026-10-03

**执行顺序**：**先把局域网跑通，再折腾穿透** —— 不着急装。

**选中理由**：
- 国内节点，延迟与可达性优于境外服务
- 支持 TCP/HTTP，可拿固定域名
- 免费额度对本项目够用（每样本 JSON ≈ 50 B，30 秒一条 → 约 4.5 MB/月）

**架构（不变）**：
```
ESP32（任意网络）
   │  POST http://<穿透地址>/api/v1/samples
   ▼
Sakura Frp 服务端 ──隧道──► frpc 客户端（K3B 上）
   ▼
127.0.0.1:8080 → FastAPI ingest() 入库
```

**服务器代码零改动** —— 局域网与穿透走同一个 `/api/v1/samples`。

**穿透前必须完成**（见第六节待办）：
- [ ] **API Token 加固**（`config.py` 加 `PHT_API_TOKEN`，`app.py` 校验 `X-PHT-Token`）
- [ ] 按 IP 限速（防暴力试探）

> ⚠️ **SSH 绝不挂到穿透上** —— 运维通道走 Tailscale（不暴露端口）。
