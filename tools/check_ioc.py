#!/usr/bin/env python3
"""Qust_version.ioc 结构校验

CubeMX 的 .ioc 有几条"少一项就静默失效"的规则，漏掉不会报错，
但在 CubeMX 里那部分配置会直接消失（HSE 被关、外设整个不见）。
这个脚本把踩过的坑固化成检查项，改完 .ioc 后跑一下：

    python tools/check_ioc.py

返回码 0 = 通过，1 = 有问题。
"""
import re
import sys
from collections import Counter

IOC = "Qust_version.ioc"

# 这些 IP 由 CubeMX 隐式处理，不需要 MX_*_Init 条目
NO_INIT_NEEDED = {"NVIC", "RCC", "SYS", "FREERTOS", "DMA", "GPIO"}
# 这些引脚必须出现在 Mcu.PinN 里，否则对应功能会被 CubeMX 丢弃
REQUIRED_PINS = {
    "PH0-OSC_IN": "HSE 12MHz 晶振（漏了会让 CubeMX 认为 HSE 未启用）",
    "PH1-OSC_OUT": "HSE 12MHz 晶振",
}

errors = []
warnings = []


def load(path):
    cfg = {}
    for line in open(path, encoding="utf-8"):
        line = line.rstrip("\n")
        if not line or line.startswith("#"):
            continue
        if "=" not in line:
            errors.append("存在不含 '=' 的行: %r" % line)
            continue
        k, v = line.split("=", 1)
        if k in cfg:
            errors.append("重复键: %s" % k)
        cfg[k] = v
    return cfg


def main():
    try:
        cfg = load(IOC)
    except FileNotFoundError:
        print("找不到 %s（请在工程根目录运行）" % IOC)
        return 1

    # --- 1. 引脚表 ---
    pm = {int(m.group(1)): v for k, v in cfg.items()
          for m in [re.fullmatch(r"Mcu\.Pin(\d+)", k)] if m}
    n = int(cfg["Mcu.PinsNb"])
    if sorted(pm) != list(range(n)):
        errors.append("Mcu.PinN 索引不连续：期望 0..%d，实际 %s" % (n - 1, sorted(pm)))
    if len(set(pm.values())) != len(pm):
        dup = [v for v, c in Counter(pm.values()).items() if c > 1]
        errors.append("Mcu.PinN 中有重复引脚: %s" % dup)
    for pin, why in REQUIRED_PINS.items():
        if pin not in pm.values():
            errors.append("引脚 %s 不在 Mcu.PinN 中 —— %s" % (pin, why))

    # --- 2. 每个引脚要有 Signal ---
    for pin in pm.values():
        if pin.startswith("VP_"):
            continue
        if pin + ".Signal" not in cfg:
            errors.append("引脚 %s 没有 .Signal 声明" % pin)

    # --- 3. 外设信号不能冲突 ---
    sig = {m.group(1): v for k, v in cfg.items()
           for m in [re.fullmatch(r"(P[A-I]\d{1,2}(?:-[A-Z0-9_]+)?)\.Signal", k)] if m}
    for s, c in Counter(sig.values()).items():
        if c > 1 and not s.startswith(("GPIO", "GPXTI")):
            who = [p for p, v in sig.items() if v == s]
            errors.append("外设信号 %s 被多个引脚占用: %s" % (s, who))

    # --- 4. IP 列表 ---
    ips = sorted(((int(m.group(1)), v) for k, v in cfg.items()
                  for m in [re.fullmatch(r"Mcu\.IP(\d+)", k)] if m))
    if [i for i, _ in ips] != list(range(len(ips))):
        errors.append("Mcu.IPN 索引不连续: %s" % [i for i, _ in ips])
    if len(ips) != int(cfg["Mcu.IPNb"]):
        errors.append("Mcu.IPNb=%s 与实际条目数 %d 不符" % (cfg["Mcu.IPNb"], len(ips)))
    ipnames = [v for _, v in ips]
    if len(set(ipnames)) != len(ipnames):
        errors.append("Mcu.IP 中有重复外设: %s" % ipnames)

    # --- 5. functionlistsort 必须覆盖所有外设 ---
    fl = cfg.get("ProjectManager.functionlistsort", "")
    entries = [e for e in fl.split(",") if e]
    owning = {e.split("-")[2] for e in entries if len(e.split("-")) >= 3}
    for name in ipnames:
        if name not in NO_INIT_NEEDED and name not in owning:
            errors.append(
                "外设 %s 在 Mcu.IP 里但不在 ProjectManager.functionlistsort 中"
                " —— CubeMX 会认为它未配置（这就是 ADC3 消失的原因）" % name)
    nums = [int(e.split("-")[0]) for e in entries]
    if nums != list(range(1, len(nums) + 1)):
        warnings.append("functionlistsort 序号不连续: %s" % nums)

    # --- 6. HSE / PLL 一致性 ---
    if cfg.get("RCC.PLLSourceVirtual") == "RCC_PLLSOURCE_HSE":
        try:
            hse = int(cfg["RCC.HSE_VALUE"])
            pllm = int(cfg["RCC.PLLM"])
            vco_in = hse / pllm
            if not (1e6 <= vco_in <= 2e6):
                errors.append(
                    "PLL 输入 = HSE/PLLM = %.3f MHz，超出 STM32F4 要求的 1~2 MHz"
                    % (vco_in / 1e6))
        except (KeyError, ValueError) as e:
            errors.append("HSE/PLL 参数缺失或非法: %s" % e)

    # --- 7. FreeRTOS 任务与 HAL 时基 ---
    tb = cfg.get("NVIC.TimeBaseIP")
    if tb != "TIM1":
        warnings.append("NVIC.TimeBaseIP=%s；FreeRTOS 占用 SysTick 时通常用 TIM1" % tb)

    for w in warnings:
        print("[警告] " + w)
    for e in errors:
        print("[错误] " + e)
    print("\n%s：%d 个引脚 / %d 个外设 / %s" % (
        "通过" if not errors else "失败", n, len(ips),
        "%d 个 FreeRTOS 任务" % len(cfg.get("FREERTOS.Tasks01", "").split(";"))))
    return 1 if errors else 0


if __name__ == "__main__":
    sys.exit(main())
