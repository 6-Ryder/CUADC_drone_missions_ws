#!/usr/bin/env python3
"""直接通过 MAVLink FTP 从飞控抓 ULog 日志,并校验完整性。

QGC 的 "Log Download" 在链路不好的时候会静默产出全 0 的文件(大小对、内容空),
本脚本用同一套 MAVLink FTP 协议但自己控制重传与超时,并在下载后强制校验
ULog magic + 非空,失败会明确报出来。

用法:
    # 1) 列 SD 卡日志目录(先看清有哪些架次)
    python3 tools/fetch_ulog.py list
    python3 tools/fetch_ulog.py list /fs/microsd/log/2026-09-27/

    # 2) 下载指定文件
    python3 tools/fetch_ulog.py get /fs/microsd/log/2026-09-27/08_08_12.ulg -o ~/Desktop/

    # 3) 校验本地文件(QGC 下完先跑这个)
    python3 tools/fetch_ulog.py check ~/Desktop/log_206_2026-9-27-16-08-12.ulg

注意: 串口是独占的,运行前必须在 QGC 里点 Disconnect(或关掉 QGC)。
"""

from __future__ import annotations

import argparse
import copy
import hashlib
import logging
import os
import sys
import time

from pymavlink import mavutil
from pymavlink.mavftp import FtpError, MAVFTP, MAVFTPSettings

ULOG_MAGIC = bytes([0x55, 0x4C, 0x6F, 0x67, 0x01, 0x12, 0x35])
LOG_ROOT = "/fs/microsd/log"


def patch_pymavlink_add_message():
    """修 pymavlink 2.4.49 的 add_message 崩溃。

    同一种消息如果先以"无 instance 值"的形式到达, 会走 simple 分支直接覆盖
    messages[mtype], 此时该对象的 _instances 仍是 None; 之后同一类型再带
    instance 值到来时, 原实现无条件执行 messages[mtype]._instances[...] = msg,
    于是 TypeError: 'NoneType' object does not support item assignment —— 表现为
    wait_heartbeat() 刚收到第一条消息就炸。这里补上 None 检查走重建分支。
    """
    if getattr(mavutil.add_message, "_ulog_patched", False):
        return

    def add_message(messages, mtype, msg):
        if msg._instance_field is None or getattr(msg, msg._instance_field, None) is None:
            messages[mtype] = msg
            return
        instance_value = getattr(msg, msg._instance_field)
        if mtype not in messages or getattr(messages[mtype], "_instances", None) is None:
            messages[mtype] = copy.copy(msg)
            messages[mtype]._instances = {instance_value: msg}
            messages["%s[%s]" % (mtype, str(instance_value))] = copy.copy(msg)
            return
        prev_instances = messages[mtype]._instances
        prev_instances[instance_value] = msg
        messages[mtype] = copy.copy(msg)
        messages[mtype]._instances = prev_instances
        messages["%s[%s]" % (mtype, str(instance_value))] = copy.copy(msg)

    add_message._ulog_patched = True  # type: ignore[attr-defined]
    mavutil.add_message = add_message


def connect(args):
    """打开 MAVLink 链路并等待心跳。"""
    if args.port.startswith(("udp", "tcp")) or ":" in args.port:
        mav = mavutil.mavlink_connection(args.port, source_system=255)
    else:
        if not os.path.exists(args.port):
            sys.exit(f"错误: 串口 {args.port} 不存在")
        mav = mavutil.mavlink_connection(args.port, baud=args.baud, source_system=255)

    print(f"等待心跳 {args.port} ...", flush=True)
    hb = mav.wait_heartbeat(timeout=args.heartbeat_timeout)
    if hb is None:
        sys.exit(
            "错误: 未收到心跳。常见原因: QGC 仍占用串口(请在 QGC 点 Disconnect), "
            "或飞控没上电/线没插好。"
        )
    print(
        f"已连接: system={mav.target_system} component={mav.target_component} "
        f"type={mavutil.mavlink.enums['MAV_TYPE'][hb.type].name if hb.type in mavutil.mavlink.enums['MAV_TYPE'] else hb.type}",
        flush=True,
    )
    return mav


def make_ftp(mav, args):
    """构造 MAVFTP 客户端。burst 调大提速, idle_detection 调小让完成判定更快。"""
    settings = MAVFTPSettings(
        [
            ("debug", int, args.debug),
            ("pkt_loss_tx", int, 0),
            ("pkt_loss_rx", int, 0),
            ("max_backlog", int, 5),
            ("burst_read_size", int, 239),
            ("write_size", int, 80),
            ("write_qsize", int, 5),
            ("idle_detection_time", float, 2.0),
            ("read_retry_time", float, 0.4),
            ("retry_time", float, 0.2),
        ]
    )
    return MAVFTP(mav, mav.target_system, mav.target_component, settings=settings)


def entry_size(e) -> int:
    """取目录项大小。pymavlink 的 DirectoryEntry 字段叫 size_b。"""
    for attr in ("size_b", "st_size", "size"):
        v = getattr(e, attr, None)
        if v is not None:
            return int(v)
    return 0


def human(n: float) -> str:
    for unit in ("B", "KB", "MB", "GB"):
        if n < 1024 or unit == "GB":
            return f"{n:.1f}{unit}" if unit != "B" else f"{n:.0f}B"
        n /= 1024.0
    return f"{n:.1f}GB"


def cmd_list(mav, ftp, args):
    """列出日志目录。不给参数时先列 /fs/microsd/log/ 再逐个进日期目录。"""
    targets = [args.path] if args.path else [LOG_ROOT]

    for target in targets:
        ftp.list_result = []
        ret = ftp.cmd_list([target])
        entries = list(ftp.list_result)
        print(f"\n=== {target}  ({len(entries)} 项) ===")

        dirs, files = [], []
        for e in entries:
            name = getattr(e, "name", str(e))
            if name in (".", ".."):
                continue
            is_dir = bool(getattr(e, "is_dir", getattr(e, "st_mode", 0) & 0o40000))
            (dirs if is_dir else files).append(e)

        for e in sorted(dirs, key=lambda x: x.name):
            print(f"  [dir ] {e.name}")
        for e in sorted(files, key=lambda x: x.name):
            size = entry_size(e)
            print(f"  [file] {e.name:28s} {human(size):>10s}  ({size} 字节)")

        if not entries:
            print(f"  (空或读取失败: {ret.error_code})")

    print(
        f"\n提示: `list {LOG_ROOT}/<日期>/` 看具体架次; "
        f"`get {LOG_ROOT}/<日期>/<文件名> -o <本地目录>` 下载。"
    )


def cmd_get(mav, ftp, args):
    """下载一个文件,带进度、超时和完整性校验。"""
    remote = args.path
    if not remote.startswith("/"):
        sys.exit("错误: 远端路径必须是绝对路径, 例如 /fs/microsd/log/2026-09-27/08_08_12.ulg")

    outdir = os.path.expanduser(args.out)
    os.makedirs(outdir, exist_ok=True)
    local = os.path.join(outdir, os.path.basename(remote))
    if os.path.exists(local) and not args.force:
        sys.exit(f"错误: {local} 已存在(加 --force 覆盖)")

    # 先确认远端大小, 好判断传输是否真的完成
    ftp.list_result = []
    ftp.cmd_list([os.path.dirname(remote)])
    remote_size = None
    for e in ftp.list_result:
        if getattr(e, "name", None) == os.path.basename(remote):
            remote_size = entry_size(e)
    if remote_size is None:
        print(f"警告: 没在目录列表里找到 {os.path.basename(remote)}, 仍然尝试下载", flush=True)
    else:
        print(f"远端大小: {human(remote_size)} ({remote_size} 字节)", flush=True)

    total = remote_size or 0
    state = {"last": 0.0, "t0": time.time()}

    def on_progress(frac):
        # pymavlink 在 TerminateSession 时会回调 callback_progress(None), 表示结束
        if frac is None:
            return
        now = time.time()
        if now - state["last"] < 0.5 and frac < 1.0:
            return
        state["last"] = now
        dt = max(now - state["t0"], 1e-6)
        done = int(frac * total)
        rate = done / dt if done else 0.0
        eta = (total - done) / rate if rate > 0 else 0.0
        print(
            f"\r  {frac * 100:5.1f}%  {human(done)}/{human(total) if total else '?'}  "
            f"{human(rate)}/s  ETA {eta:5.1f}s   ",
            end="",
            flush=True,
        )

    print(f"开始下载 {remote} -> {local}", flush=True)
    ret = ftp.cmd_get([remote, local], progress_callback=on_progress)
    if ret.error_code != FtpError.Success:
        sys.exit(f"\n错误: OpenFileRO 请求失败: {ret.operation_name} {ret.error_code}")

    ret2 = ftp.process_ftp_reply("OpenFileRO", timeout=args.timeout)
    print(flush=True)
    if ret2.error_code != FtpError.Success:
        sys.exit(f"错误: 传输失败: {ret2.operation_name} {ret2.error_code} (system_error={ret2.system_error})")

    ok = check_file(local, expect_size=remote_size, quiet=False)
    if not ok:
        sys.exit(1)
    print(f"\n完成: {local}")


def check_file(path: str, expect_size: int | None = None, quiet: bool = True) -> bool:
    """校验一个本地文件是不是真的 ULog(QGC 下载失败会留下全 0 文件)。"""
    if not os.path.exists(path):
        print(f"  ✗ 不存在: {path}")
        return False

    size = os.path.getsize(path)
    with open(path, "rb") as fh:
        head = fh.read(16)
        fh.seek(0)
        h = hashlib.sha256()
        nonzero = 0
        chunk_total = 0
        while True:
            chunk = fh.read(1 << 20)
            if not chunk:
                break
            h.update(chunk)
            chunk_total += len(chunk)
            nonzero += sum(1 for b in chunk if b)
            if nonzero > 0 and chunk_total > (1 << 22):
                # 已确认有内容, 剩下只为算哈希
                h.update(fh.read())
                break

    print(f"  文件: {path}")
    print(f"  大小: {size} 字节")
    print(f"  头部: {head.hex(' ')}")

    ok = True
    if head[:4] != ULOG_MAGIC[:4]:
        print(f"  ✗ 不是 ULog(应为 55 4c 6f 67 = 'ULog')")
        ok = False
    if nonzero == 0:
        print("  ✗ 全部为 0x00 —— 空的下载文件, 没有数据")
        ok = False
    if expect_size is not None and size != expect_size:
        print(f"  ✗ 大小不符(远端 {expect_size} 字节)")
        ok = False
    if ok:
        print(f"  ✓ ULog 头正确, 含 {nonzero}+ 个非零字节")
        print(f"  sha256: {h.hexdigest()}")
    return ok


def main():
    p = argparse.ArgumentParser(
        description="通过 MAVLink FTP 从飞控抓取并校验 ULog 日志",
        formatter_class=argparse.RawDescriptionHelpFormatter,
    )
    p.add_argument(
        "action",
        choices=["list", "get", "check"],
        help="list=列目录, get=下载, check=校验本地文件",
    )
    p.add_argument("path", nargs="?", help="list: 远端目录; get: 远端文件; check: 本地文件")
    p.add_argument("-o", "--out", default=".", help="get: 本地输出目录 (默认当前目录)")
    p.add_argument("--port", default="/dev/ttyACM0", help="串口或 udp:... (默认 /dev/ttyACM0)")
    p.add_argument("--baud", type=int, default=57600, help="波特率 (USB CDC 会忽略)")
    p.add_argument("--timeout", type=float, default=600.0, help="单次传输总超时秒数")
    p.add_argument("--heartbeat-timeout", type=float, default=10.0, help="等心跳秒数")
    p.add_argument("--force", action="store_true", help="覆盖已存在的本地文件")
    p.add_argument("--debug", type=int, default=0, help="MAVFTP 调试等级")
    args = p.parse_args()

    logging.basicConfig(level=logging.DEBUG if args.debug else logging.WARNING, format="%(message)s")
    patch_pymavlink_add_message()

    if args.action == "check":
        if not args.path:
            sys.exit("错误: check 需要本地文件路径")
        sys.exit(0 if check_file(os.path.expanduser(args.path)) else 1)

    mav = connect(args)
    try:
        ftp = make_ftp(mav, args)
        if args.action == "list":
            cmd_list(mav, ftp, args)
        else:
            if not args.path:
                sys.exit("错误: get 需要远端文件路径")
            cmd_get(mav, ftp, args)
    finally:
        mav.close()


if __name__ == "__main__":
    main()
