"""Decode PX4 ULog `event` messages using the log's own embedded metadata.

用法:
    python3 tools/ulog_events.py <log.ulg> [more.ulg ...]

事件表(id → 消息模板 + 参数名)与枚举定义**直接从 ULog 自带的 metadata_events 提取**,
不需要外部文件,也不绑定 PX4 源码版本 —— 每个 id 的含义由写这份日志的固件自己给出。

提取路径(两个 pyulog 坑):
  1. 分块在 `u.msg_info_multiple_dict['metadata_events']` 里,不是 `msg_info_dict`。
     内层是一串 bytes 分块(长日志可多组),**必须按序全部拼接**再解压。
  2. 拼接后是 lzma 压缩的 JSON,不是明文。少数日志未压缩。

⚠️ 已知限制 —— 为什么多数日志解不出事件:
  本仓库 73 份真机日志(2026-07 ~ 2026-10, PX4 v1.17)实测,日志里 `event.id` 全部落在
  0x01000000–0x018AFFFF 的高位空间,与 metadata 表里的事件 ID **零交集**(1654 个事件,
  命中 0),metadata 中也没有任何 hash 字段。也就是说飞控写进日志的是一个哈希命名空间,
  而随日志携带的 metadata 是另一个。没有反查表就无法还原。
  本工具因此做两件事:能对上时正常解码;**对不上时明确报告数量与 id 范围** ——
  而不是像早期版本那样对每条事件打印一行 `<unknown>`,看着像在工作其实什么都没解出。

  排查数据请改用同目录的 `ulog_timeline.py`(状态变化时间轴)与
  `ulog_offboard.py`(Offboard 模式位生命周期),它们不依赖事件表。
"""
import lzma
import json
import sys

from pyulog import ULog


def metadata_json(ulog: ULog):
    """从 ULog 内嵌 metadata 取出事件定义 JSON;缺失时返回 None。"""
    groups = ulog.msg_info_multiple_dict.get("metadata_events")
    if not groups:
        return None
    raw = b"".join(chunk for group in groups for chunk in group)
    if not raw:
        return None
    # 少数日志的 metadata 未压缩,按明文 JSON 兜底。
    try:
        return json.loads(lzma.decompress(raw))
    except lzma.LZMAError:
        return json.loads(raw)


def build_tables(meta: dict):
    """→ (lut, enums)

    lut:   {event_id: (message_template, [arg_name, ...])}
    enums: {enum_type: {value: name}}
    """
    lut, enums = {}, {}
    for component in meta.get("components", {}).values():
        for group in component.get("event_groups", {}).values():
            for event_id, event in group.get("events", {}).items():
                args = [a["name"] for a in event.get("arguments", [])]
                lut[int(event_id)] = (event.get("message", ""), args)
        for enum_name, values in component.get("enums", {}).items():
            # 枚举字典里混有 "description" 这类非取值键,只取数字键。
            enums[enum_name] = {int(v): d.get("name", str(v))
                                for v, d in values.items()
                                if isinstance(d, dict) and str(v).isdigit()}
    return lut, enums


def arg_types(meta: dict):
    """→ {event_id: [type_name, ...]},用于把枚举参数翻译成可读名字。"""
    types = {}
    for component in meta.get("components", {}).values():
        for group in component.get("event_groups", {}).values():
            for event_id, event in group.get("events", {}).items():
                types[int(event_id)] = [a.get("type", "") for a in event.get("arguments", [])]
    return types


def render(value, type_name, enums):
    """把参数渲染成字符串;枚举类型映射为名字。"""
    if type_name in enums:
        try:
            return enums[type_name].get(int(value), str(value))
        except (TypeError, ValueError):
            return str(value)
    if isinstance(value, float):
        return f"{value:g}"
    return str(value)


def dump(ulog: ULog, path: str):
    meta = metadata_json(ulog)
    header = path.rsplit("/", 1)[-1]
    print("=" * 74)
    print(header)

    if meta is None:
        print("  该日志不含 metadata_events,无法解码事件表(常见于较早的 PX4 版本)")
        return

    lut, enums = build_tables(meta)
    types = arg_types(meta)

    data = {d.name: d for d in ulog.data_list}
    if "event" not in data:
        print("  该日志没有 event 消息")
        return

    events = data["event"].data
    t0 = ulog.start_timestamp
    total = len(events["timestamp"])
    seen_unknown = set()
    decoded = 0

    for i in range(total):
        event_id = int(events["id"][i])
        template, names = lut.get(event_id, (None, None))
        if template is None:
            seen_unknown.add(event_id)
            continue
        args = []
        for j, name in enumerate(names):
            key = f"arguments[{j}]"
            if key not in events:
                continue
            type_name = types.get(event_id, [""] * (j + 1))[j] if j < len(types.get(event_id, [])) else ""
            args.append(f"{name}={render(events[key][i], type_name, enums)}")
        rel = (events["timestamp"][i] - t0) / 1e6
        print("  %7.2fs [%s] %s  %s" % (rel, events["log_levels"][i],
                                        template, " ".join(args)))
        decoded += 1

    print("  事件表 %d 条;本日志 %d 个事件,解出 %d 个。" % (len(lut), total, decoded))
    if seen_unknown:
        lo, hi = min(seen_unknown), max(seen_unknown)
        print("  ⚠ %d 个事件 id 不在事件表内(范围 0x%06X–0x%06X),无法解码。"
              % (len(seen_unknown), lo, hi))
        print("    这是事件 id 的哈希命名空间与 metadata 表不相交所致,见模块 docstring 的说明。")


def main(argv):
    if len(argv) < 2:
        print((__doc__ or "").strip())
        return 1
    for path in argv[1:]:
        dump(ULog(path), path)
    return 0


if __name__ == "__main__":
    sys.exit(main(sys.argv))
