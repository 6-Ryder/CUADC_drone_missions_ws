"""
统一配置加载 — 从 YAML 文件加载 ROS2 节点参数.

每个包的 config/*.yaml → Node.declare_parameter + get_parameter.
支持命令行覆盖 YAML 默认值.
"""

import os
from typing import Any, Dict, Optional

import yaml


def load_yaml_file(yaml_path: str) -> Dict[str, Any]:
    """
    加载单个 YAML 配置文件, 返回展平后的参数字典.

    YAML 格式:
      /node_name:
        ros__parameters:
          key: value

    Returns
    -------
    dict: {key: value, ...}  或 {} 如果文件不存在/解析失败
    """
    if not yaml_path or not os.path.isfile(yaml_path):
        return {}

    try:
        with open(yaml_path, "r") as f:
            raw = yaml.safe_load(f)
    except (yaml.YAMLError, OSError) as e:
        print(f"[config_loader] YAML 解析失败 {yaml_path}: {e}")
        return {}

    if not isinstance(raw, dict):
        return {}

    # 展平: 取第一个命名空间下的 ros__parameters
    for node_ns, node_config in raw.items():
        if isinstance(node_config, dict) and "ros__parameters" in node_config:
            return dict(node_config["ros__parameters"])

    # 如果已经是展平格式
    return dict(raw)


def load_config(node, yaml_path: str) -> Dict[str, Any]:
    """
    从 YAML 加载参数并注册到 ROS2 Node.

    对于每个 YAML 参数:
      1. 调用 node.declare_parameter(key, value)
      2. 用 node.get_parameter(key).value 读取 (允许多层覆盖)

    Parameters
    ----------
    node : rclpy.node.Node
        ROS2 节点实例
    yaml_path : str
        YAML 配置文件路径

    Returns
    -------
    dict: 最终生效的 {key: value} 映射
    """
    params = load_yaml_file(yaml_path)
    result = {}

    for key, default_value in params.items():
        try:
            node.declare_parameter(key, default_value)
            result[key] = node.get_parameter(key).value
        except Exception as e:
            node.get_logger().warn(f"参数注册失败 {key}: {e}")
            result[key] = default_value

    return result


def get_param(node, key: str, default: Any = None) -> Any:
    """
    安全获取 ROS2 参数值.

    优先从 node 已声明的参数读取, 否则返回 default.
    """
    try:
        return node.get_parameter(key).value
    except Exception:
        return default
