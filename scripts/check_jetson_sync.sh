#!/usr/bin/env bash
# Read-only 漂移检查: Optimization(开发机) 相对 jetson(真机) 的差异是否仍在白名单内.
# 本脚本只读 git 对象, 不修改任何分支、工作树或索引.
#
# 白名单只允许两类差异:
#   1. 开发机专有: 路径/软链接/维护者元数据/TensorRT 二进制/视觉模型配置
#   2. 已确认的修复: 5 个过期测试断言 + launch 文件去重
# 任何任务逻辑文件 (mission_state_machine / states/ / vision_pipeline 等) 出现
# 未列入白名单的差异 → 退出码 1, 说明开发机与真机的代码逻辑已漂移.
#
# 用法:
#   scripts/check_jetson_sync.sh          # 检查 HEAD vs jetson
#   scripts/check_jetson_sync.sh --list   # 列出当前全部差异 (不判定)
#   BASE_REF=origin/jetson scripts/check_jetson_sync.sh
# 不能用 nounset (-u): 与 start_full_real_mission.sh 同因, setup.bash 无保护.
set -eo pipefail

WORKSPACE_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
BASE_REF="${BASE_REF:-jetson}"

# ---- 白名单: 允许与真机不同的路径 (glob) ----
WHITELIST=(
  # 维护者元数据: 真机是 nvidia, 开发机是 ryder
  "src/*/package.xml"
  "src/*/setup.py"
  # 开发机无 TensorRT: 模型用 .onnx + device cuda
  "src/drone_vision/config/vision_params.yaml"
  "src/drone_vision/drone_vision/models/*.engine"
  # launch: device 参数 + 去掉 jetson 上的重复块 (行为等价, 见下方注释)
  "src/drone_bringup/launch/real_flight.launch.py"
  # 过期测试断言修复 (这些断言在 jetson 上同样失败)
  "src/drone_bringup/test/test_real_full_mission_profile.py"
  "src/drone_bringup/test/test_real_mission_scripts.py"
  "src/drone_mission/test/test_flight_safety_contract.py"
  "src/drone_mission/test/test_layout_mission_config.py"
  # 仅存在于开发机的工具
  "scripts/check_jetson_sync.sh"
  # 脚本内的开发机路径
  "scripts/build_sitl_px4_msgs.sh"
  "scripts/simple_mission.sh"
  "scripts/start_full_simulation.sh"
  "src/drone_servo2/scripts/*.sh"
)

is_whitelisted() {
  local path="$1" pattern
  for pattern in "${WHITELIST[@]}"; do
    # shellcheck disable=SC2254  # glob 需在 case 中展开
    case "$path" in $pattern) return 0 ;; esac
  done
  return 1
}

cd "$WORKSPACE_DIR"

if ! git rev-parse --verify --quiet "$BASE_REF^{commit}" >/dev/null; then
  printf 'ERROR: 找不到基线 ref: %s\n' "$BASE_REF" >&2
  exit 2
fi

mapfile -t changed < <(git diff --name-only "$BASE_REF" HEAD -- src/ scripts/)

if [[ "${1:-}" == "--list" ]]; then
  printf '基线 %s -> HEAD 共 %d 个差异文件:\n' "$BASE_REF" "${#changed[@]}"
  if (( ${#changed[@]} )); then printf '  %s\n' "${changed[@]}"; fi
  exit 0
fi

unexpected=()
for path in "${changed[@]}"; do
  is_whitelisted "$path" || unexpected+=("$path")
done

if (( ${#unexpected[@]} )); then
  printf 'DRIFT: %d 个文件与真机 %s 不同且不在白名单内:\n' \
    "${#unexpected[@]}" "$BASE_REF" >&2
  printf '  %s\n' "${unexpected[@]}" >&2
  printf '\n若是有意的逻辑改动, 请同步到真机; 若有意保留在开发机, 请加入本脚本 WHITELIST.\n' >&2
  exit 1
fi

printf 'OK: %d 个差异文件全部在白名单内 (基线 %s, 代码逻辑已同步)\n' \
  "${#changed[@]}" "$BASE_REF"
