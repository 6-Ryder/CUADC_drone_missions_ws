#!/usr/bin/env bash
# Read-only 漂移检查: Optimization(开发机) 相对 jetson(真机) 的差异是否已知.
# 本脚本只读 git 对象, 不修改任何分支、工作树或索引.
#
# 比较对象是**工作区**, 不是 HEAD: 未提交的改动和新文件同样会被列出,
# 因此开发中途就能跑, 不必先 commit.
#
# 差异分两类, 语义完全不同:
#   A. 开发机专有 (WHITELIST_MACHINE): 路径/维护者元数据/模型/二进制.
#      永不合并到真机, 属于长期共存.
#   B. 待上游合并的 seam (WHITELIST_SEAM): 刻意的逻辑分叉, 目标是收缩到 0.
#      每一条都该有"合并到 jetson 后删除"的出口.
# 不在任何白名单内的 src/ 或 scripts/ 差异 = 意外漂移, 退出码 1.
#
# 用法:
#   scripts/check_jetson_sync.sh          # 检查 HEAD vs jetson
#   scripts/check_jetson_sync.sh --list   # 列出当前全部差异 (不判定)
#   BASE_REF=origin/jetson scripts/check_jetson_sync.sh
# 不能用 nounset (-u): 与 start_full_real_mission.sh 同因, setup.bash 无保护.
set -eo pipefail

WORKSPACE_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
BASE_REF="${BASE_REF:-jetson}"

# ---- A. 开发机专有内容 (不打算合并到真机) ----
WHITELIST_MACHINE=(
  # 维护者元数据: 真机是 nvidia, 开发机是 ryder
  "src/*/package.xml"
  "src/*/setup.py"
  # 开发机无 TensorRT: 模型用 .onnx + device cuda
  "src/drone_vision/config/vision_params.yaml"
  "src/drone_vision/drone_vision/models/*.engine"
  # launch: device 参数 + 去掉 jetson 上的重复块 (行为等价)
  "src/drone_bringup/launch/real_flight.launch.py"
  # 仅存在于开发机的工具
  "scripts/check_jetson_sync.sh"
  # 脚本内的开发机路径
  "scripts/build_sitl_px4_msgs.sh"
  "scripts/simple_mission.sh"
  "scripts/start_full_simulation.sh"
  "src/drone_servo2/scripts/*.sh"
)

# ---- B. 待上游合并的 seam 分叉 (目标: 收缩到 0) ----
# 背景: 096cdf5 为修 8/12-8/14 quick 失败, 把三处放宽直接改在共享底层,
# 未加任何模式门控, full 一并继承. 这里把它们恢复成显式能力开关
# (默认值 = 当前放宽行为, 故真机行为不变), 使 quick/full 可各自演进.
# 全部改动都是"恢复代码 + 包一层默认关闭的开关", 合到 jetson 后本列表应清空.
WHITELIST_SEAM=(
  # 直径过滤: 静态方法 + 调用点开关 drop_marker_filter_by_diameter_enable
  "src/drone_vision/drone_vision/marker_detector.py"
  "src/drone_vision/drone_vision/vision_pipeline.py"
  # 记忆世界坐标门控: 拒绝块 + 评分项开关 precision_align_world_gate_enable
  "src/drone_mission/drone_mission/precision_align_controller.py"
  "src/drone_mission/drone_mission/mission_state_machine.py"
  # 上述 seam 的契约测试 (真机上同样失败的两个既有过期断言已修)
  "src/drone_bringup/test/test_real_full_mission_profile.py"
  "src/drone_bringup/test/test_real_mission_scripts.py"
  "src/drone_mission/test/test_flight_safety_contract.py"
  "src/drone_mission/test/test_layout_mission_config.py"
  "src/drone_mission/test/test_precision_align_controller.py"
  "src/drone_vision/test/test_marker_refiner.py"
)

matches_any() {
  local path="$1" pattern
  shift
  for pattern in "$@"; do
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

# 对比对象是**工作区**而非 HEAD: 未提交的改动同样要能看见, 否则开发中途
# 这个检查形同虚设. git diff 不含未跟踪文件, 故再并入 ls-files --others
# (两者天然不相交: 一个列已跟踪改动, 一个列新文件).
changed=()
while IFS= read -r path; do
  [[ -n "$path" ]] && changed+=("$path")
done < <(
  git diff --name-only "$BASE_REF" -- src/ scripts/
  git ls-files --others --exclude-standard -- src/ scripts/
)

if [[ "${1:-}" == "--list" ]]; then
  printf '基线 %s -> HEAD 共 %d 个差异文件:\n' "$BASE_REF" "${#changed[@]}"
  if (( ${#changed[@]} )); then printf '  %s\n' "${changed[@]}"; fi
  exit 0
fi

unexpected=()
seam=()
for path in "${changed[@]}"; do
  if matches_any "$path" "${WHITELIST_MACHINE[@]}"; then
    continue
  elif matches_any "$path" "${WHITELIST_SEAM[@]}"; then
    seam+=("$path")
  else
    unexpected+=("$path")
  fi
done

if (( ${#seam[@]} )); then
  printf 'SEAM: %d 个文件是待合并到真机的刻意分叉 (非漂移):\n' "${#seam[@]}"
  printf '  %s\n' "${seam[@]}"
  printf '  → 这些改动应在下次真机窗口合并到 jetson, 然后从本脚本 WHITELIST_SEAM 删除.\n'
fi

if (( ${#unexpected[@]} )); then
  printf 'DRIFT: %d 个文件与 %s 不同且不在任何白名单内:\n' \
    "${#unexpected[@]}" "$BASE_REF" >&2
  printf '  %s\n' "${unexpected[@]}" >&2
  printf '\n若是意外改动 → 检查最近的提交与未提交的工作区改动; 若是有意的 →\n' >&2
  printf '先确认是否该同步到真机, 再决定加入 WHITELIST_MACHINE 还是 WHITELIST_SEAM.\n' >&2
  exit 1
fi

printf 'OK: %d 个差异文件全部已知 (开发机专有 %d, 待合并 seam %d)\n' \
  "${#changed[@]}" \
  "$(( ${#changed[@]} - ${#seam[@]} ))" \
  "${#seam[@]}"
