# Documentation Refresh Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** 将当前仓库文档重构为准确、可执行、可维护的 SITL 与真机任务说明，并删除已迁移的过时方案文档。

**Architecture:** 保留并重写 `docs/README.md` 和 `docs/ARCHITECTURE.md` 作为入口与实现基线；新增部署手册和按日期维护的工作记录。历史文档中的有效结论先进入工作记录，再删除旧提案，最后用仓库搜索和 Markdown 链接检查保证一致性。

**Tech Stack:** Markdown、ROS 2 Humble、PX4/PX4-DDS、Gazebo SITL、Bash、现有 pytest 测试。

## Global Constraints

- 必须保留 `docs/README.md` 和 `docs/ARCHITECTURE.md`，只能重写，不能删除。
- 真机默认使用 `/fmu/out/vehicle_status_v1`；SITL 通过 `scripts/sitl_env.sh` 使用 v4 overlay 和 `/fmu/out/vehicle_status_v4`。
- 精准降落必须描述视觉对心与盲降；不得把 PX4 LAND 写成主路径或降落替代方案。
- 当前场地按北向布局描述：Home World `(4,10)`，投放区 NED north `30..35`，侦察区 NED north `55..60`、east `-4..4`。
- 文档必须以当前源码、参数、启动脚本和 `log/simulation_20260730_192807` 为事实基线。
- 删除前须确保有效结论已迁移到 `docs/WORKLOG.md`。
- 只使用 `apply_patch` 修改或删除文件；不修改业务代码。

---

### Task 1: Rewrite the project entry point

**Files:**
- Modify: `docs/README.md`
- Reference: `scripts/start_full_simulation.sh`, `scripts/sitl_env.sh`, `src/drone_bringup/launch/sitl.launch.py`, `src/drone_bringup/launch/real_flight.launch.py`

**Interfaces:**
- Consumes: current launch/script names, mission state names, deployment topic versions.
- Produces: a concise entry point linking to `ARCHITECTURE.md`, `DEPLOYMENT.md`, `REAL_HARDWARE_DEBUG_GUIDE.md`, `WORKLOG.md`, `field/README.md`, and the real-flight parameter record.

- [ ] **Step 1: Replace stale overview content**

  Write sections for purpose, current verified status, quick navigation, mission flow, SITL/real differences, repository map, build/test commands, and documentation map. Remove missing links, old east-facing coordinates, old `.pt` runtime claims, broad process-kill instructions, and copied stale parameter tables.

- [ ] **Step 2: Check entry-point facts**

  Run:

  ```bash
  rg -n "DEBUG_2026|CHANGELOG_2026-07-10|\.pt|向东|PX4 LAND|vehicle_status_v1|vehicle_status_v4" docs/README.md
  ```

  Expected: only the intentional v1/v4 deployment explanation remains; no missing historical links or obsolete runtime claims remain.

---

### Task 2: Rewrite the current architecture reference

**Files:**
- Modify: `docs/ARCHITECTURE.md`
- Reference: `src/drone_mission/drone_mission/mission_state_machine.py`, `src/drone_mission/drone_mission/states/*.py`, `src/drone_vision/drone_vision/vision_pipeline.py`, `src/px4_control/px4_control/px4_bridge.py`, `src/drone_servo2/src/servo_controller.cpp`, `src/drone_interfaces/{msg,srv,action}/*`

**Interfaces:**
- Consumes: actual package responsibilities, ROS topics/services, FSM transitions, coordinate and parameter contracts.
- Produces: architecture truth source covering runtime topology, control/data flow, state machine, key algorithms, safety, cleanup, and SITL/real adapter boundaries.

- [ ] **Step 1: Define the document outline**

  Use these sections in dependency order: scope and invariants; package ownership; runtime topology; coordinate frames; FSM transition flow; vision modes and outputs; drop algorithm; recon coverage algorithm; precision landing algorithm; PX4/servo control; v1/v4 deployment seam; failure handling and resource lifecycle; verification evidence; known limitations.

- [ ] **Step 2: Record exact interfaces**

  Include the real topics `/drone/vision/cylinders`, `/drone/vision/markers`, `/drone/vision/landing_target`, `/drone/vision/coverage`, `/drone/vision/servo_cmd`, `/drone/vision/set_mode`, `/drone/servo/trigger_drop`, `/drone/pose`, `/drone/cmd_velocity_ned`, and PX4 in/out topics only where they are present in code.

- [ ] **Step 3: Record landing and cleanup contracts**

  State that visual H alignment is latched before blind descent, blind descent continues through Offboard vertical setpoints, high-altitude vision loss and blind timeout enter error, real touchdown uses `vehicle_land_detected`, SITL uses strict local touchdown fallback, and touchdown stops velocity then force-disarms. State that drop detection models are released on recon entry and the mission timer is cancelled on completion.

- [ ] **Step 4: Validate architecture claims**

  Run:

  ```bash
  rg -n "ReportHazards|vehicle_status\]|vehicle_status\||PX4 LAND|\.pt|向东|classify|TensorRT" docs/ARCHITECTURE.md
  ```

  Expected: no obsolete interface or algorithm claims; any mention of TensorRT or hazard code is explicitly marked non-mainline or historical only.

---

### Task 3: Add deployment and operations guidance

**Files:**
- Create: `docs/DEPLOYMENT.md`
- Modify: `docs/REAL_HARDWARE_DEBUG_GUIDE.md`
- Reference: `scripts/start_full_simulation.sh`, `scripts/simple_mission_real.sh`, `src/drone_bringup/launch/real_flight.launch.py`

**Interfaces:**
- Consumes: current build commands, SITL environment setup, launch arguments, log layout, real hardware transport options.
- Produces: repeatable SITL procedure and a safe real-flight procedure with preflight checks and troubleshooting.

- [ ] **Step 1: Document build and SITL execution**

  Include ROS 2 setup, `colcon build --symlink-install`, model/config prerequisites, `scripts/start_full_simulation.sh`, the generated `log/simulation_*` directory, and the v4 message overlay behavior.

- [ ] **Step 2: Document real deployment**

  Include `real_flight.launch.py`, V4L2 camera path, MicroXRCEAgent UDP/serial choices, real v1 topic behavior, RC arm requirement, and the distinction between the full mission launch and the legacy `simple_mission_real.sh` smoke-test path.

- [ ] **Step 3: Replace unsafe or stale field checks**

  Update `REAL_HARDWARE_DEBUG_GUIDE.md` to use bounded commands and explicit stop conditions. Keep the real MAIN5/PX4 servo checks and link to the parameter record.

- [ ] **Step 4: Validate commands and links**

  Check every command target with `test -e`/`rg --files`, and ensure all links in both files point to existing paths.

---

### Task 4: Consolidate history into a work log

**Files:**
- Create: `docs/WORKLOG.md`
- Reference: the nine files listed in the design spec for deletion and `log/simulation_20260730_192807`

**Interfaces:**
- Consumes: historical problem/root-cause/fix/verification facts.
- Produces: compact dated records that explain why the current architecture and parameters exist.

- [ ] **Step 1: Add milestone records**

  Record the v1.17 real-hardware interface alignment, north-facing layout migration, landing_H ONNX integration, recon coverage and landing fixes, resource release behavior, and the 2026-07-30 full SITL result.

- [ ] **Step 2: Add current known risks**

  Distinguish verified SITL behavior from still-required field validation: camera calibration, real PX4 touchdown behavior, MAIN5 actuator wiring, and real-flight safety checks.

- [ ] **Step 3: Review migration coverage**

  Search the old files for unique conclusions and ensure each relevant conclusion is represented in `WORKLOG.md` before deletion.

---

### Task 5: Remove superseded documents and refresh field notes

**Files:**
- Delete: the nine superseded files listed in `docs/superpowers/specs/2026-07-30-docs-refresh-design.md`
- Modify: `field/README.md`

**Interfaces:**
- Consumes: `WORKLOG.md` migration and the current `field/drone_field.sdf`.
- Produces: no stale duplicate operational documents and a field README matching the north-oriented scene.

- [ ] **Step 1: Verify exact deletion targets**

  Run `test -f` for each named file and inspect `git diff --name-status` before applying deletions.

- [ ] **Step 2: Delete only migrated targets**

  Use `apply_patch` delete blocks for the nine named files. Do not delete `README.md`, `ARCHITECTURE.md`, `REAL_HARDWARE_DEBUG_GUIDE.md`, `field/README.md`, or the real-flight parameter record.

- [ ] **Step 3: Align field README**

  Derive zone axes and coordinates from `field/drone_field.sdf` and current mission parameters; remove claims about an obsolete east-facing default.

---

### Task 6: Run documentation verification

**Files:**
- Check: all changed and new Markdown files

- [ ] **Step 1: Check Markdown references**

  Run a repository-local link/path audit for relative Markdown links and report any missing target.

- [ ] **Step 2: Check stale phrases**

  Run:

  ```bash
  rg -n "DEBUG_2026|CHANGELOG_2026-07-10|向东|东向|\.pt|PX4 LAND|待实施|规划中" docs field/README.md
  ```

  Expected: no stale operational claims; historical context may only appear in `WORKLOG.md` when clearly marked as historical.

- [ ] **Step 3: Run existing tests**

  Run the existing functional suite and `git diff --check`. Documentation changes must not alter source behavior; all previously passing tests should remain passing.

- [ ] **Step 4: Summarize the diff**

  Report retained files, new files, deleted files, validation results, and any remaining real-flight-only risks. Do not commit or push unless separately requested.
