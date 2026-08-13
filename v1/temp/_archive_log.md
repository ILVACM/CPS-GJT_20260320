# temp 归档日志

本文档记录 `temp/` 目录中所有归档文件的归档原因与时间。

## 归档清单

| 文件 | 归档时间 | 归档理由 |
|------|----------|----------|
| `Design-NodeA.md` | 2026-07-31 04:44 | Agent-Alpha 并行设计阶段的原始输出，与最终版 `Design-AI_detect.md` 内容完全一致（58,213 B），属多 Agent 协作中间产物，具里程碑意义 |
| `Design-node-b.md` | 2026-07-31 04:44 | Agent-Beta 并行设计阶段的原始输出，与最终版 `Design-server.md` 内容完全一致（64,516 B），属多 Agent 协作中间产物，具里程碑意义 |
| `camera_config.json` | 2026-07-31 | AGENTS.md §2.4 明确不再使用（336L 已改为 USB 直连节点 B），网络摄像头历史 URL 配置，仅留作历史参考 |
| `new-predict.py` | 2026-07-31 | V53 钢筋测量脚本（UNet 推理 + RebarMeasureV53 测量换算），AGENTS.md §2.4 所列现有代码资产，原型整理阶段归档，后续工程实现阶段从归档版本迁移至 node_a/ 与 node_b/ 各模块 |
| `test_camera.py` | 2026-07-31 | 相机环境与设备诊断脚本，AGENTS.md §2.4 所列现有代码资产，原型整理阶段归档，后续按 §7.4 思路为两节点各配备硬件自检脚本 |
| `test_gui.py` | 2026-07-31 | GUI 环境诊断脚本，AGENTS.md §2.4 所列现有代码资产，原型整理阶段归档，后续按 §7.4 思路为两节点各配备硬件自检脚本 |
| `demo.py` | 2026-07-31 | 旧版单节点入口（USB 摄像头 + 激光测距 tkinter 应用），AGENTS.md §2.4 已降级为"节点 B 原型组件"，代码已重构至 `node_b/` 各模块，根目录保留易误导部署，初代代码审计阶段归档 |
| `requirements_root_old.txt` | 2026-07-31 | 根目录旧版依赖清单（仅 3 项：opencv/pyserial/Pillow），与 `node_b/requirements.txt`（6 项）重复且不全，初代代码审计阶段归档。原路径：`requirements.txt`（根目录），归档时重命名以避免与节点 B 清单混淆 |
| `node_a_test.log` | 2026-07-31 | 节点 A 测试遗留日志（164 B），非生产运行日志，初代代码审计阶段归档。原路径：`node_a/logs/node_a_test.log` |
| `node_a_test2.log` | 2026-07-31 | 节点 A 测试遗留日志（240 B），非生产运行日志，初代代码审计阶段归档。原路径：`node_a/logs/node_a_test2.log` |
| `result_20260731-095032/` | 2026-07-31 | 节点 B 联调测试结果目录（含 ClassMask.png / report.csv / result.jpg），非交付物，初代代码审计阶段归档。原路径：`node_b/result/20260731-095032/` |
| `result_20260731-130425/` | 2026-07-31 | 同上，节点 B 联调测试结果目录。原路径：`node_b/result/20260731-130425/` |
| `result_20260731-131447/` | 2026-07-31 | 同上，节点 B 联调测试结果目录。原路径：`node_b/result/20260731-131447/` |
| `SYSTEM_OVERVIEW.md` | 2026-07-31 | 系统运作流程归纳（面向非技术人员的 450 字大白话版），初代代码审计阶段生成于根目录，由用户手动归档至 `temp/`。原路径：`SYSTEM_OVERVIEW.md`（根目录） |

## 备注

- 以上文件为多 Agent 协作设计阶段（Phase 2 并行设计）的原始输出，保留以供回溯设计决策过程。
- 如未来确认不再需要，请在项目团队确认后，方可从 `temp/` 中物理删除。
- 2026-07-31 初代代码审计阶段新增归档：`demo.py` `requirements_root_old.txt` 及测试日志/结果，详见上表各自条目。
- 2026-07-31 用户手动归档：`SYSTEM_OVERVIEW.md`（系统运作流程归纳）。
- 2026-07-31 节点 A 依赖清单迁移：`requirements_node_a.txt`（原根目录）→ `node_a/requirements.txt`，与 `node_b/requirements.txt` 对称。原文件未归档（非废弃，仅位置调整），已同步更新 `Design-AI_detect.md`、`node_a/README.md`、`node_a/QUICKSTART.md` 中所有引用。
- `node_b/logs/node_b.log`（运行时日志）未归档，直接删除：运行时产物，部署后自动重新生成，不属"废弃源文件"范畴（AGENTS.md §7.6 归档约束针对源代码与设计文档）。
- 2026-07-31 目录统一重命名：`node_a`→`node_detect`，`node_b`→`node_server`。归档目录 `temp/` 下的历史文件（`Design-NodeA.md` / `Design-node-b.md` 等）保持原样不修改，其内容中 `node_a`/`node_b`/`节点 A`/`节点 B` 为历史命名，仅作回溯参考。
