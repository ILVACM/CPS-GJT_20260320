"""
检测节点 状态机（StateMachine）— 管理推理节点的生命周期状态。

状态定义（对齐 Design-AI_detect.md §3.4 状态机）：
  INIT          — 初始态（刚创建，未加载模型）
  IDLE          — 空闲（可接受推理请求）
  INFERENCING   — 推理中（GPU 正在处理）
  RETURNING     — 结果处理中（后处理 + 掩码编码）
  SHUTDOWN      — 终态（不再接受新任务）

合法状态转换：
  INIT      → IDLE, SHUTDOWN
  IDLE      → INFERENCING, SHUTDOWN
  INFERENCING → RETURNING, SHUTDOWN
  RETURNING → IDLE, SHUTDOWN
  SHUTDOWN  → (终态，不可再转换)

线程安全：所有状态转换操作通过 threading.Lock 串行化。
"""

import logging
import threading
from enum import Enum
from typing import Dict, List, Optional, Tuple

logger = logging.getLogger("node_detect.system.lifecycle")


class NodeState(Enum):
    """节点工作状态枚举（5 值，proto NodeStatus 仅映射其中 4 个业务态）"""

    INIT = 0            # 初始（gRPC 启动前）
    IDLE = 1            # 空闲（等待推理）
    INFERENCING = 2     # 推理中（GPU 计算）
    RETURNING = 3       # 结果处理（后处理 + PNG 编码）
    SHUTDOWN = 4        # 终态（不再接新任务）


# proto NodeStatus 枚举映射（对齐 node_detect/proto/rebar_inference.proto NodeStatus）
_STATE_TO_PROTO: Dict[NodeState, int] = {
    NodeState.INIT: 0,           # NODE_STATUS_UNKNOWN
    NodeState.IDLE: 1,           # NODE_STATUS_IDLE
    NodeState.INFERENCING: 2,    # NODE_STATUS_BUSY
    NodeState.RETURNING: 2,      # NODE_STATUS_BUSY（返回过程中也算忙）
    NodeState.SHUTDOWN: 3,       # NODE_STATUS_SHUTTING_DOWN
}

# 合法状态转换表
_VALID_TRANSITIONS: Dict[NodeState, List[NodeState]] = {
    NodeState.INIT: [NodeState.IDLE, NodeState.SHUTDOWN],
    NodeState.IDLE: [NodeState.INFERENCING, NodeState.SHUTDOWN],
    NodeState.INFERENCING: [NodeState.RETURNING, NodeState.SHUTDOWN],
    NodeState.RETURNING: [NodeState.IDLE, NodeState.SHUTDOWN],
    NodeState.SHUTDOWN: [],
}


class StateTransitionError(RuntimeError):
    """非法状态转换异常"""
    pass


class StateMachine:
    """
    线程安全的节点状态机。

    使用方式：
        sm = StateMachine()
        sm.transition(NodeState.IDLE)      # 合法转换
        sm.transition(NodeState.INFERENCING)

        if sm.can_transition(NodeState.IDLE):  # 预判是否允许（不实际转换）
            ...

        status = sm.current_proto_status()  # 返回 proto NodeStatus 整数 (0-3)
    """

    def __init__(self, initial: NodeState = NodeState.INIT) -> None:
        self._state: NodeState = initial
        self._lock = threading.Lock()
        self._transition_count: int = 0  # 转换计数（调试用）
        logger.info(f"[StateMachine] 初始状态: {self._state.name}")

    @property
    def current(self) -> NodeState:
        """获取当前状态（线程安全读）"""
        with self._lock:
            return self._state

    def transition(self, target: NodeState) -> None:
        """
        执行状态转换（阻塞式，线程安全）。

        Args:
            target: 目标状态

        Raises:
            StateTransitionError: 非法转换（如 IDLE → RETURNING）
        """
        with self._lock:
            self._transition_unlocked(target)

    def try_transition(self, target: NodeState) -> bool:
        """
        尝试状态转换（非阻塞）。

        若当前状态不允许转换到 target，返回 False 而不抛异常。
        适用于 gRPC Servicer 中尝试获取推理锁的场景。

        Returns:
            True — 转换成功；False — 转换被拒绝
        """
        with self._lock:
            if target not in _VALID_TRANSITIONS.get(self._state, []):
                return False
            self._transition_unlocked(target)
            return True

    def can_transition(self, target: NodeState) -> bool:
        """预判目标状态是否可达（不实际转换）"""
        with self._lock:
            return target in _VALID_TRANSITIONS.get(self._state, [])

    def current_proto_status(self) -> int:
        """
        返回当前状态对应的 proto NodeStatus 枚举值。

        Returns:
            int — 0(UNKNOWN) / 1(IDLE) / 2(BUSY) / 3(SHUTTING_DOWN)
        """
        with self._lock:
            return _STATE_TO_PROTO.get(self._state, 0)

    @property
    def is_shutdown(self) -> bool:
        """是否已到达终态"""
        with self._lock:
            return self._state == NodeState.SHUTDOWN

    @property
    def is_busy(self) -> bool:
        """是否处于忙碌态（INFERENCING / RETURNING）"""
        with self._lock:
            return self._state in (NodeState.INFERENCING, NodeState.RETURNING)

    @property
    def transition_count(self) -> int:
        """已完成的状态转换次数（调试/监控指标）"""
        with self._lock:
            return self._transition_count

    def _transition_unlocked(self, target: NodeState) -> None:
        """无锁版本（调用者必须持有 _lock）"""
        if target not in _VALID_TRANSITIONS.get(self._state, []):
            raise StateTransitionError(
                f"非法状态转换: {self._state.name} → {target.name}"
            )
        old_state = self._state
        self._state = target
        self._transition_count += 1
        logger.info(
            f"[StateMachine] {old_state.name} → {target.name} "
            f"(累计 {self._transition_count} 次转换)"
        )
