# Copyright 2026 MobileManipulationCore Contributors
# Licensed under the Apache License, Version 2.0.
"""Skill framework primitives: the param schema, the result, and the Skill ABC.

A *skill* is one high-level robot capability an AI client can invoke by name
(pick, place, home, ...). Every skill is a small class that declares:
  * a unique ``name`` (the registry key + MCP tool name),
  * a human ``description`` (shown to the LLM as the tool docstring),
  * a list of ``params`` (typed arguments, validated before execute runs),
  * an ``execute`` method containing the actual robot sequence.

Skills never own ROS plumbing. They receive a ``SkillContext`` (the running
skill_server node) and call its shared helpers — publish a detection prompt,
set a remote parameter, move the arm, read live state. This keeps each skill
focused on *what* to do, not *how* to talk to ROS, so a new skill is one file.
"""
from __future__ import annotations

from abc import ABC, abstractmethod
from dataclasses import dataclass, field
from typing import Any, Callable, Dict, List

# JSON-Schema-ish type tags a skill param can declare. Mapped to Python types
# for MCP signature generation and to validators for server-side checking.
_PY_TYPES: Dict[str, type] = {
    "string": str,
    "number": float,
    "integer": int,
    "boolean": bool,
    "array": list,        # JSON array; elements coerced to float (e.g. a joint pose)
}


@dataclass
class SkillParam:
    """One typed argument of a skill."""

    name: str
    type: str = "string"                 # one of _PY_TYPES keys
    description: str = ""
    required: bool = False
    default: Any = None

    def py_type(self) -> type:
        if self.type not in _PY_TYPES:
            raise ValueError(f"param '{self.name}': unknown type '{self.type}'")
        return _PY_TYPES[self.type]

    def coerce(self, value: Any) -> Any:
        """Validate + coerce an incoming value to this param's Python type."""
        t = self.py_type()
        if t is list:
            if not isinstance(value, (list, tuple)):
                raise ValueError(
                    f"param '{self.name}' expects array, got {value!r}")
            try:
                return [float(v) for v in value]
            except (TypeError, ValueError):
                raise ValueError(
                    f"param '{self.name}' expects array of numbers, got {value!r}")
        if isinstance(value, bool) and t is not bool:
            raise ValueError(f"param '{self.name}' expects {self.type}, got bool")
        try:
            # bool() of any non-empty value is True, so guard it above; for the
            # rest int/float/str coercion is what we want from JSON.
            return t(value)
        except (TypeError, ValueError):
            raise ValueError(
                f"param '{self.name}' expects {self.type}, got {value!r}")


@dataclass
class SkillResult:
    """What a skill hands back. ``data`` is JSON-serialised into result_json."""

    success: bool
    message: str
    data: Dict[str, Any] = field(default_factory=dict)


class SkillContext(ABC):
    """The capabilities the skill_server exposes to skills.

    Implemented by the skill_server node. Declared abstractly here so skills
    type-check against the interface, not the concrete node, and so the shared
    robot plumbing lives in exactly one place.
    """

    # --- live robot state (kept fresh by the server's subscriptions) ---
    @property
    @abstractmethod
    def vs_state(self) -> str:
        """Latest visual-servo state-machine label (e.g. 'GUARDED_APPROACH')."""

    @property
    @abstractmethod
    def gripper_width(self) -> float:
        """Latest gripper opening in metres."""

    # --- shared actuators / IO ---
    @abstractmethod
    def get_param(self, name: str, default: Any = None) -> Any:
        """Read a server ROS parameter (skills read tunables, never declare them)."""

    @abstractmethod
    def publish_prompt(self, text: str) -> None:
        """Publish a detection/target prompt to the visual-servo prompt topic."""

    @abstractmethod
    def set_bool_param(self, name: str, value: bool,
                       node: str | None = None, timeout: float = 4.0) -> bool:
        """Set a bool parameter on a remote node (default: the visual-servo node)."""

    @abstractmethod
    def move_arm_to(self, positions: List[float],
                    time_sec: float = 5.0, timeout: float = 12.0) -> bool:
        """Send a single-point joint trajectory and wait for it to finish."""

    @abstractmethod
    def set_gripper(self, position: float, max_effort: float = 5.0,
                    timeout: float = 10.0) -> bool:
        """Command the gripper to an opening in metres (0=closed, ~0.07=open).

        max_effort must be > 0 or the gripper will not move. Returns True once
        the GripperCommand action reports done (or the timeout elapses).
        """

    @abstractmethod
    def log(self, message: str) -> None:
        """Log through the server's ROS logger."""

    @abstractmethod
    def sleep(self, seconds: float) -> None:
        """Sleep without blocking the executor's other callbacks."""

    @abstractmethod
    def now(self) -> float:
        """Monotonic seconds, for measuring elapsed time in skill loops."""

    @abstractmethod
    def ok(self) -> bool:
        """False once rclpy is shutting down — long loops must check this."""


class Skill(ABC):
    """Base class for every robot skill. Subclass, set the class attributes,
    implement ``execute``, then register with ``@register_skill``."""

    name: str = ""
    description: str = ""
    params: List[SkillParam] = []

    def validate(self, raw: Dict[str, Any]) -> Dict[str, Any]:
        """Check required params are present and coerce all to declared types.

        Unknown keys are rejected so a typo'd argument fails loudly rather than
        being silently ignored by the skill.
        """
        known = {p.name: p for p in self.params}
        unknown = set(raw) - set(known)
        if unknown:
            raise ValueError(f"unknown param(s): {', '.join(sorted(unknown))}")
        out: Dict[str, Any] = {}
        for p in self.params:
            if p.name in raw and raw[p.name] is not None:
                out[p.name] = p.coerce(raw[p.name])
            elif p.required:
                raise ValueError(f"missing required param '{p.name}'")
            else:
                out[p.name] = p.default
        return out

    @abstractmethod
    def execute(self, ctx: SkillContext, params: Dict[str, Any],
                feedback: Callable[[str, float], None],
                is_cancelled: Callable[[], bool]) -> SkillResult:
        """Run the skill.

        Args:
            ctx: shared robot plumbing (see SkillContext).
            params: validated + coerced arguments (every declared name present).
            feedback: call ``feedback(state, progress)`` to stream progress.
            is_cancelled: returns True if the client requested cancel — long
                skills should poll it and return early.
        """
