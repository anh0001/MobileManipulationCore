# Copyright 2026 MobileManipulationCore Contributors
# Licensed under the Apache License, Version 2.0.
"""Skill registry: the single source of truth for what skills exist.

Add a skill by decorating its class with ``@register_skill`` (and importing the
module in ``skills/__init__.py`` so the decorator runs). The skill_server and
the MCP bridge both read this registry — neither hardcodes any skill name.
"""
from __future__ import annotations

from typing import Dict, List, Type

from .base import Skill

_REGISTRY: Dict[str, Skill] = {}


def register_skill(cls: Type[Skill]) -> Type[Skill]:
    """Class decorator: instantiate and register a Skill by its ``name``."""
    if not getattr(cls, "name", ""):
        raise ValueError(f"{cls.__name__} must set a non-empty class attr 'name'")
    inst = cls()
    if inst.name in _REGISTRY:
        raise ValueError(f"duplicate skill name '{inst.name}'")
    _REGISTRY[inst.name] = inst
    return cls


def get_skill(name: str) -> Skill | None:
    return _REGISTRY.get(name)


def all_skills() -> List[Skill]:
    return list(_REGISTRY.values())
