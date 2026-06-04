# Copyright 2026 MobileManipulationCore Contributors
# Licensed under the Apache License, Version 2.0.
"""Robot skill framework.

Public API:
    Skill, SkillParam, SkillResult, SkillContext  -- to author a skill
    register_skill, get_skill, all_skills          -- the registry

To add a skill:
    1. create skills/<name>_skill.py with a Skill subclass + @register_skill
    2. import it below so the decorator runs on package import
That's it — the skill_server dispatches it and the MCP bridge exposes it.
"""
from .base import Skill, SkillParam, SkillResult, SkillContext
from .registry import register_skill, get_skill, all_skills

# Import every skill module so registration runs on `import ...skills`.
from . import pick_skill  # noqa: F401
from . import place_skill  # noqa: F401
from . import home_skill  # noqa: F401

__all__ = [
    "Skill", "SkillParam", "SkillResult", "SkillContext",
    "register_skill", "get_skill", "all_skills",
]
