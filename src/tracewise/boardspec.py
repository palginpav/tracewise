"""Board specification: the `tracewise.yaml` the user writes next to a project.

Minimal on purpose — the fields that actually drive constraints. Everything
has a sane 2-layer/1oz default so an empty file is valid.

    layers: 4
    copper_oz: 1
    min_track_mm: 0.15
    min_clearance_mm: 0.15
    via: { diameter_mm: 0.6, drill_mm: 0.3 }
    power_track_mm: 0.5
    notes: "USB FS on J2; keep antenna keepout at top-left"
"""

from __future__ import annotations

from pathlib import Path

from pydantic import BaseModel, Field


class ViaSpec(BaseModel):
    diameter_mm: float = Field(default=0.6, gt=0)
    drill_mm: float = Field(default=0.3, gt=0)


class BoardSpec(BaseModel):
    layers: int = Field(default=2, ge=1, le=32)
    copper_oz: float = Field(default=1.0, gt=0)
    min_track_mm: float = Field(default=0.15, gt=0)
    min_clearance_mm: float = Field(default=0.15, gt=0)
    via: ViaSpec = Field(default_factory=ViaSpec)
    power_track_mm: float = Field(default=0.5, gt=0)
    notes: str = ""

    @classmethod
    def load(cls, path: str | Path) -> BoardSpec:
        import yaml

        text = Path(path).read_text(encoding="utf-8")
        data = yaml.safe_load(text) or {}
        return cls.model_validate(data)

    @classmethod
    def for_project(cls, project_dir: str | Path) -> BoardSpec:
        p = Path(project_dir) / "tracewise.yaml"
        return cls.load(p) if p.exists() else cls()
