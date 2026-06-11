"""Datasheet store and lightweight retrieval for per-part verification.

Same licensing reality as any datasheet tooling: vendor PDFs are downloadable
but not redistributable, so the store is a manifest (part → URL) plus a
fetcher; PDFs live in a local cache directory and never enter git.

Retrieval is deliberately simple: extract text per page (pypdf), split into
paragraph windows, score by keyword overlap with the query. At one-datasheet
scope a vector index buys nothing — the queries are literal (pin names,
"recommended operating", "decoupling"), and keeping the dependency surface
small matters more than recall on a 30-page document.
"""

from __future__ import annotations

import json
import re
from dataclasses import dataclass
from pathlib import Path

import httpx

DEFAULT_STORE = Path.home() / ".cache" / "tracewise" / "datasheets"


@dataclass
class DatasheetStore:
    root: Path = DEFAULT_STORE

    @property
    def manifest_path(self) -> Path:
        return self.root / "manifest.json"

    def load_manifest(self) -> dict[str, str]:
        if self.manifest_path.exists():
            return json.loads(self.manifest_path.read_text(encoding="utf-8"))
        return {}

    def save_manifest(self, manifest: dict[str, str]) -> None:
        self.root.mkdir(parents=True, exist_ok=True)
        self.manifest_path.write_text(
            json.dumps(manifest, indent=2, sort_keys=True) + "\n", encoding="utf-8"
        )

    def pdf_path(self, part: str) -> Path:
        return self.root / f"{part.upper()}.pdf"

    def add(self, part: str, url: str) -> None:
        manifest = self.load_manifest()
        manifest[part.upper()] = url
        self.save_manifest(manifest)

    def fetch(self, part: str) -> Path | None:
        """Download the datasheet for a part if mapped and not cached."""
        part = part.upper()
        target = self.pdf_path(part)
        if target.exists():
            return target
        url = self.load_manifest().get(part)
        if not url:
            return None
        self.root.mkdir(parents=True, exist_ok=True)
        with httpx.Client(timeout=60, follow_redirects=True,
                          headers={"User-Agent": "tracewise/0.0.1"}) as c:
            r = c.get(url)
            if r.status_code != 200 or not r.content.startswith(b"%PDF"):
                return None
            target.write_bytes(r.content)
        return target

    def available(self, part: str) -> Path | None:
        p = self.pdf_path(part.upper())
        return p if p.exists() else None


# --- text extraction + retrieval ---------------------------------------------


def extract_text(pdf_path: Path) -> str:
    from pypdf import PdfReader

    reader = PdfReader(str(pdf_path))
    return "\n\n".join((page.extract_text() or "") for page in reader.pages)


def windows(text: str, size: int = 600, overlap: int = 100) -> list[str]:
    """Paragraph-merged sliding windows of ~size chars."""
    paras = [p.strip() for p in re.split(r"\n\s*\n", text) if p.strip()]
    out: list[str] = []
    cur = ""
    for p in paras:
        if len(cur) + len(p) + 1 > size and cur:
            out.append(cur)
            cur = cur[-overlap:] if overlap else ""
        cur = (cur + "\n" + p).strip()
    if cur:
        out.append(cur)
    return out


_WORD = re.compile(r"[a-z0-9µ.+-]+")


def _terms(s: str) -> set[str]:
    return set(_WORD.findall(s.lower()))


def retrieve(text: str, query: str, k: int = 4) -> list[str]:
    """Top-k windows by keyword overlap with the query."""
    q = _terms(query)
    if not q:
        return []
    scored = []
    for w in windows(text):
        hits = len(q & _terms(w))
        if hits:
            scored.append((hits, w))
    scored.sort(key=lambda t: t[0], reverse=True)
    return [w for _, w in scored[:k]]
