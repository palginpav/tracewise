"""Minimal local-first LLM client (Ollama HTTP).

Carries the qwen-family lesson learned elsewhere: reasoning models sometimes
leak their thinking with only a closing ``</think>`` tag, so everything before
the last closing tag is dropped before any downstream parsing.
"""

from __future__ import annotations

import json
import re

import httpx

_THINK = re.compile(r"<think>.*?</think>\s*", re.DOTALL)
_JSON_ARRAY = re.compile(r"\[.*\]", re.DOTALL)


def strip_thinking(text: str) -> str:
    text = text.rsplit("</think>", 1)[-1] if "</think>" in text else text
    return _THINK.sub("", text).strip()


def extract_json_array(text: str):
    """Best-effort extraction of a JSON array from model output; None if absent."""
    m = _JSON_ARRAY.search(strip_thinking(text))
    if not m:
        return None
    try:
        return json.loads(m.group(0))
    except json.JSONDecodeError:
        return None


class OllamaClient:
    def __init__(
        self,
        model: str = "qwen3:4b",
        base_url: str = "http://localhost:11434",
        timeout: float = 300.0,
    ) -> None:
        self.model = model
        self._base_url = base_url.rstrip("/")
        self._timeout = timeout

    def available(self) -> bool:
        try:
            r = httpx.get(f"{self._base_url}/api/tags", timeout=5)
            return r.status_code == 200
        except httpx.HTTPError:
            return False

    def chat(self, messages: list[dict]) -> str:
        payload = {
            "model": self.model,
            "messages": messages,
            "stream": False,
            "think": False,
            "options": {"temperature": 0.1},
        }
        with httpx.Client(timeout=self._timeout) as client:
            resp = client.post(f"{self._base_url}/api/chat", json=payload)
            resp.raise_for_status()
            return resp.json()["message"]["content"]
