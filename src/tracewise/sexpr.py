"""Lossless s-expression editing for KiCad files.

Spike 0 showed why this module exists: the available KiCad file libraries
either crash on current formats or silently drop content on rewrite. TraceWise
patches *user* files, so the file layer's contract is absolute:

    parse(text) -> tree;  write(tree) == text   (byte-identical, always)

The design that guarantees it: a concrete syntax tree (CST), not a semantic
model. Every token stores its original text verbatim plus the *trivia*
(whitespace/newlines) that preceded it. Serialization is pure concatenation,
so an untouched tree reproduces the file exactly — and an edit changes only
the bytes of the edited region. There is nothing format-version-specific here;
KiCad format evolution cannot break round-tripping by construction.

Quoted strings are kept raw (no unescape/re-escape cycle). Helpers exist for
the common KiCad shape ``(name arg1 arg2 (child ...))``: find nodes by name,
read/set atom values, insert new children with inferred indentation.
"""

from __future__ import annotations

from dataclasses import dataclass, field


class SexprError(ValueError):
    """Malformed s-expression input."""


@dataclass
class Atom:
    """A bare token or quoted string, stored verbatim."""

    text: str  # exact source text, including quotes for strings
    trivia: str = ""  # whitespace that preceded this token in the source

    @property
    def value(self) -> str:
        """Token text with surrounding quotes removed (escapes left intact)."""
        if len(self.text) >= 2 and self.text[0] == '"' and self.text[-1] == '"':
            return self.text[1:-1]
        return self.text

    def write(self) -> str:
        return self.trivia + self.text


@dataclass
class Node:
    """A parenthesized list: ``(name ...children)``."""

    children: list[Atom | Node] = field(default_factory=list)
    trivia: str = ""  # whitespace before the opening paren
    close_trivia: str = ""  # whitespace before the closing paren

    @property
    def name(self) -> str | None:
        first = self.children[0] if self.children else None
        return first.value if isinstance(first, Atom) else None

    # --- navigation ---------------------------------------------------------

    def nodes(self, name: str | None = None) -> list[Node]:
        """Child nodes, optionally filtered by name."""
        out = [c for c in self.children if isinstance(c, Node)]
        return out if name is None else [n for n in out if n.name == name]

    def first(self, name: str) -> Node | None:
        found = self.nodes(name)
        return found[0] if found else None

    def atoms(self) -> list[Atom]:
        return [c for c in self.children if isinstance(c, Atom)]

    def arg(self, index: int = 1) -> str | None:
        """Positional atom value (index 0 is the node name)."""
        a = self.atoms()
        return a[index].value if index < len(a) else None

    def walk(self):
        """Depth-first iteration over all descendant nodes (self included)."""
        yield self
        for c in self.children:
            if isinstance(c, Node):
                yield from c.walk()

    def find_all(self, name: str) -> list[Node]:
        """All descendant nodes with the given name (depth-first)."""
        return [n for n in self.walk() if n is not self and n.name == name]

    # --- editing ------------------------------------------------------------

    def _child_indent(self) -> str:
        """Infer the indentation used by existing child nodes."""
        for c in self.children:
            if isinstance(c, Node) and "\n" in c.trivia:
                return c.trivia[c.trivia.rfind("\n") + 1 :]
        # fall back: one level deeper than our own indentation
        own = self.trivia[self.trivia.rfind("\n") + 1 :] if "\n" in self.trivia else ""
        return own + "\t"

    def insert(self, node: Node, index: int | None = None) -> Node:
        """Insert a child node with newline + inferred indentation trivia."""
        node.trivia = "\n" + self._child_indent()
        if not self.close_trivia.startswith("\n") and self.children:
            # keep the closing paren on its own line once we have multiline children
            own = self.trivia[self.trivia.rfind("\n") + 1 :] if "\n" in self.trivia else ""
            self.close_trivia = self.close_trivia or "\n" + own
        if index is None:
            self.children.append(node)
        else:
            self.children.insert(index, node)
        return node

    def remove(self, node: Node) -> None:
        self.children.remove(node)

    def set_arg(self, index: int, text: str) -> None:
        """Replace a positional atom's text (caller provides quoting)."""
        a = self.atoms()
        if index >= len(a):
            raise IndexError(f"node {self.name!r} has {len(a)} atoms, no index {index}")
        a[index].text = text

    def write(self) -> str:
        parts = [self.trivia, "("]
        parts += [c.write() for c in self.children]
        parts += [self.close_trivia, ")"]
        return "".join(parts)


def atom(value: str, quote: bool = False) -> Atom:
    """Build a new atom; ``quote=True`` wraps in double quotes."""
    return Atom(text=f'"{value}"' if quote else value, trivia=" ")


def node(name: str, *args: str | Atom | Node) -> Node:
    """Build a new node: ``node("pin", "1")`` → ``(pin 1)`` (single-line)."""
    n = Node()
    n.children.append(Atom(text=name, trivia=""))
    for a in args:
        if isinstance(a, (Atom, Node)):
            if not a.trivia:
                a.trivia = " "
            n.children.append(a)
        else:
            n.children.append(atom(str(a)))
    return n


# --- parser -----------------------------------------------------------------


def parse(text: str) -> Node:
    """Parse a complete KiCad file (one top-level form). Lossless."""
    pos = 0
    n = len(text)

    def read_trivia() -> str:
        nonlocal pos
        start = pos
        while pos < n and text[pos] in " \t\r\n":
            pos += 1
        return text[start:pos]

    def read_atom() -> str:
        nonlocal pos
        start = pos
        if text[pos] == '"':
            pos += 1
            while pos < n:
                if text[pos] == "\\":
                    pos += 2
                    continue
                if text[pos] == '"':
                    pos += 1
                    return text[start:pos]
                pos += 1
            raise SexprError(f"unterminated string starting at byte {start}")
        while pos < n and text[pos] not in ' \t\r\n()"':
            pos += 1
        if start == pos:
            raise SexprError(f"empty token at byte {pos}: {text[pos]!r}")
        return text[start:pos]

    def read_node(trivia: str) -> Node:
        nonlocal pos
        assert text[pos] == "("
        pos += 1
        nd = Node(trivia=trivia)
        while True:
            t = read_trivia()
            if pos >= n:
                raise SexprError("unexpected end of input inside node")
            ch = text[pos]
            if ch == ")":
                nd.close_trivia = t
                pos += 1
                return nd
            if ch == "(":
                nd.children.append(read_node(t))
            else:
                nd.children.append(Atom(text=read_atom(), trivia=t))

    lead = read_trivia()
    if pos >= n or text[pos] != "(":
        raise SexprError("expected '(' at top level")
    root = read_node(lead)
    tail = read_trivia()
    if pos != n:
        raise SexprError(f"trailing content at byte {pos}")
    root.close_trivia += ""  # no-op; keep shape explicit
    # stash the file's trailing whitespace on the root for exact reproduction
    root._tail = tail  # type: ignore[attr-defined]
    return root


def write(root: Node) -> str:
    """Serialize a tree parsed by :func:`parse` (byte-identical if unedited)."""
    return root.write() + getattr(root, "_tail", "")


def parse_file(path) -> Node:
    from pathlib import Path

    return parse(Path(path).read_text(encoding="utf-8"))


def write_file(root: Node, path) -> None:
    from pathlib import Path

    Path(path).write_text(write(root), encoding="utf-8")
