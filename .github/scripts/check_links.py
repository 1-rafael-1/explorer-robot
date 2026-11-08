#!/usr/bin/env python3
"""Check relative links and images in the repository's Markdown.

Every tracked Markdown file is scanned, plus untracked files that git does not
ignore. ``docs/adr/`` (historical records) and anything under ``.scratch/`` (the
local tracker) are excluded.

External URLs, pure-fragment links and web-root (``/...``) links are not
resolved; web-root links are reported because the docs are read from a clone,
not from a web server. Every other target is resolved against the containing
file's directory and must name an existing file or directory. A Markdown target
with a fragment must also match a GitHub-style heading slug.

Exit status is non-zero if any relative link is broken.
"""

from __future__ import annotations

import os
import re
import subprocess
import sys
import urllib.parse
from pathlib import Path

SCHEME_RE = re.compile(r"^[A-Za-z][A-Za-z0-9+.-]*:")
FENCE_RE = re.compile(r"^\s{0,3}(`{3,}|~{3,})")
HEADING_RE = re.compile(r"^ {0,3}(#{1,6})\s+(.*?)\s*#*\s*$")
REF_DEF_RE = re.compile(r"^ {0,3}\[([^\]]+)\]:\s*(.*)$")

# Inline Markdown constructs used when reducing a heading to rendered text.
IMAGE_RE = re.compile(r"!\[([^\]]*)\]\([^)]*\)")
LINK_RE = re.compile(r"\[([^\]]*)\]\([^)]*\)")
REF_LINK_RE = re.compile(r"\[([^\]]*)\]\[[^\]]*\]")
HTML_TAG_RE = re.compile(r"<[^>]+>")


def repo_root() -> Path:
    result = subprocess.run(
        ["git", "rev-parse", "--show-toplevel"],
        check=True,
        capture_output=True,
        text=True,
    )
    return Path(result.stdout.strip())


def markdown_files(root: Path) -> list[str]:
    result = subprocess.run(
        ["git", "ls-files", "--cached", "--others", "--exclude-standard", "-z", "*.md"],
        cwd=root,
        check=True,
        capture_output=True,
    )
    paths = [os.fsdecode(path) for path in result.stdout.split(b"\0") if path]
    return sorted(
        path
        for path in paths
        if not path.startswith("docs/adr/")
        and not path.startswith(".scratch/")
        and path != ".scratch"
    )


def strip_code_spans(line: str) -> str:
    """Replace inline code spans with a space so links inside them are ignored."""
    out: list[str] = []
    index = 0
    length = len(line)
    while index < length:
        if line[index] != "`":
            out.append(line[index])
            index += 1
            continue
        tick_end = index
        while tick_end < length and line[tick_end] == "`":
            tick_end += 1
        ticks = line[index:tick_end]
        close = line.find(ticks, tick_end)
        if close == -1:
            out.append(ticks)
            index = tick_end
            continue
        out.append(" ")
        index = close + len(ticks)
    return "".join(out)


def replace_code_spans(line: str) -> str:
    """Replace inline code spans with their content, as GitHub does for headings."""
    out: list[str] = []
    index = 0
    length = len(line)
    while index < length:
        if line[index] != "`":
            out.append(line[index])
            index += 1
            continue
        tick_end = index
        while tick_end < length and line[tick_end] == "`":
            tick_end += 1
        ticks = line[index:tick_end]
        close = line.find(ticks, tick_end)
        if close == -1:
            out.append(ticks)
            index = tick_end
            continue
        out.append(line[tick_end:close])
        index = close + len(ticks)
    return "".join(out)


def slugify(text: str) -> str:
    text = replace_code_spans(text)
    text = IMAGE_RE.sub(r"\1", text)
    text = LINK_RE.sub(r"\1", text)
    text = REF_LINK_RE.sub(r"\1", text)
    text = HTML_TAG_RE.sub("", text)
    text = text.replace("*", "").replace("~~", "").replace("`", "")
    text = text.lower()
    text = re.sub(r"[^\w\- ]", "", text, flags=re.UNICODE)
    return text.replace(" ", "-")


def heading_slugs(path: Path) -> set[str]:
    slugs: set[str] = set()
    counts: dict[str, int] = {}
    in_fence = False
    fence_char = ""
    fence_length = 0

    for line in path.read_text(encoding="utf-8", errors="replace").splitlines():
        fence = FENCE_RE.match(line)
        if fence:
            marker = fence.group(1)
            if not in_fence:
                in_fence, fence_char, fence_length = True, marker[0], len(marker)
            elif marker[0] == fence_char and len(marker) >= fence_length:
                in_fence = False
            continue
        if in_fence:
            continue

        heading = HEADING_RE.match(line)
        if not heading:
            continue

        base = slugify(heading.group(2))
        if base not in counts:
            counts[base] = 0
            slugs.add(base)
            continue
        count = counts[base]
        while True:
            count += 1
            candidate = f"{base}-{count}"
            if candidate not in slugs:
                break
        counts[base] = count
        slugs.add(candidate)

    return slugs


def parse_destination(raw: str) -> str | None:
    """Return the destination from a link body, dropping any URL title."""
    text = raw.strip()
    if not text:
        return None
    if text.startswith("<"):
        close = text.find(">")
        if close == -1:
            return None
        return text[1:close]
    return text.split(None, 1)[0]


def inline_targets(line: str) -> list[str]:
    """Yield the destination of every inline link or image on one line."""
    targets: list[str] = []
    index = 0
    length = len(line)
    while index < length:
        if line[index] != "[":
            index += 1
            continue

        # Find the matching `]`, allowing nested brackets and escapes.
        depth = 0
        close = index
        while close < length:
            char = line[close]
            if char == "\\":
                close += 2
                continue
            if char == "[":
                depth += 1
            elif char == "]":
                depth -= 1
                if depth == 0:
                    break
            close += 1
        if close >= length or close + 1 >= length or line[close + 1] != "(":
            index += 1
            continue

        # Find the matching `)`, ignoring parentheses inside `<...>`.
        cursor = close + 2
        parens = 1
        in_angle = False
        while cursor < length:
            char = line[cursor]
            if char == "\\":
                cursor += 2
                continue
            if char == "<":
                in_angle = True
            elif char == ">":
                in_angle = False
            elif not in_angle:
                if char == "(":
                    parens += 1
                elif char == ")":
                    parens -= 1
                    if parens == 0:
                        break
            cursor += 1
        if cursor >= length or parens != 0:
            index += 1
            continue

        target = parse_destination(line[close + 2 : cursor])
        if target is not None:
            targets.append(target)
        index = cursor + 1

    return targets


def check_target(
    target: str,
    source: str,
    line_number: int,
    root: Path,
    broken: list[tuple[str, int, str, str]],
    warnings: list[tuple[str, int, str]],
) -> int:
    """Check one target. Return 1 when a relative target was resolved."""
    if not target or target.startswith("#"):
        return 0
    if target.startswith("//") or SCHEME_RE.match(target):
        return 0
    if target.startswith("/"):
        warnings.append((source, line_number, target))
        return 0

    path_part, _, fragment = target.partition("#")
    path_part = path_part.split("?", 1)[0]
    path_part = urllib.parse.unquote(path_part)
    fragment = urllib.parse.unquote(fragment)
    if not path_part:
        return 0

    relative = os.path.normpath(os.path.join(os.path.dirname(source), path_part))
    if relative == ".scratch" or relative.startswith(".scratch" + os.sep):
        return 0

    resolved = root / relative
    if not resolved.exists():
        broken.append((source, line_number, target, relative))
        return 1

    if fragment and resolved.is_file() and resolved.suffix.lower() == ".md":
        if fragment not in heading_slugs(resolved):
            broken.append((source, line_number, target, f"{relative}#{fragment}"))
            return 1

    return 1


def main() -> int:
    root = repo_root()
    files = markdown_files(root)
    broken: list[tuple[str, int, str, str]] = []
    warnings: list[tuple[str, int, str]] = []
    checked = 0

    for source in files:
        path = root / source
        try:
            lines = path.read_text(encoding="utf-8", errors="replace").splitlines()
        except OSError as error:
            print(f"{source}: cannot read: {error}", file=sys.stderr)
            return 1

        in_fence = False
        fence_char = ""
        fence_length = 0

        for line_number, line in enumerate(lines, start=1):
            fence = FENCE_RE.match(line)
            if fence:
                marker = fence.group(1)
                if not in_fence:
                    in_fence, fence_char, fence_length = True, marker[0], len(marker)
                elif marker[0] == fence_char and len(marker) >= fence_length:
                    in_fence = False
                continue
            if in_fence:
                continue

            clean = strip_code_spans(line)

            definition = REF_DEF_RE.match(clean)
            if definition:
                target = parse_destination(definition.group(2))
                if target is not None:
                    checked += check_target(
                        target, source, line_number, root, broken, warnings
                    )
                continue

            for target in inline_targets(clean):
                checked += check_target(
                    target, source, line_number, root, broken, warnings
                )

    print(f"Scanned {len(files)} Markdown files; checked {checked} relative links.")
    for source, line_number, target in warnings:
        print(f"{source}:{line_number}: web-root link skipped: {target}")
    for source, line_number, target, resolved in broken:
        print(f"{source}:{line_number}: {target} -> {resolved}")

    if broken:
        print(f"Broken links: {len(broken)}")
        return 1

    print("All relative links resolve.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
