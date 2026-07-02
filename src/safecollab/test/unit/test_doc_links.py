"""Doc-consistency spec: internal Markdown links resolve (P6, ground rule 1).

Every *relative* link (and image) in README.md and docs/*.md must point at a file
that exists, resolved relative to the file the link lives in. External links
(http/https/mailto) and pure in-page anchors (#section) are out of scope. Catches
the classic docs rot: a renamed/moved file leaving a dangling link.
"""

import re
from pathlib import Path

import pytest

_REPO_ROOT = Path(__file__).resolve().parents[4]

# Markdown inline links and images: [text](target) / ![alt](target)
_LINK_RE = re.compile(r"!?\[[^\]]*\]\(([^)]+)\)")
# Fenced code blocks (```...```) hold illustrative snippets, not live links.
_FENCE_RE = re.compile(r"```.*?```", re.DOTALL)


def _markdown_files():
    files = [_REPO_ROOT / "README.md"]
    files += sorted((_REPO_ROOT / "docs").glob("*.md"))
    return [f for f in files if f.is_file()]


def _relative_targets(md_file):
    text = _FENCE_RE.sub("", md_file.read_text(encoding="utf-8"))
    for raw in _LINK_RE.findall(text):
        target = raw.strip()
        # Drop an optional link title:  (path "Title")
        target = target.split(" ", 1)[0]
        # Strip a trailing #anchor.
        target = target.split("#", 1)[0]
        if not target:
            continue  # pure in-page anchor
        if target.startswith(("http://", "https://", "mailto:")):
            continue  # external
        yield target


def test_repo_has_markdown_to_check():
    assert _markdown_files(), "no Markdown files found to link-check"


@pytest.mark.parametrize("md_file", _markdown_files(), ids=lambda p: p.name)
def test_internal_links_resolve(md_file):
    broken = []
    for target in _relative_targets(md_file):
        resolved = (md_file.parent / target).resolve()
        if not resolved.exists():
            broken.append(target)
    assert not broken, f"{md_file.name} has dangling relative link(s): {broken}"
