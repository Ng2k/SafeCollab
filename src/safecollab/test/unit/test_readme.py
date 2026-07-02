"""Doc-consistency spec for README.md (P6, ground rule 1).

The README is the entry point and the CI *package* stage verifies its
one-command run (AGENTS.md §7 stage 5). This test locks the README's promises to
the real build: the two-command run (§8), the headless launch line, the demo GIF,
and links to the architecture / risk / demo docs P6 produces.
"""

from pathlib import Path

_REPO_ROOT = Path(__file__).resolve().parents[4]
_README = _REPO_ROOT / "README.md"

# Every relative link the README must carry maps to a file that must exist.
_REQUIRED_LINKS = [
    "docs/DEMO.md",
    "docs/ARCHITECTURE.md",
    "docs/RISK.md",
    "docs/media/safecollab-demo.gif",
]


def _readme_text():
    assert _README.is_file(), f"missing README: {_README}"
    return _README.read_text(encoding="utf-8")


def test_documents_the_one_command_run():
    text = _readme_text()
    # The §8 build-and-run the CI package stage checks.
    assert "docker build" in text, "README must document `docker build`"
    assert "docker run" in text, "README must document `docker run`"


def test_documents_the_headless_launch():
    text = _readme_text()
    assert "cell.launch.py" in text, "README must name the launch file"
    assert (
        "headless:=true" in text
    ), "README must document the headless launch (CI path)"


def test_links_to_the_p6_docs_and_they_exist():
    text = _readme_text()
    for link in _REQUIRED_LINKS:
        assert link in text, f"README does not link to {link}"
        assert (_REPO_ROOT / link).is_file(), f"README links to a missing file: {link}"


def test_names_the_safety_standard():
    text = _readme_text()
    assert "ISO/TS 15066" in text, "README must name the safety standard it implements"
