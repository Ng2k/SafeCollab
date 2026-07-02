"""Doc-consistency spec for docs/ARCHITECTURE.md (P6, ground rule 1).

The architecture diagram must stay consistent with the *build*: it has to name
every ROS node the package actually ships (the authoritative list is the
``console_scripts`` in ``setup.py``), the two pure logic modules, and the key
topics of the §3 interface contract. If a node is added, renamed, or removed
without updating the diagram, this test fails.

Pure ``pathlib`` + regex; no ROS graph, no new dependency.
"""

import re
from pathlib import Path

import pytest

_REPO_ROOT = Path(__file__).resolve().parents[4]
_ARCH_DOC = _REPO_ROOT / "docs" / "ARCHITECTURE.md"
_SETUP_PY = _REPO_ROOT / "src" / "safecollab" / "setup.py"

# Key channels from the AGENTS.md §3 interface contract that the SSM story
# hinges on -- the diagram must show the speed-scaling loop end to end.
_CONTRACT_TOPICS = [
    "/camera/image",
    "/human/uncertainty",
    "/safety/scale",
    "/safety/zone",
    "/motion/nominal_trajectory",
    "/arm_controller/joint_trajectory",
]

# Pure (no-ROS) modules the safety maths lives in.
_PURE_MODULES = ["risk", "safety_logic"]


def _node_names_from_setup():
    """The authoritative node list: left-hand side of each console_scripts entry."""
    text = _SETUP_PY.read_text(encoding="utf-8")
    names = re.findall(r"(\w+)\s*=\s*safecollab\.\w+:main", text)
    assert names, "no console_scripts entry points found in setup.py"
    return names


@pytest.fixture(scope="module")
def arch_text():
    assert _ARCH_DOC.is_file(), f"missing architecture doc: {_ARCH_DOC}"
    return _ARCH_DOC.read_text(encoding="utf-8")


def test_has_a_rendered_mermaid_diagram(arch_text):
    # A GitHub-native rendered diagram (```mermaid fenced block), not just prose.
    assert re.search(
        r"```mermaid\b", arch_text
    ), "ARCHITECTURE.md must contain a ```mermaid fenced diagram block"


def test_references_every_shipped_node(arch_text):
    missing = [n for n in _node_names_from_setup() if n not in arch_text]
    assert not missing, f"architecture doc does not mention node(s): {missing}"


def test_references_the_pure_logic_modules(arch_text):
    missing = [m for m in _PURE_MODULES if m not in arch_text]
    assert not missing, f"architecture doc does not mention pure module(s): {missing}"


def test_references_the_contract_topics(arch_text):
    missing = [t for t in _CONTRACT_TOPICS if t not in arch_text]
    assert (
        not missing
    ), f"architecture doc does not mention contract topic(s): {missing}"
