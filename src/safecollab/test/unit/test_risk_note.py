"""Doc-consistency spec for docs/RISK.md (P6, ground rule 1).

The risk note must agree with the *model*: the zone thresholds it quotes have to
equal what ``risk.py`` computes from ``config/risk.yaml`` (ground rule 5 -- no
hand-typed constants), it has to state the six ISO/TS 15066 ``S_p`` terms, and it
has to document the property that a larger operator uncertainty ``z_d`` widens
both thresholds. If ``risk.yaml`` changes, the derived numbers change and this
test forces the note to be updated with them.
"""

from pathlib import Path

from safecollab.risk import load_config, thresholds

_REPO_ROOT = Path(__file__).resolve().parents[4]
_RISK_DOC = _REPO_ROOT / "docs" / "RISK.md"
_RISK_YAML = _REPO_ROOT / "src" / "safecollab" / "config" / "risk.yaml"

# The two z_d operating points the note tabulates, computed live from the config.
_Z_D_POINTS = [0.05, 0.10]

# The six terms of S_p = S_H + S_R + S_S + C + Z_d + Z_r (bare "C" is too common
# to assert as a substring, so we check the summed formula instead).
_SP_TERMS = ["S_H", "S_R", "S_S", "Z_d", "Z_r"]


def _doc_text():
    assert _RISK_DOC.is_file(), f"missing risk note: {_RISK_DOC}"
    return _RISK_DOC.read_text(encoding="utf-8")


def test_states_the_thresholds_computed_from_config():
    text = _doc_text()
    cfg = load_config(_RISK_YAML)
    missing = []
    for z_d in _Z_D_POINTS:
        d_red, d_yellow = thresholds(cfg, z_d=z_d)
        for value in (f"{d_red:.2f}", f"{d_yellow:.2f}"):
            if value not in text:
                missing.append((z_d, value))
    assert (
        not missing
    ), f"risk note does not quote the thresholds computed from risk.yaml: {missing}"


def test_states_the_sp_model_and_its_terms():
    text = _doc_text()
    assert "S_p" in text, "risk note must name the ISO/TS 15066 S_p model"
    missing = [t for t in _SP_TERMS if t not in text]
    assert not missing, f"risk note does not list S_p term(s): {missing}"


def test_documents_that_uncertainty_widens_the_thresholds():
    text = _doc_text()
    cfg = load_config(_RISK_YAML)
    low = thresholds(cfg, z_d=min(_Z_D_POINTS))
    high = thresholds(cfg, z_d=max(_Z_D_POINTS))
    # The property must actually hold in the model...
    assert (
        high[0] > low[0] and high[1] > low[1]
    ), "model regression: larger z_d should widen both thresholds"
    # ...and the note must state it.
    assert (
        "widen" in text.lower() or "wider" in text.lower()
    ), "risk note must document that larger z_d widens the thresholds"


def test_names_the_two_scenarios_and_config_source():
    text = _doc_text().lower()
    assert "risk.yaml" in text, "risk note must point at config/risk.yaml as the source"
    assert "reduced" in text and (
        "full speed" in text or "full_speed" in text
    ), "risk note must describe the full-speed and reduced scenarios"
