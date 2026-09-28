"""2026-09-10: MOVED. The canonical frozen config now lives at
projects/teleop_left_master_right_slave/hardware_validation/frozen_config.py
(item 7, "complete canonical configuration adoption" -- one authoritative
source, not a copy alongside it). This file re-exports from there so every
test/script already importing from THIS path keeps working unmodified.
Do not add new fields here -- edit the canonical file instead.

Loaded via importlib with an explicit distinct module name (NOT a plain
`sys.path.insert` + `import frozen_config`) because THIS file is itself
named frozen_config.py -- a naive import would resolve back to this
partially-initialized module instead of the canonical one.
"""
from __future__ import annotations

import importlib.util
import sys
from pathlib import Path

_canonical_path = (Path(__file__).resolve().parents[2] / "hardware_validation" / "frozen_config.py")
_spec = importlib.util.spec_from_file_location("_canonical_frozen_config", _canonical_path)
_canonical = importlib.util.module_from_spec(_spec)
sys.modules["_canonical_frozen_config"] = _canonical  # dataclasses needs this registered
_spec.loader.exec_module(_canonical)

FROZEN = _canonical.FROZEN
FrozenConfig = _canonical.FrozenConfig
bridge_cli_args = _canonical.bridge_cli_args
config_fingerprint = _canonical.config_fingerprint
write_manifest = _canonical.write_manifest
