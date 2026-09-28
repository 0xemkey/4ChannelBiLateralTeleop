from __future__ import annotations

import os
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from check_touch_devices import main  # noqa: E402


LEFT_CFG = """\
ModelType=OMNI
CardType=USB
Channel=300
DeviceSerialNumber=18019000358
"""

RIGHT_CFG = """\
ModelType=OMNI
CardType=USB
Channel=301
DeviceSerialNumber=18019000365
"""


def _make_fake_topology(tmp_path: Path) -> tuple[Path, Path, Path]:
    cfg_dir = tmp_path / "config"
    dev = tmp_path / "dev"
    cfg_dir.mkdir()
    dev.mkdir()
    (cfg_dir / "Left.config").write_text(LEFT_CFG)
    (cfg_dir / "Default Device.config").write_text(RIGHT_CFG)
    (dev / "ttyACM0").write_text("")
    (dev / "ttyACM1").write_text("")
    os.symlink("ttyACM0", dev / "ttyACM300")
    os.symlink("ttyACM1", dev / "ttyACM301")
    os.symlink("ttyACM0", dev / "touch_18019000358")
    os.symlink("ttyACM1", dev / "touch_18019000365")
    return cfg_dir / "Left.config", cfg_dir / "Default Device.config", dev


def test_stable_channel_and_identity_links_pass(tmp_path):
    left_cfg, right_cfg, dev = _make_fake_topology(tmp_path)
    rc = main([
        "--left-config", str(left_cfg),
        "--right-config", str(right_cfg),
        "--dev-root", str(dev),
        "--allow-regular-files",
    ])
    assert rc == 0


def test_raw_tty_order_is_not_required_when_stable_links_are_valid(tmp_path):
    left_cfg, right_cfg, dev = _make_fake_topology(tmp_path)
    (dev / "ttyACM0").rename(dev / "ttyACM7")
    (dev / "ttyACM1").rename(dev / "ttyACM8")
    (dev / "ttyACM300").unlink()
    (dev / "ttyACM301").unlink()
    (dev / "touch_18019000358").unlink()
    (dev / "touch_18019000365").unlink()
    os.symlink("ttyACM7", dev / "ttyACM300")
    os.symlink("ttyACM8", dev / "ttyACM301")
    os.symlink("ttyACM7", dev / "touch_18019000358")
    os.symlink("ttyACM8", dev / "touch_18019000365")

    rc = main([
        "--left-config", str(left_cfg),
        "--right-config", str(right_cfg),
        "--dev-root", str(dev),
        "--allow-regular-files",
    ])
    assert rc == 0


def test_identity_mismatch_fails(tmp_path, capsys):
    left_cfg, right_cfg, dev = _make_fake_topology(tmp_path)
    (dev / "touch_18019000365").unlink()
    os.symlink("ttyACM0", dev / "touch_18019000365")

    rc = main([
        "--left-config", str(left_cfg),
        "--right-config", str(right_cfg),
        "--dev-root", str(dev),
        "--allow-regular-files",
    ])

    captured = capsys.readouterr()
    assert rc == 1
    assert "resolve to different targets" in captured.err


def test_wrong_config_channel_fails(tmp_path, capsys):
    left_cfg, right_cfg, dev = _make_fake_topology(tmp_path)
    left_cfg.write_text(LEFT_CFG.replace("Channel=300", "Channel=0"))

    rc = main([
        "--left-config", str(left_cfg),
        "--right-config", str(right_cfg),
        "--dev-root", str(dev),
        "--allow-regular-files",
    ])

    captured = capsys.readouterr()
    assert rc == 1
    assert "Channel='0', expected 300" in captured.err
