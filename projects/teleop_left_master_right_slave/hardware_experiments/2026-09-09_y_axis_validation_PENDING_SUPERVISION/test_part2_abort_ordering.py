"""2026-09-10 item 3: pure test proving Part 2's abort sequence disables
FIRST and never publishes a new (possibly reverse-pulling) target.

No ROS/rclpy -- tests abort_sequence() directly.
"""
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).parent))
from part2_master_only_y_motion import abort_sequence, DISABLE_REPEAT_N  # noqa: E402


def test_abort_disables_before_recording_pose():
    actions = abort_sequence()
    assert len(actions) >= 1
    disable_indices = [i for i, a in enumerate(actions) if a == ("enable", False)]
    record_indices = [i for i, a in enumerate(actions) if a[0] == "record_pose"]
    assert disable_indices, "abort must disable at least once"
    assert record_indices, "abort must record the pose at abort"
    assert max(disable_indices) < min(record_indices), (
        "ALL disable actions must come before pose recording"
    )
    print(f"[test] disable indices={disable_indices} record indices={record_indices} PASS")


def test_abort_repeats_disable_multiple_times():
    actions = abort_sequence()
    n_disable = sum(1 for a in actions if a == ("enable", False))
    assert n_disable == DISABLE_REPEAT_N
    assert n_disable >= 3, "a single disable publish is not bounded-reliable over best-effort ROS"
    print(f"[test] disable repeated {n_disable} times PASS")


def test_abort_never_publishes_a_target():
    actions = abort_sequence()
    target_actions = [a for a in actions if a[0] == "target"]
    assert not target_actions, (
        f"abort must NEVER publish a new target (risk of reverse spring pull "
        f"if the device has already moved past the latched origin), found: {target_actions}"
    )
    print("[test] no target action anywhere in abort_sequence() PASS")


def test_abort_enable_value_is_always_false():
    actions = abort_sequence()
    for a in actions:
        if a[0] == "enable":
            assert a[1] is False, f"abort must only ever disable, found enable action {a}"
    print("[test] every enable action in abort_sequence() is False PASS")


if __name__ == "__main__":
    test_abort_disables_before_recording_pose()
    test_abort_repeats_disable_multiple_times()
    test_abort_never_publishes_a_target()
    test_abort_enable_value_is_always_false()
    print("ALL TESTS PASSED")
