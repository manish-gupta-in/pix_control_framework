#!/usr/bin/env python3
"""
Unit tests for pix_command_manager arbitration logic.

Tests priority routing, timeout expiry, and standby fallback
WITHOUT requiring a running ROS2 system.

Run with:
    cd pix_control_framework
    python3 -m pytest src/pix_command_manager/test/test_arbitration_logic.py -v
"""
import time
import pytest

# ---------------------------------------------------------------------------
# Pure-Python re-implementation of the arbitration logic
# ---------------------------------------------------------------------------

PRIORITIES = [
    'EMERGENCY_STOP',
    'COLLISION_AVOIDANCE',
    'HUMAN_AVOIDANCE',
    'LANE_FOLLOWING',
    'CRUISE_CONTROL',
]


class Arbitrator:
    """Mirrors PixCommandArbitrator priority selection logic."""
    def __init__(self, active_timeout=0.4):
        self.active_timeout = active_timeout
        self.storage = {name: {'msg': None, 'time': 0.0} for name in PRIORITIES}

    def publish(self, source, msg, t):
        """Simulate a message arriving from `source` at time `t`."""
        self.storage[source]['msg']  = msg
        self.storage[source]['time'] = t

    def arbitrate(self, now):
        """Return (selected_source, selected_msg) or (None, None) for standby."""
        for name in PRIORITIES:
            data = self.storage[name]
            if data['msg'] is not None:
                if (now - data['time']) < self.active_timeout:
                    return name, data['msg']
        return None, None


# ---------------------------------------------------------------------------
# Tests
# ---------------------------------------------------------------------------

class TestPriorityRouting:
    def test_single_source_selected(self):
        arb = Arbitrator()
        arb.publish('LANE_FOLLOWING', {'steer': 100.0}, 1.0)
        source, _ = arb.arbitrate(1.1)
        assert source == 'LANE_FOLLOWING'

    def test_higher_priority_overrides_lower(self):
        arb = Arbitrator()
        t = 10.0
        arb.publish('LANE_FOLLOWING', {'steer': 100.0}, t)
        arb.publish('HUMAN_AVOIDANCE', {'steer': -200.0}, t)
        source, _ = arb.arbitrate(t + 0.01)
        assert source == 'HUMAN_AVOIDANCE'

    def test_emergency_stop_is_highest(self):
        arb = Arbitrator()
        t = 20.0
        arb.publish('LANE_FOLLOWING', {}, t)
        arb.publish('HUMAN_AVOIDANCE', {}, t)
        arb.publish('COLLISION_AVOIDANCE', {}, t)
        arb.publish('EMERGENCY_STOP', {'estop': True}, t)
        source, _ = arb.arbitrate(t + 0.01)
        assert source == 'EMERGENCY_STOP'

    def test_priority_order_all_active(self):
        """Verify strict priority ordering when all sources active."""
        arb = Arbitrator()
        t = 5.0
        for name in PRIORITIES:
            arb.publish(name, {'src': name}, t)

        # Check each priority level by removing highest and re-arbitrating
        for expected in PRIORITIES:
            source, _ = arb.arbitrate(t + 0.01)
            assert source == expected, f"Expected {expected}, got {source}"
            # Expire this source
            arb.storage[expected]['time'] = 0.0


class TestTimeoutBehavior:
    def test_expired_source_not_selected(self):
        arb = Arbitrator(active_timeout=0.4)
        arb.publish('LANE_FOLLOWING', {'steer': 50.0}, 0.0)
        # Arbitrate well past timeout
        source, _ = arb.arbitrate(1.0)
        assert source is None

    def test_fresh_source_replaces_expired(self):
        arb = Arbitrator(active_timeout=0.4)
        t = 0.0
        arb.publish('LANE_FOLLOWING', {'steer': 50.0}, t)
        # Both available at start
        source, _ = arb.arbitrate(t + 0.1)
        assert source == 'LANE_FOLLOWING'
        # Lane following expires
        arb.publish('CRUISE_CONTROL', {'speed': 2.0}, 1.0)
        source, _ = arb.arbitrate(1.1)
        assert source == 'CRUISE_CONTROL'

    def test_standby_when_all_expired(self):
        arb = Arbitrator(active_timeout=0.4)
        for name in PRIORITIES:
            arb.publish(name, {}, 0.0)
        source, msg = arb.arbitrate(10.0)
        assert source is None
        assert msg is None

    def test_exactly_at_timeout_boundary(self):
        """At exactly timeout, message should be expired."""
        arb = Arbitrator(active_timeout=0.4)
        arb.publish('LANE_FOLLOWING', {}, 0.0)
        # elapsed == timeout exactly: should be expired (< not <=)
        source, _ = arb.arbitrate(0.4)
        assert source is None

    def test_just_before_timeout(self):
        """Just before timeout, message should still be active."""
        arb = Arbitrator(active_timeout=0.4)
        arb.publish('LANE_FOLLOWING', {}, 0.0)
        source, _ = arb.arbitrate(0.399)
        assert source == 'LANE_FOLLOWING'


class TestStandbyFallback:
    def test_no_messages_gives_none(self):
        arb = Arbitrator()
        source, msg = arb.arbitrate(0.0)
        assert source is None
        assert msg is None

    def test_message_recovery_after_standby(self):
        arb = Arbitrator(active_timeout=0.4)
        arb.publish('LANE_FOLLOWING', {'steer': 10.0}, 0.0)
        # Goes to standby
        src, _ = arb.arbitrate(5.0)
        assert src is None
        # Recovers with new message
        arb.publish('LANE_FOLLOWING', {'steer': 20.0}, 5.5)
        src, msg = arb.arbitrate(5.6)
        assert src == 'LANE_FOLLOWING'
        assert msg['steer'] == 20.0


if __name__ == '__main__':
    pytest.main([__file__, '-v'])
