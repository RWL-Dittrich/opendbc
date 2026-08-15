import unittest
from unittest import mock

from opendbc.car import DT_CTRL, structs
from opendbc.car.can_definitions import CanData
from opendbc.car.car_helpers import interfaces
from opendbc.car.psa.carcontroller import RADAR_DISABLE_FRAME
from opendbc.car.psa.interface import RADAR_ENABLE_TIMEOUT_FRAMES

DISABLE_RADAR = (0x6B6, b'\x02\x10\x02\x80\x00\x00\x00\x00')
ENABLE_RADAR = (0x6B6, b'\x02\x10\x01\x80\x00\x00\x00\x00')
RADAR_EMULATION = (0x2B6, 0x2F6)


def make_car_interface(alpha_long: bool):
  CarInterface = interfaces["PSA_PEUGEOT_208"]
  CP = CarInterface.get_params("PSA_PEUGEOT_208", {i: {} for i in range(8)}, [],
                               alpha_long=alpha_long, is_release=False, docs=False)
  CP_SP = CarInterface.get_params_sp(CP, "PSA_PEUGEOT_208", {i: {} for i in range(8)}, [],
                                     alpha_long=alpha_long, is_release_sp=False, docs=False)
  return CarInterface(CP, CP_SP)


class TestPsaRadarKnockout(unittest.TestCase):
  """The radar ECU knockout has to stay in step with what is actually on the ADAS bus.

  The ESP (UC_FREIN) marks its ACC fields invalid after ~150 ms without 0x2B6, which
  openpilot reports as accFaulted, so 0x2B6 must never be missing from the bus for
  longer than that and must never be sent by two ECUs at once.
  """

  def setUp(self):
    self.CI = make_car_interface(alpha_long=True)
    self.assertTrue(self.CI.CP.openpilotLongitudinalControl)
    self.CC = structs.CarControl().as_reader()
    self.CC_SP = structs.CarControlSP()
    self.now_nanos = 0

  def step(self, radar_alive: bool):
    self.CI.update([])
    self.CI.CS.radar_alive = radar_alive
    _, can_sends = self.CI.apply(self.CC, self.CC_SP, self.now_nanos)
    self.now_nanos += int(DT_CTRL * 1e9)
    return can_sends

  @staticmethod
  def addrs(can_sends):
    return [msg[0] for msg in can_sends]

  def test_no_knockout_before_safety_mode_is_certain(self):
    # pandad still has the panda in ELM327 mode early on, where the diagnostic
    # knockout is allowed through but the emulation that replaces it is not
    for _ in range(RADAR_DISABLE_FRAME):
      addrs = self.addrs(self.step(radar_alive=True))
      self.assertNotIn(0x6B6, addrs)
      for addr in RADAR_EMULATION:
        self.assertNotIn(addr, addrs)

  def test_knockout_sent_once_then_waits_for_radar_to_go_quiet(self):
    for _ in range(RADAR_DISABLE_FRAME):
      self.step(radar_alive=True)

    # the knockout goes out exactly once
    assert DISABLE_RADAR in [(m[0], m[1]) for m in self.step(radar_alive=True)]

    # the real radar is still transmitting, so we must not emulate yet
    for _ in range(50):
      addrs = self.addrs(self.step(radar_alive=True))
      self.assertNotIn(0x6B6, addrs)
      for addr in RADAR_EMULATION:
        self.assertNotIn(addr, addrs, "emulating while the real radar is still on the bus")

    # once it goes quiet, emulation takes over
    seen: set[int] = set()
    for _ in range(4):
      seen.update(self.addrs(self.step(radar_alive=False)))
    self.assertTrue(RADAR_EMULATION[0] in seen and RADAR_EMULATION[1] in seen)

  def test_no_knockout_without_openpilot_longitudinal(self):
    self.CI = make_car_interface(alpha_long=False)
    self.assertFalse(self.CI.CP.openpilotLongitudinalControl)
    for _ in range(RADAR_DISABLE_FRAME * 2):
      addrs = self.addrs(self.step(radar_alive=True))
      self.assertNotIn(0x6B6, addrs)
      for addr in RADAR_EMULATION:
        self.assertNotIn(addr, addrs)


class FakeBus:
  """Stands in for card's can_recv/can_send callbacks."""

  def __init__(self, radar_returns_after=None, radar_src=1, seed=None):
    self.sent: list = []
    self.calls = 0
    self.radar_returns_after = radar_returns_after
    self.radar_src = radar_src
    self.seed = seed

  def can_recv(self, wait_for_one=False):
    self.calls += 1
    if self.calls == 1:  # the counter-seeding read
      return [list(self.seed)] if self.seed else []
    if self.radar_returns_after is not None and self.calls > self.radar_returns_after:
      return [[CanData(0x2B6, b'\x00' * 8, self.radar_src)]]
    return []

  def can_send(self, msgs):
    self.sent.extend(msgs)

  def addrs(self):
    return [m[0] for m in self.sent]


class TestPsaDeinit(unittest.TestCase):
  """deinit() has to close the gap between openpilot leaving the bus and the radar returning."""

  def setUp(self):
    patcher = mock.patch('opendbc.car.psa.interface.time.sleep')
    patcher.start()
    self.addCleanup(patcher.stop)

  def test_noop_without_openpilot_longitudinal(self):
    CI = make_car_interface(alpha_long=False)
    bus = FakeBus(radar_returns_after=1)
    CI.deinit(CI.CP, bus.can_recv, bus.can_send)
    self.assertEqual(bus.sent, [], "touched the radar without openpilot longitudinal")

  def test_asks_the_radar_back_and_covers_the_gap(self):
    CI = make_car_interface(alpha_long=True)
    bus = FakeBus(radar_returns_after=10)
    CI.deinit(CI.CP, bus.can_recv, bus.can_send)

    # the very first thing on the bus is the request to come back
    self.assertEqual((bus.sent[0][0], bus.sent[0][1]), ENABLE_RADAR)
    # and we keep emulating until it does
    for addr in RADAR_EMULATION:
      assert addr in bus.addrs(), "left the bus silent while the radar restarted"

  def test_stops_as_soon_as_the_real_radar_transmits(self):
    CI = make_car_interface(alpha_long=True)
    bus = FakeBus(radar_returns_after=5)
    CI.deinit(CI.CP, bus.can_recv, bus.can_send)
    # two ECUs on 0x2B6 would collide, so we must get out of the way promptly
    self.assertLess(bus.calls, RADAR_ENABLE_TIMEOUT_FRAMES)
    self.assertLessEqual(bus.addrs().count(0x2B6), 4)

  def test_our_own_tx_echo_is_not_mistaken_for_the_radar(self):
    CI = make_car_interface(alpha_long=True)
    # src 129 is bus 1 with the panda's "returned" flag, i.e. our own emulated frame
    bus = FakeBus(radar_returns_after=1, radar_src=129)
    CI.deinit(CI.CP, bus.can_recv, bus.can_send)
    self.assertGreater(bus.addrs().count(0x2B6), 10, "quit on its own echo")

  def test_terminates_if_the_radar_never_comes_back(self):
    CI = make_car_interface(alpha_long=True)
    bus = FakeBus(radar_returns_after=None)
    CI.deinit(CI.CP, bus.can_recv, bus.can_send)
    self.assertEqual(bus.calls, RADAR_ENABLE_TIMEOUT_FRAMES + 1)

  def test_continues_the_counter_sequence(self):
    CI = make_car_interface(alpha_long=True)
    # last 0x2B6 on the bus carried COUNTER 7 (high nibble of byte 7)
    bus = FakeBus(radar_returns_after=None, seed=[CanData(0x2B6, b'\x00' * 7 + b'\x70', 129)])
    CI.deinit(CI.CP, bus.can_recv, bus.can_send)

    counters = [m[1][7] >> 4 for m in bus.sent if m[0] == 0x2B6]
    self.assertEqual(counters[:3], [8, 9, 10], "restarted the counter instead of continuing it")


if __name__ == "__main__":
  unittest.main()
