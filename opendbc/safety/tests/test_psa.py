#!/usr/bin/env python3
import unittest

from opendbc.car.structs import CarParams
from opendbc.safety.tests.libsafety import libsafety_py
import opendbc.safety.tests.common as common
from opendbc.safety.tests.common import CANPackerSafety

LANE_KEEP_ASSIST = 0x3F2


class TestPsaSafetyBase(common.CarSafetyTest, common.AngleSteeringSafetyTest):
  RELAY_MALFUNCTION_ADDRS = {0: (LANE_KEEP_ASSIST,)}
  FWD_BLACKLISTED_ADDRS = {2: [LANE_KEEP_ASSIST]}
  TX_MSGS = [[1010, 0], [1106, 1], [1718, 1], [1942, 1], [1270, 1], [694, 1], [758, 1]]

  MAIN_BUS = 0
  ADAS_BUS = 1
  CAM_BUS = 2

  STEER_ANGLE_MAX = 390
  DEG_TO_CAN = 10

  ANGLE_RATE_BP = [0., 5., 25.]
  ANGLE_RATE_UP = [2.5, 1.5, .2]
  ANGLE_RATE_DOWN = [5., 2., .3]

  def setUp(self):
    self.packer = CANPackerSafety("psa_aee2010_r3")
    self.safety = libsafety_py.libsafety
    self.safety.set_safety_hooks(CarParams.SafetyModel.psa, 0)
    self.safety.init_tests()

  def _angle_cmd_msg(self, angle: float, enabled: bool):
    values = {"SET_ANGLE": angle, "TORQUE_FACTOR": 100 if enabled else 0}
    return self.packer.make_can_msg_safety("LANE_KEEP_ASSIST", self.MAIN_BUS, values)

  def _angle_meas_msg(self, angle: float):
    values = {"ANGLE": angle}
    return self.packer.make_can_msg_safety("STEERING_ALT", self.MAIN_BUS, values)

  def _pcm_status_msg(self, enable):
    values = {"RVV_ACC_ACTIVATION_REQ": enable}
    return self.packer.make_can_msg_safety("HS2_DAT_MDD_CMD_452", self.ADAS_BUS, values)

  def _speed_msg(self, speed):
    kph = speed * 3.6
    values = {
      "P263_VehV_VPsvValWhlFrtL": kph,
      "P264_VehV_VPsvValWhlFrtR": kph,
      "P265_VehV_VPsvValWhlBckL": kph,
      "P266_VehV_VPsvValWhlBckR": kph,
    }
    return self.packer.make_can_msg_safety("Dyn4_FRE", self.MAIN_BUS, values)

  def _vehicle_moving_msg(self, speed: float):
    values = {"VEHICLE_STANDSTILL": 0 if speed > self.STANDSTILL_THRESHOLD else 1}
    return self.packer.make_can_msg_safety("HS2_DYN_UCF_MDD_32D", self.ADAS_BUS, values)

  def _user_brake_msg(self, brake):
    values = {"P013_MainBrake": brake}
    return self.packer.make_can_msg_safety("Dat_BSI", self.CAM_BUS, values)

  def _user_gas_msg(self, gas):
    values = {"GAS_PEDAL": int(gas * 100)}
    return self.packer.make_can_msg_safety("DRIVER", self.CAM_BUS, values)

  def test_rx_hook(self):
    # cruise
    for _ in range(10):
      self.assertTrue(self._rx(self._pcm_status_msg(0)))
    msg = self._pcm_status_msg(0)
    # invalidate checksum
    msg[0].data[5] = 0x00
    self.assertFalse(self._rx(msg))
    msg = self._pcm_status_msg(0)
    # write to unused payload byte
    msg[0].data[6] = 0xAB
    self.assertTrue(self._rx(msg))


class TestPsaStockSafety(TestPsaSafetyBase):

  def setUp(self):
    self.packer = CANPackerSafety("psa_aee2010_r3")
    self.safety = libsafety_py.libsafety
    self.safety.set_safety_hooks(CarParams.SafetyModel.psa, 0)
    self.safety.init_tests()


if __name__ == "__main__":
    unittest.main()
