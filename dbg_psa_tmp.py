import numpy as np
from opendbc.car.lateral import get_max_angle_delta_vm, get_max_angle_vm
from opendbc.car.psa.carcontroller import get_safety_CP
from opendbc.car.psa.values import CarControllerParams
from opendbc.car.structs import CarParams
from opendbc.car.vehicle_model import VehicleModel
from opendbc.safety.tests.libsafety import libsafety_py
from opendbc.safety.tests.common import CANPackerSafety, away_round, round_speed, MAX_SAMPLE_VALS


def round_angle(apply_angle, can_offset=0):
  rnd_offset = 1e-5 if apply_angle >= 0 else -1e-5
  return (away_round(apply_angle / 0.1 + rnd_offset) + can_offset) * 0.1


VM = VehicleModel(get_safety_CP())
packer = CANPackerSafety("psa_aee2010_r3")
safety = libsafety_py.libsafety
safety.set_safety_hooks(CarParams.SafetyModel.psa, 0)
safety.init_tests()

cnt = [0]

def angle_cmd(angle, enabled=True):
  values = {"SET_ANGLE": angle, "TORQUE_FACTOR": 100 if enabled else 0}
  safety.set_timer(cnt[0] * int(1e6 / 100))
  cnt[0] += 1
  msg = packer.make_can_msg_safety("LANE_KEEP_ASSIST", 0, values)
  return safety.safety_tx_hook(msg)

def speed_msg(speed):
  kph = speed * 3.6
  values = {f"P{i}_VehV_VPsvValWhl{p}": kph for i, p in zip(range(263, 267), ["FrtL", "FrtR", "BckL", "BckR"])}
  return packer.make_can_msg_safety("Dyn4_FRE", 0, values)

def reset_speed(speed):
  for _ in range(MAX_SAMPLE_VALS):
    safety.safety_rx_hook(speed_msg(speed))

failures = 0
for speed in np.linspace(0, 40, 100):
  speed = round_speed(away_round(max(speed, 1) * 3.6 / 0.01) * 0.01 / 3.6)
  for sign in (-1, 1):
    safety.set_controls_allowed(True)
    reset_speed(speed + 1)
    angle_cmd(0)

    delta = get_max_angle_delta_vm(speed, VM, CarControllerParams)
    at_limit = round_angle(delta) * sign
    above = round_angle(delta, 1) * sign

    ok_at = angle_cmd(at_limit)
    safety.set_desired_angle_last(0)
    ok_above = angle_cmd(above)
    if not ok_at or ok_above:
      failures += 1
      if failures < 8:
        print(f"speed={speed:.4f} sign={sign} vmin={safety.get_vehicle_speed_min()} delta={delta:.6f} "
              f"delta_can={delta*10:.4f} at={at_limit} above={above} ok_at={ok_at} ok_above={ok_above}")
    safety.set_desired_angle_last(0)
print("failures:", failures)
