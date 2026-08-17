from opendbc.car.can_definitions import CanData


def psa_checksum(address: int, sig, d: bytearray) -> int:
  chk_ini = {0x452: 0x4, 0x38D: 0x7, 0x2f6: 0x8, 0x2b6: 0xC}.get(address, 0xB)
  byte = sig.start_bit // 8
  d[byte] &= 0x0F if sig.start_bit % 8 >= 4 else 0xF0
  checksum = sum((b >> 4) + (b & 0xF) for b in d)
  return (chk_ini - checksum) & 0xF


def create_lka_steering(packer, lat_active: bool, apply_angle: float, status: int, lka_drive_mode: int):
  # DRIVE 0 means normal D mode, 1 means B (brake) mode — echo raw value from car
  values = {
    'DRIVE': lka_drive_mode,
    'STATUS': status,
    'LXA_ACTIVATION': 1,
    'TORQUE_FACTOR': lat_active * 100,
    'SET_ANGLE': apply_angle,
  }

  return packer.make_can_msg('LANE_KEEP_ASSIST', 0, values)


def create_resume_acc(packer, counter, status, hs2_dat_mdd_cmd_452):
  hs2_dat_mdd_cmd_452['COUNTER'] = counter
  hs2_dat_mdd_cmd_452['COCKPIT_GO_ACC_REQUEST'] = status
  return packer.make_can_msg('HS2_DAT_MDD_CMD_452', 1, hs2_dat_mdd_cmd_452)


def create_drive_away_request(packer, hs2_dyn_mdd_etat_2f6):
  hs2_dyn_mdd_etat_2f6['DRIVE_AWAY_REQUEST'] = 0
  return packer.make_can_msg('HS2_DYN_MDD_ETAT_2F6', 1, hs2_dyn_mdd_etat_2f6)


# Radar, 50 Hz
def create_HS2_DYN1_MDD_ETAT_2B6(packer, frame: int, desired_decel: float, decel_active: bool, enabled: bool,
                                 gasPressed: bool, brakePressed: bool, standstill: bool, torque: int):
  # TODO: tune torque multiplier
  # TODO: check difference between GMP_POTENTIAL_WHEEL_TORQUE and GMP_WHEEL_TORQUE
  # TODO: transition from waiting to active enables torque control. For now, deactivate autohold or enable on brake pressed

  # decel_active can outlive enabled: an active deceleration request has to be ramped
  # out, not stepped, or the ESP latches a fault — see the release ramp in carcontroller.
  # While it does, ACC_STATUS must keep advertising active (4): the stock radar holds the
  # full active pattern after a driver brake press and then drops every signal in one
  # frame; sending the off pattern (2) while the decel request was still up latched the
  # ESP fault within 50 ms of the brake press, with the ramp itself running correctly.
  torque_mode = enabled and not decel_active
  values = {
    'MDD_DESIRED_DECELERATION': desired_decel, # m/s²
    'POTENTIAL_WHEEL_TORQUE_REQUEST': 2 if decel_active else (1 if enabled else 0),
    'MIN_TIME_FOR_DESIRED_GEAR': 6.2 if torque_mode else 0.0,
    'GMP_POTENTIAL_WHEEL_TORQUE': torque if torque_mode else -4000,
    'ACC_STATUS': (5 if gasPressed else 2 if brakePressed and not standstill else 4) if enabled else (4 if decel_active else 2 if brakePressed else 3),
    'GMP_WHEEL_TORQUE': torque if torque_mode else -4000,
    'WHEEL_TORQUE_REQUEST': 1 if torque_mode else 0, # TODO: test 1: high torque range 2: low torque range
    'AUTO_BRAKING_STATUS': 3, # AEB # TODO: testing ALWAYS ENABLED to resolve DTC errors if enabled else 3, # maybe disabled on too high steering angle
    'MDD_DECEL_TYPE': 1 if decel_active else 0,
    'MDD_DECEL_CONTROL_REQ': 1 if decel_active else 0,
  }

  return packer.make_can_msg('HS2_DYN1_MDD_ETAT_2B6', 1, values)


# Radar, 50 Hz
def create_HS2_DYN_MDD_ETAT_2F6(packer, decel_active: bool, lead_visible: bool, lead_distance_bars: int):
  values = {
    'TARGET_DETECTED': lead_visible,
    # 'REQUEST_TAKEOVER': 0, # TODO potential signal for HUD message from OP
    # 'BLIND_SENSOR': 0,
    # 'REQ_VISUAL_COLL_ALERT_ARC': 0,
    # 'REQ_AUDIO_COLL_ALERT_ARC': 0,
    # 'REQ_HAPTIC_COLL_ALERT_ARC': 0,
    # 'INTER_VEHICLE_DISTANCE': 255.5,#255.5, # TODO: <distance> if enabled else 255.5,
    # 'ARC_STATUS': 6,  # 12 after 50 frames (1 sec) after AUTO_BRAKING_STATUS else 6
    # 'AUTO_BRAKING_IN_PROGRESS': 0,
    # 'AEB_ENABLED': 0,
    # 'DRIVE_AWAY_REQUEST': 0, # TODO: potential RESUME request?
    'DISPLAY_INTERVEHICLE_TIME': 5.0, # TODO: <time to vehicle> if enabled else 6.2,
    'MDD_DECEL_CONTROL_REQ': decel_active,
    # 'AUTO_BRAKING_STATUS': 3, # AEB # TODO: testing ALWAYS ENABLED to resolve DTC errors if enabled else 3, # maybe disabled on too high steering angle
    'TARGET_POSITION': lead_distance_bars, # distance to lead car, far - 4, 3, 2, 1 - near
  }

  return packer.make_can_msg('HS2_DYN_MDD_ETAT_2F6', 1, values)


# TODO: do this in interface.py init()
# Disable radar ECU by setting it to programming mode
def create_disable_radar():
  addr = 0x6B6
  bus = 1
  dat = [0x02, 0x10, 0x02, 0x80]
  dat.extend([0x0] * (8 - len(dat)))

  return CanData(addr, bytes(dat), bus)


# Put the radar ECU back into the default session so it resumes broadcasting.
# Without this it stays in the programming session until the S3 timer expires,
# measured at 5.22 s after the last 0x6B6 frame, and the ESP flags ACC data
# invalid for the whole of that gap.
def create_enable_radar():
  addr = 0x6B6
  bus = 1
  dat = [0x02, 0x10, 0x01, 0x80]
  dat.extend([0x0] * (8 - len(dat)))

  return CanData(addr, bytes(dat), bus)
