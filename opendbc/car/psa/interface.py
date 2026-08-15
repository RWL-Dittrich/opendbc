import time

from opendbc.can.packer import CANPacker
from opendbc.car import Bus, DT_CTRL, structs, get_safety_config
from opendbc.car.interfaces import CarInterfaceBase
from opendbc.car.psa.carcontroller import CarController
from opendbc.car.psa.carstate import CarState
from opendbc.car.psa.psacan import create_enable_radar, create_HS2_DYN1_MDD_ETAT_2B6, create_HS2_DYN_MDD_ETAT_2F6
from opendbc.car.psa.values import DBC

TransmissionType = structs.CarParams.TransmissionType

PSA_ADAS_BUS = 1
RADAR_MSG = 0x2B6
# The radar resumed 220-230 ms after leaving the programming session in the logs, but
# that was via the S3 timeout, so allow generous margin. The handover ends as soon as
# the real 0x2B6 reappears, so this only bounds the case where it never does.
RADAR_ENABLE_TIMEOUT_FRAMES = 200  # 2.0 s


class CarInterface(CarInterfaceBase):
  CarState = CarState
  CarController = CarController

  @staticmethod
  def deinit(CP, can_recv, can_send):
    """Hand the ADAS bus back to the radar ECU that openpilot longitudinal knocks out.

    Dropping straight off the bus leaves the radar in its programming session until
    the S3 timer expires ~5 s later. The ESP (UC_FREIN) marks its ACC fields invalid
    after ~150 ms of that, which is what faults the car when alpha long is switched
    off mid-drive. So ask the radar back and keep emulating until it is transmitting.
    """
    if not CP.openpilotLongitudinalControl:
      return

    def radar_frames(packets):
      # our own emulated 0x2B6 comes back as a TX echo with bit 7 set in src, so an
      # exact bus match is the real ECU and nothing else
      return [c for p in packets for c in p if c.address == RADAR_MSG and c.src == PSA_ADAS_BUS]

    packer = CANPacker(DBC[CP.carFingerprint][Bus.pt])

    # continue the counter the control loop was using rather than restarting at 0,
    # picking it up from whichever 0x2B6 was last on the bus (ours or the radar's)
    recent = [c for p in can_recv() for c in p if c.address == RADAR_MSG]
    if recent:
      packer.counters[RADAR_MSG] = ((recent[-1].dat[7] >> 4) + 1) % 16

    can_send([create_enable_radar()])

    for frame in range(RADAR_ENABLE_TIMEOUT_FRAMES):
      if radar_frames(can_recv()):
        break
      # neutral request, the same content the emulation sends while disengaged
      if frame % 2 == 0:
        can_send([create_HS2_DYN1_MDD_ETAT_2B6(packer, frame // 2, 0.0, False, False, False, False, False, 0),
                  create_HS2_DYN_MDD_ETAT_2F6(packer, False, False, 4)])
      time.sleep(DT_CTRL)

  @staticmethod
  def _get_params(ret: structs.CarParams, candidate, fingerprint, car_fw, alpha_long, is_release, docs) -> structs.CarParams:
    ret.brand = 'psa'

    ret.safetyConfigs = [get_safety_config(structs.CarParams.SafetyModel.psa)]

    ret.dashcamOnly = False

    # measured command->wheel-angle lag is ~0.25s at corner speeds (NCC on 200s of engaged
    # driving, route 7d73189a89fc24fd/0000001a--9eab9524db). 0.35 over-led and cut apexes.
    ret.steerActuatorDelay = 0.25
    ret.steerLimitTimer = 0.1
    ret.steerAtStandstill = True

    ret.steerControlType = structs.CarParams.SteerControlType.angle
    ret.radarUnavailable = True

    ret.alphaLongitudinalAvailable = True
    ret.openpilotLongitudinalControl = alpha_long

    return ret