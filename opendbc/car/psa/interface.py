from opendbc.car import structs, get_safety_config
from opendbc.car.interfaces import CarInterfaceBase
from opendbc.car.psa.carcontroller import CarController
from opendbc.car.psa.carstate import CarState

TransmissionType = structs.CarParams.TransmissionType


class CarInterface(CarInterfaceBase):
  CarState = CarState
  CarController = CarController

  def release_ecus(self) -> bool:
    # hand the ADAS bus back to the radar ECU openpilot longitudinal knocks out
    return self.CC.release_radar()

  @staticmethod
  def _get_params(ret: structs.CarParams, candidate, fingerprint, car_fw, alpha_long, is_release, docs) -> structs.CarParams:
    ret.brand = 'psa'

    ret.safetyConfigs = [get_safety_config(structs.CarParams.SafetyModel.psa)]

    ret.dashcamOnly = False

    # measured command->wheel-angle lag is ~0.25s at corner speeds (NCC over 200s of
    # engaged driving). 0.35 over-led and cut apexes.
    ret.steerActuatorDelay = 0.25
    ret.steerLimitTimer = 0.1
    ret.steerAtStandstill = True

    ret.steerControlType = structs.CarParams.SteerControlType.angle
    ret.radarUnavailable = True

    ret.alphaLongitudinalAvailable = True
    ret.openpilotLongitudinalControl = alpha_long

    return ret