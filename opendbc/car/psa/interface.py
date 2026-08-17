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

    # The planner leads corners by lagd's liveDelay, whose prior is steerActuatorDelay
    # + 0.2 and which only learns above 80 km/h (MIN_VEGO), so on this car the prior
    # dominates. Measured full-chain delay (desired curvature -> yaw, NCC over 550 s of
    # clean engaged driving, angle-limiter frames excluded) peaks at ~0.30 s; the EPS
    # alone is ~0.15 s command->wheel. 0.10 + 0.2 puts the prior on the measured value.
    # Earlier values over-led and cut corner apexes: 0.35 badly, 0.25 still by ~0.12 s.
    # Naive NCC on this car overstates the lag: the angle limiter couples the command to
    # the measured angle in the same frame, which is where the old 0.25 figure came from.
    ret.steerActuatorDelay = 0.10
    ret.steerLimitTimer = 0.1
    ret.steerAtStandstill = True

    ret.steerControlType = structs.CarParams.SteerControlType.angle
    ret.radarUnavailable = True

    ret.alphaLongitudinalAvailable = True
    ret.openpilotLongitudinalControl = alpha_long

    # the accel->torque map in carcontroller is pure feedforward and speed-blind, so any
    # unmodeled load (aero drag above ~15 m/s, grade residual, headwind) becomes a
    # permanent steady-state error: measured -0.15 m/s² at 80 km/h on flat ground, the
    # car sagging 2-3 km/h under the set speed with no lead. The integrator trims that
    # out. ki only — kp would fight the map's slope errors frame-by-frame.
    ret.longitudinalTuning.kiBP = [0.]
    ret.longitudinalTuning.kiV = [0.5]

    return ret