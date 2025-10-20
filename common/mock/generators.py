from cereal import messaging


LOCATION1 = (32.7174, -117.16277)
LOCATION2 = (32.7558, -117.2037)

LLK_DECIMATION = 10
RENDER_FRAMES = 15
DEFAULT_ITERATIONS = RENDER_FRAMES * LLK_DECIMATION


def generate_liveLocationKalman(location=LOCATION1):
  msg = messaging.new_message('liveLocationKalman')
  msg.liveLocationKalman.positionGeodetic = {'value': [*location, 0], 'std': [0., 0., 0.], 'valid': True}
  msg.liveLocationKalman.positionECEF = {'value': [0., 0., 0.], 'std': [0., 0., 0.], 'valid': True}
  msg.liveLocationKalman.calibratedOrientationNED = {'value': [0., 0., 0.], 'std': [0., 0., 0.], 'valid': True}
  msg.liveLocationKalman.velocityCalibrated = {'value': [0., 0., 0.], 'std': [0., 0., 0.], 'valid': True}
  msg.liveLocationKalman.status = 'valid'
  msg.liveLocationKalman.gpsOK = True
  return msg


def generate_livePose():
  msg = messaging.new_message('livePose')
  meas = {'x': 0.0, 'y': 0.0, 'z': 0.0, 'xStd': 0.0, 'yStd': 0.0, 'zStd': 0.0, 'valid': True}
  msg.livePose.orientationNED = meas
  msg.livePose.velocityDevice = meas
  msg.livePose.angularVelocityDevice = meas
  msg.livePose.accelerationDevice = meas
  msg.livePose.inputsOK = True
  msg.livePose.posenetOK = True
  msg.livePose.sensorsOK = True
  return msg
