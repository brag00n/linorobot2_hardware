"""modules.tracking - Suivi de visage par camera motorisee.

  RobotWebCamMotorized  subsystem : compose device.sensor (webcam),
                        interaction.FaceDetection (detection + tracker) et
                        lib.KalmanPredictor (prediction de trajectoire), et
                        asservit device.motion.RobotServoMotor (pan/tilt).
"""
