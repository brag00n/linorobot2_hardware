"""interaction - Traitements de perception (au-dessus de device.sensor).

  FaceDetection  detecteurs de visage commutables (haar/dnn/yunet) + tracker
                 visuel (mil/vit) et machine detect-then-track. Travaille sur
                 une image reduite (coords 'small') ; le remap pleine resolution,
                 la normalisation et la prediction sont dans le subsystem
                 modules.tracking.RobotWebCamMotorized.
"""
