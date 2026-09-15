r"""nodes - les 4 nodes du banc d'essai (enveloppent les couches de robot_control).

CameraNode (capteur) -> TrackingNode (traitement) -> ServoNode (actionneur) ->
BoardNode (liaison serie). Chacun compose une classe de couche eprouvee de
robot_control (import) et n'ajoute que l'interface ROS (Ports + process()).
"""
from .CameraNode import CameraNode
from .TrackingNode import TrackingNode
from .ServoNode import ServoNode
from .BoardNode import BoardNode
from .WSEsp32Node import WSEsp32Node
from .TeensyNode import TeensyNode
from .GrovePiNode import GrovePiNode
from .FaceRecogNode import FaceRecogNode
from .FaceTrainNode import FaceTrainNode

__all__ = ["CameraNode", "TrackingNode", "ServoNode", "BoardNode",
           "WSEsp32Node", "TeensyNode", "GrovePiNode", "FaceRecogNode", "FaceTrainNode"]
