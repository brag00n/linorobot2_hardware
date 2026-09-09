r"""roslite - micro-framework « ROS2 en process » (Node / Port / Executor).

Expose les 3 primitives du banc d'essai : Node et Port (node.py), Executor
(executor.py). Voir les docstrings de module pour le modele de transport
(files profondeur 1, boucle sequentielle set/process/get).
"""
from .node import Node, Port
from .executor import Executor

__all__ = ["Node", "Port", "Executor"]
