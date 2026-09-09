r"""robot_controlv3 - banc d'essai « nodes ROS2 » (framework roslite sequentiel).

Refonte de robot_control en NODES simules pour preparer la migration ROS2 :
chaque traitement (camera, suivi, servo, carte) est isole derriere une interface
standard (roslite.Node) testable sans queue ni environnement ROS. Le Core
(RobotMain) n'execute plus le traitement, il orchestre les nodes via un executeur
sequentiel (roslite.Executor) et fournit les services HMI/clavier/MCP/log.

Les ALGORITHMES ne sont pas reecrits : les nodes composent, PAR IMPORT, les classes
de couche eprouvees de robot_control (detection, Kalman, slew servo, mix drive,
protocole serie). robot_control et robot_control/old restent intacts et lancables.

Entry point :  python -m robot_controlv3.RobotMain  (memes flags que robot_control)
"""
