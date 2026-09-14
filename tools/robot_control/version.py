r"""Version applicative unique (app teleop + suivi + reconnaissance).

A INCREMENTER a chaque changement de code notable : la version est ecrite dans
state.json par la telemetrie, affichee sur le HUD de l'app et lisible via MCP
(robot-analysis `status`). Sert de repere pour verifier qu'une app en cours
execute bien le dernier code (evite les faux diagnostics « code non recharge »).

Format : MAJEUR.MINEUR.CORRECTIF (SemVer souple, sans signification stricte).
"""

APP_VERSION = "3.6.3"
