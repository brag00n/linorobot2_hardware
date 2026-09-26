r"""mcp - Serveurs MCP (Model Context Protocol) de robot_control.

Deux serveurs stdio a role net, pair des autres couches (communication/, device/,
modules/, lib/, interaction/) :

  analysis  serveur 1 : ANALYSE, strictement lecture seule des logs de l'app.
  action    serveur 2 : ACTION carte + config suivi ; relais via l'app quand elle
            tourne (unique proprietaire de COM4), repli COM4 direct quand elle est
            arretee.

Partage :
  gateway   protocole socket loopback app<->MCP (CommandServer + send_command) et
            handle_command() (executeur de commandes carte commun aux deux chemins).
  _rpc      boucle JSON-RPC 2.0 stdio + acces lecture seule aux fichiers de logs.

DEUX SERVEURS SONT PARTIS le 2026-09-26 (lot 3.0), et ce n'est pas une perte : `ros2.py`
(ros2-analysis) et `orchestrator.py` (ros2-orchestrator) vivent desormais dans
`linorobot2/tools/ros_control/mcp/`, avec le nouveau `ros2_action.py`. Motif : ils ne
parlent qu'au graphe ROS et aux services du `docker-compose.yaml` de l'AUTRE depot ; les
garder ici faisait qu'une allow-list et un compose pouvaient diverger sans qu'un seul
commit les voie tous les deux. Ils n'utilisaient de `_rpc` que `make_log` et `serve`, d'ou
une copie minimale la-bas plutot qu'un import croise par PYTHONPATH -- qui aurait casse les
serveurs ROS des que ce depot n'est pas a cote. Le `.mcp.json` de ce depot reste le SEUL du
projet et porte les cinq entrees : Claude Code lit celui du dossier de lancement.
"""
