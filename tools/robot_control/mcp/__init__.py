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
"""
