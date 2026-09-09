r"""roslite.executor - executeur+middleware mono-thread (mime rclpy.SingleThreadedExecutor).

Tient l'ORDRE de la pipeline (liste de nodes) et un `bus` : dict topic -> derniere
valeur publiee (l'equivalent des files ROS profondeur 1, cote middleware). Une
seule boucle sequentielle, pas de thread : c'est le point de depart « simple »
demande par l'utilisateur (on parallelisera plus tard en glissant un thread
derriere le Port, sans toucher les process()).

spin_once(), pour chaque node dans l'ordre :
  1. SET     : livre bus[topic] dans chaque Port d'entree du node ;
  2. PROCESS : appelle node.process() (le callback) ;
  3. GET     : latche chaque Port de sortie fraiche dans le bus.

Le Core (hote) peut injecter des messages « de l'exterieur » via publish() : c'est
l'equivalent d'un publisher cote Core (clavier -> /tracking/config, /servo/cmd...).
"""


class Executor:
    """Executeur sequentiel : spin_once() fait tourner tous les nodes une fois."""

    def __init__(self):
        self.nodes = []           # ordre de la pipeline
        self.bus = {}             # topic -> derniere valeur (files profondeur 1)

    def add_node(self, node):
        """Ajoute un node en fin de pipeline (l'ordre = l'ordre d'appel)."""
        self.nodes.append(node)
        return node

    def publish(self, topic, msg):
        """Injecte un message dans le bus (publisher cote Core : clavier, MCP...)."""
        self.bus[topic] = msg

    def latest(self, topic, default=None):
        """Derniere valeur publiee sur un topic (pour l'HMI/affichage du Core)."""
        return self.bus.get(topic, default)

    def start(self):
        """Ouvre les ressources de tous les nodes (on_start), dans l'ordre."""
        for node in self.nodes:
            node.on_start()
        return self

    def spin_once(self):
        """Un tour de pipeline : pour chaque node, set entrees -> process -> get sorties."""
        for node in self.nodes:
            # 1) SET : livrer les dernieres valeurs du bus dans les Ports d'entree
            for topic, port in node.inputs.items():
                if topic in self.bus:
                    port.set(self.bus[topic])
            # 2) PROCESS : le callback du node traite ses entrees
            node.process()
            # 3) GET : latcher les sorties reellement publiees dans le bus
            for topic, port in node.outputs.items():
                if port.fresh:
                    self.bus[topic] = port.get()
                    port.clear()

    def stop(self):
        """Libere les ressources de tous les nodes (on_stop), meme en cas d'erreur."""
        for node in self.nodes:
            try:
                node.on_stop()
            except Exception:
                pass
