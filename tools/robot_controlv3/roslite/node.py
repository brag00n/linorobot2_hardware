r"""roslite.node - primitives Node / Port (mime rclpy, en process).

Banc d'essai « nodes ROS2 » : on isole chaque traitement sous forme de Node a
interface standard, testable SANS queue ni environnement ROS, pour qu'un node
soit ensuite implementable presque tel quel sous rclpy (« swap import, corps du
node inchange »).

Modele de transport (decide avec l'utilisateur) : un Port represente la donnee
unitaire d'une file ROS de PROFONDEUR 1. L'executeur (voir executor.py) fait,
pour chaque node dans l'ordre de la pipeline :
  - set   des entrees   (= une entree de file ROS)      -> Port.set(msg)
  - appel process()      (= le callback du node ROS)
  - get   des sorties    (= une sortie de file ROS)      -> Port.get()

Le corps de process() ne lit/ecrit QUE ses Ports (jamais le bus de l'executeur) :
c'est ce qui le rend portable tel quel sous ROS2. La frontiere set/get est
exactement la ou un vrai thread+file (ou un topic DDS) se glissera plus tard
sans toucher les process().
"""


class Port:
    """Emplacement de file ROS de profondeur 1 (semantique KEEP_LAST(1)).

    `set(msg)` ecrit la donnee (ecrase la precedente : profondeur 1) et la marque
    fraiche ; `get()` la relit. `fresh` indique qu'une valeur a ete produite/livree
    depuis le dernier acquittement (`clear()`), ce qui permet a l'executeur de ne
    latcher dans le bus que les sorties reellement publiees ce tour.
    """

    def __init__(self, topic, msg_type):
        self.topic = topic
        self.msg_type = msg_type
        self._msg = None
        self.fresh = False

    def set(self, msg):
        """Entree de file : ecrit la donnee unitaire (profondeur 1, ecrase)."""
        self._msg = msg
        self.fresh = True

    def get(self):
        """Sortie de file : relit la derniere donnee (None si aucune)."""
        return self._msg

    def clear(self):
        """Acquitte la donnee (consommee) : la valeur reste, `fresh` retombe."""
        self.fresh = False


class Node:
    """Base d'un node ROS simule : declare ses Ports d'entree/sortie et process().

    A surcharger :
      - process()   = le callback du node (traite les entrees -> remplit les sorties) ;
      - on_start()  = ouverture des ressources (camera, serie...) au demarrage ;
      - on_stop()   = liberation propre a l'arret.
    """

    def __init__(self, name):
        self.name = name
        self.inputs = {}          # topic -> Port (souscriptions)
        self.outputs = {}         # topic -> Port (publications)

    def create_input(self, topic, msg_type):
        """Declare une souscription (= file d'entree ROS). Retourne le Port."""
        p = Port(topic, msg_type)
        self.inputs[topic] = p
        return p

    def create_output(self, topic, msg_type):
        """Declare une publication (= file de sortie ROS). Retourne le Port."""
        p = Port(topic, msg_type)
        self.outputs[topic] = p
        return p

    # --- cycle de vie (surchargeables) --------------------------------------
    def on_start(self):
        """Appele une fois par Executor.start() (ouverture ressources)."""

    def process(self):
        """Callback du node : a SURCHARGER. Lit self.inputs, remplit self.outputs."""

    def on_stop(self):
        """Appele une fois par Executor.stop() (liberation ressources)."""
