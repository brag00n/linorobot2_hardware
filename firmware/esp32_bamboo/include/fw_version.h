#ifndef FW_VERSION_H
#define FW_VERSION_H

// =====================================================================
//  Identite du microcode de la carte WaveShare "General Driver for
//  Robots" (module ESP32-WROOM-32UE).
//
//  Forme canonique : "<MCU>_bamboo vX.Y.Z", ici
//      ESP32-WROOM-32UE_bamboo v0.1.0
//
//  Cette chaine est RETOURNEE par le microcode, elle n'est pas
//  seulement documentaire : voir ConnectorMavlink::sendVersion()
//  (STATUSTEXT + AUTOPILOT_VERSION, en reponse a
//  MAV_CMD_REQUEST_AUTOPILOT_CAPABILITIES / MAV_CMD_REQUEST_MESSAGE, et
//  emise une fois au demarrage). C'est le seul identifiant que l'hote
//  doit citer pour designer une revision de microcode.
//
//  Regle de version (semantique, decidee ici) :
//    MAJOR : rupture du contrat de fil (ordre des index PARAM, ajout ou
//            retrait d'un message du dialecte, changement de sysid).
//    MINOR : fonction nouvelle compatible (ex. ecriture SRAM des
//            parametres de geometrie, gains PID par moteur).
//    PATCH : correction sans effet sur le contrat de fil.
//
//  Historique :
//    0.1.0  identite du microcode ; MAVLink v2 (dialecte bamboo) ;
//           telemetrie complete ; parametres en LECTURE seule
//           (les index 15-18 echotent sans ecrire) ; pas de
//           persistance ; gains PID communs aux 4 moteurs.
//    0.2.0  ecriture SRAM des parametres de geometrie (index 15-18 :
//           WHEEL_CPR / WHEEL_CIRC / WHEEL_APB / CAR_TYPE), appliquee a
//           chaud a Kinematics et aux encodeurs ; correction du callback
//           PID de la voie de controle (les gains etaient inertes en
//           MAVLink) ; journal de la carte en STATUSTEXT avec seuil de
//           severite reglable par le parametre LOG_LEVEL (index 19,
//           ajoute EN FIN de table : le contrat 0-18 est intact) ;
//           amorcages alignes sur les valeurs constructeur WaveShare.
//           Toujours aucune persistance : le driver repousse la
//           configuration a chaque connexion.
// =====================================================================

#define FW_BOARD_NAME    "ESP32-WROOM-32UE_bamboo"

#define FW_VERSION_MAJOR 0
#define FW_VERSION_MINOR 2
#define FW_VERSION_PATCH 0

#define FW_STR_(x) #x
#define FW_STR(x)  FW_STR_(x)

// "0.1.0"
#define FW_VERSION_STR   FW_STR(FW_VERSION_MAJOR) "." FW_STR(FW_VERSION_MINOR) "." FW_STR(FW_VERSION_PATCH)

// "ESP32-WROOM-32UE_bamboo v0.1.0" (<= 50 caracteres : tient dans un
// champ texte STATUSTEXT sans decoupage en chunks).
#define FW_IDENT_STR     FW_BOARD_NAME " v" FW_VERSION_STR

// Version empaquetee pour AUTOPILOT_VERSION.flight_sw_version :
// (major << 24) | (minor << 16) | (patch << 8), convention MAVLink.
#define FW_VERSION_PACKED (((uint32_t)FW_VERSION_MAJOR << 24) | \
                           ((uint32_t)FW_VERSION_MINOR << 16) | \
                           ((uint32_t)FW_VERSION_PATCH << 8))

#endif // FW_VERSION_H
