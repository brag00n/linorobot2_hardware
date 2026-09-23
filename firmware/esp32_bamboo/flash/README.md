# Transit des binaires de flash (ESP32 General Driver)

Le flash se fait **depuis un conteneur** sur le RPi, pour ne rien installer sur l'hote
dietpi (ni `esptool` ni `pip`). Voir `linorobot2/docker/flash_esp32.sh` et le service
compose `flash.esp32` (profil `tools`, donc jamais demarre par `up`).

Cycle complet, depuis le poste de developpement :

```bash
# 1. construire (l'env n'est PAS celui de default_envs)
cd firmware/esp32_bamboo && pio run -e bamboov3-wirshare_bamboo_mavlink

# 2. pousser le binaire (il n'est pas versionne, et rsync exclut .pio/)
scp .pio/build/bamboov3-wirshare_bamboo_mavlink/firmware.bin \
    dietpi@<rpi>:/home/dietpi/prj_robotique/Bamboo4WD_V4/linorobot2_hardware/firmware/esp32_bamboo/flash/

# 3. flasher (sur le RPi, dans linorobot2/docker/)
docker compose stop driver.real
docker compose --profile tools run --rm flash.esp32
docker compose up -d driver.real
```

`backup/` recoit la relecture integrale de la flash (4 MiB) faite **avant** chaque
ecriture : c'est le seul chemin de retour arriere si le microcode ecrit ne demarre pas.
Restauration :

```bash
esptool.py --port /dev/esp32 --baud 115200 write_flash 0 backup/firmware_<horodatage>.bin
```
