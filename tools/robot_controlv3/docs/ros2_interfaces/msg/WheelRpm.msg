# Vitesse de rotation par roue, tours/MINUTE (= tics/s / cpr * 60), M1..M4.
# Complete la telemetrie STM32 eclatee (BatteryState + Imu + TwistStamped).
# Alternative simple : std_msgs/Float32MultiArray -- ce type dedie est plus lisible.

std_msgs/Header header    # stamp = ts horloge interne carte (trame encodeurs 0x0D)
float32[4] rpm            # [M1, M2, M3, M4]
