# Number of reported motor's id have failure,Current (tap esc) we support 8 ESCs
# motor_state bit(denote motor number):         |  15  |  14  |  13  |  12  | ... |  1  |  0  |
# vaule(1:motor is error 0:motor is normal):    |  1|0 |  1|0 |  1|0 |  1|0 | 1|0 | 1|0 | 1|0 |
uint16 motor_state
float32 delta_pwm