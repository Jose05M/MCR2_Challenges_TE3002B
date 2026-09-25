# Motion primitive requested by the behavior FSM and executed by the motion controller.
# DRIVE and ROTATE report completion on /motion_done.

uint8 FOLLOW_LINE=0
uint8 STOP=1
uint8 DRIVE=2
uint8 ROTATE=3

uint8 mode

float32 linear_scale   # FOLLOW_LINE: linear speed multiplier (1.0 = nominal)
float32 angular_scale  # FOLLOW_LINE: angular speed multiplier (1.0 = nominal)
float32 distance       # DRIVE: distance to travel [m]
float32 speed          # DRIVE: linear speed [m/s]
float32 angle          # ROTATE: relative heading change [rad], + = left
