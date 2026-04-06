# Exploration state shared between frontier explorer and task manager

uint8 IDLE=0
uint8 RUNNING=1
uint8 STOPPED=2
uint8 COMPLETED=3
uint8 STUCK=4

builtin_interfaces/Time stamp
uint8 state
string detail
