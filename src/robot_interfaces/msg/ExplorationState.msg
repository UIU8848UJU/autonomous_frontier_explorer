# Frontier 策略节点与 task_manager 共享的探索状态

uint8 IDLE=0
uint8 RUNNING=1
uint8 STOPPED=2
uint8 COMPLETED=3
uint8 STUCK=4
uint8 FAILED=5

builtin_interfaces/Time stamp
uint8 state
string detail
