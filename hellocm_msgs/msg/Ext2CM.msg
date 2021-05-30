#
# Message from external ROS Node to CarMaker ROS Node
#

# General
Header  header                                 # General ROS Header (optional)
time    time                                   # ROS time when message was sent (optional)
int64   cycleno                                # Cycle number since simulation start (optional)


float64 SteeringWheel	# 
float64 Ax		# 가속도
float64 GearNo		# 기어
float64 VC_SwitchOn	# 자율주행 모드 스위치
float64 Light_Indicator	# 방향지시등
float64 Light_Hazard	# 비상등
