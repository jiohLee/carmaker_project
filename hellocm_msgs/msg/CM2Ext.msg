#
# Message from CarMaker ROS Node to external ROS Node
#

# General
Header  header                                 # General ROS Header (optional)
time    time                                   # ROS time when message was sent (optional)
int64   cycleno                                # Cycle number since simulation start (optional)
float64 synthdelay                             # Synthetic delay to check synchronization (optional)

CarMaker_Out CMOut			#폴더 내 carmakerout.msg

float64 ego_GK_x		#World coordinate x (GK = Gauss–Kruger)
float64 ego_GK_y		#World coordinate y

RoadSensorPath[] RoadPath 	#9WEEK
CameraSensor camearaSensor
	
float64 ego_orientation         # rad	

