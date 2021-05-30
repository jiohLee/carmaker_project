# maken by dk
# Message from CarMaker ROS Node to external ROS Node
#

# General
Header  header                                 # General ROS Header (optional)
time    time                                   # ROS time when message was sent (optional)
int64   cycleno                                # Cycle number since simulation start (optional)
float64 synthdelay                             # Synthetic delay to check synchronization (optional)

int16 status                                # 0: Off / 1: Green / 2: Yello / 3: Red / 4: RedYellow
float64 distance                            # Distance [m]
