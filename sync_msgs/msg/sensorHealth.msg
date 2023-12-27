# ROS message for sensor health

string name
uint8 status
float32 rate

# Sensor status enum
uint8 UNKNOWN = 0
uint8 HEALTHY = 1
uint8 UNHEALTHY = 2
uint8 MISSING = 3
