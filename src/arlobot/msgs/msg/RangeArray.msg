# Note, this is an extension to the Range() message
# It supports an array of ranges in a single message
# The index to the array maps to a particular sensor
uint8 ULTRASOUND=0
uint8 INFRARED=1
std_msgs/Header header
uint8 radiation_type
float32 field_of_view
float32 min_range
float32 max_range
float32[] ranges