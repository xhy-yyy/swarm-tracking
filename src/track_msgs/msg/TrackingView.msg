# Associated image observation for the currently assigned tracking aircraft.
# stamp is the RGB image epoch, frame_id is world. No timer republishing.
std_msgs/Header header
string assignment
uint8 drone_id
float64 bearing
float64 horizontal_error_px
float64 confidence
