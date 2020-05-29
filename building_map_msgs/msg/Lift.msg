string name
string[] levels
Door[] doors
Graph wall_graph

# (ref_x, ref_y, ref_yaw) is a "reference orientation" of the lift cabin
# which can be used to align floors.
float32 ref_x
float32 ref_y
float32 ref_yaw

# width and depth of the cabin
float32 width
float32 depth