string name

# CONVENTIONS
# ===========
# single hinge doors:
#   * hinge is located at (v1_x, v1_y)
#   * door extends till (v2_x, v2_y)
#   * motion_range = door swing range in DEGREES
#   * there are two possible motions: clockwise and anti-clockwise
#     selected by the motion_direction parameter, which is +1 or -1
#
# double hinge doors:
#   * hinges are located at both (v1_x, v1_y) and (v2_x, v2_y)
#   * motion range = door swing ranges in DEGREES (assume symmetric)
#   * same motion-direction selection as single hinge
#
# single sliding doors:
#   * the door slides from (v2_x, v2_y) towards (v1_x, v1_y)
#   * range of motion is entire distance from v2->v1. No need to specify.
#
# double sliding doors:
#   * door panels slide from the centerpoint of v1<->v2 towards v1 and v2
#
# single/double telescoping doors:
#   * common in elevators; same parameters as sliding doors; they just
#     open/close faster and take up less space inside the wall.

float32 v1_x
float32 v1_y

float32 v2_x
float32 v2_y

uint8 door_type
uint8 DOOR_TYPE_UNDEFINED=0
uint8 DOOR_TYPE_SINGLE_SLIDING=1
uint8 DOOR_TYPE_DOUBLE_SLIDING=2
uint8 DOOR_TYPE_SINGLE_TELESCOPE=3
uint8 DOOR_TYPE_DOUBLE_TELESCOPE=4
uint8 DOOR_TYPE_SINGLE_SWING=5
uint8 DOOR_TYPE_DOUBLE_SWING=6

float32 motion_range
int32 motion_direction
