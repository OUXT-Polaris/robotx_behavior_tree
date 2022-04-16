uint8 BUOY_RED = 1
uint8 BUOY_GREEN = 2
uint8 BUOY_WHITE = 3
uint8 BUOY_BLACK = 4
uint8 FIELD_BOUNDARY_MARKER = 5
uint8 OBSTACLE = 6

uint8 MARINE_CREATURE = 41

uint8 LIGHT_TOWER = 51
uint8 PLATYPUS = 52
uint8 CROCODILE = 53
uint8 TURTLE = 54

uint8 FLOATING_DOCK = 61
uint8 FLAT_PANEL_RED = 62
uint8 FLAT_PANEL_GREEN = 63
uint8 FLAT_PANEL_BLUE = 64

uint8 FLOATING_PANEL = 71
uint8 HOLE_L = 72
uint8 HOLE_S = 73

uint8 HELIPAD = 81

uint8 OBJECT_N = 91
uint8 OBJECT_R = 92

uint8 object_kind
uint32 unique_id
float32 x
float32 y
float32[<=1] z
float32[<=1] theta 
bool detected
builtin_interfaces/Time last_detected
float32 reliability