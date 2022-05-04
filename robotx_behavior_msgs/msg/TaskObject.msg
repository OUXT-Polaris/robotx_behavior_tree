uint8 BUOY_RED = 1
uint8 BUOY_GREEN = 2
uint8 BUOY_WHITE = 3
uint8 BUOY_BLACK = 4
uint8 FIELD_BOUNDARY_MARKER = 5 // orange bouy
uint8 OBSTACLE = 6              // black round bouy

uint8 MARINE_CREATURE = 41
uint8 PLATYPUS = 42
uint8 CROCODILE = 43
uint8 TURTLE = 44

uint8 LIGHT_TOWER = 51          // tower to display color pattorn

uint8 FLOATING_DOCK = 61        // floating dock to dock
uint8 FLAT_PANEL_RED = 62
uint8 FLAT_PANEL_GREEN = 63
uint8 FLAT_PANEL_BLUE = 64   

uint8 FLOATING_PANEL = 71   // Floating dock with colored panels and holes on Task 7 
uint8 HOLE_L = 72           // larger hole into which throw boll
uint8 HOLE_S = 73           // smaller hole

uint8 HELIPAD = 81          // UAV launch and landing port

uint8 OBJECT_N = 91         // Task9 Search and Report N
uint8 OBJECT_R = 92         // R

uint8 object_kind
uint32 unique_id
float32 x
float32 y
float32[<=1] z
float32[<=1] theta 
bool detected
builtin_interfaces/Time last_detected
float32 reliability