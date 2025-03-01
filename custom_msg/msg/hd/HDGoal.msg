# Predefined positions of the arm
# HOME, ZERO and COBRA are predefined pose. If you want to use them, set the target = NAMED_POSE and predefined_pose = HOME for ex 

string                      PREDEFINED_POSE = "named_pose"
string                      HOME = "home"
string                      ZERO = "zero"
string                      COBRA = "cobra"


# Tasks to perform

## Switches

string                      BUTTON_1 = "switch_1"
string                      BUTTON_2 = "switch_2"
string                      BUTTON_3 = "switch_3"
string                      BUTTON_4 = "switch_4"
string                      BUTTON_5 = "switch_5"
string                      BUTTON_6 = "switch_6"
string                      BUTTON_7 = "switch_7"
string                      BUTTON_8 = "switch_8"
string                      BUTTON_9 = "switch_9"
string                      BUTTON_10 = "switch_10"
string                      BUTTON_11 = "switch_11"
string                      BUTTON_12 = "switch_12"
string                      BUTTON_13 = "switch_13"

string                      BUTTON_MAIN = "switch_main"

## Panel

# socket steps: pickup, align aruco, align socket, insert socket, pullout
# metalbar steps: find, approach, pose estimation, pickup, align aruco, go left of electromagnet, place, release, go back
# ethernet steps: find, pickup, place in tool, close tool, grab back, open tool, align aruco, detect ethernet socket (?), align, insert, release, go back
# rock steps: find, go above, request detection, go to ground, grab, lift a bit, human verif, go above container, release
# brick steps: same as rock steps until grab step
# probe steps: same as rock until grab step, place into container

# useless or outdated or unclear custom messages to change:
    # Brick
    # BrickArary
    # Ethernet
    # EthernetArray
    # Probe
    # ProbeArray
    # Rock
    # RockArray
    # ArucoObject
    

string                      VOLTMETER_COMPLETE = "voltmeter"
string                      VOLTMETER_ALIGN = "voltmeter align"
string                      ETHERNET = "ethernet"
string                      ROCK = "rocks"
string                      BRICK = "bricks"
string                      PROBE = "probes"
string                      PROBE_STORE = "probe store
string                      DROP_SAMPLE = "drop sample"


## Tools manipulation

string                      TOOL_PICKUP = "tool pickup"
string                      TOOL_PLACEBACK = "tool place back"
# available tools
string                      SHOVEL_TOOL = "shovel"
string                      SOCKET_TOOL = "socket"
string                      BUTTON_TOOL = "buttons"


# Functionalities

string                      NEXT = "next"
string                      PREVIOUS = "previous"

string                      ABORT = "abort"
string                      NONE = "none"


# ERROR_CODES

string                      OK = "ok"
string                      ALREADY_HAS_GOAL = "already has goal"
string                      UNKNOWN_GOAL = "unknown goal"
string                      UNDEFINED = "undefined"




# MESSAGE DATA
string                      target              NONE
string NAMED_POSE = "named_pose"

# additional data for describing target
bool                        is_button_top       True        # random default value
string                      predefined_pose     "undefined"
string                      tool                "undefined"

string                      TOP_GRAB = "top grab"
string                      SIDE_GRAB = "side grab"
string                      probe_grab_option   "undefined"
uint8                       probe_station        0