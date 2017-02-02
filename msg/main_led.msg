# RGB LED color definitions
uint8 LED_COLOR_OFF = 0
uint8 LED_COLOR_RED = 1
uint8 LED_COLOR_YELLOW = 2
uint8 LED_COLOR_PURPLE = 3
uint8 LED_COLOR_GREEN = 4
uint8 LED_COLOR_BLUE = 5
uint8 LED_COLOR_WHITE = 6
uint8 LED_COLOR_AMBER = 7
uint8 LED_COLOR_DIM_RED = 8
uint8 LED_COLOR_DIM_YELLOW = 9
uint8 LED_COLOR_DIM_PURPLE = 10
uint8 LED_COLOR_DIM_GREEN = 11
uint8 LED_COLOR_DIM_BLUE = 12
uint8 LED_COLOR_DIM_WHITE = 13
uint8 LED_COLOR_DIM_AMBER = 14

# RGB LED modes definitions
uint8 LED_MODE_OFF = 0
uint8 LED_MODE_ON = 1
uint8 LED_MODE_BLINK_SLOW = 2
uint8 LED_MODE_BLINK_NORMAL = 3
uint8 LED_MODE_BLINK_FAST = 4
uint8 LED_MODE_BREATHE = 5
uint8 LED_MODE_PATTERN = 6


uint8 color # color of the main LED
uint8 mode  # mode of the main LED
# TODO: is the pattern definition necessary?
# TODO: implement definition for the array size
# uint8[20] pattern_colors
# uint8[20] pattern_duration
