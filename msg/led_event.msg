# RGB LED modes definitions
uint8 LED_MODE_OFF = 0
uint8 LED_MODE_ON = 1
uint8 LED_MODE_BLINK_SLOW = 2
uint8 LED_MODE_BLINK_NORMAL = 3
uint8 LED_MODE_BLINK_FAST = 4
uint8 LED_MODE_BREATHE = 5
uint8 LED_MODE_PATTERN = 6

uint8 enabled  # 0 / 255
uint8 prio     # 0-255
uint32 duration # in ms
uint16[8] color		# tap led value
uint8[8] mode
