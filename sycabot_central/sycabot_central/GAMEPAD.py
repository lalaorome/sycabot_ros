#  Here is an example to use the gamepad :
# 
# 
# from evdev import InputDevice 
# import GAMEPAD
# 
# 
# for event in gamepad.read_loop():
#     if event.type != 0 :
#         if event.code == GAMEPAD.JOYSTICK_LEFT_UP_DOWN:
#             if event.value > GAMEPAD.JOYSTICK_MID_UDOWN + GAMEPAD.JOYSTICK_DEADZONE:
#                 print('left : backward')
#             if event.value < GAMEPAD.JOYSTICK_MID_UDOWN - GAMEPAD.JOYSTICK_DEADZONE:
#                 print('left : forward')
#         if event.code == GAMEPAD.JOYSTICK_LEFT_RIGHT_LEFT:
#             if event.value > GAMEPAD.JOYSTICK_MID_RLEFT + GAMEPAD.JOYSTICK_DEADZONE:
#                 print('left : right')
#             if event.value < GAMEPAD.JOYSTICK_MID_RLEFT - GAMEPAD.JOYSTICK_DEADZONE:
#                 print('left : left')
#         if event.code == GAMEPAD.JOYSTICK_RIGHT_UP_DOWN:
#             if event.value > GAMEPAD.JOYSTICK_MID_UDOWN + GAMEPAD.JOYSTICK_DEADZONE:
#                 print('right : backward')
#             if event.value < GAMEPAD.JOYSTICK_MID_UDOWN - GAMEPAD.JOYSTICK_DEADZONE:
#                 print('right : forward')
#         if event.code == GAMEPAD.JOYSTICK_RIGHT_RIGHT_LEFT:
#             if event.value > GAMEPAD.JOYSTICK_MID_RLEFT + GAMEPAD.JOYSTICK_DEADZONE:
#                 print('right : right')
#             if event.value < GAMEPAD.JOYSTICK_MID_RLEFT - GAMEPAD.JOYSTICK_DEADZONE:
#                 print('right : left')
#         if event.code == GAMEPAD.BUTTON_A :
#             if event.value == GAMEPAD.BUTTON_DOWN :
#                 print('Button A')
#         if event.code == GAMEPAD.BUTTON_X :
#             if event.value == GAMEPAD.BUTTON_DOWN :
#                 print('Button X')
#         if event.code == GAMEPAD.BUTTON_B :
#             if event.value == GAMEPAD.BUTTON_DOWN :
#                 print('Button B')
#                 break
#         if event.code == GAMEPAD.BUTTON_Y :
#             if event.value == GAMEPAD.BUTTON_DOWN :
#                 print('Button Y')
#         if event.code == GAMEPAD.BUTTON_R1 :
#             if event.value == GAMEPAD.BUTTON_DOWN :
#                 print('Button R1')
#         if event.code == GAMEPAD.BUTTON_R2 :
#             if event.value == GAMEPAD.BUTTON_DOWN :
#                 print('Button R2')
#         if event.code == GAMEPAD.BUTTON_L1 :
#             if event.value == GAMEPAD.BUTTON_DOWN :
#                 print('Button L1')
#         if event.code == GAMEPAD.BUTTON_L2 :
#             if event.value == GAMEPAD.BUTTON_DOWN :
#                 print('Button L2')



# Event type code
JOYSTICK_LEFT_UP_DOWN = 1
JOYSTICK_LEFT_RIGHT_LEFT = 0

JOYSTICK_RIGHT_UP_DOWN = 5
JOYSTICK_RIGHT_RIGHT_LEFT = 2

ARROW_RIGHT_LEFT = 16
ARROW_UP_DOWN = 17

BUTTON_A = 306
BUTTON_B = 305
BUTTON_X = 307
BUTTON_Y = 304
BUTTON_R1 = 309
BUTTON_R2 = 311
BUTTON_L1 = 308
BUTTON_L2 = 310
BUTTON_START = 313
BUTTON_SELECT = 312

# Event value
JOYSTICK_UP_MIN = 0
JOYSTICK_DOWN_MAX = 255
JOYSTICK_RIGHT_MAX = 255
JOYSTICK_LEFT_MIN = 0
JOYSTICK_MID_UDOWN = 128
JOYSTICK_MID_RLEFT = 127
JOYSTICK_DEADZONE = 10
BUTTON_DOWN = 1
BUTTON_UP = 0
ARROW_UP_LEFT_DOWN = -1
ARROW_DOWN_RIGHT_DOWN = 1
ARROW_RELEASED = 0

if __name__ == '__main__':
    raise NotImplementedError("GAMEPAD is a macro file, it should not be launched directly ...")