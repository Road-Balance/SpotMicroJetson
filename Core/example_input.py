from inputs import get_key
from thinputs_keyboard import ThreadedInputsKeyBoard

# while 1:
#     events = get_key()
#     for event in events:
#         print(event.ev_type, event.code, event.state)


keyboardInputs = {
                    'ABS_X': 128, 'ABS_RZ': 127, 'ABS_Y': 126, 'ABS_Z': 125,
                    'BTN_TIGGER': 124, 'BTN_THUMB': 123, 'BTN_THUMB2': 122, 'BTN_TOP': 121, # right side of gamepad
                    'ABS_HAT0X': 120, 'ABS_HAT0Y': 119, # left
                    'BTN_TOP2': 118, 'BTN_BASE': 117, # left top
                    'BTN_PINKIE': 116, 'BTN_BASE2': 115, # right top
                    'SPACE': 57,
                    'UPPER_ARROW': 200, 'RIGHT_ARROW': 205, 'DOWN_ARROW': 208, 'LEFT_ARROW': 203
                 }

keyboard = ThreadedInputsKeyBoard(keyboardInputs)
keyboard.start()

def handleKeyboard():
    # TODO: globals are bad
    global joy_x, joy_z, joy_y, joy_rz
    commandInput, commandValue = keyboard.read()
    print(commandInput)
    # Gamepad button command filter
    if commandInput == 'UPPER_ARROW':
        joy_x = commandValue
    if commandInput == 'RIGHT_ARROW':
        joy_y = commandValue
    if commandInput == 'DOWN_ARROW':
        joy_z = commandValue
    if commandInput == 'LEFT_ARROW':
        joy_rz = commandValue
    if commandInput == 'SPACE':
        pass
        # resetPose()

if __name__ == "__main__":
    
    try:
        while 1:
            handleKeyboard()
    except Exception as e:
        print(e)
    finally:
        keyboard.stop()
