'''
Multiprocess Keyboard Interrupt Handler
You can get Keyboard inputs while running 
another endless Loops 
'''

import keyboard
from multiprocessing import Process, Queue


# keyboard Initialisation
# Dictionary of keyboard controller buttons we want to include.
key_value_default = {'w': 0, 'a': 0, 's': 0, 'd': 0, 'q': 0, 'e': 0}
control_offset = {'IDstepLength': 0.0, 'IDstepWidth': 0.0, 'IDstepAlpha': 0.0}


class KeyInterrupt(): 

    def __init__(self): 
        self.key_status = Queue()
        self.key_status.put(key_value_default)
        
        self.command_status = Queue()
        self.command_status.put(control_offset)

    def start(self):
        self.KeyInterrupt.start()

    def resetStatus(self, key_status):
        result_dict = self.key_status.get()
        self.key_status.put(key_value_default)

    def keyCounter(self, key_status, character):
        result_dict = self.key_status.get()
        result_dict[character] += 1
        self.key_status.put(result_dict)

    def calcRbStep(self, key_status, command_status):
        result_dict = key_status.get()
        command_dict = command_status.get()
        command_dict['IDstepLength'] = 10.0 * result_dict['s'] - 10.0 * result_dict['w']
        command_dict['IDstepWidth'] = 5.0 * result_dict['d'] - 5.0 * result_dict['a']
        command_dict['IDstepAlpha'] = 3.0 * result_dict['q'] - 3.0 * result_dict['e']

        key_status.put(result_dict)
        command_status.put(command_dict)

    def keyInterrupt(self, id, key_status, command_status):
        
        was_pressed = False

        while True:
            if keyboard.is_pressed('w'):
                if not was_pressed:
                    self.keyCounter(key_status, 'w')
                    was_pressed = True
            elif keyboard.is_pressed('a'):
                if not was_pressed:
                    self.keyCounter(key_status, 'a')
                    was_pressed = True
            elif keyboard.is_pressed('s'):
                if not was_pressed:
                    self.keyCounter(key_status, 's')
                    was_pressed = True
            elif keyboard.is_pressed('d'):
                if not was_pressed:
                    self.keyCounter(key_status, 'd')
                    was_pressed = True
            elif keyboard.is_pressed('q'):
                if not was_pressed:
                    self.keyCounter(key_status, 'q')
                    was_pressed = True
            elif keyboard.is_pressed('e'):
                if not was_pressed:
                    self.keyCounter(key_status, 'e')
                    was_pressed = True
            elif keyboard.is_pressed('space'):
                if not was_pressed:
                    self.resetStatus(key_status)
                    was_pressed = True
            else:
                was_pressed = False

            self.calcRbStep(key_status, command_status)


def testWhile(id, command_status):
    while True:
        result_dict = command_status.get()
        print(result_dict)
        command_status.put(result_dict)


# Basic Usage
if __name__ == "__main__":
    try:
        KeyTest = KeyInterrupt()
        KeyProcess = Process(target=KeyTest.keyInterrupt, args=(1, KeyTest.key_status, KeyTest.command_status))
        KeyProcess.start()

        testWhile(2, KeyTest.command_status)
    except Exception as e:
        print(e)
    finally:
        print("Done... ")