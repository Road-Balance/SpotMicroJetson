# import keyboard

# keys = [
#     "down",
#     "up",
#     "left",
#     "right",
#     "w",
#     "s",
#     "a",
#     "d",
#     "1",
#     "2",
#     "3",
#     "4",
#     "q",
#     "e",
#     "f"
# ]

# while 1:
#     for key in keys:
#         if keyboard.is_pressed(key):
#             print(key)

import getch
while True:
    char = getch.getche()
    print(char + " : " + char)
    # if getch.kbhit():
    #     key_stroke = getch.getch()
    #     print(key_stroke)