import sys
sys.path.append("..")

from Common.multiprocess_kb import KeyInterrupt
from multiprocessing import Process, Queue


# Test Endless While Loop
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