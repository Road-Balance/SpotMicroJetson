"""
Python MultiProcessing Example

This example is copied from this blog 
    https://monkey3199.github.io/develop/python/2018/12/04/python-pararrel.html 

Thank you monkey3199 :)
"""

import time
from multiprocessing import Process, Queue

def work(id, start, end, result):
    total = 0
    for i in range(start, end):
        total += i
    result.put(total)
    return

if __name__ == "__main__":
    START, END = 0, 100000000
    result = Queue()
    th1 = Process(target=work, args=(1, START, END//2, result))
    th2 = Process(target=work, args=(2, END//2, END, result))
    
    start = time.time()
    th1.start()
    th2.start()
    th1.join()
    th2.join()

    result.put('STOP')
    total = 0
    while True:
        tmp = result.get()
        if tmp == 'STOP':
            break
        else:
            total += tmp
    
print(f"Result: {total}")
print("time :", time.time() - start)

