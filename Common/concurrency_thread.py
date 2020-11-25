"""
Python MultiThreading Example

This example is copied from this blog 

https://monkey3199.github.io/develop/python/2018/12/04/python-pararrel.html 

Thank you monkey3199 :)
"""

import time
from threading import Thread

def work(id, start, end, result):
    total = 0
    for i in range(start, end):
        total += i
    result.append(total)
    return

# if __name__ == "__main__":
#     START, END = 0, 100000000
#     result = list()
#     th1 = Thread(target=work, args=(1, START, END, result))
    
#     start = time.time()
#     th1.start()
#     th1.join()

if __name__ == "__main__":
    START, END = 0, 100000000
    result = list()
    th1 = Thread(target=work, args=(1, START, END//2, result))
    th2 = Thread(target=work, args=(2, END//2, END, result))
    
    start = time.time()
    th1.start()
    th2.start()
    th1.join()
    th2.join()

print("time :", time.time() - start)
print(f"Result: {sum(result)}")