import time
import threading
from agv.AGV_Main import AGV

if __name__== '__main__':

    # Create threads
    threads= []

    # Start AGV
    z = threading.Thread(target=AGV, args=('localhost', 10001, (10, 30)))
    threads.append(z)
    z.start()
    time.sleep(1)

    # Start AGV
    q = threading.Thread(target=AGV, args=('localhost', 10002, (10, 50)))
    threads.append(q)
    q.start()
    time.sleep(1)
    
    # Start AGV
    s = threading.Thread(target=AGV, args=('localhost', 10003, (10, 70)))
    threads.append(s)
    s.start()
    time.sleep(1)

    for index, thread in enumerate(threads):
        thread.join()
