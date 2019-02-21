import rospy
import threading
import time
from queue import Queue


def collision():
    # Halt if collision(s) detected by bumper(s).
    print('collision')


def get_input():
    # Accept keyboard movement commands from a human user.
    print('get_input')


def escape_symmetric():
    # Escape from (roughly) symmetric obstacles within 1ft in front of the robot.
    print('escape_asymmetric')


def escape_asymmetric():
    # Avoid asymmetric obstacles within 1ft in front of the robot.
    print('escape_asymmetric')


def turn_random():
    # Turn randomly (uniformly sampled within ±15°) after every 1ft of forward movement.
    print('turn_random')


def go_forward():
    # Drive forward.
    print('got_forward')


def main():
    tasks = [collision,
             get_input,
             escape_symmetric,
             escape_asymmetric,
             turn_random,
             go_forward]

    lock = threading.Lock()

    def do_work(task):
        time.sleep(.1)  # pretend to do some lengthy work.
        # Make sure the whole print completes or threads can mix up output in one line.
        task()
        with lock:
            print(threading.current_thread().name, task)

    # The worker thread pulls an item from the queue and processes it
    def worker():
        while True:
            task = q.get()
            do_work(task)
            q.task_done()

    # Create the queue and thread pool.
    q = Queue()
    for i in range(6):
        t = threading.Thread(target=worker)
        t.daemon = True  # thread dies when main thread (only non-daemon thread) exits.
        t.start()

    # put tasks on the queue
    start = time.perf_counter()
    for item in tasks:
        q.put(item)

    q.join()


if __name__ == '__main__':
    main()
