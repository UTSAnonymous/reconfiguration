#!/usr/bin/env python
import time
import math

def main():
    # Time counter
    t = 1.
    s = 100.
    n = 3
    # Circle loop
    while True:
        for i in range(n):
            if i == 1:
                theta = t / s + i * 2 * math.pi / n
                print("theta :" + str(theta))
                print("x :" + str(math.cos(theta)))
                print("y :" + str(math.sin(theta)))
                print("z :" + str(0.2*math.sin(1*theta)+5))
                print("yaw :" + str(theta + math.pi/2))

        t += 1
        time.sleep(.1)


if __name__ == "__main__":
    main()
