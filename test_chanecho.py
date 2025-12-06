from pidog import Pidog
import time

dog = Pidog()

print("=== TEST ULTRASONIC ===")

while True:
    dist = dog.ultrasonic.read()
    print("DISTANCE =", dist)
    time.sleep(0.2)
