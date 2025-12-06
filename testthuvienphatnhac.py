from robot_hat import Buzzer, Pin, Music
import time

music = Music()

# Active buzzer only
buzzer = Buzzer(Pin("D0"))   # ĐỔI D0 → D1 nếu buzzer nằm ở chân khác

music.tempo(120, 1/4)

def beep(note, beat):
    freq = music.note(note)
    duration = music.beat(beat)

    buzzer.on()
    time.sleep(duration)
    buzzer.off()
    time.sleep(0.05)

def shark():
    beep("C5", 1/8)
    beep("C5", 1/8)
    beep("C5", 1/8)
    beep("C5", 1/16)
    beep("C5", 1/8)
    beep("C5", 1/16)
    beep("C5", 1/8)

for _ in range(2):
    print("Measure 1")
    beep("G4", 1/4)
    beep("A4", 1/4)

    print("Measure 2")
    shark()
    beep("G4", 1/8)
    beep("A4", 1/8)

    print("Measure 3")
    shark()
    beep("G4", 1/8)
    beep("A4", 1/8)

    print("Measure 4")
    shark()
    beep("C5", 1/8)
    beep("C5", 1/8)

    print("Measure 5")
    beep("B4", 1/4)
    time.sleep(music.beat(1/4))
