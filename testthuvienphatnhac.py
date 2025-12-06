# Import Buzzer class
from robot_hat import Buzzer
# Import Pin for active buzzer
from robot_hat import Pin
# Import PWM for passive buzzer
from robot_hat import PWM
# import Music class for tones
from robot_hat import Music
# Import time for sleep
import time

music = Music()
# Create Buzzer object for passive buzzer
p_buzzer = Buzzer(PWM(0))
# Create Buzzer object for active buzzer
a_buzzer = Buzzer(Pin("D0"))

music.tempo(120, 1/4)

# Make a Shark-doo-doo function as is all about it
def shark_doo_doo():
    p_buzzer.play(music.note("C5"), music.beat(1/8))
    p_buzzer.play(music.note("C5"), music.beat(1/8))
    p_buzzer.play(music.note("C5"), music.beat(1/8))
    p_buzzer.play(music.note("C5"), music.beat(1/16))
    p_buzzer.play(music.note("C5"), music.beat(1/16 + 1/16))
    p_buzzer.play(music.note("C5"), music.beat(1/16))
    p_buzzer.play(music.note("C5"), music.beat(1/8))

# loop any times you want from baby to maybe great great great grandpa!
for _ in range(3):
    print("Measure 1")
    p_buzzer.play(music.note("G4"), music.beat(1/4))
    p_buzzer.play(music.note("A4"), music.beat(1/4))
    print("Measure 2")
    shark_doo_doo()
    p_buzzer.play(music.note("G4"), music.beat(1/8))
    p_buzzer.play(music.note("A4"), music.beat(1/8))
    print("Measure 3")
    shark_doo_doo()
    p_buzzer.play(music.note("G4"), music.beat(1/8))
    p_buzzer.play(music.note("A4"), music.beat(1/8))
    print("Measure 4")
    shark_doo_doo()
    p_buzzer.play(music.note("C5"), music.beat(1/8))
    p_buzzer.play(music.note("C5"), music.beat(1/8))
    print("Measure 5")
    p_buzzer.play(music.note("B4"), music.beat(1/4))
    time.sleep(music.beat(1/4))