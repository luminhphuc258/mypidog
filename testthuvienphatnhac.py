from pidog.audio import Audio
from time import sleep

audio = Audio()

print("Playing...")
audio.play("testmic.wav")

while audio.is_playing():
    sleep(0.1)

audio.stop()
print("Done.")
