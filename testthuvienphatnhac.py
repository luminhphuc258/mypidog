from pidog.audio import Audio
from time import sleep

audio = Audio()          # KHỞI TẠO I2S AUDIO DRIVER
audio.play('testmic.wav')

while audio.is_playing():
    time.sleep(0.1)

audio.stop()
