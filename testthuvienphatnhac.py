from robot_hat import Audio
import time

audio = Audio()          # KHỞI TẠO I2S AUDIO DRIVER
audio.play('testmic.wav')

while audio.is_playing():
    time.sleep(0.1)

audio.stop()
