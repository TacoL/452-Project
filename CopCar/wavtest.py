# This code works, but the speaker we have may need an amplifier

import pygame

def playSound(filename):
    pygame.mixer.music.load(filename)
    pygame.mixer.music.play()

def audio_player():

    pygame.init()
    playSound('CopCar/Siren.wav')
    while pygame.mixer.music.get_busy() == True:
        continue
    return

audio_player()