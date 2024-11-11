# Below are a couple methods of attempting to play the audio file
# None work, I was also having issues simply instaling pygame, although this might not be necessary once we have the audio jack
# Theres a decent amount of sample code for playing with the audio jack, also worth looking at


# import os
# os.system('aplay /home/pi/Downloads/Siren.wav')

# def playSound(filename):
#     pygame.mixer.music.load(filename)
#     pygame.mixer.music.play()
# def audio_player():
#     pygame.init()
#     playSound('Siren.wav')
#     while pygame.mixer.music.get_busy() == True:
#         continue
#     return