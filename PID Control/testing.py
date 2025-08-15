import keyboard

'''while True:
    if keyboard.is_pressed('a'):
        print("You pressed a")
    elif keyboard.is_pressed('b'):
        print("You pressed b")
    elif keyboard.is_pressed('esc'):
        break'''

events = keyboard.record('esc')

keyboard.play(events)

