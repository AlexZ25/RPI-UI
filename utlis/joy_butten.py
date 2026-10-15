import pygame, time
import os
import json

import curses
def main():
    pygame.init(); pygame.joystick.init()
    pad = pygame.joystick.Joystick(0); pad.init()

    while True:
        # print out all joystick axis values
        pygame.event.pump()
        axes = pad.get_numaxes()
        buttons = pad.get_numbuttons()
        axis_values = [round(pad.get_axis(i), 2) for i in range(axes)]
        button_values = [pad.get_button(i) for i in range(buttons)]
        print(f"\rAxes: {axis_values}", end="")
        print(f" Buttons: {button_values}", end="")
        time.sleep(0.1)

if __name__ == "__main__":
    main()