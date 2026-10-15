import pygame, time
import os
import json

import curses
from rpi_hardware_pwm import HardwarePWM

def load_config(path = '~/pwm_config.json'):
    # Placeholder for loading configuration if needed in future
    try :
        with open(os.path.expanduser(path), 'r') as f:
            config = json.load(f)
            return config
    except:
        # return default config
        return {'steer_center': 22.5,
                'steer_range':2.7,
                'throttle_rest':21,
                'throttle_start':0.5,
                'throttle_max':1,
                }

def save_config(config):
    # Placeholder for saving configuration if needed in future
    try:
        with open(os.path.expanduser('~/pwm_config.json'), 'w') as f:
            json.dump(config, f)
    except:
        pass


def main(stdscr):
    curses.curs_set(0); stdscr.keypad(True)
    pygame.init(); pygame.joystick.init()
    pad = pygame.joystick.Joystick(0); pad.init()
    # print("Detected:", pad.get_name())
    config = load_config()

    steer_center = config['steer_center']
    steer_range = config['steer_range']
    throttle_rest = config['throttle_rest']
    throttle_start = config['throttle_start']
    throttle_max = config['throttle_max']

    steer_center_adj_step = 0.1
    steer_range_adj_step = 0.1
    throttle_rest_adj_step = 0.1
    throttle_start_adj_step = 0.1
    throttle_max_adj_step = 0.1

    FREQ_HZ = 150  # 150Hz

    pwm_steer = HardwarePWM(pwm_channel=0, hz=FREQ_HZ)  # uses /sys/class/pwm
    duty = steer_center  
    pwm_steer.start(duty)
    pwm_throttle = HardwarePWM(pwm_channel=1, hz=FREQ_HZ)  # uses /sys/class/pwm
    duty = throttle_rest 
    pwm_throttle.start(duty)

    try:
        while True:
            pygame.event.pump()
            steer = round(pad.get_axis(3),2)
            throttle = round((pad.get_axis(5)+1)/2,2)  # Invert Y axis
            reverse = round((pad.get_axis(2)+1)/2,2)
            # print(f"\rsteer:{steer} throttle:{throttle}", end="")
            steer_pwm = steer_center - steer_range * steer
            
            pwm_steer.change_duty_cycle(steer_pwm)

            if throttle > 0.05:
                throttle_pwm = throttle_rest - throttle_start - (throttle_max - throttle_start) * throttle
            elif reverse > 0.05:
                throttle_pwm = throttle_rest + throttle_start + (throttle_max - throttle_start) * reverse
            else:
                throttle_pwm = throttle_rest

            # Adjust steering config with dpad
            steer_range_adj = pad.get_axis(1)
            if abs(steer_range_adj) > 0.1:
                steer_range -= steer_range_adj_step * steer_range_adj
                steer_range = round(steer_range,2)
                time.sleep(0.2)  # avoid too fast change
            steer_center_adj = pad.get_axis(0)
            if abs(steer_center_adj) > 0.1:
                steer_center += steer_center_adj_step * steer_center_adj
                steer_center = round(steer_center,2)
                time.sleep(0.2)  # avoid too fast change

            y = pad.get_button(3)
            a = pad.get_button(0)
            if abs(y) > 0.1:
                throttle_rest += throttle_rest_adj_step
                throttle_rest = round(throttle_rest,2)
                time.sleep(0.2)  # avoid too fast change
            if abs(a) > 0.1:
                throttle_rest -= throttle_start_adj_step
                throttle_rest = round(throttle_rest,2)
                time.sleep(0.2)  # avoid too fast change

            

            pwm_throttle.change_duty_cycle(throttle_pwm)

            stdscr.clear()
            stdscr.addstr(0, 0, f"Using joystick: {pad.get_name()}")
            stdscr.addstr(1, 0, f"Steer: {steer} Throttle: {throttle} Reverse: {reverse}")
            stdscr.addstr(3, 0, f"Steering PWM: {steer_pwm}")
            stdscr.addstr(4, 0, f"Throttle PWM: {throttle_pwm}")
            stdscr.addstr(6, 0, f"Steer Center: {steer_center} Range: {steer_range}")
            stdscr.addstr(7, 0, f"Throttle Rest: {throttle_rest} Start: {throttle_start} Max: {throttle_max}")
            stdscr.refresh()

            time.sleep(0.03)
    except KeyboardInterrupt:
        # save config on exit
        config['steer_center'] = steer_center
        config['steer_range'] = steer_range
        config['throttle_rest'] = throttle_rest
        config['throttle_start'] = throttle_start
        config['throttle_max'] = throttle_max
        save_config(config)
        print()



if __name__ == "__main__":
    curses.wrapper(main)
