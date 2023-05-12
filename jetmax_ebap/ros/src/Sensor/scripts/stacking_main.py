import sys
import math
import hiwonder
import time


jetmax = hiwonder.JetMax()
sucker = hiwonder.Sucker()
target_positions = [(238, 20, 80),(238, 20, 123),(238, 20, 166)]
if __name__ == '__main__':
    jetmax.go_home()
    sucker.set_state(True)
    time.sleep(0.5)
    sucker.release(3)
    time.sleep(2)
    overlay = 0
    
    while(overlay<3):       # Pick up the block
        hiwonder.pwm_servo1.set_position(90 , 0.1)
        jetmax.set_position((0, 80, 120), 1)
        time.sleep(1)
        sucker.set_state(True)  # Turn on the air pump
        jetmax.set_position((0, 80, 85 - 5), 1)
        time.sleep(1)

        jetmax.set_position((0, 80, 180), 1)
        time.sleep(1)
        hiwonder.pwm_servo1.set_position(90, 0.1)
        
        # Go to the target position
        (x, y, z) = target_positions[overlay]
        jetmax.set_position((x, y, 180), 1)
        time.sleep(1)
        jetmax.set_position((x, y, z), 1)
        time.sleep(1)

        # Put down the block
        sucker.release(3)  # Turn off the air pump
        jetmax.set_position((x, y, 180), 1)
        time.sleep(1)
        jetmax.go_home()
        time.sleep(1)
        overlay = overlay+1
