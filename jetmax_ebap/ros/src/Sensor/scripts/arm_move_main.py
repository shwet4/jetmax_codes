import time
import hiwonder

jetmax = hiwonder.JetMax()
sucker = hiwonder.Sucker()

if __name__ == '__main__':
    jetmax.go_home()
    time.sleep(1)
    hiwonder.pwm_servo1.set_position(90 , 0.1)
    time.sleep(1)
    cur_x, cur_y, cur_z = jetmax.position
    while True:       
        jetmax.set_position((cur_x-100, cur_y, cur_z), 1)
        time.sleep(2)
        jetmax.set_position((cur_x+100, cur_y, cur_z), 1)
        time.sleep(2)
        jetmax.set_position((cur_x, cur_y, cur_z), 1)
        time.sleep(2)

        jetmax.set_position((cur_x, cur_y-50, cur_z), 1)
        time.sleep(2)
        jetmax.set_position((cur_x, cur_y+50, cur_z), 1)
        time.sleep(2)
        jetmax.set_position((cur_x, cur_y, cur_z), 1)
        time.sleep(2)
        
        jetmax.set_position((cur_x, cur_y, cur_z-100), 1)
        time.sleep(2)
        jetmax.set_position((cur_x, cur_y, cur_z), 0.5)
        time.sleep(2)
        