import time
import pigpio


class ESC:
    MIN_WIDTH = 1150
    MAX_WIDTH = 1900
    
    pin1 = 5
    pin2 = 13
    pin3 = 21
    pin4 = 27

    def __init__(self):
        self.conn = pigpio.pi()
        #self.pin = pin
        #print(f'ESC initialized on pin {pin}')

    def pwm(self, width, sleep=0):
        print('pwm:', width)
        self.conn.set_servo_pulsewidth(self.pin1, width)
        self.conn.set_servo_pulsewidth(self.pin2, width)
        self.conn.set_servo_pulsewidth(self.pin3, width)
        self.conn.set_servo_pulsewidth(self.pin4, width)
        if sleep:
            time.sleep(sleep)
        return
        
    '''
    def pwm_dutycycle(self, width, sleep=0):
        print('dutycycle:', width)
        self.conn.set_PWM_dutycycle(self.pin, width)
        if sleep:
            time.sleep(sleep)
        return
    '''
     
    def calibrate(self):
        print('Calibrating...')
        input('Disconnect power and press Enter...')
        self.pwm(width=self.MAX_WIDTH)
        input('Connect power and press Enter to calibrate...')
        self.pwm(width=self.MAX_WIDTH, sleep=4)
        self.pwm(width=self.MIN_WIDTH, sleep=4)
        input('Calibration complete! Re-connect and press Enter...')

    def arm(self):
        print('Arming...')
        self.pwm(width=self.MIN_WIDTH, sleep=4)
        print('Armed')

    def disarm(self):
        print('Disarming...')
        self.pwm(0)
        self.conn.stop()
        print('Disarm')
    
    '''    
    def reset(self):
        self.pwm(width=self.MAX_WIDTH, sleep=3)
        input('Wait until all beeps pass...')
    '''
    
    def test(self):
        max_width = self.MAX_WIDTH - 500
        min_width = self.MIN_WIDTH + 50
        #max_width = 250
        #min_width = 0
        step = 25
        
        sleep = 1.

        print("Increasing...")
        for width in range(min_width, max_width, step):
            self.pwm(width=width, sleep=sleep)
            #self.pwm_dutycycle(width=width, sleep=sleep)

        print("Decreasing...")
        for width in range(max_width, min_width, -step):
            self.pwm(width=width, sleep=sleep)
            #self.pwm_dutycycle(width=width, sleep=sleep)

if __name__ == "__main__":
    esc = ESC()
    try:
        #esc.calibrate()
        esc.arm()
        esc.test()
    except KeyboardInterrupt:
        pass
    finally:
        esc.disarm()
        pass
