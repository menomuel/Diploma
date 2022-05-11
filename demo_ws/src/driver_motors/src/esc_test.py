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
        
    def pwm(self, width, sleep=0):
        print('pwm:', width)
        self.conn.set_servo_pulsewidth(self.pin1, width)
        self.conn.set_servo_pulsewidth(self.pin2, width)
        self.conn.set_servo_pulsewidth(self.pin3, width)
        self.conn.set_servo_pulsewidth(self.pin4, width)
        if sleep:
            time.sleep(sleep)
        return
        
    def pwm_single(self, pin, width, sleep=0):
        print(f'pwm {pin}:', width)
        self.conn.set_servo_pulsewidth(pin, width)
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
        max_width = self.MAX_WIDTH - 600
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
            
    def motor_characteristic(self):
        max_width = 1900
        min_width = 1200
        step = 25
        
        sleep = 0.

        for width in range(min_width, max_width, step):
            self.pwm(width, sleep)
            input()
            
            #self.pwm_dutycycle(width=width, sleep=sleep)

    def u2_test(self):
        width12 = 1200
        width34 = 1350
        
        self.pwm_single(self.pin1, width12, 0);
        self.pwm_single(self.pin2, width12, 0);
        self.pwm_single(self.pin3, width34, 0);
        self.pwm_single(self.pin4, width34, 0);
        
        time.sleep(20)
        
    def u3_test(self):
        width14 = 1200
        width23 = 1350
        
        self.pwm_single(self.pin1, width14, 0);
        self.pwm_single(self.pin2, width23, 0);
        self.pwm_single(self.pin3, width23, 0);
        self.pwm_single(self.pin4, width14, 0);
        
        time.sleep(20)

if __name__ == "__main__":
    esc = ESC()
    try:
        #esc.calibrate()
        
        esc.arm()
        #esc.test()
        
        esc.motor_characteristic()
        #esc.pwm_single(esc.pin1, 1200, 0)
        #esc.pwm_single(esc.pin2, 1230, 0) #1230
        #esc.pwm_single(esc.pin3, 1235, 0) #1235
        #esc.pwm_single(esc.pin4, 1200, 0)
        #time.sleep(10)
        
        #esc.u2_test()
        #esc.u3_test()
    except KeyboardInterrupt:
        pass
    finally:
        esc.disarm()
        pass
