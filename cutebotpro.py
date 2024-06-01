from microbit import i2c
from microbit import pin8, pin12 # ultrasoon
from microbit import pin15 # neopixel
from microbit import accelerometer
import time
import machine
import neopixel

# source:
# https://github.com/elecfreaks/pxt-Cutebot-Pro/blob/master/main.ts

class CBP: # CuteBotPro class
    I2C_ADDRESS = 0x10
    M_L = 0x01 # left
    M_R = 0x02 # right
    M_B = 0x03 # both
    
    
    def __init__(self):
        self.pixel = neopixel.NeoPixel(pin15, 2) # RGB
        self.full_stop()
        self.set_pixels_off()
    
    def ba(self): # generate empty bytearray
        buf = bytearray(7)
        buf[0] = 0x99
        buf[6] = 0x88
        return buf
    
    def _map(self, number, in_min=0, in_max=100, out_min=10, out_max=100):
        if number > in_max:
            number = in_max
        elif number < in_min:
            number = in_min
        #print(number, in_max, in_min, out_max, out_min)
        return int(out_min + ((out_max - out_min) / (in_max - in_min)) * (number -  in_min))
        
    def get_motor_speed(self, cm=0x05):
        """
            cm: 0x05 for cm,
                0x06 for revolutions since last reset
        """
        buf = self.ba()
        buf[1] = cm
        buf[2] = 0x01 # motor left
        i2c.write(self.I2C_ADDRESS, buf)
        M1speed = int.from_bytes(i2c.read(self.I2C_ADDRESS, 1),'little', False)
        buf[2] = 0x02 # motor right
        i2c.write(self.I2C_ADDRESS, buf)
        M2speed = int.from_bytes(i2c.read(self.I2C_ADDRESS, 1),'little', False)
        return M1speed, M2speed
                
   
    def set_motor(self, motor, speed):
        """
            speed: forward (positive, 0,100)
                   backward (negative (0, -100)
                   stop = 0
            motor: M_L, M_R or M_B (left, right or both)
        """
        if speed == 0:
            b3 = 0x00 # doesnt matter
            b4 = 0xc8 # 200
        elif speed > 0:
            b3 = 0x01
            b4 = self._map(speed)
        else:
            b3 = 0x00
            b4 = self._map(abs(speed))
        buf = self.ba()
        buf[1] = 0x01
        buf[2] = motor
        buf[3] = b3
        buf[4] = b4
        i2c.write(self.I2C_ADDRESS, buf)
        
        
    def full_power(self, direction='forward'):
        buf = self.ba()
        if direction == 'forward':
            b1 = 0x07
        else:
            b1 = 0x08
        buf[1] = b1
        i2c.write(self.I2C_ADDRESS, buf)
        
    def full_stop(self):
        buf = self.ba()
        buf[1] = 0x09
        buf[2] = self.M_B # both motors
        i2c.write(self.I2C_ADDRESS, buf)
        
    def _distance(self, timeout = 3000):
        """
        Returns object distance from sonar module in cms.
        If negative number is returned, the timeout was reached during waiting for echo.
        """
        # trigger
        pin8.write_digital(0)
        time.sleep_us(5)
        pin8.write_digital(1)
        time.sleep_us(10)
        pin8.write_digital(0)

        # catch echo
        echo_time = machine.time_pulse_us(pin12, 1, timeout)
        if echo_time < 0:
            return echo_time
        else:
            return (echo_time / 2) / 29.1
        return -1.0
    
    def get_distance(self, timeout = 3000, N=15):
        """ gets N distances and returns the median
            measurement errors are not included in the media calcuation
        """
        distances = []
        for ii in range(N):
            d = self._distance(timeout)
            if d > -1:
                distances.append(d)
        print('distances: ', distances)
        N = len(distances)
        if N > 0:
            return sorted(distances)[(3*N)//4]
        else:
            return -1
    
    def set_headlight(self, light, rgb):
        """
            light: 0x01 (left)
                   0x02 (right)
                   0x03 (both)
            
        """
        buf = self.ba()
        buf[1] = 0x0f
        buf[2] = light
        buf[3] = rgb[0]
        buf[4] = rgb[1]
        buf[5] = rgb[2]
        i2c.write(self.I2C_ADDRESS, buf)
        
    def set_headlight_off(self):
        buf = self.ba()
        buf[1] = 0x10
        buf[2] = 0x01
        i2c.write(self.I2C_ADDRESS, buf)
        
    def set_pixels(self, color):
        self.pixel.fill(color)
        self.pixel.show()
        
    def set_pixels_off(self):
        self.pixel.fill((0,0,0))
        self.pixel.show()
        
    def get_line_state(self):
        buf = self.ba()
        buf[1] = 0x12
        i2c.write(self.I2C_ADDRESS, buf)
        state = i2c.read(self.I2C_ADDRESS, 1)[0]
        #print("{0:04b}".format(state))
        return state
        
    def get_line_offset(self):
        buf = self.ba()
        buf[1] = 0x14
        i2c.write(self.I2C_ADDRESS, buf)
        low = i2c.read(self.I2C_ADDRESS, 1)[0]
        buf[2] = 0x01
        i2c.write(self.I2C_ADDRESS, buf)
        high = i2c.read(self.I2C_ADDRESS, 1)[0]
        offset = (high << 8) | low
        offsetm = self._map(offset,0,6000,-3000,3000)
        print(offsetm, offset, low, high)
        
    def tracking_values(self):
        buf = self.ba()
        buf[1] = 0x11
        vals = []
        for ii in range(16):
            buf[2] = ii
            i2c.write(self.I2C_ADDRESS, buf)
            vals.append(i2c.read(self.I2C_ADDRESS, 1)[0])
        print(vals)
        
    def read_rotations(self):
        buf = self.ba()
        buf[1] = 0x16
        i2c.write(self.I2C_ADDRESS, buf)
        d = i2c.read(self.I2C_ADDRESS, 10)
        pulseL = (d[0]<<24) | (d[1]<<16) | (d[2]<<8) | d[3]
        pulseR = (d[4]<<24) | (d[5]<<16) | (d[6]<<8) | d[7]
        if d[8] == 1:
            pulseL *= -1
        if d[9] == 1:
            pulseR *= -1
        return pulseL/1428, pulseR/1428
    
    def clear_rotations(self, motor=0x03):
        buf = self.ba()
        buf[1] = 0x0A
        buf[2] = motor
        i2c.write(self.I2C_ADDRESS, buf)
        
    def version(self):
        buf = self.ba()
        buf[1] = 0x15
        i2c.write(self.I2C_ADDRESS, buf)
        v_dec = i2c.read(self.I2C_ADDRESS, 1)[0]
        buf[2] = 0x01
        i2c.write(self.I2C_ADDRESS, buf)
        v_int = i2c.read(self.I2C_ADDRESS, 1)[0]
        print("CuteBot Pro: V%d.%d"%(v_int, v_dec))
        
    def PID(self, kp = 0.06, kd=0, ki=0, dt=0.01, collide=True):
        # goal from sensors: 600 (100 times 6, middle light on)
        # max_speed = max_error * kp
        # 100 / 600 -> kp = 0.6
        max_speed = 30
        min_speed = 0
        set_speed = 20
        multi = 100
        goal = 6 * multi
        last_error = 0
        self.set_headlight(0x03, (255,0,0))
        running = True
        #dt = 0.01
        while running:
            pos = self.get_line_state() * multi
            error = pos - goal
            adjustment = error * kp + (error - last_error)*kd 
            last_error  = error
            rset = set_speed - adjustment
            lset = set_speed + adjustment
            if rset > max_speed:
                rset = max_speed
            if rset < min_speed:
                rset = min_speed
            if lset > max_speed:
                lset = max_speed
            if lset < min_speed:
                lset = min_speed

            distance = self.get_distance()
            guesture = accelerometer.current_gesture()
            if guesture is not 'up':
                running = False
            print('PID', distance, adjustment, lset, rset)
            if collide:
                if distance == -1:
                    rset /= 2
                    lset /= 2
                elif (distance < 5):
                    running = False
            if running:
                print('right: ', rset)
                print('left:  ', lset)
                self.set_motor(0x02, rset)
                self.set_motor(0x01, lset)
                #time.sleep(0.1)
                #self.set_motor(0x02, 0)
                #self.set_motor(0x01, 0)
                #time.sleep(1)
            time.sleep(dt)
        self.full_stop()
        self.set_headlight(0x03, (0,0,0))
        
        
        
if __name__ == "__main__":
    cbp = CBP()
