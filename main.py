# main.py
import time
from cutebotpro import CBP
import microbit

def print_line_state():
    while True:
        print(bot.get_line_state())
        time.sleep(0.5)
    
bot = CBP()
#bot.set_headlight(0x03, (255,0,255))
bot.set_pixels((255,255,255))
light_toggle = 1

#bot.set_motor(0x03, 100)
#bot.PID()
run = True
while run:
    try:
        if microbit.button_a.is_pressed():
            bot.PID(kp=0.6, dt=0.02, collide=False)
        if microbit.button_b.is_pressed():
            if light_toggle == 0:
                bot.set_pixels((255,255,255))
                light_toggle = 1
            else:
                bot.set_pixels((0,0,0))
                light_toggle = 0
            
    except KeyboardInterrupt:
        run = False
