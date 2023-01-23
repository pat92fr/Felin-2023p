from UDPComms import Publisher, Subscriber, timeout

## Configurable ##
MESSAGE_RATE = 20

import time

import pygame

 
# Initialize the joysticks
pygame.init()
pygame.joystick.init()
# Get count of joysticks
joystick_count = pygame.joystick.get_count()
print("Joystick count:" + str(joystick_count))
if joystick_count == 1:
    # Initialize the joystick #1
    joystick = pygame.joystick.Joystick(0)
    joystick.init()
    # Get the name from the OS for the controller/joystick
    name = joystick.get_name()
    print("Joystick name:" + name)
    # Get the properties of the joystick
    axes = joystick.get_numaxes()
    print("Joystick axes:" + str(axes))
    buttons = joystick.get_numbuttons()
    print("Joystick buttons:" + str(buttons))
    hats = joystick.get_numhats()
    print("Joystick hats:" + str(hats))
    
    # Communication
    joystick_pub = Publisher(8830)
    joystick_subcriber = Subscriber(8840, timeout=0.01)
    # Game loop
    done = False
    while not done:
        
        #event = pygame.event.wait()
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                done = True  # Flag that we are done so we exit this loop.

        left_x = joystick.get_axis(0)
        left_y = -joystick.get_axis(1)
        
        right_x = joystick.get_axis(2)
        right_y = joystick.get_axis(3)

        x = joystick.get_button(2)
        circle = joystick.get_button(3)
        triangle = joystick.get_button(0)
        square = joystick.get_button(1)

        L1 = joystick.get_button(4)
        R1 = joystick.get_button(5)
        
        R2 = joystick.get_axis(5)
        L2 = joystick.get_axis(4)

        left_button = joystick.get_button(6)
        right_button = joystick.get_button(7)

        dpadx = joystick.get_hat(0)[0]
        dpady = joystick.get_hat(0)[1]

        msg = {
            "ly": left_y,
            "lx": left_x,
            "rx": right_x,
            "ry": right_y,
            "L2": L2,
            "R2": R2,
            "R1": R1,
            "L1": L1,
            "dpady": dpady,
            "dpadx": dpadx,
            "x": x,
            "square": square,
            "circle": circle,
            "triangle": triangle,
            "message_rate": MESSAGE_RATE,
        }
        joystick_pub.send(msg)
        
        try:
            msg = joystick_subcriber.get()
        except timeout:
            pass

        time.sleep(1 / MESSAGE_RATE)
    
pygame.quit()
 