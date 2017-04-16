import numpy as np


# This is basically a decision tree for determining throttle, brake and steer 
# commands based on the output of the perception_step() function
def decision_step(Rover):

    # Conditionals to decide what to do given perception data
    # Default mode is forward motion
    new_steer = np.clip(np.mean(Rover.nav_angles/(np.pi/4)), -1, 1)
    nav_total = len(Rover.nav_angles)

    if Rover.mode == 'forward':  

        if nav_total > Rover.go_forward:
            
            if Rover.vel < Rover.max_vel:
                Rover.throttle = Rover.throttle_set
            else: 
                Rover.throttle = 0
            Rover.brake = 0
            Rover.steer = new_steer

        else:
            Rover.mode = 'stop'
            Rover.throttle = 0
            Rover.brake = Rover.brake_set
            Rover.steer = 0.0

    elif Rover.mode == 'stop':
    
        if Rover.vel > 0.2:
            Rover.throttle = 0
            Rover.brake = Rover.brake_set
            Rover.steer = 0.0

        elif Rover.vel <= 0.2 and nav_total < Rover.go_forward:
            Rover.throttle = 0
            Rover.brake = 0
            Rover.steer = 1

        elif Rover.vel <= 0.2 and nav_total >= Rover.go_forward and new_steer < 0.3:
            Rover.throttle = Rover.throttle_set
            Rover.brake = 0
            Rover.steer = new_steer
            Rover.mode = 'forward'

        else:
            throttle_value = 0
            brake_value = Rover.brake_set
            steer_value = 0.0
            
    
    return Rover
