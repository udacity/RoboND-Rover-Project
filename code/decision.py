import numpy as np


# This is basically a decision tree for determining throttle, brake and steer 
# commands based on the output of the perception_step() function
def decision_step(Rover):

    # Implement conditionals to decide what to do given perception data
    # Default mode is forward motion

    # Example
    if Rover.mode == 'forward':  
        
        if Rover.vel < Rover.max_vel:
            Rover.throttle = Rover.throttle_set
        else: 
            Rover.throttle = 0
    
    return Rover
