import numpy as np
import time 
import random
from perception import to_polar_coords


# change to Rover cord


# This is where you can build a decision tree for determining throttle, brake and steer 
# commands based on the output of the perception_step() function
def decision_step(Rover):
    print("Rover mode: "+str(Rover.mode))
    # print("samples_to_find: "+ str(Rover.samples_to_find))
    # print("samples found : " +str(Rover.samples_found))
    # print("Origin Positon : " +str(Rover.origin))
    # print("current locaiton: " + str(Rover.pos))

    #print("Rover Len nav_angles: " + str(len(Rover.nav_angles)))

    is_stuck = Rover.stuck_flag > 40
    #colleted = Rover.samples_found >= 1

    # Implement conditionals to decide what to do given perception data
    # Here you're all set up with some basic functionality but you'll need to
    # improve on this decision tree to do a good job of navigating autonomously!

    # Example:
    # Check if we have vision data to make decisions with
    #print(Rover.nav_angles)
    if Rover.nav_angles is not None:
        Rover.throttle = 0.2
        # Check for Rover.mode status



        if Rover.mode == 'forward': 
            # newx,newy = to_rover_cord(Rover.pos,Rover.origin)

        
            # if colleted : # check if it has collected all the samples ( this function is not finished yet)

            #     if newy >= 0:
                         
            #         origin_dist, origin_angle  = to_polar_coords(newx,newy)
            #         print("new origin: x, y: " + str(newx)+","+str(newy))

    
            #         print("Origin dist, angle: " + str(origin_dist)+","+ str(origin_angle))


            #         v1  = Rover.vel
            #         Rover.steer = np.clip(np.int_(np.mean(origin_angle*180/np.pi)),-15,15)
            #         print(Rover.steer)
            #         if Rover.vel > 0.5:

            #              Rover.throttle = - (v1*v1/(2 * np.mean(origin_dist)))  

            #         if origin_dist < 0.2:
            #             Rover.brake  = Rover.brake_set
            # else:

                if abs(Rover.vel)< 0.1 and Rover.throttle > 0 :
                        Rover.stuck_flag += 1

                else:
                        Rover.stuck_flag = 0


                if is_stuck:
                    Rover.mode = 'stop'
                    print("ah,I'm stuck!!")
                else:


                    # Check the extent of navigable terrain
                    rock_angle = np.clip(np.mean(Rover.rock_angles * 180/np.pi), -15, 15)

                    if Rover.rock_angles.any()   : # go towards the rock

                        if Rover.near_sample :
                              Rover.brake = Rover.brake_set
                              print("brake near sample ...")

                      

                        else: # don't collect samples across the road

                            if np.mean(Rover.rock_dists) > 3:
                                Rover.steer = np.clip(np.mean(Rover.rock_angles * 180/np.pi)-6, -15, 15)
                            else:


                                Rover.steer = np.clip(np.mean(Rover.rock_angles * 180/np.pi)-3, -15, 15)

                            v0 = Rover.vel 

                            if Rover.vel > 0.5:
                                Rover.throttle = - (v0*v0/(2 * np.mean(Rover.rock_dists)))  
                                # calculation the stopping accelator
                            else:
                               Rover.throttle = 0.2              
                    else:



                        if len(Rover.nav_angles) >= Rover.stop_forward:  

                        
                            Rover.steer = np.clip(np.mean(Rover.nav_angles * 180/np.pi)+ random.randint(2,10), -15, 15)

                            # If mode is forward, navigable terrain looks good 
                            # and velocity is below max, then throttle 
                            if Rover.vel < Rover.max_vel:
                                # Set throttle value to throttle setting
                                Rover.throttle = Rover.throttle_set
                            else: # Else coast
                                Rover.throttle = 0
                            Rover.brake = 0

                            # Set steering to average angle clipped to the range +/- 15


                                
                        # If there's a lack of navigable terrain pixels then go to 'stop' mode
                        elif len(Rover.nav_angles) < Rover.stop_forward :
                                # Set mode to "stop" and hit the brakes!
                                Rover.throttle = 0
                                # Set brake to stored brake value
                                Rover.brake = Rover.brake_set
                                Rover.steer = 0
                                Rover.mode = 'stop'

                



        # If we're already in "stop" mode then make different decisions
        elif Rover.mode == 'stop':

            if Rover.near_sample :
                Rover.brake = Rover.brake_set

            elif  Rover.rock_angles.any(): # if it has not be near the sample but stop ,need to continue forward

                    Rover.brake = 0
                    Rover.steer +=  -Rover.steer*2
                    Rover.throttle = -0.4

                    Rover.mode ='forward'

            else:

                if Rover.vel > 0.2:
                    Rover.throttle = 0
                    Rover.brake = Rover.brake_set
                    Rover.steer = 0
                # If we're not moving (vel < 0.2) then do something else
                elif Rover.vel <= 0.2 :
                    # Now we're stopped and we have vision data to see if there's a path forward


                    if is_stuck :
                        Rover.brake = 0
                        Rover.steer = 15# randomly choose direction
                        print("im stuck in stop mode")
                        #Rover.mode = 'forward'
                        Rover.throttle =  0 # don't know how to get out the stuck ,try some random luck
                        if random.random()>0.8:

                            Rover.throttle = -2

                        if random.random() > 0.7:
                            print("randmomly choose forwad mode")

                            Rover.mode ='forward'




                    else:
           

                        if len(Rover.nav_angles) < Rover.go_forward :
                                    Rover.throttle = 0
                                    # Release the brake to allow turning
                                    Rover.brake = 0
                                    # Turn range is +/- 15 degrees, when stopped the next line will induce 4-wheel turning
                                    Rover.steer = -15 # Could be more clever here about which way to turn
                                # If we're stopped but see sufficient navigable terrain in front then go!
                        if len(Rover.nav_angles) >= Rover.go_forward  :
                                    # Set throttle back to stored value
                                    # Release the brake
                                    Rover.brake = 0
                                    Rover.throttle = 0.2
                                    # Set steer to mean angle
                                    Rover.steer = -15

                                    Rover.mode = 'forward'
                 
    # Just to make the rover do something 
    # even if no modifications have been made to the code
    else:
        Rover.throttle = Rover.throttle_set
        Rover.steer = 0
        Rover.brake = 0
        
    # If in a state where want to pickup a rock send pickup command
    if Rover.near_sample and Rover.vel == 0 and not Rover.picking_up:
        Rover.send_pickup = True
    
    return Rover

