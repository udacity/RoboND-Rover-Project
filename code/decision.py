import numpy as np


# This is where you can build a decision tree for determining throttle, brake and steer 
# commands based on the output of the perception_step() function
def decision_step(Rover):

    # Implement conditionals to decide what to do given perception data
    # Here you're all set up with some basic functionality but you'll need to
    # improve on this decision tree to do a good job of navigating autonomously!

    # Example:
    # Check if we have vision data to make decisions with and that it is not near a sample (i.e. don't go to stop mode if near sample)
    if Rover.nav_angles is not None or len(Rover.sample_angles) > 0:
        # Check for Rover.mode status
        if Rover.mode == 'forward': 
            # Check the extent of navigable terrain
            if len(Rover.nav_angles) >= Rover.stop_forward or len(Rover.sample_angles) > 0:  
                # If mode is forward, navigable terrain looks good 
                # and velocity is below max, then throttle 
                if Rover.vel < Rover.max_vel:
                    # Set throttle value to throttle setting
                    Rover.throttle = Rover.throttle_set
                else: # Else coast
                    Rover.throttle = 0
                Rover.brake = 0
                
                # Set steering to average angle clipped to the range +/- 15
                #Rover.steer = np.clip(np.mean(Rover.nav_angles * 180/np.pi + 8), -15, 15)
                if (np.any(Rover.nav_angles > 0)):
                    nav_left = np.average(Rover.nav_angles * 180/np.pi, weights=(Rover.nav_angles > 0))
                else:
                    nav_left = -1
                #Rover.nav_left_angle = nav_left
                #nav_right = np.average(Rover.nav_angles * 180/np.pi, Rover.nav_angles < 0)
                #wall_straight = np.average(Rover.obst_angles * 180/np.pi, (Rover.obst_angles < 2 and Rover.obst_angles > -2))
                wall_left_dist = np.average(Rover.obst_dists * 180/np.pi, weights=(Rover.obst_angles > 0))
                wall_right_dist = np.average(Rover.obst_dists * 180/np.pi, weights=(Rover.obst_angles < 0))
                ########----------- Tested --------------#########
                Rover.steer = np.clip(np.mean(Rover.nav_angles * 180/np.pi + 8), -15, 15)
                #wall_angle = np.mean(Rover.obst_angles * 180/np.pi)
                #if wall_angle < 0.:
                #    Rover.steer = 15
                #if wall_angle > 6.:
                #    Rover.steer = -15
                #################################################
                
                #if((wall_right_dist < wall_left_dist)): # and wall_left_dist > 1
                #    Rover.steer = 15
                if(nav_left > 18):
                    Rover.steer = 15
                elif (nav_left < 7 and (nav_left != -1)):
                    Rover.steer = -15
                ###### Taking care of roll and pitch variations #####
                if((Rover.roll > 1 and Rover.roll < 359) or (Rover.pitch > 0.5 and Rover.pitch < 0.5)):
                    Rover.throttle = 0
                    if Rover.vel < 0.2:
                        Rover.steer = -15
                        Rover.mode = 'stop'
                    elif Rover.vel > 0.2:
                        Rover.steer = 0
                        Rover.brake = Rover.break_set
                        Rover.mode = 'forward'
                #########################################################
                ########################## Return to original position if all samples are found ################
                if not np.any(Rover.samples_found == 0): # should be put before if(sample_angles < 5 ...) condition
                    x0 = Rover.start_pos[0] - Rover.pos[0]
                    y0 = Rover.start_pos[1] - Rover.pos[1]
                    start_pos_angle = np.arctan2(y0, x0) * 180/np.pi
                    for x2 in range(len(Rover.worldmap)):
                        for y2 in range(len(Rover.worldmap[0])):
                            if np.absolute(x2 - Rover.pos[0]) > 2 and Rover.worldmap[x2, y2, 2] is not 0:
                                x2 = x2 - Rover.pos[0]
                                y2 = y2 - Rover.pos[1]
                                break
                    heading_angle = np.arctan2(y2, x2) * 180/np.pi
                    ### using the angles w.r.t real world x-axix
                    #Rover.steer = np.clip(-start_pos_angle + heading_angle, -15, 15) ## very important step
                    #if wall_left_dist < 1:
                    #    Rover.steer = -15
                    #elif wall_right_dist < 1:
                    #    Rover.steer = 15
                    dist_from_start_pos = np.sqrt(x0**2 + y0**2)
                    if dist_from_start_pos < 4:
                        Rover.throttle = 0
                        Rover.steer = 0
                        Rover.brake = Rover.brake_set
                ##########################################################################################
                ########################## Pick up sample rock ##################################
                if Rover.near_sample or Rover.picking_up:
                    Rover.throttle = 0
                    Rover.brake = Rover.brake_set
                    #Rover.steer = 0
                    if not Rover.picking_up:
                        Rover.send_pickup = True
                    Rover.mode = 'forward'
                elif len(Rover.sample_angles) > 0 :
                    Rover.steer = np.clip(np.mean(Rover.sample_angles * 180/np.pi), -15, 15)
                    Rover.throttle = Rover.throttle_set / 2
                    Rover.brake = 0
                    Rover.mode = 'forward'
                    if Rover.vel > Rover.max_vel/2:
                        Rover.throttle = 0
                        Rover.brake = Rover.brake_set
                    elif Rover.vel == 0.0:
                        Rover.throttle = 0
                        if wall_left_dist < wall_right_dist: # Turn according to wall distances
                            Rover.steer = -15
                        else:
                            Rover.steer = 15
                ###################################################################################    
                
                ################################# check if robot is stuck #########################
                if(Rover.vel > 0):
                    Rover.stuck_time = 0
                elif (Rover.total_time > 1 and Rover.vel < 0.2 and (not Rover.picking_up)):
                    if Rover.stuck_time == 0:
                        Rover.stuck_time = Rover.total_time
                    elif (Rover.total_time - Rover.stuck_time) > 1:
                        Rover.mode = 'stuck'
                ##################################################################################
                
                Rover.picking_up = False # This is required as even after picking up the stone, the flag remains true
                #Rover.near_sample = False

            # If there's a lack of navigable terrain pixels then go to 'stop' mode
            elif len(Rover.nav_angles) < Rover.stop_forward:
                    # Set mode to "stop" and hit the brakes!
                    Rover.throttle = 0
                    # Set brake to stored brake value
                    Rover.brake = Rover.brake_set
                    Rover.steer = 0
                    Rover.mode = 'stop'

        # If we're already in "stop" mode then make different decisions
        elif Rover.mode == 'stop':
            # If we're in stop mode but still moving keep braking
            if Rover.vel > 0.2:
                Rover.throttle = 0
                Rover.brake = Rover.brake_set
                Rover.steer = 0
            # If we're not moving (vel < 0.2) then do something else
            elif Rover.vel <= 0.2:
                # Now we're stopped and we have vision data to see if there's a path forward
                if len(Rover.nav_angles) < Rover.go_forward:
                    Rover.throttle = 0
                    # Release the brake to allow turning
                    Rover.brake = 0
                    # Turn range is +/- 15 degrees, when stopped the next line will induce 4-wheel turning
                    Rover.steer = -15 # Could be more clever here about which way to turn
                    #Rover.steer = np.clip(np.mean(Rover.nav_angles * 180/np.pi), -15, 15)
                # If we're stopped but see sufficient navigable terrain in front then go!
                if len(Rover.nav_angles) >= Rover.go_forward:
                    # Set throttle back to stored value
                    Rover.throttle = Rover.throttle_set
                    # Release the brake
                    Rover.brake = 0
                    # Set steer to mean angle
                    steer_nav = np.clip(np.mean(Rover.nav_angles * 180/np.pi), -15, 15)
                    Rover.steer = steer_nav
                    Rover.mode = 'forward'
        elif Rover.mode == 'stuck':
            Rover.throttle = 0
            Rover.steer == -15
            if(Rover.total_time - Rover.stuck_time) > 2.4:
                Rover.mode = 'forward'
    # Just to make the rover do something 
    # even if no modifications have been made to the code
    else:
        Rover.throttle = Rover.throttle_set
        Rover.steer = -15
        Rover.brake = 0

    return Rover