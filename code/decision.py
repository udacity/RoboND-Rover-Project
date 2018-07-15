import numpy as np
import time
from datetime import datetime as dt
from scipy import spatial
from termcolor import colored, cprint


def decision_step(Rover):
    """
    Make rover decisions
    Inputs:
        Rover (Rover object)
    Returns:
        Rover (Rover object)
    """
    # Rover has vision data
    if Rover.nav_angles is not None:
        # get positions of any located samples
        sample_world_pos = np.transpose(Rover.worldmap[:,:,1].nonzero())[:,[1, 0]]
        if sample_world_pos.any():
            Rover.nearest_sample_pos = sample_world_pos[spatial.KDTree(sample_world_pos).query(Rover.pos)[1]]
            Rover.distance_to_nearest_sample = np.linalg.norm(Rover.nearest_sample_pos-Rover.pos)
        # Rover should stay home
        if Rover.samples_collected == 6 and np.linalg.norm(Rover.home_pos-Rover.pos) <= 10:
            if not Rover.arrived_home:
                Rover.arrived_home = True
                Rover.arrival_time = time.time()
                hours, rem = divmod(Rover.arrival_time-Rover.start_time, 3600)
                minutes, seconds = divmod(rem, 60)
                cprint('\n******** Congrats!!! You successfully collected all six samples and returned home. *************', 
                    'green', 'on_white', attrs=['reverse', 'blink'])
                cprint('\n\t\t * Arrived home at: {} *************'.format(dt.fromtimestamp(Rover.arrival_time).strftime('%m-%d-%y %H:%M:%S %p')), 
                    'green', 'on_white', attrs=['reverse'])
                cprint('\n\t\t * Time elapsed: {:0>2}:{:0>2}:{:05.2f} *************'.format(int(hours),int(minutes), seconds), 'green', 'on_white', attrs=['reverse'])
            Rover = stop_rover(Rover, check_go_forward=False)
            Rover.brake = Rover.brake_set
        # Rover recently collected sample
        elif Rover.latest_completed_sample_collection_time and time.time() - Rover.latest_completed_sample_collection_time <= 5:
            Rover = reverse_rover(Rover)
        # Rover is stuck / was recently stuck
        elif is_rover_stuck(Rover) or (Rover.free_rover_start_time and not Rover.free_rover_end_time):
            Rover = free_rover(Rover)
        # Rover is in perpetual loop / was recently in perpetual loop
        elif is_rover_in_perpetual_loop(Rover) or (Rover.stop_perpetual_loop_start_time and not Rover.stop_perpetual_loop_end_time):
            Rover = stop_perpetual_loop(Rover)
        # Rover sees and is near sample (<= 10 meters away) that has not already been collected
        elif Rover.sees_sample and Rover.nearest_sample_pos.any() and Rover.distance_to_nearest_sample <= 10 and not is_nearest_sample_already_collected(Rover):
            Rover = stalk_nearby_sample(Rover)
        # move forward (general)
        elif Rover.mode == 'forward': 
            Rover = advance_rover(Rover)
        # stop (general)
        elif Rover.mode == 'stop':
            Rover = stop_rover(Rover)
    else: # Do nothing
        Rover.throttle = Rover.throttle_set
        Rover.steer = 0
        Rover.brake = 0
    return Rover


def advance_rover(Rover, alter_rover_steer=True, desired_steer=None):
    """
    Advance rover
    Inputs:
        Rover (Rover object)
        alter_rover_steer (bool/True)
        desired_steer (float/None)
    Returns:
        Rover (Rover object)
    """
    Rover.mode = "forward"
    # there is sufficient navigable terrain
    if len(Rover.nav_angles) >= Rover.stop_forward:  
        if Rover.vel < Rover.max_vel:
            Rover.throttle = Rover.throttle_set
        else: # Else coast
            Rover.throttle = 0
        Rover.brake = 0
        # Set steering to average angle clipped to the range +/- 15
        # Add offset of 10 to favor left steering
        if alter_rover_steer and not desired_steer:
            Rover.steer = np.clip(np.mean(Rover.nav_angles * 180 / np.pi) + 10, -15, 15)
        elif alter_rover_steer and desired_steer:
            Rover.steer = desired_steer
    # there is a lack of navigable terrain
    elif len(Rover.nav_angles) < Rover.stop_forward:
            Rover.throttle = 0
            Rover.brake = Rover.brake_set
            Rover.steer = 0
            Rover.mode = 'stop'
    return Rover

def stop_rover(Rover, check_go_forward=True, desired_steer=None):
    """
    Stop rover / turn rover while rover velocity is approx. 0
    Inputs:
        Rover (Rover object)
        check_go_forward (bool/True)
        desired_steer (float/None)
    Returns:
        Rover (Rover object)
    """
    Rover.mode = 'stop'
    approx_zero_cutoff  = 0.2
    # velocity is higher than approx. zero cutoff
    if Rover.vel > approx_zero_cutoff:
        Rover.throttle = 0
        Rover.brake = Rover.brake_set
        Rover.steer = 0
    # velocity is lower or equal to approx. zero cutoff
    elif Rover.vel <= approx_zero_cutoff:
        # lack of navigable terrain ahead / attempt to turn right
        if check_go_forward and len(Rover.nav_angles) < Rover.go_forward:
            Rover.throttle = 0
            Rover.brake = 0
            Rover.steer = -15
        # there is navigable terrain ahead / attempt to move forward
        elif check_go_forward and len(Rover.nav_angles) >= Rover.go_forward:
            Rover.throttle = Rover.throttle_set
            Rover.brake = 0
            Rover.steer = np.clip(np.mean(Rover.nav_angles * 180/np.pi) + 10, -15, 15)
            Rover.mode = 'forward'
        # do not attempt to move forward / turn using desired steer
        elif not check_go_forward and desired_steer:
            Rover.throttle = 0
            Rover.brake = 0
            Rover.steer = desired_steer
        # do not attempt to move forward / only apply brakes
        elif not check_go_forward and not desired_steer:
            Rover.throttle = 0
            Rover.brake = Rover.brake_set
            Rover.steer = 0
    return Rover


def reverse_rover(Rover):
    """
    Reverse rover
    Inputs:
        Rover (Rover object)
    Returns:
        Rover (Rover object)
    """
    Rover.mode = "forward"
    # rover velocity is not min
    if Rover.vel > -Rover.max_vel:
        # Set throttle value to throttle setting
        Rover.throttle = -Rover.throttle_set
    else: # Else coast
        Rover.throttle = 0
    Rover.brake = 0
    Rover.steer = 0
    return Rover


def is_rover_stuck(Rover, std_threshold=1.0, ignore_context=False):
    """
    Check to see if rover is stuck
    Inputs:
        Rover (Rover object)
        std_threshold (float/1)
        ignore_context (bool/False)
    Returns:
        std of recent rover positions compared to threshold (bool)
    """
    # wait for sufficient amount of data points before checking
    if not len(Rover.last_pos) > 350:
        return False
    ##### eventually remove below
    last_pos = Rover.last_pos
    last_xpos = Rover.last_pos[:,0]
    last_ypos = Rover.last_pos[:,1]
    std_of_last_xpos = np.std(last_xpos)
    std_of_last_ypos = np.std(last_ypos)
    ####### eventually remove above
    # check potential condition of rover currently in act of approaching sample
    # if currently approaching sample, return False
    if not ignore_context and Rover.angling_towards_sample_start_time and not Rover.approaching_sample_end_time and time.time() - Rover.angling_towards_sample_start_time <= 75:
        print(colored('\n********** NOT STUCK (IGNORING CONTEXT) ************', 'green'))
        print('** Rover is not stuck.')
        print('* Rover.is_stuck:', Rover.is_stuck)
        print('* Rover.is_in_perpetual_loop:', Rover.is_in_perpetual_loop)
        print('* Rover.is_angling_towards_sample:', Rover.is_angling_towards_sample)
        print('* Rover.is_approaching_sample:', Rover.is_approaching_sample)
        print('* std of last xpos:', std_of_last_xpos)
        print('* std of last ypos:', std_of_last_ypos)
        print('\t* avg of last xpos:', np.mean(last_xpos))
        print('\t* avg of last ypos:', np.mean(last_ypos))
        print(colored('**********************', 'green'))
        is_rover_stuck =  False
    else:
        is_rover_stuck = np.std(Rover.last_pos[:,0]) < std_threshold and np.std(Rover.last_pos[:,1]) < std_threshold
    if is_rover_stuck:
        print(colored('\n********** STUCK ************', 'red'))
        print('** Rover is stuck.')
        print('* Rover.is_stuck:', Rover.is_stuck)
        print('* Rover.is_in_perpetual_loop:', Rover.is_in_perpetual_loop)
        print('* Rover.is_angling_towards_sample:', Rover.is_angling_towards_sample)
        print('* Rover.is_approaching_sample:', Rover.is_approaching_sample)
        print('* len(Rover.last_pos):', len(Rover.last_pos))
        print('* std of last xpos:', std_of_last_xpos)
        print('* std of last ypos:', std_of_last_ypos)
        print('\t* avg of last xpos:', np.mean(last_xpos))
        print('\t* avg of last ypos:', np.mean(last_ypos))
        print(colored('**********************', 'red'))
    else:
        print(colored('\n********** NOT STUCK ************', 'green'))
        print('** Rover is not stuck.')
        print('* Rover.is_stuck:', Rover.is_stuck)
        print('* Rover.is_in_perpetual_loop:', Rover.is_in_perpetual_loop)
        print('* Rover.is_angling_towards_sample:', Rover.is_angling_towards_sample)
        print('* Rover.is_approaching_sample:', Rover.is_approaching_sample)
        print('* len(Rover.last_pos):', len(Rover.last_pos))
        print('* std of last xpos:', std_of_last_xpos)
        print('* std of last ypos:', std_of_last_ypos)
        print('\t* avg of last xpos:', np.mean(last_xpos))
        print('\t* avg of last ypos:', np.mean(last_ypos))
        print(colored('**********************', 'green'))
    return is_rover_stuck


def free_rover(Rover):
    """
    Perform a maneuver to free rover from stuck position
    Inputs:
        Rover (Rover object)
    Returns:
        Rover (Rover object)
    """
    cprint('\n******** FREEING ROVER *************', 'yellow', 'on_white', attrs=['reverse'])
    Rover.is_stuck = True
    if not Rover.free_rover_start_time:
        cprint('\t********* Phase 0: Defining Free Rover Parameters *************', 'yellow')
        Rover.free_rover_start_time = time.time()
        Rover.free_rover_end_time = None
        Rover = stop_rover(Rover, check_go_forward=False)
        return Rover
    time_elapsed = time.time() - Rover.free_rover_start_time
    # first two seconds of "get free" maneuver
    if time_elapsed <= 2:
        cprint('\t********* Phase 1: Stopping Rover *************', 'yellow')
        Rover = stop_rover(Rover, check_go_forward=False)
    # fourth to sixth seconds of "get free" maneuver
    elif 2 < time_elapsed <= 6:
        if len(Rover.nav_angles) <  Rover.stop_forward:
            cprint('\t********* Phase 2: Stopping Rover *************', 'yellow')
            Rover = stop_rover(Rover, check_go_forward=False, desired_steer=-15)
        else:
            cprint('\t********* Phase 2: Advancing Rover *************', 'yellow')
            Rover = advance_rover(Rover)
    # sixth to tenth seconds of "get free" maneuver
    elif 6 < time_elapsed <= 10:
        cprint('\t********* Phase 3: Reversing Rover *************', 'yellow')
        if len(Rover.nav_angles) <  Rover.stop_forward:
            Rover = reverse_rover(Rover)
        else:
            Rover = reverse_rover(Rover)
            # Rover = advance_rover(Rover)
    # tenth to eleventh seconds of "get free" maneuver
    elif 10 < time_elapsed <= 11:
        cprint('\t********* Phase 4: Stopping/Turning Rover *************', 'yellow')
        Rover = stop_rover(Rover, check_go_forward=False, desired_steer=np.clip(np.mean(Rover.nav_angles * 180/np.pi) + 10, -15, 15))
    # eleventh to thirteenth seconds of "get free" maneuver
    elif 11 < time_elapsed <= 13:
        cprint('\t********* Phase 5: Stopping/Turning Rover *************', 'yellow')
        Rover = stop_rover(Rover, check_go_forward=False, desired_steer=-15)
    else:
        # print('* Clearing Free Rover Start Time')
        Rover.is_stuck = False
        Rover.last_pos = Rover.last_pos[:1]
        Rover.last_steers = Rover.last_steers[:1]
        Rover.free_rover_start_time = None
        Rover.free_rover_end_time = time.time()
        Rover = advance_rover(Rover)
    cprint('**********************', 'yellow', 'on_white', attrs=['reverse'])
    return Rover


def is_rover_in_perpetual_loop(Rover):
    """
    Check to see if rover is is in perpetual loop
    Inputs:
        Rover (Rover object)
    Returns:
        bool statement of the sign uniformity of the latest Rover steers
    """
    if Rover.angling_towards_sample_start_time and not Rover.approaching_sample_end_time and time.time() - Rover.angling_towards_sample_start_time <= 75:
        return False
    elif Rover.free_rover_end_time and time.time() - Rover.free_rover_end_time <= 5:
        return False
    else:
        return ((np.all(Rover.last_steers > 0) or np.all(Rover.last_steers < 0)))

def stop_perpetual_loop(Rover):
    """
    Perform a maneuver to stop perpetual looping
    Inputs:
        Rover (Rover object)
    Returns:
        Rover (Rover object)
    """
    cprint('\n******** STOPPING PERPETUAL LOOP *************', 'blue', 'on_white', attrs=['reverse'])
    Rover.is_in_perpetual_loop = True
    if not Rover.stop_perpetual_loop_start_time:
        cprint('\t********* Phase 0: Defining Stop Perpetual Loop Parameters *************', 'blue')
        Rover.stop_perpetual_loop_start_time = time.time()
        Rover.stop_perpetual_loop_end_time = None
        Rover = stop_rover(Rover, check_go_forward=False)
        return Rover
    time_elapsed = time.time() - Rover.stop_perpetual_loop_start_time
    if time_elapsed <= 2:
        cprint('\t********* Phase 1: Stopping Rover *************', 'blue')
        Rover = stop_rover(Rover, check_go_forward=False)
    elif 2 < time_elapsed <= 7:
        cprint('\t********* Phase 2: Stopping / Turning Rover *************', 'blue')
        # desired_steer = Rover.last_steers.flat[np.abs(Rover.last_steers + 15).argmin()]
        desired_steer = -10 if np.mean(Rover.last_steers) + 8 > 0 else 10
        Rover = stop_rover(Rover, check_go_forward=False, desired_steer=desired_steer)
    else:
        Rover.is_in_perpetual_loop = False
        Rover.last_steers = Rover.last_steers[:1]
        Rover.stop_perpetual_loop_start_time = None
        Rover.stop_perpetual_loop_end_time = time.time()
        Rover = advance_rover(Rover)
    cprint('**********************', 'blue', 'on_white', attrs=['reverse'])
    return Rover

def stalk_nearby_sample(Rover):
    """
    Stalk nearby sample
    Inputs:
        Rover (Rover object)
    Returns:
        Rover (Rover object)
    """
    # establish time elapsed since start time of stalking
    cprint('\n******** STALKING NEARBY SAMPLE *************', 'magenta', 'on_white', attrs=['reverse'])
    if not Rover.is_angling_towards_sample and not Rover.is_approaching_sample:
        time_elapsed = 0
    else:
        time_elapsed = time.time() - Rover.angling_towards_sample_start_time
    # establish the current angle to the nearest sample and the avg sample pos in Rover centric coords
    avg_nearest_sample_xpos_in_rover_coords = np.mean(Rover.last_nearest_sample_pos_in_rover_coords[:,0])
    avg_nearest_sample_ypos_in_rover_coords = np.mean(Rover.last_nearest_sample_pos_in_rover_coords[:,1])
    angle_between_rover_and_nearest_sample = np.rad2deg(np.arctan2(
        avg_nearest_sample_ypos_in_rover_coords, 
        avg_nearest_sample_xpos_in_rover_coords))
    # initiate stalking activity
    if not Rover.is_angling_towards_sample and not Rover.is_approaching_sample:
        cprint('\t********* Phase 0: Defining Stalk Parameters *************', 'magenta')
        Rover.is_angling_towards_sample = True
        Rover.angling_towards_sample_start_time = time.time()
        Rover.approaching_sample_end_time = None
        Rover = stop_rover(Rover, check_go_forward=False)
    # slow down Rover before making steering adjustments to angle towards sample
    elif Rover.vel > 0.2 and Rover.is_angling_towards_sample and time_elapsed <= 90:
        cprint('\t********* Phase 1: Angling Towards Sample *************', 'magenta')
        Rover = stop_rover(Rover, check_go_forward=False, desired_steer=np.clip(angle_between_rover_and_nearest_sample, -10, 10))
    # make steering adjustments to Rover to angle towards sample
    elif Rover.vel <= 0.2 and Rover.is_angling_towards_sample and time_elapsed <= 90:
        cprint('\t********* Phase 2: Angling Towards Sample *************', 'magenta')
        # if sample is directly infront of Rover, start approaching the sample
        if not np.abs(avg_nearest_sample_ypos_in_rover_coords) < 1:
            Rover = stop_rover(Rover, check_go_forward=False, desired_steer=np.clip(angle_between_rover_and_nearest_sample, -10, 10))
        else:
            Rover.is_angling_towards_sample = False
            Rover.is_approaching_sample = True
    # Rover can see sample and is approaching sample
    elif Rover.sees_sample and Rover.is_approaching_sample and time_elapsed <= 90:
        # not near sample and rover is stuck
        if not Rover.near_sample and is_rover_stuck(Rover, std_threshold=.3, ignore_context=True):
            cprint('\t********* Phase 3: Approaching Sample (not near sample and stuck) *************', 'magenta')
            Rover = advance_rover(Rover)
        # not near sample but approaching
        if not Rover.near_sample and np.abs(avg_nearest_sample_ypos_in_rover_coords) >= 1:
            cprint('\t********* Phase 3: Approaching Sample (not near sample) *************', 'magenta')
            Rover = advance_rover(Rover, desired_steer=np.clip(angle_between_rover_and_nearest_sample, -4, 4))
        # not near sample but approaching
        elif not Rover.near_sample and np.abs(avg_nearest_sample_ypos_in_rover_coords) < 1:
            cprint('\t********* Phase 3: Approaching Sample (not near sample) *************', 'magenta')
            Rover = advance_rover(Rover, desired_steer=0)
        # near sample and need to slow down
        elif Rover.near_sample and Rover.vel > 0:
            cprint('\t********* Phase 3: Approaching Sample (near sample) *************', 'magenta')
            Rover = stop_rover(Rover, check_go_forward=False)
        # near sample but not picking up sample
        elif Rover.near_sample and Rover.vel == 0 and not Rover.picking_up:
            cprint('\t********* Phase 4: Send Pick Up Command *************', 'magenta')
            Rover.send_pickup = True
        elif Rover.picking_up:
            cprint('\t********* Phase 5: Collecting Sample *************', 'magenta', 'on_white', attrs=['reverse', 'blink'])
            if not Rover.samples_collected_pos.any():
                Rover.samples_collected_pos = np.array(Rover.pos)
            else:
                Rover.samples_collected_pos = np.vstack([Rover.pos, Rover.samples_collected_pos])
            Rover.was_picking_up = True
        # just finished picking up sample 
        elif Rover.was_picking_up and not Rover.picking_up:
            Rover.last_pos = Rover.last_pos[:1]
            Rover.last_steers = Rover.last_steers[:1]
            Rover.angling_towards_sample_start_time = None
            Rover.approaching_sample_end_time = time.time()
            Rover.latest_completed_sample_collection_time = Rover.approaching_sample_end_time
            Rover.is_angling_towards_sample = False
            Rover.is_approaching_sample = False
            Rover.send_pickup = False
            Rover.was_picking_up = False
            Rover = reverse_rover(Rover)
    # approaching sample but lose sight of sample
    elif not Rover.sees_sample and Rover.is_approaching_sample or time_elapsed > 90:
        cprint('\t********* Lost Sight Of Nearby Sample *************', 'red', 'on_white', attrs=['reverse', 'blink'])
        Rover.last_pos = Rover.last_pos[:1]
        Rover.last_steers = Rover.last_steers[:1]
        Rover.angling_towards_sample_start_time = None
        Rover.approaching_sample_end_time = time.time()
        Rover.is_angling_towards_sample = False
        Rover.is_approaching_sample = False
        Rover.send_pickup = False
        Rover.was_picking_up = False
        Rover = reverse_rover(Rover)
    cprint('**********************', 'magenta', 'on_white', attrs=['reverse'])
    return Rover

def is_nearest_sample_already_collected(Rover):
    """
    Determine sample nearest to rover has already been collected
    Inputs:
        Rover (Rover object)
    Returns:
        Rover (Rover object)
    """
    # check if rover has collected any samples
    if not Rover.samples_collected_pos.any():
        return False
    # rover has collected only one sample
    elif Rover.samples_collected_pos.ndim == 1:
        if np.linalg.norm(Rover.samples_collected_pos-Rover.nearest_sample_pos) < 4:
            return True 
    # rover has collected more than one sample
    elif Rover.samples_collected_pos.ndim == 2:
        nearest_sample_collected_pos = Rover.samples_collected_pos[spatial.KDTree(Rover.samples_collected_pos).query(Rover.nearest_sample_pos)[1]]
        if np.linalg.norm(nearest_sample_collected_pos-Rover.nearest_sample_pos) < 4:
            return True
    return False
