import numpy as np
from perception import check_rocks, to_polar_coords, pix_to_world


# This is where you can build a decision tree for determining throttle, brake and steer
# commands based on the output of the perception_step() function
def decision_step(Rover):
    # Implement conditionals to decide what to do given perception data
    # Here you're all set up with some basic functionality but you'll need to
    # improve on this decision tree to do a good job of navigating autonomously!

    # Example:
    # Check if we have vision data to make decisions with
    if Rover.nav_angles is not None:
        # print some info for debugging
        print("Current mode: ", Rover.mode)
        print("throttle_count:", Rover.throttle_count)

        make_decision(Rover)

    # Just to make the rover do something
    # even if no modifications have been made to the code
    else:
        Rover.throttle = Rover.throttle_set[0]
        Rover.steer = 0
        Rover.brake = 0

    # If in a state where want to pickup a rock send pickup command
    if Rover.near_sample and Rover.vel == 0 and not Rover.picking_up:
        Rover.send_pickup = True

    return Rover


def make_decision(Rover):
    # Check for Rover.mode status
    # Forward
    if Rover.mode == 'forward':
        if Rover.home is None:
            # note down home
            Rover.home = Rover.pos

        if Rover.samples_found == 6:
            Rover.is_going_home = True

        if Rover.is_going_home:
            if (Rover.pos[0] - Rover.home[0] < 0.5) and (Rover.pos[1] - Rover.home[1] < 0.5):
                Rover.is_done = True
                Rover.is_going_home = False
                Rover.mode = 'stop'

        # Check the extent of navigable terrain
        if len(Rover.nav_angles) >= Rover.stop_forward:
            # If mode is forward, navigable terrain looks good
            # and velocity is below max, then throttle
            if Rover.vel < Rover.max_vel:
                # keep accelerating
                set_rover_throttle(Rover)
            else:  # Else coast
                Rover.throttle = 0
            Rover.brake = 0

            # Get new steer value
            new_steer = get_navi_steer(Rover)

            # If the steer difference is large, slow it down
            if np.absolute(Rover.last_steer - new_steer) > 3:
                Rover.throttle = 0
                Rover.brake = Rover.brake_soft

            # Steer the rover and save the steer value
            Rover.steer = new_steer
            Rover.last_steer = new_steer

        # If there's a decrease of navigable terrain pixels then go to 'decelerate' mode
        elif len(Rover.nav_angles) < Rover.decelerate_forward:
            # Set mode to "decelerate" and hit the brakes!
            Rover.throttle = 0
            # Set brake to stored brake value
            if Rover.vel > 1:
                Rover.brake = Rover.brake_soft
            else:  # coast
                Rover.brake = 0
            Rover.mode = 'decelerate'

        check_stuck(Rover, 30)
        check_circle(Rover)

        if check_rocks(Rover):  # got rocks
            if Rover.vel > 1:
                Rover.throttle = 0
                Rover.brake = Rover.brake_soft
            elif Rover.vel < 0.5:
                Rover.throttle = Rover.throttle_set[0]
                Rover.brake = 0
            else:
                Rover.throttle = 0
                Rover.brake = 0
                Rover.mode = 'approach'

    # Not enough available terrain in front, need to slow down
    elif Rover.mode == 'decelerate':
        # If there's a lack of navigable terrain pixels then go to 'stop' mode
        if len(Rover.nav_angles) < Rover.stop_forward:
            # Set mode to "stop" and hit the brakes!
            Rover.throttle = 0
            # Set brake to stored brake value
            Rover.brake = Rover.brake_hard
            Rover.steer = 0
            Rover.mode = 'stop'
        elif len(Rover.nav_angles) >= Rover.go_forward:
            # Set throttle back to stored value
            Rover.throttle = Rover.throttle_set[0]
            # Release the brake
            Rover.brake = 0
            Rover.mode = 'forward'

    # To get the rover out of a stuck situation
    elif Rover.mode == 'unstuck':
        # Release throttle
        Rover.throttle = 0
        # Stop the rover first
        if Rover.vel > 0.2:
            Rover.brake = Rover.brake_hard
        else:
            # Release the brake to allow turning
            Rover.brake = 0

        # get current time
        time_now = Rover.total_time
        # spin a little
        if time_now > Rover.stuck_time + 0.5:
            Rover.steer = 0
            Rover.is_stuck = False
            Rover.stuck_time = 0
            # try move
            Rover.mode = 'test_run'
        else:
            Rover.steer = -15

    # Try to move along current direction
    elif Rover.mode == 'test_run':
        # Full throttle
        Rover.throttle = Rover.throttle_set[2]

        if Rover.throttle >= 0.1 and Rover.vel < 0.2:
            # Still stuck, need to turn again
            if Rover.throttle_count > 30:
                Rover.throttle_count = 0
                Rover.stuck_time = Rover.total_time
                Rover.mode = 'unstuck'
            else:
                Rover.throttle_count += Rover.throttle_set[1]
        elif Rover.vel >= 0.2:
            # this direction is movable
            Rover.mode = 'forward'

    # Got rock in sight, approach it
    elif Rover.mode == 'approach':
        # Near sample, stop the rover and pick it up
        if Rover.near_sample:
            Rover.throttle = 0
            Rover.brake = Rover.brake_hard
            # If in a state where want to pickup a rock send pickup command
            if Rover.vel == 0:
                Rover.mode = 'pickup'
        else:
            # Right beside the rock, turn the rover to get a good angle
            if np.mean(Rover.rock_dists) < 2:
                rover_maintain_speed(Rover, 0, 0.2)
                if np.mean(Rover.rock_angles) * 180 / np.pi > 0:
                    Rover.steer = 15
                elif np.mean(Rover.rock_angles) * 180 / np.pi < 0:
                    Rover.steer = -15
            else:
                # Adjust speed based on the dist with the rock
                if np.mean(Rover.rock_dists) > 30:
                    rover_maintain_speed(Rover, 1.7, 2)
                elif np.mean(Rover.rock_dists) > 20:
                    rover_maintain_speed(Rover, 1.4, 1.7)
                elif np.mean(Rover.rock_dists) > 10:
                    rover_maintain_speed(Rover, 1, 1.4)
                else:
                    rover_maintain_speed(Rover, 0.6, 1)

                # Turn the rover towards the rock
                try:
                    if (np.max(Rover.rock_angles) * 180 / np.pi >= -180) \
                            and (np.max(Rover.rock_angles) * 180 / np.pi <= 180):
                        Rover.steer = np.max(Rover.rock_angles) * 180 / np.pi
                    else:
                        rover_maintain_speed(Rover, 0.1, 0.3)
                        if np.mean(Rover.rock_angles) * 180 / np.pi > 0:
                            Rover.steer = 15
                        elif np.mean(Rover.rock_angles) * 180 / np.pi < 0:
                            Rover.steer = -15
                except ValueError:
                    rover_maintain_speed(Rover, 0.1, 0.3)
                    if np.mean(Rover.rock_angles) * 180 / np.pi > 0:
                        Rover.steer = 15
                    elif np.mean(Rover.rock_angles) * 180 / np.pi < 0:
                        Rover.steer = -15

        # Check if the rover is stuck with a longer waiting time
        check_stuck(Rover, 100)

    # Pick up the rock
    elif Rover.mode == 'pickup':
        # If in a state where want to pickup a rock send pickup command
        if Rover.near_sample and not Rover.picking_up:
            Rover.send_pickup = True
        elif Rover.picking_up:
            Rover.mode = 'stop'

    # If we're already in "stop" mode then make different decisions
    elif Rover.mode == 'stop':
        # If we're in stop mode but still moving keep braking
        if Rover.vel > 0.2:
            Rover.throttle = 0
            Rover.brake = Rover.brake_hard
            Rover.steer = 0
        # If we're not moving (vel < 0.2) then do something else
        else:
            if not Rover.is_done:  # if done (returned with all 6 rocks), no longer looking for sufficient terrain
                # Now we're stopped and we have vision data to see if there's a path forward
                nav_ang_len = len(Rover.nav_angles)
                if nav_ang_len < Rover.go_forward:
                    Rover.throttle = 0
                    # Release the brake to allow turning
                    Rover.brake = 0
                    # Turn range is +/- 15 degrees, when stopped the next line will induce 4-wheel turning
                    # if Rover.steer > 0:
                    Rover.steer = -15
                    # else: # wrong direction
                    #    Rover.steer = -15
                # If we're stopped but see sufficient navigable terrain in front then go!
                if nav_ang_len >= Rover.go_forward:
                    # Set throttle back to stored value
                    Rover.throttle = Rover.throttle_set[0]
                    # Release the brake
                    Rover.brake = 0
                    Rover.steer = 0
                    Rover.mode = 'forward'


# Check if the rover is stuck
def check_stuck(Rover, threshold):
    # Throttle is on but the rover is not speeding up
    if Rover.throttle >= 0.1 and Rover.vel < 0.2:  # stuck
        if Rover.throttle_count > threshold:
            Rover.throttle_count = 0
            Rover.is_stuck = True
            Rover.stuck_time = Rover.total_time
            Rover.mode = 'unstuck'
        else:
            Rover.throttle_count += Rover.throttle_set[1]
    # Rover is running
    elif Rover.vel >= 0.2:
        Rover.is_stuck = False
        Rover.throttle_count = 0
    # For the strange situation when throttle cannot be set in forward mode
    elif Rover.mode == 'forward' and Rover.vel < 0.2:
        if Rover.forward_count > 1000:
            Rover.forward_count = 0
            Rover.is_stuck = True
            Rover.stuck_time = Rover.total_time
            Rover.mode = 'unstuck'
        else:
            Rover.forward_count += 1


# Check if the rover is making circle
def check_circle(Rover):
    if Rover.steer == 15 and Rover.vel > 0.5:
        # circling
        if Rover.circle_time > 50:
            Rover.circle_time = 0
            Rover.is_stuck = True
            Rover.stuck_time = Rover.total_time
            Rover.mode = 'unstuck'
        elif Rover.steer != 15:
            Rover.circle_time = 0
            Rover.is_stuck = False
            Rover.stuck_time = 0
        else:
            Rover.circle_time += 0.2
    elif Rover.steer == -15 and Rover.vel > 0.5:
        # circling
        if Rover.circle_time > 50:
            Rover.circle_time = 0
            Rover.is_stuck = True
            Rover.stuck_time = Rover.total_time
            Rover.mode = 'unstuck'
        elif Rover.steer != -15:
            Rover.circle_time = 0
            Rover.is_stuck = False
            Rover.stuck_time = 0
        else:
            Rover.circle_time += 0.2


# Maintain a certain speed
def rover_maintain_speed(Rover, min, max):
    if Rover.vel > max:
        Rover.throttle = 0
        Rover.brake = Rover.brake_soft
    elif Rover.vel < min:
        Rover.throttle = Rover.throttle_set[1]
        Rover.brake = 0
    else:
        Rover.throttle = 0
        Rover.brake = 0


# Set throttle for rover
def set_rover_throttle(Rover):
    if len(Rover.nav_angles) >= Rover.go_forward * 3:
        Rover.throttle += Rover.throttle_set[2]
    elif len(Rover.nav_angles) >= Rover.go_forward * 2:
        Rover.throttle += Rover.throttle_set[1]
    elif len(Rover.nav_angles) > Rover.go_forward:
        Rover.throttle += Rover.throttle_set[0]
    else:
        Rover.throttle = 0


# Get next steer
def get_navi_steer(Rover):
    if Rover.is_going_home:
        # get home pos
        home = Rover.home
        # get current rover pos
        pos = Rover.pos
        # convert them into world coords
        home_world_x, home_world_y = pix_to_world(home[0], home[1], pos[0], pos[1], Rover.yaw, Rover.worldmap.shape[0],
                                                  10)
        home_world_dist, home_world_angle = to_polar_coords(home_world_x, home_world_y)

        mean_angle = np.mean(Rover.nav_angles)

        # Steer toward home
        if mean_angle > home_world_angle:
            steer = -10
        else:
            steer = +10
    else:
        # Set steering to average angle clipped to the range +/- 15
        steer = np.mean(Rover.nav_angles * 180 / np.pi) + 14.9  # lean towards left side wall

    return np.clip(steer, -15, 15)
