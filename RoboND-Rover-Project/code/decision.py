import numpy as np
import time
import pdb

"""
Points to improve on decision process

Efficiency
- cling to walls as rocks are near walls
Problems
- Bot getting stuck between rocks
- Only driving in circles due to large open space. Solved by sticking to the edges
- missed objective as it wasn't facing it
- keep moving back due to stuck
"""

# This is where you can build a decision tree for determining throttle, brake and steer
# commands based on the output of the perception_step() function

def rover_stuck(Rover):
    """
    1. Check if collected samples is 6
    1. if in the same position for a certain time
    2. if see navigable terrian but stuck in same location reverse
    3. going forward

    """

    # Check if stuck
    if Rover.vel <= 0.2 and Rover.vel >= -0.2 and not Rover.is_stuck:
        # start the stuck timer
        # check to see if timer has been started already
        if Rover.stuck_timer == None:
            Rover.stuck_timer = Rover.total_time
        # else check if timer if over wait time.
        elif Rover.total_time - Rover.stuck_timer >= Rover.wait:
            Rover.is_stuck = True
    # check
    if Rover.is_stuck and (Rover.throttle!= 0 or Rover.brake !=0):
        # moving forward
        if Rover.vel >= 0.5 or Rover.vel <= -0.5:
            Rover.stuck_timer = None
            Rover.is_stuck = False
            Rover.steer = np.clip(np.mean(Rover.nav_angles * 180/np.pi), -15, 15)
        else:
            Rover.brake = 0
            Rover.throttle = -Rover.throttle_set
            Rover.steer = 0
    else:
        Rover.is_stuck = False
        Rover.stuck_time = None

    # rover is stuck and rover is not moving forward
    # elif Rover.is_stuck and (Rover.throttle!= 0 or Rover.brake !=0):
    #     print("stuck!!!")
    #
    #     if Rover.vel >= 0.5 or Rover.vel <= -0.5:
    #         Rover.stuck_timer = None
    #         Rover.is_stuck = False
    #         Rover.steer = Rover.steer = np.max((Rover.nav_angles) * 180/np.pi) + 30
    #     else:
    #         Rover.brake = 0
    #         Rover.throttle = -Rover.throttle_set
    #         Rover.steer = 0
    # else:
    #     Rover.is_stuck = False
    #     Rover.stuck_time = None

    return Rover

def decision_step(Rover):

    # Implement conditionals to decide what to do given perception data
    # Here you're all set up with some basic functionality but you'll need to
    # improve on this decision tree to do a good job of navigating autonomously!

    # Example:
    # Check if we have vision data to make decisions with

    # store starting position
    if Rover.start_pos == None:
        Rover.start_pos = Rover.pos


    if Rover.samples_collected == 6:
        print("return to mothership")
        if abs(Rover.pos[0] - Rover.start_pos[0]) < 20 and abs(Rover.pos[1] - Rover.start_pos[1]) < 20:
            Rover.throttle = 0
            Rover.brake = Rover.brake_set
            Rover.steer = 0
            print("Send signal to return mothership")
            return Rover

    # check if stuck in cricle
    if abs(Rover.steer) == 15:
        if time.time() - Rover.circles_time < Rover.max_circle_time:
            # Initiate doughnut mode
            print('STEERING LOCK DETECTED')
            Rover.stuck_circle = True

    # Rover is caught in a doughnut so try to escape it
    if Rover.stuck_circle:
        print('Getting out of circle motion')
        if time.time() - Rover.circles_time> (Rover.max_circle_time + 5):
            Rover.mode = 'forward'
            Rover.circles_time = time.time()
        else:
            # Perform evasion to get out of doughnut
            Rover.throttle = 0
            Rover.brake = Rover.brake_set
            if Rover.steer > 0:
                Rover.steer = -15
            else:
                Rover.steer = 15
        return Rover

    if Rover.nav_angles is not None:
        # Check for Rover.mode status
        if Rover.mode == 'forward':
            # Check the extent of navigable terrain
            if len(Rover.nav_angles) >= Rover.stop_forward:
                # If mode is forward, navigable terrain looks good
                # and velocity is below max, then throttle
                if Rover.vel < Rover.max_vel:
                    # Set throttle value to throttle setting
                    Rover.throttle = Rover.throttle_set
                else: # Else coast
                    Rover.throttle = 0
                Rover.brake = 0
                # Set steering to average angle clipped to the range +/- 15
                #Rover.steer = np.clip(np.mean(Rover.nav_angles * 180/np.pi), -15, 15)
                #Rover.steer = np.mean(Rover.nav_angles * 180/np.pi)
                #Rover.steeri = np.max((Rover.nav_angles) * 180/np.pi) - 30
                Rover.steer = np.clip(np.mean(Rover.nav_angles * 180/np.pi),
                              -15, 15)

            #If there's a lack of navigable terrain pixels then go to 'stop' mode
            elif len(Rover.nav_angles) < Rover.stop_forward:
                # Set mode to "stop" and hit the brakes!
                Rover.throttle = 0
                # Set brake to stored brake value
                Rover.brake = Rover.brake_set
                Rover.steer = 0
                Rover.mode = 'stop'

            # sample seen
            if Rover.sample_seen:
                if Rover.picking_up !=0 :
                    Rover.sample_seen = False

                avg_rock_angle = np.mean(Rover.rock_angle * 180/np.pi)

                if -15 < avg_rock_angle < 15:

                    if max(Rover.rock_dist) < 10:
                        Rover.throttle = 0
                        Rover.brake = Rover.brake_set
                        Rover.steer = avg_rock_angle
                    # SLOW DOWN when approaching sample
                    elif 20 < max(Rover.rock_dist) < 40:
                        Rover.throttle = 0.1
                        Rover.steer = avg_rock_angle
                    else:
                        print('Approach rock sample')
                        Rover.throttle = Rover.throttle_set
                        Rover.steer = avg_rock_angle

                elif -50 < avg_rock_angle < 50:
                    print('Rotate to my precious')

                    if Rover.vel > 0 and max(Rover.rock_dist) < 100:
                        Rover.throttle = 0
                        Rover.brake = Rover.brake_set
                        Rover.steer = 0
                    else:
                        Rover.throttle = 0
                        Rover.brake = 0
                        Rover.steer = avg_rock_angle/6

                else:
                    print('Ive been duped. No sample here')
                    Rover.sample_seen = False

            if Rover.vel <= 0.2 and Rover.vel >= -0.2:
                if Rover.stuck_timer == None:
                    # anchor new stuck time a current time.
                    Rover.stuck_timer = Rover.total_time
                if Rover.total_time - Rover.stuck_time_time >= Rover.wait:
                    Rover.is_stuck = True

            if Rover.vel <= 0.2 and Rover.vel >= -0.2 and Rover.is_stuck:
                if Rover.stuck_time == None:
                    # anchor new stuck time a current time.
                    Rover.stuck_time = Rover.total_time
                if Rover.total_time - Rover.stuck_time_time >= Rover.wait:
                    Rover.is_stuck = True

            # rover is stuck and rover is not moving forward
            elif Rover.is_stuck and (Rover.throttle!= 0 or Rover.brake !=0):
                if Rover.vel >= 0.5 or Rover.vel <= -0.5:
                    Rover.stuck_time = None
                    Rover.is_stuck = False
                    Rover.steer = np.clip(np.mean(Rover.nav_angles * 180/np.pi), -15, 15)
                else:
                    Rover.brake = 0
                    Rover.throttle = -Rover.throttle_set
                    Rover.steer = 0
            else:
                Rover.is_stuck = False
                Rover.stuck_time = None

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
                # If we're stopped but see sufficient navigable terrain in front then go!
                if len(Rover.nav_angles) >= Rover.go_forward:
                    # Set throttle back to stored value
                    Rover.throttle = Rover.throttle_set
                    # Release the brake
                    Rover.brake = 0
                    # Set steer to mean angle
                    Rover.steer = np.clip(np.mean(Rover.nav_angles * 180/np.pi), -15, 15)
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
        Rover.sample_seen = False

    return Rover
