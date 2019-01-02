import numpy as np
import pdb


# This is where you can build a decision tree for determining throttle, brake and steer 
# commands based on the output of the perception_step() function

def rover_stuck(Rover):
    # Check if rover is stuck
    
    # check if stuck in same coordinates
    
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
        
    return Rover


def decision_step(Rover):

    # Implement conditionals to decide what to do given perception data
    # Here you're all set up with some basic functionality but you'll need to
    # improve on this decision tree to do a good job of navigating autonomously!
    
    def mean_without_outlier(arr):
        low = np.percentile(arr, 25)
        high = np.percentile(arr, 75)
        total = list()
        for c in arr:
            if c >= low and c <= high:
                total.append(c)
        return np.mean(total)
    
    # Example:
    # Check if we have vision data to make decisions with
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
                Rover.steer = np.clip(np.mean(Rover.nav_angles * 180/np.pi), -15, 15)
                                
                if len(Rover.nav_angles) >= Rover.random_direction_angles:
                    if np.random.random() <= 0.4:
                        # print("get random direction because angle: ", len(Rover.nav_angles))
                        if Rover.steer >= 0:
                            Rover.steer = np.clip(np.random.normal(Rover.steer - 5, 5), -15, 15)
                        else:
                            Rover.steer = np.clip(np.random.normal(Rover.steer + 5, 5), -15, 15)
                
                #np.clip(mean_without_outlier(Rover.nav_angles * 180/np.pi), -15, 15)

                
            
            # If there's a lack of navigable terrain pixels then go to 'stop' mode
            elif len(Rover.nav_angles) < Rover.stop_forward:
                    # Set mode to "stop" and hit the brakes!
                    Rover.throttle = 0
                    # Set brake to stored brake value
                    Rover.brake = Rover.brake_set
                    Rover.steer = 0
                    Rover.mode = 'stop'
                    
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
            #Rover = rover_stuck(Rover)

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
    
    return Rover

