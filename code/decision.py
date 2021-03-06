

#!/usr/bin/env python
#     License:BSD
#     This file scanner.py is to perform scanning from fixed    locations, when this node is launched, then the locations should be passed via rosservice command in another terminal, the "number" of positions and the "coordinate" should be passed through terminal to the node, then the scanning process at each location will be repeated untils the end of scanning
#
#    Maintainer: Alexander.Kang
#
#    Email: alexander.kang@tum.de
#
#    Date: 11.02.2018

import numpy as np


# This is where you can build a decision tree for determining throttle, brake and steer 
# commands based on the output of the perception_step() function
def decision_step(Rover):

    # Implement conditionals to decide what to do given perception data
    # Here you're all set up with some basic functionality but you'll need to
    # improve on this decision tree to do a good job of navigating autonomously!

    # Example:
    # Check if we have vision data to make decisions with
    if (Rover.nav_angles is not None) and (Rover.image_close_sample != 1):
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
                    Rover.steer = 30 # Could be more clever here about which way to turn
                # If we're stopped but see sufficient navigable terrain in front then go!
                elif len(Rover.nav_angles) >= Rover.go_forward:
                    # Set throttle back to stored value
                    Rover.throttle = Rover.throttle_set
                    # Release the brake
                    Rover.brake = 0
                    # Set steer to mean angle
                    Rover.steer = np.clip(np.mean(Rover.nav_angles * 180/np.pi), -15, 15) #-15 15
                    Rover.mode = 'forward'
# Just to make the rover do something 
    # even if no modifications have been made to the code
    elif (Rover.nav_angles is not None) and (Rover.image_close_sample == 1):
        if Rover.mode == 'forward': 
            # Check the extent of navigable terrain
            if (len(Rover.rock_angles) >= 10):  
                Rover.steer = np.clip(np.mean(Rover.rock_angles * 180/np.pi), -30, 30)               
            elif(len(Rover.rock_angles) != 0):
                Rover.steer = np.clip(np.mean(Rover.rock_angles * 180/np.pi), -10, 10)
            if Rover.near_sample == 1:
                 Rover.brake = Rover.brake_set
                 Rover.mode = 'stop'
                 Rover.throttle = 0
        
                # Set steering to average angle clipped to the range +/- 15
            elif Rover.vel < Rover.max_vel:                   
                Rover.brake = 0
                Rover.throttle = Rover.throttle_set

        # If we're already in "stop" mode then make different decisions
        elif Rover.mode == 'stop':
            # If we're in stop mode but still moving keep braking
            if Rover.vel > 0.2:
                if (abs(np.mean(Rover.rock_angles * 180/np.pi)) >= 10):  
                    Rover.steer = np.clip(np.mean(Rover.rock_angles * 180/np.pi), -25, 25)               
                else:
                    Rover.steer = np.clip(np.mean(Rover.rock_angles * 180/np.pi), -10, 10)
                Rover.throttle = 0
                Rover.brake = Rover.brake_set
            # If we're not moving (vel < 0.2) then do something else
            elif Rover.vel <= 0.2:   
                if (abs(np.mean(Rover.rock_angles * 180/np.pi)) >= 10):  
                    Rover.steer = np.clip(np.mean(Rover.rock_angles * 180/np.pi), -15, 15)               
                else:
                    Rover.steer = np.clip(np.mean(Rover.rock_angles * 180/np.pi), -10, 10)
                    # Set throttle back to stored value
                if len(Rover.nav_angles) < Rover.go_forward:
                     Rover.throttle = 0
                    # Release the brake to allow turning
                     Rover.brake = 0
                    # Turn range is +/- 15 degrees, when stopped the next line will induce 4-wheel turning
                     Rover.steer = -15 # Could be more clever here about which way to turn
                # If we're stopped but see sufficient navigable terrain in front then go!
                elif len(Rover.nav_angles) >= Rover.go_forward:
                     # Set throttle back to stored value
                     Rover.throttle = Rover.throttle_set
                    # Release the brake
                     Rover.brake = 0
                    # Set steer to mean angle
                     Rover.steer = np.clip(np.mean(Rover.nav_angles * 180/np.pi), -15, 15) #-15 15
                     Rover.mode = 'forward'   
                  
    else:
        Rover.throttle = Rover.throttle_set
        Rover.steer = 0
        Rover.brake = 0    
    if Rover.near_sample and Rover.vel == 0 and not Rover.picking_up:
        Rover.send_pickup = True

    
    return Rover

