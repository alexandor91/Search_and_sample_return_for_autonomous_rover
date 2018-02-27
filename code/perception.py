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
import cv2

# Identify pixels above the threshold
# Threshold of RGB > 160 does a nice job of identifying ground pixels only
def color_thresh(img, rgb_thresh=(160, 160, 160)):
    # Create an array of zeros same xy size as img, but single channel
    color_select = np.zeros_like(img[:,:,0])
    # Require that each pixel be above all three threshold values in RGB
    # above_thresh will now contain a boolean array with "True"
    # where threshold was met
    above_thresh = (img[:,:,0] > rgb_thresh[0]) \
                & (img[:,:,1] > rgb_thresh[1]) \
                & (img[:,:,2] > rgb_thresh[2])
    # Index the array of zeros with the boolean array and set to 1
    color_select[above_thresh] = 1
    # Return the binary image
    return color_select
def obs_thresh(img, rgb_thresh=(160, 160, 160)):
    # Create an array of zeros same xy size as img, but single channel
    obs_select = np.zeros_like(img[:,:,0])
    # Require that each pixel be above all three threshold values in RGB
    # above_thresh will now contain a boolean array with "True"
    # where threshold was met
    below_thresh = (img[:,:,0] < rgb_thresh[0]) \
                & (img[:,:,1] < rgb_thresh[1]) \
                & (img[:,:,2] < rgb_thresh[2])
    # Index the array of zeros with the boolean array and set to 1
    obs_select[below_thresh] = 1
    # Return the binary image
    return obs_select

def rock_thresh(img):
    # Create an array of zeros same xy size as img, but single channel
    rock_select = np.zeros_like(img[:,:,0])
    hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)

    # define range of yellow color in HSV
    lower_yellow = np.array([90,160,140])
    upper_yellow = np.array([150,255,250])

    # Threshold the HSV image to get only blue colors
    mask = cv2.inRange(hsv, lower_yellow, upper_yellow)
    count_rock = np.count_nonzero(mask)

    # Bitwise-AND mask and original image
    res = cv2.bitwise_and(img,img, mask= mask)
    #plt.subplot(223)
    #plt.imshow(mask, cmap='gray')
    #plt.subplot(224)
    #rockres_threshed = rock_thresh(rock_img)
    #plt.imshow(res, cmap='gray')
    
    return mask, count_rock

# Define a function to convert to rover-centric coordinates
def rover_coords(binary_img):
    # Identify nonzero pixels
    ypos, xpos = binary_img.nonzero()
    # Calculate pixel positions with reference to the rover position being at the 
    # center bottom of the image.  
    x_pixel = np.absolute(ypos - binary_img.shape[0]).astype(np.float)
    y_pixel = -(xpos - binary_img.shape[0]).astype(np.float)
    return x_pixel, y_pixel


# Define a function to convert to radial coords in rover space
def to_polar_coords(x_pixel, y_pixel):
    # Convert (x_pixel, y_pixel) to (distance, angle) 
    # in polar coordinates in rover space
    # Calculate distance to each pixel
    dist = np.sqrt(x_pixel**2 + y_pixel**2)
    # Calculate angle away from vertical for each pixel
    angles = np.arctan2(y_pixel, x_pixel)
    return dist, angles

# Define a function to apply a rotation to pixel positions
def rotate_pix(xpix, ypix, yaw):
    # TODO:
    # Convert yaw to radians
    # Apply a rotation
    xpix_rotated = 0
    ypix_rotated = 0
    yaw_rad = yaw * np.pi / 180
    xpix_rotated = (xpix * np.cos(yaw_rad)) - (ypix * np.sin(yaw_rad))
                            
    ypix_rotated = (xpix * np.sin(yaw_rad)) + (ypix * np.cos(yaw_rad))
    # Return the result  
    return xpix_rotated, ypix_rotated

# Define a function to perform a translation
def translate_pix(xpix_rot, ypix_rot, xpos, ypos, scale): 
    # TODO:
    # Apply a scaling and a translation
    xpix_translated = 0
    ypix_translated = 0
    xpix_translated = (xpix_rot / scale) + xpos
    ypix_translated = (ypix_rot / scale) + ypos
    # Return the result  
    return xpix_translated, ypix_translated

# Define a function to apply rotation and translation (and clipping)
# Once you define the two functions above this function should work
def pix_to_world(xpix, ypix, xpos, ypos, yaw, world_size, scale):
    # Apply rotation
    xpix_rot, ypix_rot = rotate_pix(xpix, ypix, yaw)
    # Apply translation
    xpix_tran, ypix_tran = translate_pix(xpix_rot, ypix_rot, xpos, ypos, scale)
    # Perform rotation, translation and clipping all at once
    x_pix_world = np.clip(np.int_(xpix_tran), 0, world_size - 1)
    y_pix_world = np.clip(np.int_(ypix_tran), 0, world_size - 1)
    # Return the result
    return x_pix_world, y_pix_world

# Define a function to perform a perspective transform
def perspect_transform(img, src, dst):
           
    M = cv2.getPerspectiveTransform(src, dst)
    warped = cv2.warpPerspective(img, M, (img.shape[1], img.shape[0]))# keep same size as input image
    
    return warped


# Apply the above functions in succession and update the Rover state accordingly
def perception_step(Rover):
    # Perform perception steps to update Rover()
    # TODO: 
    # NOTE: camera image is coming to you in Rover.img
    # 1) Define source and destination points for perspective transform
    # 2) Apply perspective transform
    # 3) Apply color threshold to identify navigable terrain/obstacles/rock samples
    # 4) Update Rover.vision_image (this will be displayed on left side of screen)
        # Example: Rover.vision_image[:,:,0] = obstacle color-thresholded binary image
        #          Rover.vision_image[:,:,1] = rock_sample color-thresholded binary image
        #          Rover.vision_image[:,:,2] = navigable terrain color-thresholded binary image

    # Define calibration box in source (actual) and destination (desired) coordinates
    # These source and destination points are defined to warp the image
    # to a grid where each 10x10 pixel square represents 1 square meter
    # The destination box will be 2*dst_size on each side
    dst_size = 5 
    # Set a bottom offset to account for the fact that the bottom of the image 
    # is not the position of the rover but a bit in front of it
    # this is just a rough guess, feel free to change it!
    bottom_offset = 6
    source = np.float32([[14, 140], [301 ,140],[200, 96], [118, 96]])
    destination = np.float32([[Rover.img.shape[1]/2 - dst_size, Rover.img.shape[0] - bottom_offset],
                  [Rover.img.shape[1]/2 + dst_size, Rover.img.shape[0] - bottom_offset],
                  [Rover.img.shape[1]/2 + dst_size, Rover.img.shape[0] - 2*dst_size - bottom_offset], 
                  [Rover.img.shape[1]/2 - dst_size, Rover.img.shape[0] - 2*dst_size - bottom_offset],
                  ])
    warped = perspect_transform(Rover.img, source, destination)
    
    color_threshed = color_thresh(warped)
    obs_threshed = obs_thresh(warped)
    
    rock_threshed, count_rock_pixel = rock_thresh(warped) 
         
    Rover.vision_image[:,:,0] = obs_threshed
    Rover.vision_image[:,:,1] = rock_threshed
    Rover.vision_image[:,:,2] = color_threshed

     
    scale = 10
    # 5) Convert map image pixel values to rover-centric coords
    # 6) Convert rover-centric pixel values to world coordinates
    # 7) Update Rover worldmap (to be displayed on right side of screen)
        # Example: Rover.worldmap[obstacle_y_world, obstacle_x_world, 0] += 1
        #          Rover.worldmap[rock_y_world, rock_x_world, 1] += 1
        #          Rover.worldmap[navigable_y_world, navigable_x_world, 2] += 1
    rover_nav_xpix, rover_nav_ypix = rover_coords(color_threshed)
    nav_x_world, nav_y_world = pix_to_world(rover_nav_xpix, rover_nav_ypix,
                                            Rover.pos[0],Rover.pos[1],Rover.yaw, 
                                            200, scale)
    rover_obs_xpix, rover_obs_ypix = rover_coords(obs_threshed)
    obs_x_world, obs_y_world = pix_to_world(rover_obs_xpix, rover_obs_ypix,
                                            Rover.pos[0],Rover.pos[1],Rover.yaw, 
                                            200, scale)
    rover_rock_xpix, rover_rock_ypix = rover_coords(rock_threshed)
    rock_x_world, rock_y_world = pix_to_world(rover_rock_xpix, rover_rock_ypix,
                                            Rover.pos[0],Rover.pos[1],Rover.yaw, 
                                            200, scale)
    Rover.worldmap[obs_y_world, obs_x_world, 0] += 1
    Rover.worldmap[rock_y_world, rock_x_world, 1] += 1
    Rover.worldmap[nav_y_world, nav_x_world, 2] += 1

    # 8) Convert rover-centric pixel positions to polar coordinates
    # Update Rover pixel distances and angles
        # Rover.nav_dists = rover_centric_pixel_distances
        # Rover.nav_angles = rover_centric_angles
    rover_centric_pixel_distances, rover_centric_angles = to_polar_coords(rover_nav_xpix,rover_nav_ypix)
    Rover.nav_dists = rover_centric_pixel_distances
    Rover.nav_angles = rover_centric_angles
    
    rock_angles, rock_dists = to_polar_coords(rover_rock_xpix,rover_rock_ypix) # Angles of navigable terrain pixels
    Rover.rock_angles = rock_angles
    Rover.rock_dists = rock_dists # Distances of navigable terrain pixels
    Rover.count_rock_pixel  = count_rock_pixel
    
    if len(Rover.rock_angles) > 30:
        Rover.image_close_sample = 1
        rock_pos_distances, rock_pos_angles = to_polar_coords(rover_rock_xpix,rover_rock_ypix)

   
    return Rover
