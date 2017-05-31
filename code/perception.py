import numpy as np
import cv2
import matplotlib.image as mpimg

# Identify pixels above the threshold
# Threshold of RGB > 160 does a nice job of identifying ground pixels only
def color_thresh(img, rgb_thresh_min=(160, 160, 160), rgb_thresh_max=(256, 256, 256)):
    # Create an array of zeros same xy size as img, but single channel
    color_select = np.zeros_like(img[:,:,0])
    # Require that each pixel be above all three threshold values in RGB
    # above_thresh will now contain a boolean array with "True"
    # where threshold was met
    at_thresh = (img[:,:,0] > rgb_thresh_min[0]) \
                & (img[:,:,1] > rgb_thresh_min[1]) \
                & (img[:,:,2] > rgb_thresh_min[2]) \
                & (img[:,:,0] < rgb_thresh_max[0]) \
                & (img[:,:,1] < rgb_thresh_max[1]) \
                & (img[:,:,2] < rgb_thresh_max[2])
    # Index the array of zeros with the boolean array and set to 1
    color_select[at_thresh] = 1
    # Return the binary image
    return color_select

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
    yaw_rad = yaw * np.pi / 180
    xpix_rotated = (xpix * np.cos(yaw_rad)) - (ypix * np.sin(yaw_rad))
                            
    ypix_rotated = (xpix * np.sin(yaw_rad)) + (ypix * np.cos(yaw_rad))
    # Return the result  
    return xpix_rotated, ypix_rotated

# Define a function to perform a translation
def translate_pix(xpix_rot, ypix_rot, xpos, ypos, scale): 
    # TODO:
    # Apply a scaling and a translation
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


# Define a function to pass stored images to
# reading rover position and yaw angle from csv file
# This function will be used by moviepy to create an output video
def perception_step(Rover):
    # Example of how to use the Databucket() object defined above
    # to print the current x, y and yaw values 
    # print(data.xpos[data.count], data.ypos[data.count], data.yaw[data.count])

    # TODO: 
    # 1) Define source and destination points for perspective transform
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
    image = Rover.img
    destination = np.float32([[image.shape[1]/2 - dst_size, image.shape[0] - bottom_offset],
                      [image.shape[1]/2 + dst_size, image.shape[0] - bottom_offset],
                      [image.shape[1]/2 + dst_size, image.shape[0] - 2*dst_size - bottom_offset], 
                      [image.shape[1]/2 - dst_size, image.shape[0] - 2*dst_size - bottom_offset],
                      ])
    # 2) Apply perspective transform
    warped = perspect_transform(image, source, destination)
    # 3) Apply color threshold to identify navigable terrain/obstacles/rock samples

    navigable_threshed = color_thresh(warped, (160, 160, 160), (256, 256, 256))
    obstacle_threshed = color_thresh(warped, (0, 0, 0), (161, 161, 161))
    rock_threshed = color_thresh(warped, (110, 110, 5), (256, 256, 90))
    # 4) Update Rover.vision_image (this will be displayed on left side of screen)
    Rover.vision_image[:,:,0] = obstacle_threshed*255
    Rover.vision_image[:,:,1] = rock_threshed*255
    Rover.vision_image[:,:,2] = navigable_threshed*255
    # 5) Convert thresholded image pixel values to rover-centric coords
    #5.a navigable
    xpix_n, ypix_n = rover_coords(navigable_threshed)
    dist_n, angles_n = to_polar_coords(xpix_n, ypix_n)
    mean_dir_n = np.mean(angles_n)
    #5.b obstacle
    xpix_o, ypix_o = rover_coords(obstacle_threshed)
    dist_o, angles_o = to_polar_coords(xpix_o, ypix_o)
    mean_dir_o = np.mean(angles_o)
    #5.a rock
    xpix_r, ypix_r = rover_coords(rock_threshed)
    dist_r, angles_r = to_polar_coords(xpix_r, ypix_r)
    mean_dir_r = np.mean(angles_r)
    # 6) Convert rover-centric pixel values to world coords
    #6.a navigable
    x_world_n, y_world_n = pix_to_world(xpix_n, ypix_n, Rover.pos[0], Rover.pos[1], Rover.yaw, 200, 10)
    #6.b obstacle
    x_world_o, y_world_o = pix_to_world(xpix_o, ypix_o, Rover.pos[0], Rover.pos[1], Rover.yaw, 200, 10)
    #6.c rock
    x_world_r, y_world_r = pix_to_world(xpix_r, ypix_r, Rover.pos[0], Rover.pos[1], Rover.yaw, 200, 10)
    # 7) Update worldmap (to be displayed on right side of screen)
    #7.a
    Rover.worldmap[y_world_n, x_world_n, 2] += 1
    #7.b
    Rover.worldmap[y_world_o, x_world_o, 0] += 1
    #7.c
    Rover.worldmap[y_world_r, x_world_r, 1] += 1

    print(Rover.pos)

    # 8) Convert rover-centric pixel positions to polar coordinates
    # Update Rover pixel distances and angles
    Rover.nav_dists = dist_n
    Rover.nav_angles = angles_n
    
    return Rover