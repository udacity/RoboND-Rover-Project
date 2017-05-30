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
    color_select[above_thresh] = 255
    # Return the binary image
    return color_select

#Detect golden rocks.
#Threshold hsv image.
def rock_thresh(img):
    img1 = np.zeros_like(img[:,:,0])
    lower_yellow = np.array([20, 100, 100])
    upper_yellow = np.array([30, 255, 255])
    r = img[:, :, 0]
    g = img[:, :, 1]
    b = img[:, :, 2]
    img1 = np.dstack((b, g, r))
    hsv = cv2.cvtColor(img1, cv2.COLOR_BGR2HSV)
    return cv2.inRange(hsv, lower_yellow, upper_yellow)

#Detect obstacles
#Threshold of (R or G or B) < 160
def obstacle_thresh(img, rgb_thresh = (160, 160, 160)):
    color_select1 = np.zeros_like(img[:,:,0])
    obstacle_th = (img[:,:,0] < rgb_thresh[0]) \
                | (img[:,:,1] < rgb_thresh[1]) \
                | (img[:,:,2] < rgb_thresh[2])
    # Index the array of zeros with the boolean array and set to 1
    color_select1[obstacle_th] = 255
    # Return the binary image
    return color_select1

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
    # Convert yaw to radians# Convert yaw to radians
    yaw = yaw * np.pi / 180
    # Apply a rotation
    xpix_rotated = xpix*np.cos(yaw) - ypix*np.sin(yaw)
    ypix_rotated = xpix*np.sin(yaw) + ypix*np.cos(yaw)
    # Return the result  
    return xpix_rotated, ypix_rotated

# Define a function to perform a translation
def translate_pix(xpix_rot, ypix_rot, xpos, ypos, scale): 
    # TODO:
    # Apply a scaling and a translation
    xpix_translated = np.int_(xpos + xpix_rot/scale)
    ypix_translated = np.int_(ypos + ypix_rot/scale)
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
    img = Rover.img
    # 1) Define source and destination points for perspective transform
    dst_size = 5
    bottom_offset = 6
    source = np.float32([[14, 140], [301 ,140],[200, 96], [118, 96]])
    destination = np.float32([[img.shape[1]/2 - dst_size, img.shape[0] - bottom_offset],
                      [img.shape[1]/2 + dst_size, img.shape[0] - bottom_offset],
                      [img.shape[1]/2 + dst_size, img.shape[0] - 2*dst_size - bottom_offset], 
                      [img.shape[1]/2 - dst_size, img.shape[0] - 2*dst_size - bottom_offset],
                      ])
    # 2) Apply perspective transform
    warped_img = perspect_transform(img, source, destination)
    
    # 3) Apply color threshold to identify navigable terrain/obstacles/rock samples
    terrain = color_thresh(warped_img)
    rock_samples = rock_thresh(warped_img)
    obstacles = obstacle_thresh(warped_img)
    
    # 4) Update Rover.vision_image (this will be displayed on left side of screen)
        # Example: Rover.vision_image[:,:,0] = obstacle color-thresholded binary image
        #          Rover.vision_image[:,:,1] = rock_sample color-thresholded binary image
        #          Rover.vision_image[:,:,2] = navigable terrain color-thresholded binary image
    Rover.vision_image[:,:,0] = obstacles
    Rover.vision_image[:,:,1] = rock_samples
    Rover.vision_image[:,:,2] = terrain

    # 5) Convert map image pixel values to rover-centric coords
    navigable_xpix, navigable_ypix = rover_coords(terrain)
    obstacle_xpix, obstacle_ypix = rover_coords(obstacles)
    rock_xpix, rock_ypix = rover_coords(rock_samples)
    
    # 6) Convert rover-centric pixel values to world coordinates
    navigable_x_world, navigable_y_world = pix_to_world(navigable_xpix, navigable_ypix, Rover.pos[0], Rover.pos[1], \
                                                        Rover.yaw, 200, 20)
    obstacle_x_world, obstacle_y_world = pix_to_world(obstacle_xpix, obstacle_ypix, Rover.pos[0], Rover.pos[1], \
                                                        Rover.yaw, 200, 20)
    rock_x_world, rock_y_world = pix_to_world(rock_xpix, rock_ypix, Rover.pos[0], Rover.pos[1], \
                                                        Rover.yaw, 200, 20)
    
    # 7) Update Rover worldmap (to be displayed on right side of screen)
        # Example: Rover.worldmap[obstacle_y_world, obstacle_x_world, 0] += 1
        #          Rover.worldmap[rock_y_world, rock_x_world, 1] += 1
        #          Rover.worldmap[navigable_y_world, navigable_x_world, 2] += 1
    Rover.worldmap[obstacle_y_world, obstacle_x_world, 0] += 1
    Rover.worldmap[rock_y_world, rock_x_world, 1] += 1
    Rover.worldmap[navigable_y_world, navigable_x_world, 2] += 1

    # 8) Convert rover-centric pixel positions to polar coordinates
    # Update Rover pixel distances and angles
        # Rover.nav_dists = rover_centric_pixel_distances
        # Rover.nav_angles = rover_centric_angles
    Rover.nav_dists, Rover.nav_angles = to_polar_coords(navigable_xpix, navigable_ypix)
    Rover.obst_dists, Rover.obst_angles = to_polar_coords(obstacle_xpix, obstacle_ypix)
    Rover.sample_dists, Rover.sample_angles = to_polar_coords(rock_xpix, rock_ypix)
    
 
    
    
    return Rover