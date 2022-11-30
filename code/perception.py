import numpy as np
import cv2


# Identify pixels above the threshold
# Threshold of RGB > 160 does a nice job of identifying ground pixels only
def color_thresh(img, rgb_thresh=(160, 160, 160)):
    # Create an array of zeros same xy size as img, but single channel
    color_select = np.zeros_like(img[:, :, 0])
    # Require that each pixel be above all three threshold values in RGB
    # above_thresh will now contain a boolean array with "True"
    # where threshold was met
    above_thresh = (img[:, :, 0] > rgb_thresh[0]) \
                   & (img[:, :, 1] > rgb_thresh[1]) \
                   & (img[:, :, 2] > rgb_thresh[2])
    # Index the array of zeros with the boolean array and set to 1
    color_select[above_thresh] = 1
    # Return the binary image
    return color_select


# Invert the color threshold function to get obstacles
def get_obstacles_1(img, rgb_thresh=(160, 160, 160)):
    # Create an array of ones same xy size as img, but single channel
    obstacles = np.ones_like(img[:, :, 0])
    # Require that each pixel be above all three threshold values in RGB
    # above_thresh will now contain a boolean array with "True"
    # where threshold was met
    above_thresh = (img[:, :, 0] > rgb_thresh[0]) \
                   & (img[:, :, 1] > rgb_thresh[1]) \
                   & (img[:, :, 2] > rgb_thresh[2])
    # Index the array of zeros with the boolean array and set to 0
    obstacles[above_thresh] = 0
    # Return the binary image
    return obstacles


# Identify the rocks
def get_rock(img, rock_low_thresh=(0, 0, 0), rock_high_thresh=(255, 255, 255)):
    # Create an array of zeros same xy size as img, but single channel
    rock = np.zeros_like(img[:, :, 0])
    # Require that each pixel be above all three threshold values in RGB
    # above_thresh will now contain a boolean array with "True"
    # where threshold was met
    above_low = (img[:, :, 0] > rock_low_thresh[0]) \
                & (img[:, :, 1] > rock_low_thresh[1]) \
                & (img[:, :, 2] > rock_low_thresh[2])
    below_high = (img[:, :, 0] < rock_high_thresh[0]) \
                 & (img[:, :, 1] < rock_high_thresh[1]) \
                 & (img[:, :, 2] < rock_high_thresh[2])
    in_range = above_low & below_high
    # Index the array of zeros with the boolean array and set to 1
    rock[in_range] = 1
    # Return the binary image
    return rock


# Define a function to convert from image coords to rover coords
def rover_coords(binary_img):
    # Identify nonzero pixels
    ypos, xpos = binary_img.nonzero()
    # Calculate pixel positions with reference to the rover position being at the 
    # center bottom of the image.  
    x_pixel = -(ypos - binary_img.shape[0]).astype(np.float)
    y_pixel = -(xpos - binary_img.shape[1] / 2).astype(np.float)
    return x_pixel, y_pixel


# Define a function to convert to radial coords in rover space
def to_polar_coords(x_pixel, y_pixel):
    # Convert (x_pixel, y_pixel) to (distance, angle) 
    # in polar coordinates in rover space
    # Calculate distance to each pixel
    dist = np.sqrt(x_pixel ** 2 + y_pixel ** 2)
    # Calculate angle away from vertical for each pixel
    angles = np.arctan2(y_pixel, x_pixel)
    return dist, angles


# Define a function to map rover space pixels to world space
def rotate_pix(xpix, ypix, yaw):
    # Convert yaw to radians
    yaw_rad = yaw * np.pi / 180
    xpix_rotated = (xpix * np.cos(yaw_rad)) - (ypix * np.sin(yaw_rad))

    ypix_rotated = (xpix * np.sin(yaw_rad)) + (ypix * np.cos(yaw_rad))
    # Return the result  
    return xpix_rotated, ypix_rotated


# Translate pixels
def translate_pix(xpix_rot, ypix_rot, xpos, ypos, scale):
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
    warped = cv2.warpPerspective(img, M, (img.shape[1], img.shape[0]))  # keep same size as input image

    return warped


# Define a function to check the rocks in view
def check_rocks(Rover):
    if len(Rover.rock_angles):  # there's rock
        if np.mean(Rover.rock_dists) < 50:
            return True
    return False


# Apply the above functions in succession and update the Rover state accordingly
def perception_step(Rover):
    # Perform perception steps to update Rover()
    # TODO: 
    # NOTE: camera image is coming to you in Rover.img

    # Get the image
    rover_img = Rover.img

    # 1) Apply color threshold to identify navigable terrain/obstacles/rock samples
    navigable = color_thresh(rover_img)
    obstacles = get_obstacles_1(rover_img)
    rock_thresh_low = (160, 160, 0)
    rock_thresh_high = (255, 255, 140)
    rock = get_rock(rover_img, rock_thresh_low, rock_thresh_high)

    # 2) Define source and destination points for perspective transform
    # These source and destination points are defined to warp the image
    # to a grid where each 10x10 pixel square represents 1 square meter

    # Define source point
    source = np.float32([[14, 140], [301, 140], [200, 96], [118, 96]])
    # The destination box will be 2*dst_size on each side
    dst_size = 5
    # Set a bottom offset to account for the fact that the bottom of the image 
    # is not the position of the rover but a bit in front of it
    # this is just a rough guess, feel free to change it!
    bottom_offset = 6

    # 3) Apply perspective transform
    destination = np.float32([[rover_img.shape[1] / 2 - dst_size, rover_img.shape[0] - bottom_offset],
                              [rover_img.shape[1] / 2 + dst_size, rover_img.shape[0] - bottom_offset],
                              [rover_img.shape[1] / 2 + dst_size, rover_img.shape[0] - 2 * dst_size - bottom_offset],
                              [rover_img.shape[1] / 2 - dst_size, rover_img.shape[0] - 2 * dst_size - bottom_offset],
                              ])
    navi_warped = perspect_transform(navigable, source, destination)
    obstacles_warped = perspect_transform(obstacles, source, destination)
    rock_warped = perspect_transform(rock, source, destination)

    # 4) Update Rover.vision_image (this will be displayed on left side of screen)
    # Example: Rover.vision_image[:,:,0] = obstacle color-thresholded binary image
    #          Rover.vision_image[:,:,1] = rock_sample color-thresholded binary image
    #          Rover.vision_image[:,:,2] = navigable terrain color-thresholded binary image

    Rover.vision_image[:, :, 0] = obstacles_warped * 255
    Rover.vision_image[:, :, 1] = rock_warped * 255
    Rover.vision_image[:, :, 2] = navi_warped * 255

    # 5) Convert map image pixel values to rover-centric coords
    # limit the view of the navi image to get better result
    row1 = np.int(rover_img.shape[0] - dst_size * 5)
    row2 = np.int(rover_img.shape[0] - bottom_offset)
    col1 = np.int(rover_img.shape[1] / 2 - dst_size * 4)
    col2 = np.int(rover_img.shape[1] / 2 + dst_size * 4)

    front_navi = navi_warped[row1:row2, col1:col2]
    nav_xpix, nav_ypix = rover_coords(front_navi)
    obs_xpix, obs_ypix = rover_coords(obstacles_warped)
    rock_xpix, rock_ypix = rover_coords(rock_warped)

    # 6) Convert rover-centric pixel values to world coordinates
    # get current x, y and yaw for the world coords mapping
    xp = Rover.pos[0]
    yp = Rover.pos[1]
    yaw = Rover.yaw
    scale = 10

    nav_x_world, nav_y_world = pix_to_world(nav_xpix, nav_ypix, xp,
                                            yp, yaw,
                                            Rover.worldmap.shape[0], scale)
    obs_x_world, obs_y_world = pix_to_world(obs_xpix, obs_ypix, xp,
                                            yp, yaw,
                                            Rover.worldmap.shape[0], scale)
    rock_x_world, rock_y_world = pix_to_world(rock_xpix, rock_ypix, xp,
                                              yp, yaw,
                                              Rover.worldmap.shape[0], scale)

    # 7) Update Rover worldmap (to be displayed on right side of screen)
    # Example: Rover.worldmap[obstacle_y_world, obstacle_x_world, 0] += 1
    #          Rover.worldmap[rock_y_world, rock_x_world, 1] += 1
    #          Rover.worldmap[navigable_y_world, navigable_x_world, 2] += 1
    # filter the rover state
    roll = Rover.roll
    pitch = Rover.pitch

    if (roll > 359.5 or roll < 0.5) and (pitch < 0.5 or pitch > 359.5):
        Rover.worldmap[obs_y_world, obs_x_world, 0] += 1
        Rover.worldmap[rock_y_world, rock_x_world, 1] += 1
        Rover.worldmap[nav_y_world, nav_x_world, 2] += 1

    # 8) Convert rover-centric pixel positions to polar coordinates
    # Update Rover pixel distances and angles
    rover_centric_pixel_distances, rover_centric_angles = to_polar_coords(nav_xpix, nav_ypix)

    Rover.nav_dists = rover_centric_pixel_distances
    Rover.nav_angles = rover_centric_angles

    # Note down rock pos in view
    rock_distances, rock_angles = to_polar_coords(rock_xpix, rock_ypix)

    Rover.rock_dists = rock_distances
    Rover.rock_angles = rock_angles

    return Rover
