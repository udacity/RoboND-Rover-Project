import numpy as np
import cv2
from termcolor import colored # added by me

# Identify pixels above the threshold
# Threshold of RGB > 160 does a nice job of identifying ground pixels only
def color_thresh(img, rgb_thresh=(160, 160, 160)):
    # Create an array of zeros same xy size as img, but single channel
    color_select = np.zeros_like(img[:,:,0])
    # Require that each pixel be above all three threshold values in RGB
    # above_thresh will now contain a boolean array with "True"
    # where threshold was met
    above_thresh = (img[:,:,0] > rgb_thresh[0]) & \
                   (img[:,:,1] > rgb_thresh[1]) & \
                   (img[:,:,2] > rgb_thresh[2])
    # Index the array of zeros with the boolean array and set to 1
    color_select[above_thresh] = 1
    return color_select

# Define a function to convert from image coords to rover coords
def rover_coords(binary_img):
    # Identify nonzero pixels
    xpos, ypos = binary_img.nonzero()
    # Calculate pixel positions with reference to the rover position being at the 
    # center bottom of the image.  
    x_pixel = -xpos + binary_img.shape[0]
    y_pixel = -ypos + binary_img.shape[1] / 2
    return x_pixel, y_pixel

# Define a function to get rover coords within specified range
def get_rover_coords_within_range(binary_img, range=50):
    # Identify nonzero pixels
    xpos, ypos = binary_img.nonzero()
    # Calculate pixel positions with reference to the rover position being at the 
    # center bottom of the image.  
    x_pixel = -xpos + binary_img.shape[0]
    y_pixel = -ypos + binary_img.shape[1] / 2
    dist = np.sqrt(x_pixel**2, y_pixel**2)
    return x_pixel[dist < range], y_pixel[dist < range]

# Define a function to convert to radial coords in rover space
def to_polar_coords(x_pixel, y_pixel):
    # Convert (x_pixel, y_pixel) to (distance, angle) 
    # in polar coordinates in rover space
    # Calculate distance to each pixel
    dist = np.sqrt(x_pixel**2 + y_pixel**2)
    # Calculate angle away from vertical for each pixel
    angles = np.arctan2(y_pixel, x_pixel)
    return dist, angles

# Define a function to map rover space pixels to world space
def rotate_pix(xpix, ypix, yaw):
    # Convert yaw to radians
    yaw_rad = yaw * np.pi / 180
    xpix_rotated = (xpix * np.cos(yaw_rad)) - (ypix * np.sin(yaw_rad))
    ypix_rotated = (xpix * np.sin(yaw_rad)) + (ypix * np.cos(yaw_rad))
    return xpix_rotated, ypix_rotated

def translate_pix(xpix_rot, ypix_rot, xpos, ypos, scale): 
    # Apply a scaling and a translation
    xpix_translated = (xpix_rot / scale) + xpos
    ypix_translated = (ypix_rot / scale) + ypos
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
    return x_pix_world, y_pix_world

# Define a function to perform a perspective transform
def perspect_transform(img, src, dst):
    M = cv2.getPerspectiveTransform(src, dst)
    warped = cv2.warpPerspective(img, M, (img.shape[1], img.shape[0]))# keep same size as input image
    mask = cv2.warpPerspective(np.ones_like(img[:, :, 0]), M, (img.shape[1], img.shape[0]))
    return warped, mask


def find_rocks(img, levels=(110, 110, 50)):
    rockpix = (img[:, :, 0] > levels[0]) & \
              (img[:, :, 1] > levels[1]) & \
              (img[:, :,  2] < levels[2])
    color_select = np.zeros_like(img[:, :, 0])
    color_select[rockpix ] = 1
    return rockpix

# Apply the above functions in succession and update the Rover state accordingly
def perception_step(Rover):
    # Perform perception steps to update Rover
    dst_size = 5
    bottom_offset = 6
    # source: bottom left, bottom right, top right, top left coordinate values
    source = np.float32([[14, 140], [301, 140], [200, 96], [118, 96]])
    # destination: bottom left, bottom right, top right, top left coordinate values
    destination = np.float32([
        [Rover.img.shape[1] / 2 - dst_size, Rover.img.shape[0] - bottom_offset],
        [Rover.img.shape[1] / 2 + dst_size, Rover.img.shape[0] - bottom_offset],
        [Rover.img.shape[1] / 2 + dst_size, Rover.img.shape[0] - 2 * dst_size - bottom_offset],
        [Rover.img.shape[1] / 2 - dst_size, Rover.img.shape[0] - 2 * dst_size - bottom_offset]])
    # warp, threshold, and apply mask to Rover.img
    warped, mask = perspect_transform(Rover.img, source, destination)
    threshed = color_thresh(warped, rgb_thresh=(180, 180, 165))
    obs_map = np.absolute(np.float32(threshed) - 1) * mask
    # update current vision image
    Rover.vision_image[:, :, 2] = threshed * 255
    Rover.vision_image[:, :, 0] = obs_map * 255
    # establish Rover centric coords and world cords
    xpix, ypix = get_rover_coords_within_range(threshed, range=50)
    world_size = Rover.worldmap.shape[0]
    scale = 2 * dst_size
    x_world, y_world = pix_to_world(
        xpix=xpix,
        ypix=ypix,
        xpos=Rover.pos[0],
        ypos=Rover.pos[1],
        yaw=Rover.yaw,
        world_size=world_size,
        scale=scale)
    # establish obstacle Rover centric coords and world coords
    obsxpix, obsypix = rover_coords(obs_map)
    obs_x_world, obs_y_world = pix_to_world(
        xpix=obsxpix,
        ypix=obsypix,
        xpos=Rover.pos[0],
        ypos=Rover.pos[1],
        yaw=Rover.yaw,
        world_size=world_size,
        scale=scale)
    # apply Rover pitch and roll tolerance cutoffs to improve fidelity
    if Rover.pitch <= Rover.pitch_tolerance and Rover.roll <= Rover.roll_tolerance:
        Rover.worldmap[y_world, x_world, 2] +=  10
        Rover.worldmap[obs_y_world, obs_x_world, 0] += 1
    dist, angles = to_polar_coords(xpix, ypix)
    # convert rover-centric pixel positions to polar coordinates
    Rover.nav_angles = angles
    # see if we can find some rocks
    rock_map = find_rocks(warped, levels=(110, 110, 50))
    if rock_map.any():
        Rover.sees_sample = True
        # estabish rock (sample) Rover centric coords and world coords
        rock_x, rock_y = rover_coords(rock_map)
        rock_x_world, rock_y_world = pix_to_world(
            xpix=rock_x,
            ypix=rock_y,
            xpos=Rover.pos[0],
            ypos=Rover.pos[1],
            yaw=Rover.yaw,
            world_size=world_size,
            scale=scale)
        rock_dist, rock_ang = to_polar_coords(rock_x, rock_y)
        rock_idx = np.argmin(rock_dist)
        rock_xcen = rock_x_world[rock_idx]
        rock_ycen = rock_y_world[rock_idx]
        # update nearest sample position in rover coords of Rover state
        Rover.nearest_sample_pos_in_rover_coords = np.array([rock_x[rock_idx], rock_y[rock_idx]])
        # update last nearest sample positions in rover coords of Rover state (only keep track of last 500)
        if not Rover.last_nearest_sample_pos_in_rover_coords.any():
            Rover.last_nearest_sample_pos_in_rover_coords = Rover.nearest_sample_pos_in_rover_coords
        else:
            Rover.last_nearest_sample_pos_in_rover_coords = np.vstack([Rover.nearest_sample_pos_in_rover_coords, Rover.last_nearest_sample_pos_in_rover_coords])[:30]
        Rover.worldmap[rock_ycen, rock_xcen, 1] = 255
        Rover.vision_image[:, :, 1] = rock_map * 255
    else:
        Rover.sees_sample = False
        Rover.vision_image[:, :, 1] = 0
    return Rover

