import numpy as np
import cv2
import matplotlib.image as mpimg
import matplotlib.pyplot as plt 

#path = "img1/robocam_2017_06_12_20_10_01_537.jpg"

#test_img = mpimg.imread(path)


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
    yaw_rad = yaw * np.pi/180.0
    xpix_rotated = xpix*np.cos(yaw_rad) - ypix * np.sin(yaw_rad)
    ypix_rotated = xpix * np.sin(yaw_rad) + ypix * np.cos(yaw_rad)
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
# helper function to detect objects

def detect_navi(img,rgb_thresh=(150,140,130)):
    
    image = np.copy(img)
    nav =  color_thresh(image,rgb_thresh)
    
    return nav
    


def detect_rock(img):
    image = np.copy(img)
    navigable = detect_navi(image,rgb_thresh=(60,60,60))
    rock = color_thresh(image,rgb_thresh=(100,100,0))
    rock[navigable>0] = 0
    return rock;

def detect_obstacle(rock,nav):
    obstacle = np.ones_like(rock)
    obstacle[nav>0] =0
    obstacle[rock>0] =0
    
    return obstacle

# fig = plt.figure(figsize=(12,9))

# plt.subplot(2,2,1)

# nav = detect_navi(test_img)
# plt.imshow(nav)
# rock = detect_rock(test_img)

# plt.subplot(2,2,2)
# plt.imshow(rock)

# plt.subplot(2,2,3)

# plt.imshow(test_img)


# plt.show()

# get 4 meters pixels

def get_5_meter_pixels(xpix,ypix):
    # a range between 4 and 5

    a = (xpix * xpix + ypix * ypix) <= 2400

    return xpix[a],ypix[a]

def get_7_meter_pixels(xpix,ypix):

    a = (xpix * xpix + ypix * ypix) < 4900

    return xpix[a],ypix[a]




# Apply the above functions in succession and update the Rover state accordingly
def perception_step(Rover):
    # Perform perception steps to update Rover()
    # TODO: 
    # NOTE: camera image is coming to you in Rover.img
    # 1) Define source and destination points for perspective transform
    dst_size = 5
    bottom_offset = 6
    src = np.float32([[36,128],[270,128],[195,94],[118,94]])
    dst = np.float32([[Rover.vision_image.shape[1]/2 - dst_size, Rover.vision_image.shape[0] - bottom_offset],
                  [Rover.vision_image.shape[1]/2 + dst_size, Rover.vision_image.shape[0] - bottom_offset],
                  [Rover.vision_image.shape[1]/2 + dst_size, Rover.vision_image.shape[0] - 2*dst_size - bottom_offset], 
                  [Rover.vision_image.shape[1]/2 - dst_size, Rover.vision_image.shape[0] - 2*dst_size - bottom_offset],
                  ])  
    # 2) Apply perspective transform
    warped = perspect_transform(Rover.img,src,dst)

    # 3) Apply color threshold to identify navigable terrain/obstacles/rock samples

    rock = detect_rock(warped)
    nav = detect_navi(warped)
    obstacle = detect_obstacle(rock,nav)

    # 4) Update Rover.vision_image (this will be displayed on left side of screen)
        # Example: Rover.vision_image[:,:,0] = obstacle color-thresholded binary image
        #          Rover.vision_image[:,:,1] = rock_sample color-thresholded binary image
        #          Rover.vision_image[:,:,2] = navigable terrain color-thresholded binary image


    Rover.vision_image[:,:,0] = obstacle* 255
    Rover.vision_image[:,:,1] = rock * 255
    Rover.vision_image[:,:,2] = nav * 255

    #plt.ion
    #fig=plt.figure(figsize=(12,9))
    #plt.imshow(Rover.vision_image)
    #plt.show()

    # 5) Convert map image pixel values to rover-centric coords
    robx ,roby = rover_coords(obstacle)
    rrockx,rrocky = rover_coords(rock)
    rnavx, rnavy = rover_coords(nav)
    #rrockx,rrocky = get_6_meter_pixels(rrockx,rrocky)
    rnavx, rnavy = get_5_meter_pixels(rnavx,rnavy)

    #print("rnavx, rnavy" + str(rnavx)+str(rnavy))


    # 6) Convert rover-centric pixel values to world coordinates
    #print(Rover.yaw)
    obx,oby = pix_to_world(robx,roby,Rover.pos[0],Rover.pos[1],Rover.yaw,200,10)
    rox,roy = pix_to_world(rrockx,rrocky,Rover.pos[0],Rover.pos[1],Rover.yaw,200,10)
    navx,navy = pix_to_world(rnavx,rnavy,Rover.pos[0],Rover.pos[1],Rover.yaw,200,10)


    # 7) Update Rover worldmap (to be displayed on right side of screen)
        # Example: Rover.worldmap[obstacle_y_world, obstacle_x_world, 0] += 1
        #          Rover.worldmap[rock_y_world, rock_x_world, 1] += 1
        #          Rover.worldmap[navigable_y_world, navigable_x_world, 2] += 1
    Rover.worldmap[oby, obx,0] += 1 
    Rover.worldmap[roy, rox,1] += 1
    Rover.worldmap[navy, navx,2] += 1

    # 8) Convert rover-centric pixel positions to polar coordinates

    rover_centric_pixel_distances, rover_centric_angles = to_polar_coords(rnavx,rnavy)
    # Update Rover pixel distances and angles
    Rover.nav_dists = rover_centric_pixel_distances
    Rover.nav_angles = rover_centric_angles
    #print(rover_centric_angles)

    Rover.rock_dists, Rover.rock_angles = to_polar_coords(rrockx,rrocky)



    #print(Rover.rock_dists)
    
    
    return Rover