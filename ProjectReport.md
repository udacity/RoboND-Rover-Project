## Project: Search and Sample Return

Source code https://github.com/boldbinarywisdom/RoboND-Rover-Project

The purpose of this project is to write code to drive rover autonomoulsy in a map that is simulated environment with objects of interests and terrain where rover cannot drive. arned about in the lesson, and build upon them.


## Steps

** Training / Calibration **  

* Download the simulator and take data in "Training Mode"
* Test out the functions in the Jupyter Notebook provided
* Add functions to detect obstacles and samples of interest (golden rocks)
* Fill in the `process_image()` function with the appropriate image processing steps (perspective transform, color threshold etc.) to get from raw images to a map.  The `output_image` you create in this step should demonstrate that your mapping pipeline works.
* Use `moviepy` to process the images in your saved dataset with the `process_image()` function.  Include the video you produce as part of your submission.

** Autonomous Navigation / Mapping **

* `perception_step()` function within the `perception.py` has image processing functions to create a map and update `Rover()` object
* `decision_step()` function within the `decision.py` has conditional statements that take into consideration the outputs of the `perception_step()` in deciding how to issue throttle, brake and steering commands to Rover object. 
* drive_rover.py Iterates on perception and decision functions until rover does a reasonable job of navigating and mapping.  
### More detailed steps can be found under Readme.md

[//]: # (Image References)

[image1]: ./output/terrain2direction.jpg
[image2]: ./output/terrain2drivablemap.jpg
[image3]: ./calibration_images/example_rock1.jpg 
[image4]: ./output/foundRocks.jpg



## Submissions:


### Notebook Analysis

![Terrain to Direction of Robot][image1]


1. The `process_image()` function was modified with the appropriate analysis steps to map pixels identifying navigable terrain, obstacles and rock samples into a worldmap.  

2. Running  `process_image()` on the test data using the `moviepy` functions produces a movie which can be found in ./output folder


And another! 

![Terrain to Driveable map][image2]

### Autonomous Navigation and Mapping

The telemetry() function in driver_rover.py receives images and calls two key functions:

1. perception_step() which applies vision steps, finds navigation angle and updates Rover state

Perception steps and modified code is detailed below and summary of steps added to the function are as follows:

1. Applying persptive transform 
2. Applying color threshold
3. Converting rto over centric coordinates and updating to world map
4.   obstacles are added to world map channel red which can be seen in image2
5.   Rocks are added to the green channel in the world map
6.   Navigable terrain is added to the blue channel of the world map
7. A mosaic is created with various images. Original image, warped image, transformed image and navigable map
8. Rover pixels are coverted to angles which are stored in Rover objects for subsequent steps used in actually driving the rover
9. Finding rocks
10.   Rocks are brigher than the surrounding hence a RGB filter is applied to isolate the area in the map
11. This location of the map upon doing perspective transform gives the location in the terrain

2. decision_step() which decided to steer, throttle or brake

#### No modifications added to drive_rover.py

Updated code in perception_step() function

    # Apply the above functions in succession and update the Rover state accordingly
def perception_step(Rover):
    # Perform perception steps to update Rover()
    # NOTE: camera image is coming to you in Rover.img
    # 1) Define source and destination points for perspective transform
    # 2) Apply perspective transform
    # Define calibration box in source (actual) and destination (desired) coordinates
    # These source and destination points are defined to warp the image
    # to a grid where each 10x10 pixel square represents 1 square meter
    # The destination box will be 2*dst_size on each side
    dst_size = 5
    # Set a bottom offset to account for the fact that the bottom of the image
    # is not the position of the rover but a bit in front of it
    # this is just a rough guess, feel free to change it!
    bottom_offset = 6
    image = Rover.img
    source = np.float32([[14, 140], [301 ,140],[200, 96], [118, 96]])
    destination = np.float32([[image.shape[1]/2 - dst_size, image.shape[0] - bottom_offset],
                  [image.shape[1]/2 + dst_size, image.shape[0] - bottom_offset],
                  [image.shape[1]/2 + dst_size, image.shape[0] - 2*dst_size - bottom_offset],
                  [image.shape[1]/2 - dst_size, image.shape[0] - 2*dst_size - bottom_offset],
                  ])
    warped, mask = perspect_transform(Rover.img, source, destination)

    # 3) Apply color threshold to identify navigable terrain/obstacles/rock samples
    threshed = color_thresh(warped)
    obs_map = np.absolute(np.float32(threshed) -1) * mask
    # 4) Update Rover.vision_image (this will be displayed on left side of screen)
        # Example: Rover.vision_image[:,:,0] = obstacle color-thresholded binary image
        #          Rover.vision_image[:,:,1] = rock_sample color-thresholded binary image
        #          Rover.vision_image[:,:,2] = navigable terrain color-thresholded binary image

    Rover.vision_image[:,:,2] = threshed * 255
    Rover.vision_image[:,:,0] = obs_map * 255

    # 5) Convert map image pixel values to rover-centric coords
    xpix, ypix = rover_coords(threshed)

    # 6) Convert rover-centric pixel values to world coordinates
    world_size = Rover.worldmap.shape[0]
    scale = 2 * dst_size
    x_world, y_world = pix_to_world(xpix, ypix, Rover.pos[0], Rover.pos[1],
                                    Rover.yaw, world_size, scale)
    obsxpix, obsypix = rover_coords(obs_map)
    obs_x_world, obs_y_world = pix_to_world(obsxpix, obsypix, Rover.pos[0], Rover.pos[1],
                                    Rover.yaw, world_size, scale)


    # 7) Update Rover worldmap (to be displayed on right side of screen)
        # Example: Rover.worldmap[obstacle_y_world, obstacle_x_world, 0] += 1
        #          Rover.worldmap[rock_y_world, rock_x_world, 1] += 1
        #          Rover.worldmap[navigable_y_world, navigable_x_world, 2] += 1

    Rover.worldmap[y_world, x_world, 2] += 10
    Rover.worldmap[obs_y_world, obs_x_world, 0] += 1

    # 8) Convert rover-centric pixel positions to polar coordinates
    dist, angles = to_polar_coords(xpix, ypix)
    # Update Rover pixel distances and angles
        # Rover.nav_dists = rover_centric_pixel_distances
        # Rover.nav_angles = rover_centric_angles
    Rover.nav_angles = angles

    # See if we can find some rocks
    rock_map = find_rocks(warped, levels=(110, 110, 70))

    if rock_map.any():
        rock_x, rock_y = rover_coords(rock_map)

        rock_x_world, rock_y_world = pix_to_world(rock_x, rock_y, Rover.pos[0],
                                                    Rover.pos[1], Rover.yaw, world_size, scale)
        rock_dist, rock_ang = to_polar_coords(rock_x, rock_y)
        rock_idx = np.argmin(rock_dist)
        rock_xcen = rock_x_world[rock_idx]
        rock_ycen = rock_y_world[rock_idx]

        Rover.worldmap[rock_ycen, rock_xcen, 1] = 255
        Rover.vision_image[:, :, 1] = rock_map * 255
    else:
        Rover.vision_image[:, :, 1] = 0

    return Rover



#### The robot navigation is controlled by below parameters

        self.stop_forward = 50 # Threshold to initiate stopping --> if < 50 pixesl are present then stop going forward
        self.go_forward = 500 # Threshold to go forward again --> > 500 is good indication of path forward
        self.max_vel = 2 # Maximum velocity (meters/second) --> velocity value which can be tuned

### Improvements

Launching autonomous mode with vision processing improves Rovers navigation greatly. When world map is displayed, the red areas indicates obstacle region. The Robotics motion can be further tuned by parameters listed above. 


![And Found a Rock][image4]

### Summary

The Robot mapped over 75% of the terrain when image4 was captured.
