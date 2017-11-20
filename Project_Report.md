## Project: Search and Sample Return

Source code https://github.com/boldbinarywisdom/RoboND-Rover-Project


The purpose of this project is to write code to drive rover autonomoulsy in a map that is simulated environment with objects of interests and terrain where rover cannot drive. arned about in the lesson, and build upon them.


---


Steps followed:

** Training / Calibration **  

* Download the simulator and take data in "Training Mode"
* Test out the functions in the Jupyter Notebook provided
* Add functions to detect obstacles and samples of interest (golden rocks)
* Fill in the `process_image()` function with the appropriate image processing steps (perspective transform, color threshold etc.) to get from raw images to a map.  The `output_image` you create in this step should demonstrate that your mapping pipeline works.
* Use `moviepy` to process the images in your saved dataset with the `process_image()` function.  Include the video you produce as part of your submission.

** Autonomous Navigation / Mapping**

* `perception_step()` function within the `perception.py` has image processing functions to create a map and update `Rover()` object
* `decision_step()` function within the `decision.py` has conditional statements that take into consideration the outputs of the `perception_step()` in deciding how to issue throttle, brake and steering commands to Rover object. 
* drive_rover.py Iterates on perception and decision functions until rover does a reasonable job of navigating and mapping.  

[//]: # (Image References)

[image1]: ./output/terrain2direction.jpg
[image2]: ./output/terrain2drivablemap.jpg
[image3]: ./calibration_images/example_rock1.jpg 

---

Submissions:


### Notebook Analysis

![Terrain to Direction of Robot][image1]


1. The `process_image()` function was modified with the appropriate analysis steps to map pixels identifying navigable terrain, obstacles and rock samples into a worldmap.  

2. Running  `process_image()` on the test data using the `moviepy` functions produces a movie which can be found in ./output folder


And another! 

![Terrain to Driveable map][image2]

### Autonomous Navigation and Mapping

The telemetry() function in driver_rover.py receives images and calls two key functions:

1. perception_step() which applies vision steps, finds navigation angle and updates Rover state
2. decision_step() which decided to steer, throttle or brake

No modifications added to drive_rover.py

Steps in perception_step() function

    # 1) Define source and destination points for perspective transform
    # 2) Apply perspective transform
    # Define calibration box in source (actual) and destination (desired) coordinates
    # These source and destination points are defined to warp the image
    # to a grid where each 10x10 pixel square represents 1 square meter
    # The destination box will be 2*dst_size on each side
   
    # 3) Apply color threshold to identify navigable terrain/obstacles/rock samples
    
    # 4) Update Rover.vision_image (this will be displayed on left side of screen)
        # Example: Rover.vision_image[:,:,0] = obstacle color-thresholded binary image
        #          Rover.vision_image[:,:,1] = rock_sample color-thresholded binary image
        #          Rover.vision_image[:,:,2] = navigable terrain color-thresholded binary image

    # 5) Convert map image pixel values to rover-centric coords
    # 6) Convert rover-centric pixel values to world coordinates
    world_size = Rover.worldmap.shape[0]
    
    # 7) Update Rover worldmap (to be displayed on right side of screen)
        # Example: Rover.worldmap[obstacle_y_world, obstacle_x_world, 0] += 1
        #          Rover.worldmap[rock_y_world, rock_x_world, 1] += 1
        #          Rover.worldmap[navigable_y_world, navigable_x_world, 2] += 

    # 8) Convert rover-centric pixel positions to polar coordinates
    # Update Rover pixel distances and angles
        # Rover.nav_dists = rover_centric_pixel_distances
        # Rover.nav_angles = rover_centric_angles

    # Finally see if we can find some rocks
 
    return Rover object

The perception_step function was modified to perform vision processing on the images

The robot navigation is controlled by below parameters

        self.stop_forward = 50 # Threshold to initiate stopping --> if < 50 pixesl are present then stop going forward
        self.go_forward = 500 # Threshold to go forward again --> > 500 is good indication of path forward
        self.max_vel = 2 # Maximum velocity (meters/second) --> velocity value which can be tuned

### Improvements

Launching autonomous mode with vision processing improves Rovers navigation greatly. When world map is displayed, the red areas indicates obstacle region. The Robotics motion can be further tuned by parameters listed above. 


![alt text][image3]


