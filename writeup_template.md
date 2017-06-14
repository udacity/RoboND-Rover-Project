## Project: Search and Sample Return
### Writeup Template: You can use this file as a template for your writeup if you want to submit it as a markdown file, but feel free to use some other method and submit a pdf if you prefer.

---


**The goals / steps of this project are the following:**  

**Training / Calibration**  

* Download the simulator and take data in "Training Mode"
* Test out the functions in the Jupyter Notebook provided
* Add functions to detect obstacles and samples of interest (golden rocks)
* Fill in the `process_image()` function with the appropriate image processing steps (perspective transform, color threshold etc.) to get from raw images to a map.  The `output_image` you create in this step should demonstrate that your mapping pipeline works.
* Use `moviepy` to process the images in your saved dataset with the `process_image()` function.  Include the video you produce as part of your submission.

**Autonomous Navigation / Mapping**

* Fill in the `perception_step()` function within the `perception.py` script with the appropriate image processing functions to create a map and update `Rover()` data (similar to what you did with `process_image()` in the notebook). 
* Fill in the `decision_step()` function within the `decision.py` script with conditional statements that take into consideration the outputs of the `perception_step()` in deciding how to issue throttle, brake and steering commands. 
* Iterate on your perception and decision function until your rover does a reasonable (need to define metric) job of navigating and mapping.  

[//]: # (Image References)

[image1]: ./misc/rover_image.jpg
[image2]: ./calibration_images/example_grid1.jpg
[image3]: ./calibration_images/example_rock1.jpg 

## [Rubric](https://review.udacity.com/#!/rubrics/916/view) Points
### Here I will consider the rubric points individually and describe how I addressed each point in my implementation.  

---
### Writeup / README

#### 1. Provide a Writeup / README that includes all the rubric points and how you addressed each one.  You can submit your writeup as markdown or pdf.  

You're reading it!

### Notebook Analysis
#### 1. Run the functions provided in the notebook on test images (first with the test data provided, next on data you have recorded). Add/modify functions to allow for color selection of obstacles and rock samples.

I added three functions `detect_navi` , `detect_rock`, `detect_obstacle` to identify the navigable roads,rock and obstacle respectively.

Inside the three functions , I use color_thresh function to filter the colors through turning the parameter `rgb_thresh`.

I printed out the red, green and blue color channels for obseravation and found the road color can be affected by all three color channels ,so I focused on tuning all three channels to filter the road. The rock is affcted through red and green channels , so for rock, I mainly manipulated these two channels and set blue channel to 0.  After that , I got the obstacle by subtracting the colors of the navigable and rock .(I first got a color channel and set all value to 1, and then set the navigable road and rock area to be 0).


#### 1. Populate the `process_image()` function with the appropriate analysis steps to map pixels identifying navigable terrain, obstacles and rock samples into a worldmap.  Run `process_image()` on your test data using the `moviepy` functions provided to create video output of your result. 

Following are my steps to transform  navigable roads, obstacles and rock into a world map.

1. Define  `source` and `distantce` parameter for function `perspect_transform` . First print out the grid picture , and then measure four corners as the `source` 

2. Get top view perspective using defined function `perspect_transform`. Pass above parameters into it.

3. Get top view perspective of rock , navigable road and obstacles using previously defined function `detect_rock`,`detect_navi`,`detect_obstacle`.These three functions use color_thresh to filter the objects.
4. Transform all the rock ,navigable road and obstacles to Rover centric coordinate using defined function `rover_coords`.
5. Then conver all of them to world  coordinates using `data.count` to track all the rover locations. Using `data.count -1`as the initial value.This is a trick to solve the problem in the later on video generation as the later fuction would count `total+1` images. 
6. Then updated the location in the world map by setting  a value (5) to their location in the color channel respectively.



### Autonomous Navigation and Mapping

#### 1. Fill in the `perception_step()` (at the bottom of the `perception.py` script) and `decision_step()` (in `decision.py`) functions in the autonomous mapping scripts and an explanation is provided in the writeup of how and why these functions were modified as they were.

1. In the `perception.py` file, I added helper function `detect_navi`,`detect_rock`,`detect_obstacle`. The implemented deatails are the same as in the Jupyter notebook. I also added 'get_5_meter_pixels'. This function is use to get pixels shorter than 5 meters range. And thess selected pixels will be used to calculate the steer angles. It's too distracted using all the pixels and the Rover will go either very left or very right direction,making the Rover look very drunk. And this function also help to elimiate cyling around while using all the  pixesl .
2. In the `decision.py` file, I added a variable `is_stuck` to check whether the Rover is stuck either on the edge of the road or near the small obstacles. If it is stuck, I added some random method to make it either rotate or backward to get out of the stuck.






#### 2. Launching in autonomous mode your rover can navigate and map autonomously.  Explain your results and how you might improve them in your writeup.  

**Note: running the simulator with different choices of resolution and graphics quality may produce different results, particularly on different machines!  Make a note of your simulator settings (resolution and graphics quality set on launch) and frames per second (FPS output to terminal by `drive_rover.py`) in your writeup when you submit the project so your reviewer can reproduce your results.**

#### simulator settings: 

- resultion `1024 X 768` 
- graphics quality `Fantastic`
- FPS `30`

I make the Rover try to follow the left side of the road by setting a random `+`value to the angle.steer.When it encounters the obstacles, it will try to avoid it: if the obstacle is big, the camera can see only little road, it will just turn right until it finds room; if the obstacle is small (like small rocks in the road), it will get stuck. And it will try to get out of the stuck by some random behaviors. It will try to turn left first, if there is enough space, it will go forward. If the space is not enough, it will turn right.I also added a 20% chance to get backward. Using above methods, it can map the map  90% ~ 96%.

When the Rover sees a rock, it will approach it and  pick it up:If the rock is at the left sied, it can function well. If the rock is at the right side of the road, things is a little tricker.The Rover will go to the right side and pick it up.After picking up, the Robo first turn left and see if there is enought to continue forward. If there is not enought room, it will turn right to go back.

Further Improvement:

The Rover is still not smart enought to avoid the Obstacles. I may consider using some more smarter algorithm to better recoginze the object. 

And the mapping is not efficient enough, it will sometimes go back after picking the rocks. I may consider to a better mapping plan.







