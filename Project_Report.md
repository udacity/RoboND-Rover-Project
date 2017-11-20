## Project: Search and Sample Return

The purpose of this project is to write code to drive rover autonomoulsy in a map that is simulated environment with objects of interests and terrain where rover cannot drive. arned about in the lesson, and build upon them.


---


Steps followed:

**Training / Calibration**  

* Download the simulator and take data in "Training Mode"
* Test out the functions in the Jupyter Notebook provided
* Add functions to detect obstacles and samples of interest (golden rocks)
* Fill in the `process_image()` function with the appropriate image processing steps (perspective transform, color threshold etc.) to get from raw images to a map.  The `output_image` you create in this step should demonstrate that your mapping pipeline works.
* Use `moviepy` to process the images in your saved dataset with the `process_image()` function.  Include the video you produce as part of your submission.

**Autonomous Navigation / Mapping**

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

#### 1. Run the functions provided in the notebook on test images (first with the test data provided, next on data you have recorded). Add/modify functions to allow for color selection of obstacles and rock samples.
Here is an example of how to include an image in your writeup.

![alt text][image1]

#### 1. Populate the `process_image()` function with the appropriate analysis steps to map pixels identifying navigable terrain, obstacles and rock samples into a worldmap.  Run `process_image()` on your test data using the `moviepy` functions provided to create video output of your result. 

And another! 

![alt text][image2]

### Autonomous Navigation and Mapping

#### 1. Fill in the `perception_step()` (at the bottom of the `perception.py` script) and `decision_step()` (in `decision.py`) functions in the autonomous mapping scripts and an explanation is provided in the writeup of how and why these functions were modified as they were.


#### 2. Launching in autonomous mode your rover can navigate and map autonomously.  Explain your results and how you might improve them in your writeup.  

**Note: running the simulator with different choices of resolution and graphics quality may produce different results, particularly on different machines!  Make a note of your simulator settings (resolution and graphics quality set on launch) and frames per second (FPS output to terminal by `drive_rover.py`) in your writeup when you submit the project so your reviewer can reproduce your results.**




![alt text][image3]


