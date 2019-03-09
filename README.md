# RoboND_Rover_Project
Search and Sample Return Project

![alt text][image_0] 

The project is modeled after the [NASA sample return challenge](https://www.nasa.gov/directorates/spacetech/centennial_challenges/sample_return_robot/index.html). It discusses about the three essential elements of robotics, which are perception, decision making and actuation.  The project is carried out in a simulator environment built with the Unity game engine.  

## The Simulator
The first step is to download the simulator build that's appropriate for operating system.  Here are the links for [Linux](https://s3-us-west-1.amazonaws.com/udacity-robotics/Rover+Unity+Sims/Linux_Roversim.zip), [Mac](	https://s3-us-west-1.amazonaws.com/udacity-robotics/Rover+Unity+Sims/Mac_Roversim.zip), or [Windows](https://s3-us-west-1.amazonaws.com/udacity-robotics/Rover+Unity+Sims/Windows_Roversim.zip).  

## Dependencies
Python 3 and Jupyter Notebooks are required for the project. [RoboND-Python-Starterkit](https://github.com/ryan-keenan/RoboND-Python-Starterkit). 


## Navigating Autonomously
The file called `drive_rover.py` is used to navigate the environment in autonomous mode.  This script calls functions from within `perception.py` and `decision.py`.  The functions defined in the IPython notebook are all included in`perception.py` to update the rover map. `decision.py` includes another function called `decision_step()` to navigate autonomously. `drive_rover.py` should work if all the required Python packages are installed. Run the following:

```sh
python drive_rover.py
```  
Then launch the simulator and choose "Autonomous Mode".  The rover should drive itself now!!

## Results

The rover attains a fidelity of 71.1% after mapping 69.9% area.
![Fidelity_Results](RoboND_Rover_Project/Results/Fidelity_71.1Percent_MappedArea_69.9Percent.png)