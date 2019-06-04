# FCND - 3D Motion Planning
This project is a continuation of the Backyard Flyer project where we executed a simple square shaped flight path. In this project we will integrate the techniques that we have learned to plan a path through an urban environment. 

### Step 1: Download the Simulator
This is a new simulator environment!  

Download the Motion-Planning simulator for this project that's appropriate for your operating system from the [simulator releases respository](https://github.com/udacity/FCND-Simulator-Releases/releases).

### Step 2: Set up your Python Environment
If you haven't already, set up your Python environment and get all the relevant packages installed using Anaconda following instructions in [this repository](https://github.com/udacity/FCND-Term1-Starter-Kit)

### Step 3: Clone this Repository
```sh
git clone https://github.com/udacity/FCND-Motion-Planning
```
### Step 4: Test setup
The first task in this project is to test the [solution code](https://github.com/udacity/FCND-Motion-Planning/blob/master/backyard_flyer_solution.py) for the Backyard Flyer project in this new simulator. Verify that your Backyard Flyer solution code works as expected and your drone can perform the square flight path in the new simulator. To do this, start the simulator and run the [`backyard_flyer_solution.py`](https://github.com/udacity/FCND-Motion-Planning/blob/master/backyard_flyer_solution.py) script.

```sh
source activate fcnd # if you haven't already sourced your Python environment, do so now.
python backyard_flyer_solution.py
```
The quad should take off, fly a square pattern and land, just as in the previous project. If everything functions as expected then you are ready to start work on this project. 


### Step 5: Writing our planner

Our planning algorithm is going to look something like the following:

- Load the 2.5D map in the `colliders.csv` file describing the environment.
- Discretize the environment into a grid or graph representation.
- Define the start and goal locations. We can determine our home location from `self._latitude` and `self._longitude`. 
- Perform a search using A* or other search algorithm. 
- Use a collinearity test or ray tracing method (like Bresenham) to remove unnecessary waypoints.
- Return waypoints in local ECEF coordinates (format for `self.all_waypoints` is [N, E, altitude, heading], where the drone’s start location corresponds to [0, 0, 0, 0]). 

### Implementation
For details about implementation, please check [Implementation Details](https://github.com/UsmanIjaz/Flying_Car_Motion_Planning_Project/blob/master/3DMotionPlanningWriteup.pdf)

### Results
Grid Based Motion Planning Output Video
[![IMAGE ALT TEXT HERE](https://img.youtube.com/vi/XsBuXVgsRjk/0.jpg)](https://www.youtube.com/watch?v=XsBuXVgsRjk)

Graph based Motion Planning Output Video
[![IMAGE ALT TEXT HERE](https://img.youtube.com/vi/yVZNJJu-71M/0.jpg)](https://www.youtube.com/watch?v=yVZNJJu-71M)
