## Writing Assignment
See [here](solution.ipynb)

# Particle Filter Localization


This code implements the MCL algorithm for the RACECAR. 

[![YouTube Demo](./media/thumb.jpg)](https://www.youtube.com/watch?v=-c_0hSjgLYw)

For high efficiency in Python, it uses Numpy arrays and RangeLibc for fast 2D ray casting.

# Configuration
## 1. Install racecar_simulator
If you have not install racecar_simulator package yet, install it under racecar_ws/src
```
git clone https://github.com/mit-racecar/racecar_simulator.git
```
Then run catkin_make and catkin_make install
```
cd ~/racecar_ws
catkin_make
catkin_make install
source devel/setup.bash
```
## 2. Build the code for the lab
```
sudo apt-get install cython
cd ~/racecar_ws/src/localization-solution/src
python setup.py build_ext --inplace
```

# Usage

The majority of parameters you might want to tweak are in the param.yaml file. You may have to modify the "odom_topic", "scan_topic", "num_beams_per_particle" and "angle_step" parameters in the launch file to match your environment.

Most of the time you should be able to use it out of the box. However, you might need to change the parameters in `sensor_model.py` (such as `alpha_hit`) to reflect the intrinsics of your LiDAR.

To run particle filter solution, in sumulation environment:
```
roslaunch racecar_simulator simulate.launch
roslaunch localization-solution localize_simulation.launch
```

In RACECAR environment:
```
roslaunch localization-solution localize_real_env.launch
```


Once the particle filter is running, you can visualize the map and other particle filter visualization message in RViz. Use the "2D Pose Estimate" tool from the RViz toolbar to initialize the particle locations.
