# Multi Sensor Localizaion with Kalman Filter


## Run the following code to install dependencies

```
rosdep install --from-paths src --ignore-src -r -y
```

## For Simulation (Unpause the simulation before running other codes)

```
roslaunch final project.launch 
```

## For Localization

```
roslaunch jackal_localization localization.launch 
```

## For Movement Commands

```
rosrun final sim_time_cmd_publisher.py  
```

## For Position Plotting

```
rosrun final pos_plot.py 
```

## For Pedestrian Simulation

```
roslaunch final pedsim.launch
```