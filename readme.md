# Two Bar Linkage Kinematics and Path Planning

The repository contains c++ implementation of a two bar linkage with 2 DOF. 


## Overview
 A two bar linkage is an interesting starting point for understanding forward and inverse kinematics. The chain has two degress of freedom which is aldo evident from the fact that you can independentl control both the joint angles for this planar assembly.
 
I have also added the  path planning feature to this chain. The path planner (taken from `daancode`) implmenets A* search algorithm which I use to find the path the chain should take to move from a given starting poition to given target position.

There is also an option to add obstacles and the path planner should plan a path aroound those obstacles.
Please see `src/main.cpp` for usage examples.


The Path Planning can be done in two ways:
 - Configuration space i.e. [0, 2 Pi] for each of the angles
 - Euclidian Space i.e [-(L1 + l2), L1 + l2] for both X and Y coordinate of the end effector

The TwoBarLinkage class implements both these approaches. One could visualize the outputs from both these approaches to understand the difference.

Next Scope:
 - Add visualizations
 - Allow other obstacle shapes 
 - Implement a custom planner optimized for C Space path planning
 - Add a Command Line Interface

## Native

To build:

```sh
    mkdir build
    cd build && cmake ..
    make
    cd ..
```

To run the example:
```sh
    cd bin
    ./TwoBarLinkage
```

## Docker

To build:

```sh
    docker build -t two-bar-linkage . 
```

To run the example:
```sh
    docker run -t two-bar-linkage > output.txt 
```


> Please look at `src/main.cpp` to understand how to use the TwoBarLinkage functions

## Acknowledgements
 - https://github.com/daancode/a-star
 - http://hades.mech.northwestern.edu/index.php/Modern_Robotics

