# Two Bar Linkage Kinematics and Path Planning

The repository contains c++ implementation of a two bar linkage with 2 DOF. 

## Native

To build:

```sh
    mkdir build
    cd build && cmake ..
    make
```

To run the example:
```sh
    cd build
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


## Acknowledgements
 - https://github.com/daancode/a-star
 - http://hades.mech.northwestern.edu/index.php/Modern_Robotics

