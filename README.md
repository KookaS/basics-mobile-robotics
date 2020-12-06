# Basics mobile robotics

EPFL 2020 Project

## Members

* Olivier Charrez
* Paco Mermoud
* Tifanny Portela
* Arthur Chevalley

## Movie

![](./gif/video.gif)
    
## Jupyter

[<img align="left" alt="Jupyter" width="80px" src="https://upload.wikimedia.org/wikipedia/commons/thumb/3/38/Jupyter_logo.svg/800px-Jupyter_logo.svg.png" />][jupyter]
    
## Libraries

Anaconda is useful for setting up python, you have to set as interpreter for the project

packages to install:
    
    conda install ...
    
If you don't have Anaconda download pip3 and then to install libraries:

    pip3 install ...
    
Here is the list of packages to install:

    pyserial python-dotenv opencv-python tqdm matplotlib numpy ipywidgets
    
## .env

Create a .env and make sure not to commit and push it

This is a way to store locally variables that can be used in the project, variables that may be different for everyone

    COM_PORT=\\.\COM10
    LEFT_WHEEL_SCALING=98
    RIGHT_WHEEL_SCALING=100
    HALF_TURN_TIME=4.72
    DISTANCE_TIME=0.32
    CAMERA_PORT=1
    SPEED_80_TO_MM_S=0.3333


[jupyter]: https://github.com/KookaS/basics-mobile-robotics/tree/master/jupyter/repport_robotic_project.ipynb