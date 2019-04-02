#!/bin/bash
docker run -it --rm -v $HOME/catkin_ws/:/root/catkin_ws -p 10253-10263:10253-10263 flightgogglesros:latest bash