#!/bin/bash

# CD to the build directory (replace <BUILD_DIRECTORY> with the name of the build directory)
cd ~/Documents/tensegrity/NTRTsim/build/dev/<BUILD_DIRECTORY>

# Run the application with the arguments in format:
# [start_time] [min_proportional_actuator_length] [actuator_rate] [jump_time] [jump_delay] [actuator_1_proportional] [actuator_2_proportional] [actuator_3_proportional]
# For example, the command below will actuate at a rate of 1.0, starting at 1.0 seconds, with a jump at 3.0 seconds, zero jump delay, 
# and will reduce the length of all actuator cables by 30%
./AppisocDarYAMLdirpret isodrop3Aextalligned.yaml 1.0 1.0 3 4.0 0.0 -0.3 -0.3 -0.3
