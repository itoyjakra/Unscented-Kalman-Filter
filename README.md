## Dependencies

* cmake >= v3.5
* make >= v4.1
* gcc/g++ >= v5.4

## Basic Build Instructions

1. Clone this repo.
2. Make a build directory: `mkdir build && cd build`
3. Compile: `cmake .. && make`
4. Run it explore mode: 
        Create a file param.in, each line of which should have values for the followin gparameters:
        STD_A STD_YAWDD MULT_P1 MULT_P2
        (See parameter_package.h for definitions)
   Then run:
        `./UnscentedKF path/to/input.txt path/to/output.txt 1`
   
5. Run it normal mode: `./UnscentedKF path/to/input.txt path/to/output.txt 0`
