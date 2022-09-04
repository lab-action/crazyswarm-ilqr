# iLQR Code

While the original author didn't leave hardly anything to help us run this thing,
I've gone back and tried to figure out how things are supposed to be done. Using
a `Makefile`, I figured out a way to get the simulation compiled and running
outside of ROS's `cmake` build system. 

## Depenencies
This code uses two libraries:
  1. [`eigen3`](https://eigen.tuxfamily.org/index.php?title=Main_Page)
  1. [`ADOL-C`](https://github.com/coin-or/ADOL-C)

Follow the respective installation instructions for each library. After finished:
  1. Move the `eigen3` header (`*.h`) files into `/home/$USER/.local/include/eigen3`.
  2. Move the `adolc` headers into `/home/$USER/.local/include/adolc`.
  3. Move the `adolc` libraries (*.so) into  `/home/$USER/.local/lib/adolclib`.

## Compiling
With the necessary libraries, you should be able to say:
  - `make -j`: compile main.cpp with all available CPU cores.
  - `make clean`: remove object files if you want a fresh build.

## Running
To run the executable, you need to run:
  - `export LD_LIBRARY_PATH=/home/$USER/.local/lib/adolclib`
  - `./sim`: run the compiled and linked simulation executable.

## Contributors
  - Talha Kuvuncu: primary author.
  - Ayberk Yaraneri: hardware integration.
  - Zach Williams: this document and dusting off the cobwebs.