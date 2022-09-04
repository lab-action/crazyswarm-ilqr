# Makefile to compile the iLQR code outside of ROS.
# This assumes that you've installed:
#   1. eigen3
#     - headers (*.h) are contained in: `/home/$USER/.local/include/eigen3`
#   2. adolc
#     - headers contained in: `/home/$USER/.local/include/adolc`
#     - libraries (*.so) contained in: `/home/$USER/.local/lib/adolclib`
#
# With the necessary libraries, you should be able to say:
#   - `make -j`: this will compile main.cpp with all available CPU cores.
#   - `make clean`: this will remove object files if you want a fresh build.

HOMEDIR = /home/${USER}
INCFLAGS = -I${HOMEDIR}/.local/include/eigen3 -I${HOMEDIR}/.local/include/adolc -I.
LDLIBS = -L/home/zjw/.local/lib/adolclib -ladolc 
CXXFLAGS = -Wall $(INCFLAGS)

objects = main.o dynamics_structure.o cost_structure.o utils.o

all: $(objects)
	$(LINK.cpp) $^ $(LOADLIBES) $(LDLIBS) -o $@

clean:
	rm -f *.o all
