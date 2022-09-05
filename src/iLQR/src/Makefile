HOMEDIR = /home/${USER}
INCFLAGS = -I${HOMEDIR}/.local/include/eigen3 -I${HOMEDIR}/.local/include/adolc -I.
LDLIBS = -L/home/zjw/.local/lib/adolclib -ladolc 
CXXFLAGS = -Wall -g3 -std=c++2a $(INCFLAGS)

objects = main.o dynamics_structure.o cost_structure.o utils.o
deps = cost.h dynamics.h dynamics_structure.h cost_structure.h iLQR.h utils.h

sim: $(objects) $(deps)
	$(LINK.cpp) $^ $(LOADLIBES) $(LDLIBS) -o $@

clean:
	rm -f *.o sim
