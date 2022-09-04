HOMEDIR = /home/${USER}
INCFLAGS = -I${HOMEDIR}/.local/include/eigen3 -I${HOMEDIR}/.local/include/adolc -I.
LDLIBS = -L/home/zjw/.local/lib/adolclib -ladolc 
CXXFLAGS = -Wall $(INCFLAGS)

objects = main.o dynamics_structure.o cost_structure.o utils.o

sim: $(objects)
	$(LINK.cpp) $^ $(LOADLIBES) $(LDLIBS) -o $@

clean:
	rm -f *.o sim
