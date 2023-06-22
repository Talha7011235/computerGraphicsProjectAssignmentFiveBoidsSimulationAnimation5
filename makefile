CC          = c++ 

#-----------------------------------------
#Optimization ----------------------------
OPT   = -O3 -Wno-deprecated

#GL_LIB = -framework OpenGL -framework GLUT -framework foundation
GL_LIB = -lGL -lglut -lGLU

#-----------------------------------------

TARGETS = fishtank viewer

OBJECTS = kdTree.o boid.o food.o simulation.o

#-----------------------------------------

LIBS = 
INCS = -I/usr/local/include/eigen3 -I/usr/include/eigen3

CCOPTS = $(OPT) $(INCS) 
LDOPTS = $(OPT) $(INCS) $(LIBS) 

#-----------------------------------------
#-----------------------------------------

default: $(TARGETS)

test: fishtank
	./fishtank sample.in sample.out
	./fishtank sample.in sample-tree.out -tree

clean:
	/bin/rm -f *.o $(TARGETS)

#-----------------------------------------
#-----------------------------------------

viewer: $(OBJECTS) viewer.o
	$(CC) $(OBJECTS) viewer.o $(LDOPTS) $(GL_LIB) -o viewer

fishtank: $(OBJECTS) fishtank.o
	$(CC) $(OBJECTS) fishtank.o $(LDOPTS) -o fishtank

#-----------------------------------------
#-----------------------------------------
fishtank.o: fishtank.cpp
	$(CC)  $(CCOPTS) -c fishtank.cpp

kdTree.o: kdTree.cpp
	$(CC)  $(CCOPTS) -c kdTree.cpp

boid.o: boid.cpp
	$(CC)  $(CCOPTS) -c boid.cpp

food.o: food.cpp
	$(CC)  $(CCOPTS) -c food.cpp

simulation.o: simulation.cpp
	$(CC)  $(CCOPTS) -c simulation.cpp

viewer.o: viewer.cpp
	$(CC)  $(CCOPTS) -c viewer.cpp

.cpp.o: 
	$(CC) $(CCOPTS) -c $< 

#-----------------------------------------
#-----------------------------------------















