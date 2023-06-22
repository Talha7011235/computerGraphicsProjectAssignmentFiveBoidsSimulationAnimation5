Talha Hussain, myBoidsSimulationAnimationProjectFive

Computer Graphics Project Assignment 5 Boids Simulation Animation

Websites:
To complete my Computer Graphics Project Assignment 5 Boids Simulation and Animation, I received helpful inspiration from the following website:
http://www.red3d.com/cwr/steer/gdc99/ - Used for inspiration on possible Boid behaviors.

Usage:
Choose either the "make" command to run the fishtank and viewer simultaneously or first do "make fishtank" and then do "make viewer" to run the fishtank and viewer
separately.
make - Runs the compilation using the g++ compiler.
make fishtank - Runs the compilation of just the fishtank program using the g++ compiler.
make viewer - Runs the compilation of just the viewer program using the g++ compiler.
make clean - Cleans the intermediate files and compiled target.

After running either "make" or "make fishtank" and then "make viewer", run "make test" and then "make clean".
make test - Runs the sample.in input file through both the basic version and the K-Dimensional KD Tree version.

Summary:
This project was about simulating flock/herd behavior of agents moving in the simulation space. Similarly to a flock of birds or school of fish, there were some
basic concepts that allowed for systemic movement which emulated the natural behaviors. After parsing the Input File for my Computer Graphics Project Assignment 5
Boids Simulation and Animation, I started modeling the basic cohesion, separation, and velocity matching behaviors and this involved doing checks on the boid
flock to find the nearest neighbors using the Eigen norm function to determine the distance. As a result, this provided the basis for applying multiple forces one
after the other in order to add functionality beyond this base set. The next thing I added was the bounds of the simulation space, by detecting when the boid was or
boids were outside the bound plane and providing a constant pervasive steering force to return them to the simulation space. This proved to be a good way to have
them smoothly turn around when outside the space instead of all of a sudden changing course. Then, I added food using the rand() Function to add basic perturbations
to the horizontal velocity of the food particle leaving the vertical velocity untouched so that it continued to rain down. The pursuit behavior then became a simple
element of finding the closest piece of food in range and adding a steering force towards it and then removing the food once the boid gets too close. In terms of the 
K-Dimensional KD Tree, the K-Dimensional KD Tree Integration was fairly seamless because by calculating a vector of position points and generating a tree from that
vector of position points, it was not an extremely difficult issue to find the nearest neighbors and pass that data back to the simulation instead of using the
brute force function. Finally, the additional behaviors that I decided to add are two forms of persuit and a free behavior. First was the flock leaders which are
represented as the blue boids that is a force whee the non leader boids would essentially follow the blue leader boids similarly to a flock of geese. The other two
came in the form of a predator prey relationship where the boids identified as predators that are represented as the red boids in my Computer Graphics Project
Assignment 5 Boids Simulation Animation would use a similar force to the food but would select the closest prey and stalk the closest prey. Consequently, the prey would then flee from the predators using a
similar system to the collision to find a vector that had them be as far away as possible. Finally, adding modifications to the viewer.cpp File allowed the first 10
boids to be marked as predators, the next 5 boids to be marked as leaders and the rest of the boids to be marked as prey and adding these three different types of
boids by assigning these three different types of boids with three different colors that includes red for the boids represented as predators, blue for the boids
represented as leaders, and the default color green for the boids represented as prey made it easier to identify and follow the generated output animation and
simulation of the boids that represent the predators, the boids that represent the leaders, and the boids that represent the prey.

