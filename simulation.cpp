// Computer Graphics Project Assignment 5 Boids Simulation Animation
// simulation.cpp
#include "simulation.h"
#include <cmath>
#include <ctime>
#include <fstream>
#include <iostream>


// The Public Member Function "Simulation::Simulation(std::string fileName, bool useKDimensionalTree)" constructs a new "Simulation" object. The First Parameter
// that is in the Public Member Function "Simulation::Simulation(std::string fileName, bool useKDimensionalTree)" that is the String Variable "fileName" is defined
// as the file to read as input. The Second Parameter in the Public Member Function "Simulation::Simulation(std::string fileName, bool useKDimensionalTree)"
// that is the Boolean Variable useKDimensionalTree defines whether to use the K-Dimensional KD Tree for the nearest neighbor calculation.
Simulation::Simulation(std::string fileName, bool useKDimensionalTree) {
    // Initialize the random seed for food.
    srand(time(nullptr));

    useTree = useKDimensionalTree;

    // Open the file for reading.
    std::ifstream file(fileName);
    
    // Check if the file is opened and in a state that it can be read from.
    if(!file.good()) {
        std::cerr << "Unable to open " << fileName << "." << std::endl;
        file.close();
        return;
    }

    int numberOfBoids;
    int amountOfFood;

    // Read the Properties of the Simulation.
    file >> size >> neighborRadius >> numberOfNeighbors >> mass >> collisionForce
         >> centeringForce >> velocityForce >> hunger >> damping >> deltaTime >> length;

    // Convert to a Frame Count.
    length /= deltaTime;

    // Read the Number Of Boids.
    file >> numberOfBoids;

    // Reserve the space for the Boids and then read each single Boid in, storing in the vector.
    boids.reserve(numberOfBoids);
    for(int iIndex = 0; iIndex < numberOfBoids; ++iIndex) {
        Boid bIsForBoid;
        file >> bIsForBoid;
        boids.push_back(bIsForBoid);
    }
    
    // Read the amount of food.
    file >> amountOfFood;
    
    // Reserve the space for the food and then read each one in, storing in the vector.
    food.reserve(amountOfFood);
    for(int iIndex = 0; iIndex < amountOfFood; ++iIndex) {
        Food f;
        file >> f;
        food.push_back(f);
    }

    // Close the file so that the handle is not left dangling.
    file.close();
}

// The Destructor "Simulation::~Simulation()" clears the data from the system.
Simulation::~Simulation() {
    // Clear the Boids and the Food Vectors when the object is destroyed.
    boids.clear();
    food.clear();
}

// The Publice Member Function "const std::vector<Boid>& Simulation::getBoids() const Simulation::getBoids() const" returns a "const" reference to the Boids Vector.
const std::vector<Boid>& Simulation::getBoids() const {
    return boids;
}


// The Public Member Function "const std::vector<Food>& Simulation::getFood() const" returns a "const" reference to the Food Vector.
const std::vector<Food>& Simulation::getFood() const {
    return food;
}

// The Public Member Function "void Simulation::run(std::string fileName)" runs the simulation for the length specified in the input file and then writing output
// to the given file. The Parameter that is in the Public Member Function "void Simulation::run(std::string fileName)" that is the String Variable "fileName" is the
// name of the file to write the output to.
void Simulation::run(std::string fileName) {
    // Open the file for writing.
    std::ofstream output(fileName);

    // Check that the file opened correctly and can be written to.
    if(!output.good()) {
        std::cerr << "Unable to open " << fileName << " for writing" << std::endl;
        output.close();
        return;
    }

    // Write the length of the animation. Then, convert the Frame Count to seconds.
    output << length << std::endl;

    // Initialize the current Frame's Food Vector.
    std::vector<Food> currentFood(food.size());

    double timeStamp = 0.0;
    // Record each Frame as they occur.
    for(int iIndex = 0; iIndex < length; ++iIndex) {
        // Write the Number of Boids and each boid to the output file.
        output << boids.size() << std::endl;
        for(std::vector<Boid>::const_iterator iter = boids.begin(); iter != boids.end(); ++iter) {
            output << *iter;
        }

        // Clear the current Food Vector and fill with the food that is in the frame of simulation.
        currentFood.clear();
        for(std::vector<Food>::const_iterator iter = food.begin(); iter != food.end(); ++iter) {
            if(iter->getAdditionTime() <= iIndex && !iter->getIsEaten()) {
                currentFood.push_back(*iter);
            }
        }
        
        // Write the number of food on the frame and each food item.
        output << currentFood.size() << std::endl;
        for(std::vector<Food>::const_iterator iter = currentFood.begin(); iter != currentFood.end(); ++iter) {
            output << *iter;
        }

        // Perform the next step of the simulation.
        step(iIndex);
        timeStamp += deltaTime;
        std::cout << timeStamp << std::endl;
    }

    // Close the file.
    output.close();
}

// The Private Member Function "Simulation::Neighbor** Simulation::getNeighbors(const Boid& boid, int& num) const" returns the nearest neighbors to the
// Boid. The First Parameter in the Private Member Function "Simulation::Neighbor** Simulation::getNeighbors(const Boid& boid, int& num) const" that is the
// Reference Variable "const Boid& boid" is used as the basis for detection. In the Private Member Function "Simulation::Neighbor** Simulation::getNeighbors(const
// Boid& boid, int& num) const", the Second Parameter that is the Reference Variable "int& num" defines the number of neighbors found by the
// Function "Simulation::Neighbor** Simulation::getNeighbors(const Boid& boid, int& num) const".
Simulation::Neighbor** Simulation::getNeighbors(const Boid& boid, int& num) const {
    // Allocate enough space for the simulation's Neighbor capacity.
    Neighbor** output = (Neighbor**)malloc(sizeof(Neighbor*) * numberOfNeighbors);

    // Ensure all of the pointers are set to null so that the data does not potentially point to a bad sector.
    for(int iIndex = 0; iIndex < numberOfNeighbors; ++iIndex) {
        output[iIndex] = nullptr;
    }

    // Initialize "num" to 0. Then, create a for() Loop in order to loop through all of the boids in the simulation.
    num = 0;
    for(size_t iIndex = 0; iIndex < boids.size(); ++iIndex) {
        // Do not include the boid we are looking at.
        if(boids[iIndex] != boid) {
            // Calculate the distance between boids and skip if the distance is too large.
            double distance = (boid.getPosition() - boids[iIndex].getPosition()).norm();
            if(distance > neighborRadius) continue;

            // If the Neighbor Array is not filled, then all neighbors close enough get added.
            if(num < numberOfNeighbors) {
                output[num++] = new Neighbor(
                    &boids[iIndex],
                    distance
                );
                continue;
            }

            // If there is no more room, then find the Neighbor that is futhest away from the Boid.
            int index = 0;
            double closest = -std::numeric_limits<double>::max();
            for(int jIndex = 0; jIndex < numberOfNeighbors; ++jIndex) {
                if(output[jIndex]->distance > closest) {
                    index = jIndex;
                    closest = output[jIndex]->distance;
                }
            }

            // Delete the Neighbor and fill the slot with a new neighbor.
            delete output[index];

            output[index] = new Neighbor(
                &boids[index],
                distance
            );
        }
    }
    
    // Return the list of neighbors.
    return output;
}

// The Private Member Function "Simulation::Neighbor** Simulation::getNeighborsViaTree(KDTree* tree, const std::vector<Eigen::Vector3d>& pts,
// const Boid& boid, int& num) const" returns the nearest Neighbors to the Boid using the K-Dimensional KD Tree. The First Parameter in the
// Private Member Function "Simulation::Neighbor** Simulation::getNeighborsViaTree(KDTree* tree, const std::vector<Eigen::Vector3d>& pts, const
// Boid& boid, int& num) that is "tree" is used for the searches. In the Private Member Function "Simulation::Neighbor** Simulation::getNeighborsViaTree(KDTree* tree,
// const std::vecror<Eigen::Vector3d>& pts, const Boid& boid, int& num) const", the Second Parameter that is "pts" defines the positions of all of the Boids in the
// simulation. The Third Parameter in the Private Member Function "Simulation::Neighbor** Simulation::getNeighborsViaTree(KDTree* tree, const
// std::vector<Eigen::Vector3d>& pts, const Boid& boid, int& num) const" that is "boid" is used as the basis for detection. The Fourth Parameter in the Private
// Member Function "Simulation::Neighbor** Simulation::getNeighborsViaTree(KDTree* tree, const std::vector<Eigen::Vector3d>& pts,m const Boid& boid, int& num) const"
// that is "num" defines the number of neighbors found by the function.
Simulation::Neighbor** Simulation::getNeighborsViaTree(KDTree* tree, const std::vector<Eigen::Vector3d>& pts, const Boid& boid, int& num) const {
    // Initialize the Neighbor List to be filled by the Tree.
    std::vector<int> neighborList(numberOfNeighbors);

    // Perform the nearest Neighbor Search.
    tree->neighbors(pts, boid.getPosition(), numberOfNeighbors, neighborRadius, neighborList);

    // Allocate the memory for the Number of Boids found by the Tree.
    num = neighborList.size() - 1;
    Neighbor** output = (Neighbor**)malloc(sizeof(Neighbor*) * num);

    // Fill the Neighbor Array with the information from the Tree.
    int current = 0;
    for(std::vector<int>::const_iterator iter = neighborList.begin(); iter != neighborList.end(); ++iter) {
        // Do not include the Boid.
        if((uint32_t)(*iter) == boid.getId()) continue;
        output[current++] = new Neighbor(&boids[*iter], (boids[*iter].getPosition() - boid.getPosition()).norm());
    }

    // Return the Neighbor List.
    return output;
}

// The Function "Simulation::Neighbor** Simulation::getNearbyFood(const Boid& boid, int& num) const" returns the nearest pieces of Food to the Boid. The
// First Parameter in the Private Member Function "Simulation::Neighbor** Simulation::getNearbyFood(const Boid& boid, int& num) const" that is the
// Reference Variable "const Boid& boid" defines a Boid that is used as the basis for detection. The Second Parameter in the Private Member
// Function "Simulation::Neighbor** Simulation::getNearbyFood(const Boid& boid, int& num) const" that is the Reference Variable "int& num" defines the
// number of food found by the Function "Simulation::Neighbor** Simulation::getNearbyFood(const Boid& boid, int& num) const".
Simulation::Neighbor** Simulation::getNearbyFood(const Boid& boid, int& num) const {
    // Initialize to the Number of desired Food Targets.
    int amountOfFood = 1;
    Neighbor** output = (Neighbor**)malloc(sizeof(Neighbor*) * amountOfFood);

    // Ensure all of the pointers are set to null so that the data does not potentially point to a bad sector.
    for(int iIndex = 0; iIndex < amountOfFood; ++iIndex) {
        output[iIndex] = nullptr;
    }

    // Initialize Num to 0 and then create a for() to Loop through all of the Boids in the Simulation.
    num = 0;
    for(size_t iIndex = 0; iIndex < food.size(); ++iIndex) {
        // If the Food has been eaten, then ignore it.
        if(!food[iIndex].getIsEaten()) {
            // If the Food is not within bounds, there is no point in searching it.
            if(food[iIndex].getPosition()[2] > 0.25) continue;

            // Get the distance of the Food and check if it is close enough to be detected.
            double distance = (boid.getPosition() - food[iIndex].getPosition()).norm();
            if(distance > neighborRadius) continue;

            // If the Neighbor Array has not yet been filled, then all the Neighbors that are close enough get added.
            if(num < amountOfFood) {
                output[num++] = new Neighbor(
                    &food[iIndex],
                    distance
                );
                continue;
            }

            // If there is no more room, then find the Neighbor that is futhest away from the Boid.
            int index = 0;
            double closest = -std::numeric_limits<double>::max();
            for(int jIndex = 0; jIndex < amountOfFood; ++jIndex) {
                if(output[jIndex]->distance > closest) {
                    index = jIndex;
                    closest = output[jIndex]->distance;
                }
            }

            // Delete the Neighbor and fill the slot with a new Neighbor.
            delete output[index];

            output[index] = new Neighbor(
                &food[index],
                distance
            );
        }
    }
    
    // Return the Requested List of Food.
    return output;
}

// The Private Member Function "Eigen::Vector3d Simulation::getCenteringVector(const Boid& boid, Neighbor** neighbors, const int& count)" uses the List of Neighbors
// to determine the cohesive force by steering the Boid to the center point of the Neighbors. The First Parameter in the Private Member Function "Eigen::Vector3d
// Simulation::getCenteringVector(const Boid& boid, Neighbor** neighbors, const int& count)" that is "boid" defines the boid being used as a basis. The
// Second Parameter in the Private Member Function "Eigen::Vector3d Simulation::getCenteringVector(const Boid& boid, Neighbor** neighbors, const int& count)"
// that is "neighbors" defines the list of the nearest neighbors that the boid has. The Third Parameter in the Private Member Function "Eigen::Vector3d
// Simulation::getCenteringVector(const Boid& boid, Neighbor** neighbors, const int& count)" that is "count" defines the number of neighbors in the list.
Eigen::Vector3d Simulation::getCenteringVector(const Boid& boid, Neighbor** neighbors, const int& count) {
    // First, initialize the center vector. Then, check, if we have no neighbors or if it is a predator, then skip this vector.
    Eigen::Vector3d center(0,0,0);
    if(count == 0 || boid.isPredator()) return center;

    // Create a for() Loop in order to loop through the neighbors to collect the centered point.
    int val = 0;
    for(int iIndex = 0; iIndex < count; ++iIndex) {
        // Ignore the predators.
        if(!neighbors[iIndex]->boid->isPredator()) {
            center += neighbors[iIndex]->boid->getPosition();
            val++;
        }
    }

    // If there are predators around, then do not use the centering vector.
    if(val == 0) return center;

    // Compute the average of the points to get the proper center point.
    center /= val;

    // Convert into a vector position in relation to the Boid. Then, normalize to create a steering value.
    return (center - boid.getPosition()).normalized();
}

// The Private Member Function "Eigen::Vector3d Simulation::getPreyVector(const Boid& boid, Neighbor** neighbors, const int& count)" defines the logic that if the
// boid is prey, then this vector will point away from the predators in the Neighbor List. The First Parameter in the Private Member Function "Eigen::Vector3d
// Simulation::getPreyVector(const Boid& boid, Neighbor** neighbors, const int& count)" that is "boid" indicates the boid that we are using as a basis. The
// Second Parameter in the Private Member Function "Eigen::Vector3d Simulation::getPreyVector(const Boid& boid, Neighbor** neighbors, const int& count)"
// that is "neighbors" defines the list of the neareast Neighbors the boid has. The Third Parameter in the Private Member Function "Eigen::Vector3d
// Simulation::getPreyVector(const Boid& boid, Neighbor** neighbors, const int& count)" that is "count" defines the number of neighbors in the list.
Eigen::Vector3d Simulation::getPreyVector(const Boid& boid, Neighbor** neighbors, const int& count) {
    // First, initalize the prey vector. Then, skip if there are no neighbors or if the boid is a predator.
    Eigen::Vector3d prey(0,0,0);
    if(count == 0 || boid.isPredator()) return prey;

    // Create a for() Loop to loop through the Neighbor List in order to include it to the prey vector.
    int val = 0;
    for(int iIndex = 0; iIndex < count; ++iIndex) {        
        // Only count the predators.
        if(neighbors[iIndex]->boid->isPredator()) {
            // Add the vector from the boid to the predator as a steering vector.
            prey += (boid.getPosition() - neighbors[iIndex]->boid->getPosition()).normalized();
            val++;
        }
    }

    // If there are no predators, then return.
    if(val == 0) return prey;

    // Normalize the vector into a steering direction.
    return prey.normalized();
}

// The Private Member Function "Eigen::Vector3d Simulation::getPredatorVector(const Boid& boid, Neighbor** neighbors, const int& count)" defines the logic that
// if the boid is the predator, then this vector will point at the nearest boid that is prey for the predator to chase. The First Parameter in the Private Member
// Function "Eigen::Vector3d Simulation::getPredatorVector(const Boid& boid, Neighbor** neighbors, const int& count)" that is "boid" indicates the boid that is used
// as a basis. The Second Parameter in the Private Member Function "Eigen::Vector3d Simulation::getPredatorVector(const Boid& boid, Neighbor** neighbors, const
// int& count)" that is "neighbors" defines the list of the nearest Neighbors the boid has. The Third Parameter in the Private Member Function "Eigen::Vector3d
// Simulation::getPredatorVector(const Boid& boid, Neighbor** neighbors, const int& count)" that is "count" defines the number of neighbors in the list.
Eigen::Vector3d Simulation::getPredatorVector(const Boid& boid, Neighbor** neighbors, const int& count) {
    // First, initialize the predator vector. Then, skip if the boid is not a predator.
    Eigen::Vector3d predator(0,0,0);
    if(count == 0 || !boid.isPredator()) return predator;

    // Find the closest boid that is prey.
    double distance = std::numeric_limits<double>::max();
    for(int iIndex = 0; iIndex < count; ++iIndex) {
        // Ignore the other predators.
        if(!neighbors[iIndex]->boid->isPredator()) {
            if(neighbors[iIndex]->distance < distance) {
                predator = neighbors[iIndex]->boid->getPosition();
                distance = neighbors[iIndex]->distance;
            }
        }
    }
    
    // Return the steering value for the prey that is found.
    return (predator - boid.getPosition()).normalized();
}

// The Private Member Function "Eigen::Vector3d Simulation::getLeaderVector(const Boid& boid, Neighbor** neighbors, const int& count)" defines the logic that if the
// boid is prey, then this vector will point towards the direction of leaders to follow. The First Parameter in the Private Member Function "Eigen::Vector3d
// Simulation::getLeaderVector(const Boid& boid, Neighbor** neighbors, const int& count)" that is "boid" indicates the boid that is used as a basis. The
// Second Parameter in the Private Member Function "Eigen::Vector3d Simulation::getLeaderVector(const Boid& boid, Neighbor** neighbors, const int& count)"
// that is "neighbors" defines the list of the nearest neighbors the boid has. In the Private Member Function "Eigen::Vector3d Simulation::getLeaderVector(const
// Boid& boid, Neighbor** neighbors, const int& count), the Third Parameter that is "count" defines the number of neighbors in the list.
Eigen::Vector3d Simulation::getLeaderVector(const Boid& boid, Neighbor** neighbors, const int& count) {
    // First, initialize the leader vector and then skip if the boid is a predator or leader.
    Eigen::Vector3d leader(0,0,0);
    if(count == 0 || boid.isPredator()|| boid.isLeader()) return leader;

    // Next, find all of the leader vectors. Then, add the leader vectors together.
    int val = 0;
    for(int iIndex = 0; iIndex < count; ++iIndex) {
        // Only add leader boids.
        if(neighbors[iIndex]->boid->isLeader()) {
            // Target a position just behind the leader.
            Eigen::Vector3d target = neighbors[iIndex]->boid->getPosition() - (neighbors[iIndex]->boid->getVelocity() * 0.05);
            leader += (target - boid.getPosition()).normalized();
            val++;
        }
    }

    // If no leaders are found, then ignore the vector.
    if(val == 0) return leader;

    // Return the vector to the leader.
    return leader.normalized();
}

// The Private Member Function "Eigen::Vector3d Simulation::getCollisionVector(const Boid& boid, Neighbor** neighbors, const int& count)" provides a collision
// avoidance steering force. In the Private Member Function "Eigen::Vector3d Simulation::getCollisonVector(const Boid& boid, Neighbor** neighbors, const
// int& count)", the First Parameter that is "boid" indicates the boid that is used as a basis. The Second Parameter in the Private Member Function "Eigen::Vector3d
// Simulation::getCollisionVector(const Boid& boid, Neighbor** neighbors, const int& count)" that is "neighbors" defines the list of nearest neighbors that the boid
// has. The Third Parameter in the Private Member Function "Eigen::Vector3d Simulation::getCollisionVector(const Boid& boid, Neighbor** neighbors, const int& count)"
// that is "count" defines the number of neighbors in the list.
Eigen::Vector3d Simulation::getCollisionVector(const Boid& boid, Neighbor** neighbors, const int& count) {
    // First, initialize the Collision Vector and skip if nobody is in range.
    Eigen::Vector3d collision(0,0,0);
    if(count == 0) return collision;

    // Create a for() Loop to loop through the neighbors and find the vector that best moves away from colliding based on distance.
    for(int iIndex = 0; iIndex < count; ++iIndex) {
        collision += (boid.getPosition() - neighbors[iIndex]->boid->getPosition()).normalized() / (neighbors[iIndex]->distance + std::numeric_limits<double>::epsilon());
    }

    // Convert into steering force.
    return collision.normalized();
}

// The Private Member Function "Eigen::Vector3d Simulation::getVelocityVector(const Boid& boid, Neighbor** neighbors, const int& count)" defines the logic that
// if the boid is prey, then this vector will steer them to move alongside their prey neighbors. The First Parameter in the Private Member Function "Eigen::Vector3d
// Simulation::getVelocityVector(const Boid& boid, Neighbor** neighbors, const int& count)" that is "boid" indicates the boid used as a basis. The Second Parameter
// in the Private Member Function "Eigen::Vector3d Simulation::getVelocityVector(const Boid& boid, Neighbor** neighbors, const int& count)" that is "neighbors"
// defines the list of the nearest neighbors the boid has. The Third Parameter in the Private Member Function "Eigen::Vector3d Simulation::getVelocityVector(const
// Boid& boid, Neighbor** neighbors, const int& count)" that is "count" defines the number of neighbors in the list. 
Eigen::Vector3d Simulation::getVelocityVector(const Boid& boid, Neighbor** neighbors, const int& count) {
    // First, initialize the "neighborVelocity" vector and skip if the boid is a predator.
    Eigen::Vector3d neighborVelocity(0,0,0);
    if(count == 0 || boid.isPredator()) return neighborVelocity;

    // Second, create a for() Loop to loop through and find all other prey to flock with.
    int val = 0;
    for(int iIndex = 0; iIndex < count; ++iIndex) {
        // Ignore the predators because the boids that are identified as prey need to run from them.
        if(!neighbors[iIndex]->boid->isPredator()) {
            neighborVelocity += neighbors[iIndex]->boid->getVelocity();
            val++;
        }
    }

    // Third, If no neighbors are found, then ignore the force.
    if(val == 0) return neighborVelocity;

    // Finally, normalize the direction into a steering vector and return.
    neighborVelocity /= val;
    return neighborVelocity.normalized();
}

// The Private Member Function "Eigen::Vector3d Simulation::getFoodVector(const Boid& boid)" indicates that this vector lets the prey find food when it is in
// range and eat if it collides.
Eigen::Vector3d Simulation::getFoodVector(const Boid& boid) {
    // First, initialize the Food Vector and skip if the boid is a predator.
    Eigen::Vector3d foodVec(0,0,0);
    if(boid.isPredator()) return foodVec;

    // Second, find the nearest Food.
    int countOfFood;
    Neighbor** nearFood = getNearbyFood(boid, countOfFood);

    // Third, do a vector shift towards the food.
    for(int iIndex = 0; iIndex < countOfFood; ++iIndex) { 
        double inverseSquareOfDistanceBetweenBoids = 1.0 / sqrt(nearFood[iIndex]->distance);
        foodVec += (boid.getPosition() - nearFood[iIndex]->boid->getPosition()) * inverseSquareOfDistanceBetweenBoids;
        
        if(nearFood[iIndex]->distance <= (2 * size)) {
            ((Food*)nearFood[iIndex]->boid)->eat();
        }
    }

    // Next, clear the Food List memory.
    for(int iIndex = 0; iIndex < countOfFood; ++iIndex) {
        if(nearFood[iIndex] != nullptr) {
            delete nearFood[iIndex];
            nearFood[iIndex] = nullptr;
        }
    }
    free(nearFood);

    // Finally, return a steering vector towards the Food.
    return foodVec.normalized();
}

// The Private Member Function "Eigen::Vector3d Simulation::boundingVolume(const Boid& boid) const" checks if the boid is in the bounding volume and
// adds a steering force to turn around.
Eigen::Vector3d Simulation::boundingVolume(const Boid& boid) const {
    // First, retrieve the velocity and position of the boid.
    Eigen::Vector3d pos = boid.getPosition();
    Eigen::Vector3d velocity(0,0,0);

    // Second, check if the boids are outside the box and apply the steering force appropriately.
    if(pos[0] < -0.5) {
        velocity[0] = 1;
    } else if(pos[0] > 0.5) {
        // Flip the X velocity.
        velocity[0] = -1;
    }
    if(pos[1] < -0.25) {
        velocity[1] = 1;
    } else if(pos[1] > 0.25) {
        // Flip the Y velocity.
        velocity[1] = -1;
    }
    if(pos[2] < -0.125) {
        velocity[2] = 1;
    } else if(pos[2] > 0.125) {
        // Flip the Z velocity.
        velocity[2] = -1;
    }

    // Finally, normalize into a steering force.
    return velocity.normalized();
}

// The Function "void Simulation::step(int frame)" performs a single step of the Boids Simulation by storing the values for the next step. The Parameter in the
// Function "void Simulation::step(int Frame)" that is the Integer Variable "int frame" defines the current frame of the animation.
void Simulation::step(int frame) {
    // First, retrieve the locations of the Boids for the K-Dimensional KD Tree.
    std::vector<Eigen::Vector3d> points;
    KDTree* tree = nullptr;
    int count;

    // Second, use the K-Dimensional KD Tree store the positions of the Boids in the K-Dimensional KD Tree.
    if(useTree) {
        points.reserve(boids.size());
        for(size_t iIndex = 0; iIndex < boids.size(); ++iIndex) {
            points.push_back(boids[iIndex].getPosition());
        }
        tree = new KDTree(points);
    }

    // Third, create a for() Loop to loop through each Boid and perform it's step.
    for(std::vector<Boid>::iterator boid = boids.begin(); boid != boids.end(); ++boid) {
        Neighbor** neighbors = nullptr;

        // Next, find the Neighbors of the Boid.
        if(useTree) {
            neighbors = getNeighborsViaTree(tree, points, *boid, count);
        } else {
            neighbors = getNeighbors(*boid, count);
        }

        // Then, update the Boid.
        Eigen::Vector3d velocity = boid->getVelocity();
        Eigen::Vector3d pos = boid->getPosition();
        double speed = boid->getSpeed();

        // Perform the Flock Calculations.
        Eigen::Vector3d center = centeringForce * getCenteringVector(*boid, neighbors, count);
        Eigen::Vector3d collision = collisionForce * getCollisionVector(*boid, neighbors, count);
        Eigen::Vector3d neighborVelocity = velocityForce * getVelocityVector(*boid, neighbors, count);
        Eigen::Vector3d foodVec = hunger * getFoodVector(*boid);
        Eigen::Vector3d prey = 0.1 * getPreyVector(*boid, neighbors, count);
        Eigen::Vector3d predator = 0.1 * getPredatorVector(*boid, neighbors, count);
        Eigen::Vector3d leader = centeringForce * getLeaderVector(*boid, neighbors, count);
        Eigen::Vector3d flockBox = boundingVolume(*boid);

        // Update the Acceleration.
        Eigen::Vector3d accelleration = velocity + center + collision + neighborVelocity + flockBox + predator + prey + leader + foodVec;
        
        // Determine the desired speed from the steering vectors.
        double desiredSpeed = 0;
        if(!accelleration.isZero()) {
            desiredSpeed = accelleration.norm();
            accelleration.normalized();
        }

        // Determine the amount of speed change and the change in direction
        desiredSpeed -= speed;
        desiredSpeed *= deltaTime * deltaTime * damping;
        velocity += accelleration;
        velocity.normalize();

        // Modify the speed of the Boid.
        speed = (speed + desiredSpeed) * damping;
        if(speed > 0.2) {
            speed = 0.2;
        }

        // Update the Boid's Position.
        pos += velocity * speed * deltaTime;


        // Detect Problems.
        if(std::isnan(pos[0])) {
            std::cout << "NAN detected" << std::endl;
            std::cout << "Position: " << boid->getPosition()[0] << ", " << boid->getPosition()[1] << ", " << boid->getPosition()[2] << std::endl;
            std::cout << "Velocity: " << boid->getVelocity()[0] << ", " << boid->getVelocity()[1] << ", " << boid->getVelocity()[2] << std::endl;
            
            std::cout << "Food force: " << foodVec[0] << ", " << foodVec[1] << ", " << foodVec[2] << std::endl;
            std::cout << "Collision force: " << collision[0] << ", " << collision[1] << ", " << collision[2] << std::endl;
            std::cout << "Centering force: " << center[0] << ", " << center[1] << ", " << center[2] << std::endl;
            std::cout << "Velocity Matching force: " << neighborVelocity[0] << ", " << neighborVelocity[1] << ", " << neighborVelocity[2] << std::endl;
            std::cout << "delta Velocity: " << velocity[0] << ", " << velocity[1] << ", " << velocity[2] << std::endl;
            exit(1);
        }
        
        // Update the Boid in the simulation.
        boid->setSpeed(speed);
        boid->setVelocity(velocity);
        boid->setPosition(pos);

        // Clear the Neighbors.
        for(int iIndex = 0; iIndex < count; ++iIndex) {
            if(neighbors[iIndex] != nullptr) {
                delete neighbors[iIndex];
                neighbors[iIndex] = nullptr;
            }
        }
        free(neighbors);
    }

    // Perform movement on the Food.
    for(std::vector<Food>::iterator foodI = food.begin(); foodI != food.end(); ++foodI) {
        if(foodI->getAdditionTime() <= frame && !foodI->getIsEaten()) {
            Eigen::Vector3d pos = foodI->getPosition();
            Eigen::Vector3d vel = foodI->getVelocity();

            // Add some Perturbations.
            double randomJitter = ((rand() / (double)std::numeric_limits<int>::max()) * 2.0 - 1.0) / 10;
            vel += Eigen::Vector3d(randomJitter, 0, randomJitter);

            pos += vel * deltaTime;
            foodI->setPosition(pos);
        }
    }

    // Finally, clear the K-Dimensional KD Tree under the condition that the K-Dimensional KD Tree was used.
    if(tree != nullptr) {
        delete tree;
    }
}
