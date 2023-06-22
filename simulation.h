// Computer Graphics Project Assignment 5 Boids Simulation Animation
// simulation.h
#pragma once
#ifndef SIMULATION_H
#define SIMULATION_H

#include "boid.h"
#include "food.h"
#include "kdTree.H"
#include <string>
#include <vector>

class Simulation {
public:
    Simulation(std::string filename, bool useKDimensionalTree);
    ~Simulation();

    void run(std::string filename);
    void step(int i);
    const std::vector<Boid>& getBoids() const;
    const std::vector<Food>& getFood() const;

private:
    struct Neighbor {
        const Boid *boid;
        double distance;

        Neighbor(const Boid* bIsForBoid, double dIsForDistance) {
            boid = bIsForBoid;
            distance = dIsForDistance;
        }
    };

    Neighbor** getNeighbors(const Boid& boid, int& num) const;
    Neighbor** getNeighborsViaTree(KDTree* tree, const std::vector<Eigen::Vector3d>& positionsOfAllBoids, const Boid& boid, int& num) const;
    Neighbor** getNearbyFood(const Boid& boid, int& num) const;

    Eigen::Vector3d getCenteringVector(const Boid& boid, Neighbor** neighbors, const int& count);
    Eigen::Vector3d getPreyVector(const Boid& boid, Neighbor** neighbors, const int& count);
    Eigen::Vector3d getPredatorVector(const Boid& boid, Neighbor** neighbors, const int& count);
    Eigen::Vector3d getLeaderVector(const Boid& boid, Neighbor** neighbors, const int& count);
    Eigen::Vector3d getCollisionVector(const Boid& boid, Neighbor** neighbors, const int& count);
    Eigen::Vector3d getVelocityVector(const Boid& boid, Neighbor** neighbors, const int& count);
    Eigen::Vector3d getFoodVector(const Boid& boid);
    Eigen::Vector3d boundingVolume(const Boid& boid) const;

    std::vector<Boid> boids;
    std::vector<Food> food;

    bool useTree;
    double size;
    double neighborRadius;
    int numberOfNeighbors;
    double mass;
    double collisionForce;
    double centeringForce;
    double velocityForce;
    double hunger;
    double damping;
    double deltaTime;
    int length;
    int currentFrame;
};

#endif

