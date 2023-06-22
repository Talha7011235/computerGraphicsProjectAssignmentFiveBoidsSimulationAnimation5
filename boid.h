// Computer Graphics Project Assignment 5 Boids Simulation Animation
// boid.h
#pragma once
#ifndef BOID_H
#define BOID_H

#include <Eigen/Dense>
#include <iostream>

class Boid {
public:
    Boid();
    ~Boid();

    const uint32_t getId() const;
    const Eigen::Vector3d getPosition() const;
    const Eigen::Vector3d getVelocity() const;
    const double getSpeed() const;
    const bool isPredator() const;
    const bool isLeader() const;

    void setPosition(Eigen::Vector3d& pIsForPosition);
    void setVelocity(Eigen::Vector3d& velocity);
    void setSpeed(double s);

    const bool operator==(const Boid& bIsForBoid) const;
    const bool operator!=(const Boid& bIsForBoid) const;

    friend std::istream& operator>>(std::istream& in, Boid& bIsForBoid);
    friend std::ostream& operator<<(std::ostream& out, const Boid& bIsForBoid);
protected:
    uint32_t id;
    Eigen::Vector3d position;
    Eigen::Vector3d velocity;
    double speed;
    bool predator;
    bool leader;
};

#endif

