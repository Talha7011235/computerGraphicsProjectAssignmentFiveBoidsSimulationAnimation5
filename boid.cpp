// Computer Graphics Project Assignment 5 Boids Simulation Animation
// boid.cpp
#include "boid.h"

uint32_t idSource = 0;

Boid::Boid() : id(idSource++), position(0, 0, 0), velocity(0, 0, 0), speed(0.0)  {
    predator = id < 10;
    leader = !predator && id < 15;
}
Boid::~Boid() {}

const uint32_t Boid::getId() const {
    return id;
}

const Eigen::Vector3d Boid::getPosition() const {
    return position;
}
const Eigen::Vector3d Boid::getVelocity() const {
    return velocity;
}
const double Boid::getSpeed() const {
    return speed;
}
const bool Boid::isPredator() const {
    return predator;
}
const bool Boid::isLeader() const {
    return leader;
}

void Boid::setPosition(Eigen::Vector3d& pIsForPosition) {
    position = pIsForPosition;
}

void Boid::setVelocity(Eigen::Vector3d& newVector) {
    velocity = newVector;
}

void Boid::setSpeed(double newSpeed) {
    speed = newSpeed;
}

const bool Boid::operator==(const Boid& bIsForBoid) const {
    return id == bIsForBoid.id;
}

const bool Boid::operator!=(const Boid& bIsForBoid) const {
    return id != bIsForBoid.id;
}

std::istream& operator>>(std::istream& in, Boid& bIsForBoid) {
    char ignore;
    in >> ignore >> bIsForBoid.position[0] >> ignore >> bIsForBoid.position[1] >> ignore >> bIsForBoid.position[2] >> ignore;
    in >> ignore >> bIsForBoid.velocity[0] >> ignore >> bIsForBoid.velocity[1] >> ignore >> bIsForBoid.velocity[2] >> ignore;

    bIsForBoid.speed = bIsForBoid.velocity.norm();
    bIsForBoid.velocity /= bIsForBoid.speed;

    return in;
}

std::ostream& operator<<(std::ostream& out, const Boid& bIsForBoid) {
    Eigen::Vector3d vel = bIsForBoid.velocity * bIsForBoid.speed;
    out << "[" << bIsForBoid.position[0] << ", " << bIsForBoid.position[1] << ", " << bIsForBoid.position[2] << "] "
        << "[" << vel[0] << ", " << vel[1] << ", " << vel[2] << "]" << std::endl;

    return out;
}

