// Computer Graphics Project Assignment 5 Boids Simulation Animation
// food.cpp
#include "food.h"

Food::Food() : Boid(), additionTime(0), isEaten(false) {}
Food::~Food() {}

const double Food::getAdditionTime() const {
    return additionTime;
}
const bool Food::getIsEaten() const {
    return isEaten;
}

void Food::eat() {
    isEaten = true;
}

std::istream& operator>>(std::istream& in, Food& fIsForFood) {
    char ignore;
    in >> ignore >> fIsForFood.position[0] >> ignore >> fIsForFood.position[1] >> ignore >> fIsForFood.position[2] >> ignore;
    in >> ignore >> fIsForFood.velocity[0] >> ignore >> fIsForFood.velocity[1] >> ignore >> fIsForFood.velocity[2] >> ignore;
    in >> fIsForFood.additionTime;

    return in;
}

std::ostream& operator<<(std::ostream& out, const Food& fIsForFood) {
    out << "[" << fIsForFood.position[0] << ", " << fIsForFood.position[1] << ", " << fIsForFood.position[2] << "] " << std::endl;
    return out;
}

