// Computer Graphics Project Assignment 5 Boids Simulation Animation
// food.h
#pragma once
#ifndef FOOD_H
#define FOOD_H

#include "boid.h"

class Food : public Boid {
public:
    Food();
    ~Food();

    const double getAdditionTime() const;
    const bool getIsEaten() const;

    void eat();

    friend std::istream& operator>>(std::istream& in, Food& fIsForFood);
    friend std::ostream& operator<<(std::ostream& out, const Food& fIsForFood);
private:
    double additionTime;
    bool isEaten;
};

#endif
