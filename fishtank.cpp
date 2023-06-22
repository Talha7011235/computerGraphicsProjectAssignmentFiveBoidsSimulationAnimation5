// Computer Graphics Project Assignment 5 Boids Simulation Animation
// fishtank.cpp
#include "simulation.h"
#include <iostream>
#include <string>

int main(int argc, char** argv) {
    if(argc < 2) {
        std::cerr << "USAGE:" << std::endl;
        std::cerr << argv[0] << " [-tree] inputFile outputFile" << std::endl;
        return 1;
    }

    std::string input[] = {
        "sample.in",
        "sample.out",
        ""
    };

    int step = 0;
    for(int iIndex = 1; iIndex < argc; ++iIndex) {
        if(strcmp(argv[iIndex], "-tree") == 0) {
            input[2] = "t";
            continue;
        }

        if(step >= 2) {
            std::cerr << "Invalid values given" << std::endl;
            return 1;
        }

        input[step++] = argv[iIndex];
    }

    Simulation simulate(input[0], input[2][0] == 't');

    simulate.run(input[1]);

    return 0;
}
