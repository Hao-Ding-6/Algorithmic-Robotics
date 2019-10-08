///////////////////////////////////////
// COMP/ELEC/MECH 450/550
// Project 3
// Authors: FILL ME OUT!!
//////////////////////////////////////

#include <iostream>

// Your random tree planner
#include "RTP.h"

void benchmarkCubicles()
{
    // TODO
}

void benchmarkTwistycool()
{
    // TODO
}

int main(int /* argc */, char ** /* argv */)
{
    int environment;

    do
    {
        std::cout << "Benchmark for: " << std::endl;
        std::cout << " (1) Cubicles" << std::endl;
        std::cout << " (2) Twistycool" << std::endl;

        std::cin >> environment;
    } while (environment < 1 || environment > 2);

    switch (environment)
    {
        case 1:
            benchmarkCubicles();
            break;
        case 2:
            benchmarkTwistycool();
            break;
        default:
            std::cerr << "Invalid Environment!" << std::endl;
            break;
    }

    return 0;
}
