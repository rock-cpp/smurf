#include <iostream>
#include "Smurf.hpp"
int main(int argc, char** argv)
{
    smurf::Robot robot;
    robot.loadFromSmurf("../spaceclimber/SpaceClimber.smurf");
    
	return 0;
}
