#include <iostream>
#include "Robot.hpp"
int main(int argc, char** argv)
{
    smurf::Robot robot;
    robot.loadFromSmurf(argv[1]);

	return 0;
}
