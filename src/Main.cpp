#include <iostream>
#include "Robot.hpp"
int main(int argc, char** argv)
{
    if (argc != 2) 
        std::runtime_error("call executable with one agrument: full or relative path to the smurf file");
    
    std::string smurf_path = argv[1];
    smurf::Robot robot;
    robot.loadFromSmurf(smurf_path);
    
	return 0;
}
