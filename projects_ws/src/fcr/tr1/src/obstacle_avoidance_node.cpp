#include "tr1/obstacle_avoidance.h"


int main(int argc, char** argv)
{
    ros::init(argc, argv, "obsavoidance");
    ros::NodeHandle n;
    ObstacleAvoidance obsavoidance(n);
    obsavoidance.spin();
}

//INFO

// Com this->scan_msg_.ranges.size();
// Observou-se que o laser possui 720 leituras
// O laser passa a funcionar a partir de um certo número de iterações.
