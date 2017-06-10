#include "tr5_simu/trabalho.h"

int main(int argc, char** argv)
{
    ros::init(argc, argv, "trabalho5_simu");
    ros::NodeHandle n;                  // node handle global
    Trabalho tr(n);
    tr.spinTrab5();
}

