#include "tr5_zrobot/trabalho.h"

int main(int argc, char** argv)
{
    ros::init(argc, argv, "trabalho5_zrobot");
    ros::NodeHandle n;                  // node handle global
    Trabalho tr(n);
    tr.spinTrab5();
}

