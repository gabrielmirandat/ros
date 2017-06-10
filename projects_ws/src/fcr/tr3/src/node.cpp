#include "tr3/trabalho.h"

int main(int argc, char** argv)
{
    ros::init(argc, argv, "trabalho3");
    ros::NodeHandle n;                  // node handle global
    Trabalho tr(n);
    tr.spinTrab3();
}

