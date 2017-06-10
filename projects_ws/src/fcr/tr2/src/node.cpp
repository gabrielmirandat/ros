#include "tr2/trabalho.h"

int main(int argc, char** argv)
{
    ros::init(argc, argv, "trabalho2");
    ros::NodeHandle n;                  // node handle global
    Trabalho tr(n);
    tr.spinTrab2();
}

