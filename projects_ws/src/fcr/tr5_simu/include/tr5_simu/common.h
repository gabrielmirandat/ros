#ifndef COMMON_H
#define COMMON_H

#include <tf/tf.h>
#include <tf/transform_datatypes.h>

const int TOPO_MAP_NODE_SIZE = 18;

const int LOOP_RATE = 30;
const int BASE_TIMER = 10;
const int TIMER_FORCE = LOOP_RATE*10;
const int OBSTACLE_FORCE = LOOP_RATE*5;


enum{A,B,C,D,E,F,G,H,I,J,K,L,M,N,O,P,Q,R};

#define POSANGLE(a) (((a) > 0.0)? (a) : ((a) + 360.0))
#define RAD2DEGREE(a) (POSANGLE((a)*180.0/M_PI))

#define COSDEGREE(a) (cos((a)*M_PI/180.0))
#define SENDEGREE(a) (sin((a)*M_PI/180.0))

#endif // COMMON_H
