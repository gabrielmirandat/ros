// descobrir eixo de coordenadas do robo.
// descobrir eixo de coordenadas da odometria.
// Decisao se milimetro e metro?

// Por observação no mapa, o robo sempre começa no mesmo lugar
// de acordo com este começo do robo, as coordenadas ficaram assim
// podemos dar um jeito de deslocar a odometria do robo e os centros dos nós
// 18 nos

// A:-27.38;17.67  D:-14.09;17.58  F:-0.96;17.60  I:7.74;17.76  K:19.07;17.83  N:30.62;17.74  P:38.71;17.72
// B:-27.20;8.67                   G:-0.78;8.75                 L:19.12;8.69                  Q:38.87;8.86
// C:-27.18;-0.19  E:-14.01;-0.17  H:-0.50;-0.22  J:8.05;-0.22  M:18.98;-0.27  O:30.20;-0.23  R:38.61;-0.23

// Isto em metros, mas escolhemos centimetros para representacao

// H - E - C - B - A - D - F - G - H - J - M - L - K - I - F - G - H - J - M - O - R - Q - P - N - K

#ifndef TOPOLOGICAL_MAP_H
#define TOPOLOGICAL_MAP_H

#include <iostream>
#include <vector>
#include <string>
#include <math.h>

#include "tr3/common.h"

class Node
{
private:
    double center_x_;
    double center_y_;
    double disp_x_;
    double disp_y_;

public:
    // in centimeters
    Node(double cx, double cy, double dx, double dy)
    { center_x_ = cx; center_y_ = cy; disp_x_ = dx; disp_y_ = dy;}

    double getCenterX() const { return center_x_;}
    double getCenterY() const { return center_y_;}
    double getDispX() const { return disp_x_;}
    double getDispY() const { return disp_y_;}
};

class TopologicalMap
{
private:
    std::vector<Node*> nodes_;

    void initializeResources();

public:
    TopologicalMap();
    ~TopologicalMap();

    void createCicMap();

    int nodeGivenPosition(double pos_x, double pos_y);

    double getNodePosX(int id) const {return (double) nodes_[id]->getCenterX();}
    double getNodePosY(int id) const {return (double) nodes_[id]->getCenterY();}

    std::vector<double> nodesCenterXByIdVector(std::vector<int> id_vec);
    std::vector<double> nodesCenterYByIdVector(std::vector<int> id_vec);

    std::vector<double> getAllNodesCenterX();
    std::vector<double> getAllNodesCenterY();
    std::vector<double> getAllNodesDispX();
    std::vector<double> getAllNodesDispY();
};

#endif // TOPOLOGICAL_MAP_H
