// Retirado de referências
// http://www.geeksforgeeks.org/dijkstras-shortest-path-algorithm-using-priority_queue-stl/
// http://codeclassics.blogspot.com.br/2012/12/dijkstras-algorithm-in-c-using-stl.html

// grafo cic - 18 nos, 19 arestas
// construido com proporcoes do mapa do cic pdf *10 - em milimetros
// A->B:18
// A->D:26
// D->F:29
// F->G:19
// F->I:20
// I->K:21
// K->L:18
// K->N:24
// N->P:18
// P->Q:18
// Q->R:18
// R->O:18
// O->M:24
// M->L:18
// M->J:23
// J->H:20
// H->G:19
// H->E:28
// E->C:27
// C->B:19


#ifndef GRAPH_H
#define GRAPH_H

#include <ros/ros.h>
#include <ros/console.h>
#include <iostream>
#include <vector>
#include <algorithm>
#include <limits>
#include <queue>

#include "tr3/common.h"


class Comparator
{
public:
    int operator() ( const std::pair<int,float>& p1, const std::pair<int,float> &p2){ return p1.second>p2.second;}
};

class Graph
{
private:
    std::vector<std::vector<std::pair<int,float> > > graph_;
    std::vector<int> path_;

    void addEdge(int source, int destination, int weight);
    char mapNodeToChar(int id){ return (char) id+65;}

public:
    // Constructor
    Graph();

    void createCicGraph();
    void dijkstra(int source, int destination);

    std::vector<int> getPath() const {return path_;}
};

#endif // GRAPH_H
