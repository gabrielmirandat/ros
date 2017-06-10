#include "tr5_simu/topological_map.h"

TopologicalMap::TopologicalMap()
{
    initializeResources();
}

TopologicalMap::~TopologicalMap()
{
    for(size_t i=0; i < nodes_.size(); i++)
        delete nodes_[i];
}

void TopologicalMap::initializeResources()
{
    for(size_t i=0; i < nodes_.size(); i++)
        delete nodes_[i];

    nodes_.clear();
}

void TopologicalMap::createCicMap()
{

    initializeResources();

    nodes_.resize(TOPO_MAP_NODE_SIZE);

    double d_quad       = 1.30  + 0.2;
    double y_mid        = 7.14  + 0.2;
    double x_left       = 10.99 + 0.2;
    double x_mid_right  = 6.65  + 0.2;
    double x_corridor   = 3.47  + 0.2;

    nodes_[A] = new Node(-27.38, 17.67, d_quad       , d_quad ); //A
    nodes_[B] = new Node(-27.20,  8.67, d_quad       , y_mid  ); //B
    nodes_[C] = new Node(-27.18,  -.19, d_quad       , d_quad ); //C
    nodes_[D] = new Node(-14.09, 17.58, x_left       , d_quad ); //D
    nodes_[E] = new Node(-14.01,  -.17, x_left       , d_quad ); //E
    nodes_[F] = new Node(  -.96, 17.60, d_quad       , d_quad ); //F
    nodes_[G] = new Node(  -.78,  8.75, d_quad       , y_mid  ); //G
    nodes_[H] = new Node(  -.50,  -.22, d_quad       , d_quad ); //H
    nodes_[I] = new Node(  7.74, 17.76, x_mid_right  , d_quad ); //I
    nodes_[J] = new Node(  8.05,  -.22, x_mid_right  , d_quad ); //J
    nodes_[K] = new Node( 19.07, 17.83, x_corridor   , d_quad ); //K
    nodes_[L] = new Node( 19.12,  8.69, x_corridor   , y_mid  ); //L
    nodes_[M] = new Node( 18.98,  -.27, x_corridor   , d_quad ); //M
    nodes_[N] = new Node( 30.62, 17.74, x_mid_right  , d_quad ); //N
    nodes_[O] = new Node( 30.20,  -.23, x_mid_right  , d_quad ); //O
    nodes_[P] = new Node( 38.71, 17.72, d_quad       , d_quad ); //P
    nodes_[Q] = new Node( 38.87,  8.86, d_quad       , y_mid  ); //Q
    nodes_[R] = new Node( 38.61,  -.23, d_quad       , d_quad ); //R
}

// retorna -1 se nao esta em no algum
// recebe em metros
int TopologicalMap::nodeGivenPosition(double pos_x, double pos_y)
{
    double cx,cy,dx,dy;
    bool inside_x, inside_y;
    inside_x = inside_y = false;

    for(size_t i=0; i < nodes_.size(); i++)
    {
        cx = nodes_[i]->getCenterX();
        cy = nodes_[i]->getCenterY();
        dx = nodes_[i]->getDispX();
        dy = nodes_[i]->getDispY();

        inside_x = (pos_x < cx + dx) && (pos_x > cx - dx);
        inside_y = (pos_y < cy + dy) && (pos_y > cy - dy);

        if( inside_x && inside_y)
        {
            return i;
        }
    }
    return -1;
}

std::vector<double> TopologicalMap::nodesCenterXByIdVector(std::vector<int> id_vec)
{
    std::vector<double> output_vec;

    for(size_t i=0; i<id_vec.size(); i++)
        output_vec.push_back( nodes_[id_vec[i]]->getCenterX());

    return output_vec;
}

std::vector<double> TopologicalMap::nodesCenterYByIdVector(std::vector<int> id_vec)
{
    std::vector<double> output_vec;

    for(size_t i=0; i<id_vec.size(); i++)
        output_vec.push_back(nodes_[id_vec[i]]->getCenterY());

    return output_vec;
}

std::vector<double> TopologicalMap::getAllNodesCenterX()
{
    std::vector<double> ret_vec;

    for(size_t i=0; i< nodes_.size(); i++)
        ret_vec.push_back(nodes_[i]->getCenterX());

    return ret_vec;
}

std::vector<double> TopologicalMap::getAllNodesCenterY()
{
    std::vector<double> ret_vec;

    for(size_t i=0; i< nodes_.size(); i++)
        ret_vec.push_back(nodes_[i]->getCenterY());

    return ret_vec;
}

std::vector<double> TopologicalMap::getAllNodesDispX()
{
    std::vector<double> ret_vec;

    for(size_t i=0; i< nodes_.size(); i++)
        ret_vec.push_back(nodes_[i]->getDispX());

    return ret_vec;
}

std::vector<double> TopologicalMap::getAllNodesDispY()
{
    std::vector<double> ret_vec;

    for(size_t i=0; i< nodes_.size(); i++)
        ret_vec.push_back(nodes_[i]->getDispY());

    return ret_vec;
}
