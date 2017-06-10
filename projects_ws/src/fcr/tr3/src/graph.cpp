#include "tr3/graph.h"


Graph::Graph()
{
}

void Graph::addEdge(int source, int destination, int weight)
{
    graph_[source].push_back(std::make_pair(destination,weight));
}

void Graph::createCicGraph()
{
    graph_.resize(TOPO_MAP_NODE_SIZE);

    addEdge(A,B,18); addEdge(B,A,18);
    addEdge(A,D,26); addEdge(D,A,26);
    addEdge(D,F,29); addEdge(F,D,29);
    addEdge(F,G,19); addEdge(G,F,19);
    addEdge(F,I,20); addEdge(I,F,20);
    addEdge(I,K,21); addEdge(K,I,21);
    addEdge(K,L,18); addEdge(L,K,18);
    addEdge(K,N,24); addEdge(N,K,24);
    addEdge(N,P,18); addEdge(P,N,18);
    addEdge(P,Q,18); addEdge(Q,P,18);
    addEdge(Q,R,18); addEdge(R,Q,18);
    addEdge(R,O,18); addEdge(O,R,18);
    addEdge(O,M,24); addEdge(M,O,24);
    addEdge(M,L,18); addEdge(L,M,18);
    addEdge(M,J,23); addEdge(J,M,23);
    addEdge(J,H,20); addEdge(H,J,20);
    addEdge(H,G,19); addEdge(G,H,19);
    addEdge(H,E,28); addEdge(E,H,28);
    addEdge(E,C,27); addEdge(C,E,27);
    addEdge(C,B,19); addEdge(B,C,19);
}

void Graph::dijkstra(int source, int destination)
{
    ROS_INFO("[Graph] No inicial: %c", (char) destination + 65);
    ROS_INFO("[Graph] No final: %c", (char) source + 65);

    if(source < 0 || destination < 0)
        ROS_ERROR("[Graph] Nodo fonte ou node destino menos que zero!");


    std::vector<float> d(graph_.size());
    std::vector<int> parent(graph_.size());

    for(unsigned int i = 0 ;i < graph_.size(); i++)
    {
        d[i] = std::numeric_limits<float>::max();
        parent[i] = -1;
    }

    std::priority_queue<std::pair<int,float>, std::vector<std::pair<int,float> >, Comparator> Q;
    d[source] = 0.0f;
    Q.push(std::make_pair(source,d[source]));

    while(!Q.empty())
    {
        int u = Q.top().first;
        if(u==destination) break;

        Q.pop();

        for(unsigned int i=0; i < graph_[u].size(); i++)
        {
            int v= graph_[u][i].first;
            float w = graph_[u][i].second;

            if(d[v] > d[u]+w)
            {
                d[v] = d[u]+w;
                parent[v] = u;
                Q.push(std::make_pair(v,d[v]));
            }
        }
    }


    path_.clear();
    int p = destination;
    path_.push_back(destination);
    while(p!=source)
    {
        p = parent[p];
        path_.push_back(p);
    }

    std::cout << "\n[Graph] Caminho de Dijkstra: ";
    for(size_t i=0; i< path_.size(); i++ )
    {
        std::cout << mapNodeToChar(path_[i]) << " ";
    }
    std::cout << std::endl;
}
