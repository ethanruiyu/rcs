#ifndef GRAPH_H
#define GRAPH_H

#include<set>
#include<map>
#include<utility>
#include<vector>

#include<CoveragePathPlanner/prm_planner/types.h>



typedef unsigned int Vertex;
typedef double Weight;
typedef std::pair<Vertex, Weight> Edge;
typedef std::set<Edge> Edges;

class Graph
{
 public:
  Graph(unsigned int max_neighbors);

  bool addVertex(const Vertex v);
  bool removeVertex(const Vertex v);

  bool addEdge(const Vertex v0, const Vertex v1, const Weight w);
  void removeEdgesWithVertex(const Vertex v);

  unsigned int getEdgeCount(const Vertex v);

  std::vector<Vertex> shortestPath(const Vertex start, const Vertex goal);

  bool canConnect(const Vertex v);
  
  void clearContainer();

  std::map<Vertex, Edges> container() const;

 private:
  std::vector<Vertex> constructPath(std::map<Vertex, Vertex> parents, 
    Vertex goal);

  Vertex closestVertex(std::map<Vertex, double> distances, 
    std::vector<Vertex> q);

  unsigned int max_neighbors_;
  std::map<Vertex, Edges> container_;

};// class Graph



#endif// GRAPH_H
