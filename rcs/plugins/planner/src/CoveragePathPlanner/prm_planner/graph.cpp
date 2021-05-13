#include<CoveragePathPlanner/prm_planner/graph.h>



Graph::Graph(unsigned int max_neighbors)
{
  max_neighbors_ = max_neighbors;
}

bool Graph::addVertex(const Vertex v)
{
  if(container_.find(v) != container_.end())
    return false;

  container_.insert(std::pair<Vertex, Edges>(v, Edges()));
  return true;
}

bool Graph::removeVertex(const Vertex v)
{
  auto const it = container_.find(v);
  if(it == container_.end())
    return false;

  container_.erase(it);

  removeEdgesWithVertex(v);
  return true;
}

bool Graph::addEdge(const Vertex v0, const Vertex v1, const Weight w)
{
  if(  container_.find(v0) == container_.end() 
    || container_.find(v1) == container_.end())
  {
    return false;
  }

  if(  container_.find(v0)->second.size() >= max_neighbors_
    || container_.find(v1)->second.size() >= max_neighbors_)
  {
    return false;
  }

  // check if there is already an edge between these neighbors
  auto edges0 = container_.find(v0)->second;
  if(find_if(edges0.begin(), edges0.end(),
             [v1](const Edge &e){ return e.first == v1; }
            ) != edges0.end()
    )
  {
    return false;
  }

  auto edges1 = container_.find(v1)->second;
  if(find_if(edges1.begin(), edges1.end(),
             [v0](const Edge &e){ return e.first == v0; }
            ) != edges1.end()
    )
  {
    return false;
  }

  container_.find(v0)->second.insert(Edge(v1, w));
  container_.find(v1)->second.insert(Edge(v0, w));

  return true;
}

void Graph::removeEdgesWithVertex(const Vertex v)
{
  std::set<Edge>::iterator it;

  if(container_.find(v) != container_.end())
  {
    container_.find(v)->second.clear();
  }

  for(auto & u : container_)
  {
    for(it = u.second.begin(); it != u.second.end();)
    {
      Edge e = *it;
      if(e.first == v)
      {
        u.second.erase(it++);
      }
      else
      {
        ++it;
      }
    }
  }
}

Vertex Graph::closestVertex(std::map<Vertex, double> distances, 
  std::vector<Vertex> q)
{
  Vertex min_vertex = q.at(0);
  double min_dist = std::numeric_limits<double>::infinity();

  for(auto const & v : q)
  {
    if(distances[v] < min_dist)
    {
      min_vertex = v;
      min_dist = distances[v];
    }
  }

  return min_vertex;
}

unsigned int Graph::getEdgeCount(const Vertex v)
{
  if(container_.find(v) == container_.end())
  {
    return 0;
  }
  return container_.find(v)->second.size();
}


std::vector<Vertex> Graph::shortestPath(const Vertex start, const Vertex goal)
{
  std::map<Vertex, Vertex> parents;
  std::map<Vertex, double> distances;
  std::vector<Vertex> queue, path;

  if(container_.find(start) == container_.end()
    || container_.find(goal) == container_.end())
  {
    return path;
  }

  for(auto const & v : container_)
  {
    distances.insert(std::pair<Vertex, double>(v.first, 
      std::numeric_limits<double>::infinity()));
    queue.push_back(v.first);
  }

  distances[start] = 0;

  while(!queue.empty())
  {
    Vertex v = closestVertex(distances, queue);
    Edges neighbors = container_[v];

    for(auto const & n : neighbors)
    {
      double alt = distances[v] + n.second;
      if(alt < distances[n.first])
      {
        distances[n.first] = alt;
        parents[n.first] = v;
      }
    }

    queue.erase(std::remove(queue.begin(), queue.end(), v), queue.end());

    if(std::find(queue.begin(), queue.end(), goal) == queue.end())
    {
      break;
    }
  }

  return constructPath(parents, goal);
}

bool Graph::canConnect(const Vertex v)
{
  if(container_.find(v) == container_.end())
  {
    return false;
  }

  if(container_.find(v)->second.size() >= max_neighbors_)
  {
    return false;
  }

  return true;
}

void Graph::clearContainer()
{
  container_.clear();
}

std::map<Vertex, Edges> Graph::container() const
{
  return container_;
}

std::vector<Vertex> Graph::constructPath(std::map<Vertex, Vertex> parents, 
  Vertex goal)
{
  std::vector<Vertex> path;
  
  if(parents.find(goal) == parents.end())
  {
    return path;
  }

  path.push_back(goal);

  while(parents.find(path.back()) != parents.end())
  {
    path.push_back(parents.at(path.back()));
  }

  std::reverse(path.begin(), path.end());

  return path;
}


