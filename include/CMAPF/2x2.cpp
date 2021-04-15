#include <iostream>                  // for std::cout
#include <utility>                   // for std::pair
#include <algorithm>                 // for std::for_each
#include <boost/graph/graph_traits.hpp>
#include <boost/graph/adjacency_list.hpp>
#include <boost/graph/transpose_graph.hpp>
#include <boost/graph/topological_sort.hpp>
#include <bits/stdc++.h>
using namespace boost;
// using namespace std;

struct meta_data_t {
  typedef vertex_property_tag kind;
};

int count = 0;
struct meta_data{
  std::pair <float, float> start;
  std::pair <float, float> goal;

  std::vector <int> agent_list;

  int start_time;
  int end_time;

  int slack = 0;

  int start_vertex;
  int goal_vertex;
};

typedef property<meta_data_t, meta_data> MetaData;
typedef adjacency_list<vecS, vecS, directedS, MetaData> PrecedenceConstraintGraph;

typedef std::pair<int,int> Pair;

typedef boost::graph_traits<PrecedenceConstraintGraph>::vertex_descriptor Vertex;
typedef std::vector< Vertex > container;
/// Boost graph out edge iterator
typedef boost::graph_traits<PrecedenceConstraintGraph>::out_edge_iterator OutEdgeIter;
typedef boost::graph_traits<PrecedenceConstraintGraph>::vertex_iterator VertexIter;

bool checkpathpossible(PrecedenceConstraintGraph &G){
  count +=1;
  container c;
  topological_sort(G, std::back_inserter(c));
  property_map<PrecedenceConstraintGraph, meta_data_t>::type name = get(meta_data_t(), G);
  std::cout << "A topological ordering: ";
  for ( container::reverse_iterator ii=c.rbegin(); ii!=c.rend(); ++ii)
  {
    std::cout << std::endl;
    meta_data vertex = get(name, *ii);
    std::cout << vertex.start_time << " ";
    std::cout << vertex.end_time << " ";   
    for(auto agent: vertex.agent_list){
      std::cout << agent << " ";   
    }
    std::cout << vertex.slack << " ";   
    // cout << boost::index(*ii) << std::endl;
  }
  std::cout <<std::endl;
  return false;
}

bool generatePaths(PrecedenceConstraintGraph &G, PrecedenceConstraintGraph &G_T, container &c, container::iterator ii){
  property_map<PrecedenceConstraintGraph, meta_data_t>::type name = get(meta_data_t(), G);
  property_map<PrecedenceConstraintGraph, meta_data_t>::type name_t = get(meta_data_t(), G_T);
    
  if(c.rbegin().base()-1 == ii){
    return checkpathpossible(G);
  }

  container predecessors;
  OutEdgeIter ei, ei_end;
  for (boost::tie(ei, ei_end) = out_edges(*ii, G_T); ei != ei_end; ++ei) 
  {
      Vertex curPred = target(*ei, G_T);
      predecessors.push_back(curPred);
  }

  if(predecessors.size() == 0){
    return generatePaths(G, G_T, c, ii+1);
  }

  meta_data *curr_vertex = &get(name, *ii); 

  if(generatePaths(G, G_T, c, ii+1))
    return true;
  
  int slack = curr_vertex->slack;
  int start_time = curr_vertex->start_time;
  int end_time = curr_vertex->end_time;

  while(curr_vertex->slack){
      curr_vertex->slack--;
      curr_vertex->start_time++;
      curr_vertex->end_time++;
      for(auto pred:predecessors){
        meta_data *vertex = &get(name, pred);
        vertex->slack+=1;
      }
      if(generatePaths(G, G_T, c, ii+1))
        return true;
  }
  curr_vertex->slack = slack;
  curr_vertex->start_time = start_time;
  curr_vertex->end_time = end_time;
  for(auto pred:predecessors){
    meta_data *vertex = &get(name, pred);
    vertex->slack-=slack;
  }
  return false;
}

bool ICTS(PrecedenceConstraintGraph &G, int maxIter){
  property_map<PrecedenceConstraintGraph, meta_data_t>::type name = get(meta_data_t(), G);
  container c;
  topological_sort(G, std::back_inserter(c));

  PrecedenceConstraintGraph G_T;
  

  transpose_graph(G, G_T);
  for(int i=0; i<maxIter; i++){

    if(generatePaths(G, G_T, c, c.begin())){
      return true;
    }

    VertexIter v, vend;
    for (boost::tie(v, vend) = vertices(G); v != vend; ++v) {

      container successors;
      OutEdgeIter ei, ei_end;

      for (boost::tie(ei, ei_end) = out_edges(*v, G); ei != ei_end; ++ei) 
      {
          Vertex curSuc = target(*ei, G);
          successors.push_back(curSuc);
      }

      if(successors.size() == 0){
        meta_data *vertex = &get(name, *v);
        vertex->slack+=1;
      }

    }

  }
  
  return false;
}



int main()
{
  Pair edge_array[4] = { Pair(0,1), Pair(1,2),
                          Pair(3, 4), Pair(4, 5) };
    
  PrecedenceConstraintGraph G(6);

  property_map<PrecedenceConstraintGraph, meta_data_t>::type name = get(meta_data_t(), G);
    
  float eps = 0.0625;

  name[0] = meta_data {std::make_pair(eps*4, eps*12), std::make_pair(eps*5, eps*9), std::vector <int> {0}, 0, 4, 0,};
  name[1] = meta_data {std::make_pair(eps*5, eps*9), std::make_pair(eps*5, eps*8), std::vector <int> {0}, 4, 5, 0,};
  name[2] = meta_data {std::make_pair(eps*5, eps*8), std::make_pair(eps*5, eps*7), std::vector <int> {0}, 5, 6, 2,};

  name[3] = meta_data {std::make_pair(eps*4, eps*13), std::make_pair(eps*5, eps*11), std::vector <int> {1}, 0, 3, 0,};
  name[4] = meta_data {std::make_pair(eps*5, eps*11), std::make_pair(eps*5, eps*10), std::vector <int> {1}, 3, 4, 0,};
  name[5] = meta_data {std::make_pair(eps*5, eps*10), std::make_pair(eps*5, eps*6), std::vector <int> {1}, 4, 8, 0,};


  for (int i = 0; i < 4; ++i)
    add_edge(edge_array[i].first, edge_array[i].second, G);

  ICTS(G, 1);
  std::cout << count << std::endl;
}


