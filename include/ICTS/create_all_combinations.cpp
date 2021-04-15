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
  std::pair <int, int> start;
  std::pair <int, int> goal;

  std::vector <int> agent_list;

  int start_time;
  int end_time;

  int slack = 0;
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
  Pair edge_array[11] = { Pair(0,1), Pair(1,2), Pair(2,5), 
                          Pair(3,5), Pair(4,6), Pair(5,7), 
                          Pair(5,8), Pair(6,9), Pair(7,10), 
                          Pair(8,11), Pair(9,11) };
    
  PrecedenceConstraintGraph G(12);
  PrecedenceConstraintGraph invG(12);

  property_map<PrecedenceConstraintGraph, meta_data_t>::type name = get(meta_data_t(), G);
    
  meta_data random = {std::make_pair(1, 2), std::make_pair(1, 2), std::vector <int> {0, 1}, 0, 5, 10,};

  name[0] = meta_data {std::make_pair(1, 2), std::make_pair(1, 2), std::vector <int> {0}, 0, 5, 0,};
  name[1] = meta_data {std::make_pair(1, 2), std::make_pair(1, 2), std::vector <int> {0}, 6, 15, 0,};
  name[2] = meta_data {std::make_pair(1, 2), std::make_pair(1, 2), std::vector <int> {0}, 16, 20, 0,};
  name[3] = meta_data {std::make_pair(1, 2), std::make_pair(1, 2), std::vector <int> {1}, 0, 10, 10,};
  name[4] = meta_data {std::make_pair(1, 2), std::make_pair(1, 2), std::vector <int> {2}, 0, 10, 0,};
  name[5] = meta_data {std::make_pair(1, 2), std::make_pair(1, 2), std::vector <int> {0, 1}, 21, 30, 0,};
  name[6] = meta_data {std::make_pair(1, 2), std::make_pair(1, 2), std::vector <int> {2}, 10, 15, 0,};
  name[7] = meta_data {std::make_pair(1, 2), std::make_pair(1, 2), std::vector <int> {0}, 31, 40, 0,};
  name[8] = meta_data {std::make_pair(1, 2), std::make_pair(1, 2), std::vector <int> {1}, 31, 35, 0,};
  name[9] = meta_data {std::make_pair(1, 2), std::make_pair(1, 2), std::vector <int> {2}, 16, 25, 10,};
  name[10] = meta_data {std::make_pair(1, 2), std::make_pair(1, 2), std::vector <int> {0, 1}, 41, 50, 0,};
  name[11] = meta_data {std::make_pair(1, 2), std::make_pair(1, 2), std::vector <int> {0, 1}, 36, 45, 5,};

  for (int i = 0; i < 11; ++i)
    add_edge(edge_array[i].first, edge_array[i].second, G);

  ICTS(G, 1);
  std::cout << count << std::endl;
}


