#ifndef BGL_DEFINITIONS_
#define BGL_DEFINITIONS_

#include <vector>
#include <string>
#include <unordered_set>
#include <queue>
#include <exception>
#include <boost/graph/adjacency_list.hpp>
#include <boost/graph/properties.hpp>
#include <boost/graph/reverse_graph.hpp>
#include <boost/property_map/dynamic_property_map.hpp>
#include <boost/function.hpp>
#include <boost/graph/copy.hpp>
#include <boost/shared_ptr.hpp>
#include <boost/graph/graphml.hpp>
#include <time.h>

#include <Eigen/Dense>
#include <iostream>
#include <boost/program_options.hpp>

namespace HCBS {

namespace BGL_DEFINITIONS {

  enum CollisionStatus
  {
    BLOCKED,
    FREE
  };

///////////////////////// DEFINITIONS FOR INDIVIDUAL ROADMAP

  // Properties associated with each roadmap vertex.
  struct VProp
  {
    /// The underlying state of the vertex
    //State (configuration)
    Eigen::VectorXd state;

    /// Cost-to-Come
    double distance = std::numeric_limits<double>::infinity();

    /// Heuristic value
    double heuristic = std::numeric_limits<double>::infinity();

    /// Parent
    std::size_t parent;

    /// Visited
    bool visited = false;

    /// Collision status
    CollisionStatus status = CollisionStatus::FREE;

    /// Vertex Index
    size_t vertex_index;

  }; // struct VProp

  // Properties associated with each roadmap edge.
  struct EProp
  {
    /// The length of the edge using the space distance metric
    double length;

    /// Flag to check if edge is evaluated
    bool isEvaluated = false;

    /// Collision status
    CollisionStatus status = CollisionStatus::FREE;

    /// Prior over existence of edge
    double prior;

  }; // struct EProp

  // Helpful alias declarations
  /// Undirected Boost graph
  typedef boost::adjacency_list<boost::vecS, boost::vecS, boost::undirectedS, VProp, EProp> Graph;

  /// Boost vertex
  typedef boost::graph_traits<Graph>::vertex_descriptor Vertex;

  /// Boost vertex iterator
  typedef boost::graph_traits<Graph>::vertex_iterator VertexIter;

  /// Boost edge
  typedef boost::graph_traits<Graph>::edge_descriptor Edge;

  /// Boost edge iterator
  typedef boost::graph_traits<Graph>::edge_iterator EdgeIter;

  /// Boost graph out edge iterator
  typedef boost::graph_traits<Graph>::out_edge_iterator OutEdgeIter;

  /// Boost graph in edge iterator
  typedef boost::graph_traits<Graph>::in_edge_iterator InEdgeIter;

  /// Boost graph neighbor iterator
  typedef boost::graph_traits<Graph>::adjacency_iterator NeighborIter;

  /// Map each vertex to a unique ID
  typedef boost::property_map<Graph, size_t VProp::*>::type VPIndexMap;

  /// Map each vertex to the underlying state [read from the graphml file]
  typedef boost::property_map<Graph, Eigen::VectorXd VProp::*>::type VPStateMap;

  /// Map each edge to a unique ID
  typedef boost::property_map<Graph, boost::edge_index_t EProp::*>::type EdgeIndexMap;

  /// Map each edge to its length
  typedef boost::property_map<Graph, double EProp::*>::type EPLengthMap;

  /// Map each edge to its existence prior
  typedef boost::property_map<Graph, double EProp::*>::type EPPriorMap; 

} // namespace BGL_DEFINITIONS

} // namespace HCBS

#endif