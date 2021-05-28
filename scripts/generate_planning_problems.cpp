// Standard C++ libraries
#include <iostream>
#include <string>
#include <fstream>
#include <sstream>
#include <queue>

// Boost libraries
// #include <boost/shared_ptr.hpp>
#include <boost/property_map/dynamic_property_map.hpp>
#include <boost/graph/adjacency_list.hpp>
#include <boost/graph/graphml.hpp>
#include <boost/function.hpp>
#include <boost/program_options.hpp>

// OpenCV libraries
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

// Custom header files
#include "../include/utils/graphFileUtils.hpp"

#define INF std::numeric_limits<double>::infinity()

namespace po = boost::program_options;
using namespace BGL_DEFINITIONS;
using namespace cv;

bool evaluateConfig(Mat &img, Eigen::VectorXd config)
{
  int numberOfRows = img.rows;
  int numberOfColumns = img.cols;

  // agent
  double x_point = config[0]*numberOfColumns;
  double y_point = (1 - config[1])*numberOfRows;
  cv::Point point((int)x_point, (int)y_point);

  // Collision Check for agent with environment
  int intensity = (int)img.at<uchar>(point.y, point.x);
  if (intensity == 0) // Pixel is black
    return false;
  return true;
}

bool evaluateEdge(Mat &img, Graph &graph, Edge &e)
{
	Vertex source_vertex = source(e, graph);
	Vertex target_vertex = target(e, graph);

	Eigen::VectorXd sourceState(2);
	sourceState << graph[source_vertex].state;

	Eigen::VectorXd targetState(2);
	targetState << graph[target_vertex].state;

	double resolution = 0.005;
	unsigned int nStates = std::ceil(graph[e].length / resolution-0.000000001)+1;

	// Just start and goal
	if(nStates < 2u)
	{
		nStates = 2u;
	}
	// std::cout<<"nStates:"<<nStates<<std::endl;

	bool checkResult = true;
	
	if (checkResult && !evaluateConfig(img, sourceState))
	{
		graph[source_vertex].status = CollisionStatus::BLOCKED;
		graph[e].status = CollisionStatus::BLOCKED;
		graph[e].length = INF;
		checkResult = false;
	}

	if (checkResult && !evaluateConfig(img, targetState))
	{
		graph[target_vertex].status = CollisionStatus::BLOCKED;
		graph[e].status = CollisionStatus::BLOCKED;
		graph[e].length = INF;
		checkResult = false;
	}

	if (checkResult)
	{
		// Evaluate the States in between
		for (unsigned int i = 1; i < nStates-1; i++)
		{

			if(!evaluateConfig(img, sourceState + (resolution*i/graph[e].length)*(targetState-sourceState) ))
			{
				graph[e].status = CollisionStatus::BLOCKED;
				graph[e].length = INF;
				checkResult = false;
				break;
			}
		}
	}

	return checkResult;
}

std::vector<Vertex> AStar(Mat &img, Graph &graph, Vertex &start_vertex, Vertex &goal_vertex)
{

	VertexIter vi, vi_end;
	for (boost::tie(vi, vi_end) = vertices(graph); vi != vi_end; ++vi)
	{
	  graph[*vi].distance = std::numeric_limits<double>::infinity();
	  graph[*vi].visited = false;
	  graph[*vi].status = CollisionStatus::FREE;
	  graph[*vi].heuristic = std::abs(graph[*vi].state[0] - graph[goal_vertex].state[0])
	  			+ std::abs(graph[*vi].state[1] - graph[goal_vertex].state[1]);
	}

	// Priority Function: f-value
	auto cmpFValue = [&](const Vertex& left, const Vertex& right)
	{
		double estimateLeft = graph[left].distance + graph[left].heuristic;
		double estimateRight = graph[right].distance + graph[right].heuristic;

		if (estimateRight - estimateLeft > 0)
			return true;
		if (estimateLeft - estimateRight > 0)
			return false;
		if (left < right)
			return true;
		else
			return false;
	};

	std::set<Vertex, decltype(cmpFValue)> qUseful(cmpFValue);

	bool solutionFound = false;

	graph[start_vertex].distance = 0;
	graph[start_vertex].parent = -1;
	graph[start_vertex].visited = true;
	qUseful.insert(start_vertex);

	size_t iteration=0;
	while(qUseful.size()!=0)
	{
		iteration++;
		Vertex vTop = *qUseful.begin();
		// std::cout<<"Vertex Index: "<<graph[vTop].vertex_index<<" Distance: "<<graph[vTop].distance<<
		// " heuristic: "<<graph[vTop].heuristic<<std::endl;
		qUseful.erase(qUseful.begin());
		if(vTop == goal_vertex)
		{
			solutionFound = true;
			break;      
		}

		NeighborIter ai, ai_end;
		for (boost::tie(ai, ai_end) = adjacent_vertices(vTop, graph); ai != ai_end; ++ai) 
		{
			// displayGraph(graph);
			Vertex successor = *ai; 
			Edge uv;
			bool edgeExists;
			boost::tie(uv, edgeExists) = edge(vTop,successor, graph);
			if (edgeExists == false)
			{
				std::cout<<"Edge does not exist. Press [ENTER] to get segmentation fault :( :"<<std::endl;
		  		std::cin.get();
		  	}
			if(evaluateEdge(img, graph, uv))
			{
				double edgeLength = graph[uv].length;
				double new_cost = graph[vTop].distance + edgeLength;
					// std::cout<<"Edge is Free!"<<std::endl; 
				if(new_cost < graph[successor].distance)
				{
					graph[successor].distance = new_cost;
					qUseful.insert(successor);
					graph[successor].parent= vTop;
				}	
			}	
		}
	}
	if (!solutionFound)
		return std::vector<Vertex>();

	std::vector<Vertex> path;
	
	Vertex node = goal_vertex;
	
	while(node!=start_vertex)
	{
		path.push_back(node);
		node=graph[node].parent;
	}

	path.push_back(start_vertex);
	std::reverse(path.begin(), path.end());
	return path;
}

int main(int argc, char *argv[])
{
	srand(time(NULL));
	po::options_description desc("Planning Problems Generation Options");
	desc.add_options()
			("help,h", "produce help message")
			("num_agents,k", po::value<int>()->default_value(2), "Number of Agents")
			("graph,g", po::value<std::string>()->default_value(""), "Path to graph files")
			("obstacleFile,o", po::value<std::string>()->default_value(""), "Path to obstacles files")
			("planningProblemsFile,p",po::value<std::string>()->default_value(""), "Path to planning problems file")
	;

	// Read arguments
	po::variables_map vm;
	po::store(po::parse_command_line(argc, argv, desc), vm);
	po::notify(vm);

	if (vm.count("help"))
	{
			std::cout << desc << std::endl;
			return 1;
	}

	int numAgents = vm["num_agents"].as<int>();

	std::string graph_file_name(vm["graph"].as<std::string>());
	if (graph_file_name == "")
		graph_file_name = "/home/rajat/melodic_ws/src/C-MINT/data/graphs/epsilon_graphs_05_200/";
	
	std::string obstacle_file(vm["obstacleFile"].as<std::string>());
	if (obstacle_file == "")
		obstacle_file = "/home/rajat/melodic_ws/src/C-MINT/data/obstacles/2D_Images/";

	std::string planning_problems_file(vm["planningProblemsFile"].as<std::string>());
	if (planning_problems_file == "")
		planning_problems_file = "/home/rajat/melodic_ws/src/C-MINT/data/planning_problems/";


	std::ofstream file_stream;
	file_stream.open (planning_problems_file + "CMINT_test_set.txt");


	std::string other_planning_problems_file = planning_problems_file + "CCBS_test_set/";
	int id=0;
	// for(int obstacle_no=0; obstacle_no<10; obstacle_no++)
	int obstacle_no=10;
	{
		cv::Mat image = cv::imread(obstacle_file + std::to_string(obstacle_no) + ".png", 0);

		for(int graph_no = 0; graph_no < 10; graph_no++)
		// int graph_no = 0;
		{
			Graph graph;
			create_vertices(graph,get(&VProp::state,graph),graph_file_name +  "graph" + std::to_string(graph_no) + ".graphml",2,get(&EProp::prior,graph));
			create_edges(graph,get(&EProp::length,graph));

			int count = 0;
			while (count < 10)
			{
				count++;
				bool validProblem = true;

				Eigen::VectorXd start_config(2*numAgents);
				Eigen::VectorXd goal_config(2*numAgents);

				std::vector<int> start_vertices;
				std::vector<int> goal_vertices;
				for(int agent_id=0; agent_id<numAgents; agent_id++)
				{
					bool noPath = true;
					int tries = 0;
					while (tries < 100)
					{		
						tries++;
						int start_vertex_index = rand()%int(boost::num_vertices(graph));
						if(std::find(start_vertices.begin(), start_vertices.end(), start_vertex_index) != start_vertices.end())
							continue;
						
						Vertex start_vertex = vertex(start_vertex_index, graph);

						if(!evaluateConfig(image,graph[start_vertex].state))
							continue;

						int goal_vertex_index = rand()%int(boost::num_vertices(graph));
						if(std::find(goal_vertices.begin(), goal_vertices.end(), goal_vertex_index) != goal_vertices.end())
							continue;
						
						Vertex goal_vertex = vertex(goal_vertex_index, graph);

						if(!evaluateConfig(image,graph[goal_vertex].state))
							continue;

						std::vector<Vertex> path = AStar(image, graph, start_vertex, goal_vertex);

						if(path.size()==0)
							continue;

						std::cout<<"Start Vertex - "<<start_vertex_index<<" "<<graph[start_vertex].state[0]<<" "<<graph[start_vertex].state[1]<<std::endl;
						std::cout<<"Goal Vertex - "<<goal_vertex_index<<" "<<graph[goal_vertex].state[0]<<" "<<graph[goal_vertex].state[1]<<std::endl;

						std::cout<<"Path: ";
						for(int i=0;i<path.size(); i++)
							std::cout<<path[i]<<" ";
						std::cout<<std::endl<<std::endl;

						noPath = false;

						start_vertices.push_back(start_vertex_index);
						goal_vertices.push_back(goal_vertex_index);

						start_config[2*agent_id] = graph[start_vertex].state[0];
						start_config[2*agent_id+1] = graph[start_vertex].state[1];

						goal_config[2*agent_id] = graph[goal_vertex].state[0];
						goal_config[2*agent_id+1] = graph[goal_vertex].state[1];

						break;
					}

					if(noPath)
					{
						std::cout<<"No problem generated for: image - "<<obstacle_no<<" graph - "<<graph_no<<std::endl;
						validProblem = false;
						break;
					}
				}

				if(validProblem)
				{	
					file_stream << std::to_string(numAgents) + " ";
					file_stream << std::to_string(obstacle_no) + " " + std::to_string(graph_no) + " ";
					for(int i=0; i< 2*numAgents; i++)
						file_stream << std::to_string(start_config[i]) + " ";
					for(int i=0; i< 2*numAgents; i++)
						file_stream << std::to_string(goal_config[i]) + " ";
					file_stream << "\n";

					{
						std::ofstream gggg;
					    gggg.open(other_planning_problems_file + std::to_string(id) + "_"
					    	 + std::to_string(graph_no) + "_task.xml");
					    id++;
					    gggg<<"<?xml version=\"1.0\" ?>\n<root>\n";
					    for(int i = 0; i < numAgents; i++)
					        gggg<<"   <agent start_id=\""<<start_vertices[i]<<
					    		"\" goal_id=\""<<goal_vertices[i]<<"\"/>\n";
					    gggg<<"</root>";
					    gggg.close();
					}
				}
			}
		}
	}
	
	file_stream.close();
	return 0;
}