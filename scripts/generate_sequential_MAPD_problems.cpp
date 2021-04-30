// Standard C++ libraries
#include <iostream>
#include <string>
#include <fstream>
#include <sstream>
#include <queue>
#include <random>

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
#include "../include/CMAPF/LoadGraphfromFile.hpp"

#define INF std::numeric_limits<double>::infinity()

namespace po = boost::program_options;

using namespace CMAPF;
// using namespace BGL_DEFINITIONS;
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

	std::string graph_file_name(vm["graph"].as<std::string>());
	if (graph_file_name == "")
		graph_file_name = "/home/rajat/melodic_ws/src/CMAPF/data/graphs/epsilon_graphs_05_200/";
	
	std::string obstacle_file(vm["obstacleFile"].as<std::string>());
	if (obstacle_file == "")
		obstacle_file = "/home/rajat/melodic_ws/src/CMAPF/data/obstacles/2D_Images/";

	std::string planning_problems_file(vm["planningProblemsFile"].as<std::string>());
	if (planning_problems_file == "")
		planning_problems_file = "/home/rajat/melodic_ws/src/CMAPF/data/planning_problems/";

	std::default_random_engine generator;
	std::normal_distribution<double> agent_distribution(5.0,2.0);
	std::normal_distribution<double> task_distribution(3.0,2.0);

	std::string CBS_planning_problems_file = planning_problems_file + "CBS/";
	std::string ICTS_planning_problems_file = planning_problems_file + "ICTS/";
	int id=0;
	// for(int obstacle_no=0; obstacle_no<10; obstacle_no++)
	int obstacle_no=0;
	{
		cv::Mat image = cv::imread(obstacle_file + std::to_string(obstacle_no) + ".png", 0);

		// for(int graph_no = 0; graph_no < 10; graph_no++)
		int graph_no = 0;
		{
			Graph graph;
			create_vertices(graph,get(&VProp::state,graph),graph_file_name +  "graph" + std::to_string(graph_no) + ".graphml",2,get(&EProp::prior,graph));
			create_edges(graph,get(&EProp::length,graph));

			int count = 0;
			while (count < 10)
			{
				count++;
				bool validProblem = true;
				int numAgents;
				do
				{
					numAgents = agent_distribution(generator);
				} while (numAgents <= 0);

				std::vector<int> taken_vertices;
				std::vector<std::pair<int,int>> agent_inits;
				std::vector<std::pair<int,std::pair<std::pair<int,int>,std::pair<int,int>>>> tasks;
				std::vector<std::pair<int,int>> task_times;
				std::vector<int> task_slack;
				std::vector<int> consider_slack;
				std::vector<std::pair<int,int>> task_edges;
				int global_task_id=0;
				int num_edges=0;

				std::vector<std::pair<int,std::pair<std::pair<int,int>,std::pair<int,int>>>> cbs_tasks;
				std::vector<std::pair<int,int>> cbs_task_edges;
				int cbs_global_task_id=0;
				int cbs_num_edges=0;

				for(int agent_id=0; agent_id<numAgents; agent_id++)
				{
					int numTasks;
					do
					{
						numTasks = task_distribution(generator);
					} while (numTasks <= 0);
					num_edges += 2*numTasks-2;
					cbs_num_edges += numTasks-1;
					int begin_vertex_index;
					Vertex begin_vertex;
					while(true)
					{
						begin_vertex_index = rand()%int(boost::num_vertices(graph));
						if(std::find(taken_vertices.begin(), taken_vertices.end(), begin_vertex_index) != taken_vertices.end())
							continue;
						begin_vertex = vertex(begin_vertex_index, graph);
						if(!evaluateConfig(image,graph[begin_vertex].state))
								continue;
						break;
					}
					int init_x = (graph[begin_vertex].state[0]+0.0001)/0.04;
					int init_y = (graph[begin_vertex].state[1]+0.0001)/0.04;
					agent_inits.push_back(std::make_pair(init_x,init_y));
					int start_time = 0;
					for(int task_id=0; task_id<numTasks; task_id++)
					{
						bool noPath = true;
						int tries = 0;
						while (tries < 100)
						{		
							tries++;
							int start_vertex_index = rand()%int(boost::num_vertices(graph));
							if(std::find(taken_vertices.begin(), taken_vertices.end(), start_vertex_index) != taken_vertices.end())
								continue;
							
							Vertex start_vertex = vertex(start_vertex_index, graph);

							if(!evaluateConfig(image,graph[start_vertex].state))
								continue;

							std::vector<Vertex> go_path = AStar(image, graph, begin_vertex, start_vertex);

							if(go_path.size()==0)
								continue;

							int goal_vertex_index = rand()%int(boost::num_vertices(graph));
							if(std::find(taken_vertices.begin(), taken_vertices.end(), goal_vertex_index) != taken_vertices.end())
								continue;
							
							Vertex goal_vertex = vertex(goal_vertex_index, graph);

							if(!evaluateConfig(image,graph[goal_vertex].state))
								continue;

							std::vector<Vertex> carry_path = AStar(image, graph, start_vertex, goal_vertex);

							if(carry_path.size()==0)
								continue;

							// std::cout<<"Begin Vertex - "<<begin_vertex_index<<" "<<graph[begin_vertex].state[0]<<" "<<graph[begin_vertex].state[1]<<std::endl;
							// std::cout<<"Start Vertex - "<<start_vertex_index<<" "<<graph[start_vertex].state[0]<<" "<<graph[start_vertex].state[1]<<std::endl;
							// std::cout<<"Goal Vertex - "<<goal_vertex_index<<" "<<graph[goal_vertex].state[0]<<" "<<graph[goal_vertex].state[1]<<std::endl;

							// std::cout<<"Go Path: ";
							// for(int i=0;i<go_path.size(); i++)
							// 	std::cout<<go_path[i]<<" ";
							// std::cout<<std::endl;
							// std::cout<<"Carry Path: ";
							// for(int i=0;i<carry_path.size(); i++)
							// 	std::cout<<carry_path[i]<<" ";
							// std::cout<<std::endl<<std::endl;

							noPath = false;

							int begin_x = (graph[begin_vertex].state[0]+0.0001)/0.04;
							int begin_y = (graph[begin_vertex].state[1]+0.0001)/0.04;

							int start_x = (graph[start_vertex].state[0]+0.0001)/0.04;
							int start_y = (graph[start_vertex].state[1]+0.0001)/0.04;

							int goal_x = (graph[goal_vertex].state[0]+0.0001)/0.04;
							int goal_y = (graph[goal_vertex].state[1]+0.0001)/0.04;

							tasks.push_back(std::make_pair(agent_id,std::make_pair(std::make_pair(begin_x,begin_y),std::make_pair(start_x,start_y))));
							task_times.push_back(std::make_pair(start_time,start_time + go_path.size() - 1 ));
							task_slack.push_back(0);
							tasks.push_back(std::make_pair(agent_id,std::make_pair(std::make_pair(start_x,start_y),std::make_pair(goal_x,goal_y))));
							start_time += go_path.size() - 1;
							task_times.push_back(std::make_pair(start_time,start_time + carry_path.size() - 1 ));
							task_slack.push_back(0);
							start_time += carry_path.size() - 1;

							if(task_id == numTasks-1) // last task for agent
							{
								consider_slack.push_back(task_times.size()-1);
							}
							if(task_id!=0)
							{
								task_edges.push_back(std::make_pair(global_task_id-1,global_task_id));
								task_edges.push_back(std::make_pair(global_task_id,global_task_id+1));
							}
							else
								task_edges.push_back(std::make_pair(global_task_id,global_task_id+1));	
							global_task_id += 2;

							cbs_tasks.push_back(std::make_pair(agent_id,std::make_pair(std::make_pair(start_x,start_y),std::make_pair(goal_x,goal_y))));
							if(task_id!=0)
								cbs_task_edges.push_back(std::make_pair(cbs_global_task_id-1,cbs_global_task_id));
							cbs_global_task_id++;


							begin_vertex = goal_vertex;
							break;
						}

						if(noPath)
						{
							std::cout<<"No problem generated for: image - "<<obstacle_no<<" graph - "<<graph_no<<std::endl;
							validProblem = false;
							break;
						}
					}
					if(validProblem == false)
						break;
				}

				if(validProblem)
				{	
					{
						std::ofstream file_stream;
						file_stream.open (CBS_planning_problems_file + std::to_string(count) + ".txt");
				
						std::cout<<"valid problem!!";
						file_stream << std::to_string(cbs_global_task_id) + " ";
						file_stream << std::to_string(cbs_num_edges) + "\n\n";
						file_stream << std::to_string(0.04) + "\n\n";
						file_stream << std::to_string(1) + " ";
						file_stream << std::to_string(numAgents) + "\n\n";
						for(int i=0; i<agent_inits.size(); i++)
							file_stream<<std::to_string(agent_inits[i].first) + " " + 
								std::to_string(agent_inits[i].second) + "\n";
						file_stream <<"\n";
						for(int i=0; i<cbs_task_edges.size(); i++)
							file_stream<<std::to_string(cbs_task_edges[i].first) + " " + 
								std::to_string(cbs_task_edges[i].second) + "\n";
						file_stream <<"\n";
						for(int i=0; i<cbs_tasks.size(); i++)
						{
							file_stream << cbs_tasks[i].second.first.first <<" ";
							file_stream << cbs_tasks[i].second.first.second <<" ";
							file_stream << cbs_tasks[i].second.second.first <<" ";
							file_stream << cbs_tasks[i].second.second.second <<"\n";
							file_stream << 1 <<" "<< cbs_tasks[i].first<<"\n";
							file_stream << i <<"\n\n";
						}
						file_stream.close();
					}
					{
						std::ofstream file_stream;
						file_stream.open (ICTS_planning_problems_file + std::to_string(count) + ".txt");
				
						int first_cost = 0;
						for(int i=0; i<consider_slack.size(); i++)
							first_cost = std::max(first_cost,task_times[consider_slack[i]].second);
						for(int i=0; i<consider_slack.size(); i++)
							task_slack[consider_slack[i]] = first_cost - task_times[consider_slack[i]].second;

						std::cout<<"valid problem!!";
						file_stream << std::to_string(global_task_id) + " ";
						file_stream << std::to_string(num_edges) + "\n\n";
						file_stream << std::to_string(0.04) + "\n\n";
						file_stream << std::to_string(1) + " ";
						file_stream << std::to_string(numAgents) + "\n\n";
						// for(int i=0; i<agent_inits.size(); i++)
						// 	file_stream<<std::to_string(agent_inits[i].first) + " " + 
						// 		std::to_string(agent_inits[i].second) + "\n";
						// file_stream <<"\n";
						for(int i=0; i<task_edges.size(); i++)
							file_stream<<std::to_string(task_edges[i].first) + " " + 
								std::to_string(task_edges[i].second) + "\n";
						file_stream <<"\n";
						for(int i=0; i<tasks.size(); i++)
						{
							file_stream << tasks[i].second.first.first <<" ";
							file_stream << tasks[i].second.first.second <<" ";
							file_stream << tasks[i].second.second.first <<" ";
							file_stream << tasks[i].second.second.second <<"\n";
							file_stream << 1 <<" "<< tasks[i].first<<"\n";
							file_stream << task_times[i].first<<" "<<task_times[i].second <<" "<<task_slack[i]<<"\n\n";
						}
						file_stream.close();
					}

					// {
					// 	std::ofstream gggg;
					//     gggg.open(other_planning_problems_file + std::to_string(id) + "_"
					//     	 + std::to_string(graph_no) + "_task.xml");
					//     id++;
					//     gggg<<"<?xml version=\"1.0\" ?>\n<root>\n";
					//     for(int i = 0; i < numAgents; i++)
					//         gggg<<"   <agent start_id=\""<<start_vertices[i]<<
					//     		"\" goal_id=\""<<goal_vertices[i]<<"\"/>\n";
					//     gggg<<"</root>";
					//     gggg.close();
					// }
				}
			}
		}
	}
	return 0;
}