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
#include "../include/PCCBS/LoadGraphfromFile.hpp"

#define INF std::numeric_limits<double>::infinity()

namespace po = boost::program_options;

using namespace PCCBS;
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
					if(qUseful.find(successor)!=qUseful.end())
						qUseful.erase(successor);
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
			("num_agents,n", po::value<int>()->default_value(2), "Number of Agents")
			("num_tasks,m", po::value<int>()->default_value(5), "Number of Tasks")
			("collab_degree,c", po::value<int>()->default_value(2), "Number of Tasks")
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


	int numAgents = vm["num_agents"].as<int>();
	int numTasks = vm["num_tasks"].as<int>();

	int collabDegree = vm["collab_degree"].as<int>();

	std::cout<<"numAgents: "<<numAgents<<" numTasks: "<<numTasks<<" collabDegree: "<<collabDegree<<std::endl;

	// std::default_random_engine generator;
	// std::normal_distribution<double> agent_distribution(5.0,2.0);
	// std::normal_distribution<double> task_distribution(3.0,2.0);

	std::string CBS_planning_problems_file = planning_problems_file + "CBS/";
	std::string ICTS_planning_problems_file = planning_problems_file + "ICTS/";
	int obstacle_no=2;
	cv::Mat image = cv::imread(obstacle_file + std::to_string(obstacle_no) + ".png", 0);
	int graph_no = 0;
	Graph graph;
	create_vertices(graph,get(&VProp::state,graph),graph_file_name +  "graph" + std::to_string(graph_no) + ".graphml",2,get(&EProp::prior,graph));
	create_edges(graph,get(&EProp::length,graph));



	// std::normal_distribution<double> agent_distribution(3.5,1.0); // number of agents task will be distributed to
	// int min_assign = 2;
	// int max_assign = 4;
	int count = 0;
	srand(unsigned(time(0)));
	unsigned seed = 0;
	while (count < 100)
	{
		count++;

		while(true)
		{

			std::vector<Vertex> vertex_list;

			std::vector<std::pair<int,int>> agent_inits;

			std::vector<std::pair<int,int>> CBS_current_task(numAgents,std::make_pair(-1,0));
			std::vector<std::pair<std::pair<int,int>,std::pair<int,int>>> CBS_start_goal;
			std::vector<std::pair<int,int>> CBS_task_times;
			std::vector<std::vector<int>> CBS_agent_list;
			std::vector<std::pair<int,int>> CBS_task_edges;
			int CBS_task_id=0;

			std::vector<std::pair<int,int>> ICTS_current_task(numAgents,std::make_pair(-1,0));

			std::vector<std::pair<std::pair<int,int>,std::pair<int,int>>> ICTS_start_goal;
			std::vector<std::vector<int>> ICTS_agent_list;
			std::vector<std::pair<std::pair<int,int>,int>> ICTS_task_times;
			std::vector<std::pair<int,int>> ICTS_task_edges;
			int ICTS_task_id=0;

			std::vector<bool> ICTS_terminal(numAgents,true);

			int vertex_list_index=0;

			VertexIter vi, vi_end;
			for (boost::tie(vi, vi_end) = vertices(graph); vi != vi_end; ++vi)
				if(evaluateConfig(image,graph[*vi].state))
					vertex_list.push_back(*vi);

			std::cout<<"Vertex List Size: "<<vertex_list.size()<<" ";

			random_shuffle(vertex_list.begin(), vertex_list.end());

			for(int agent_id=0; agent_id<numAgents; agent_id++)
			{
				Vertex begin_vertex = vertex_list[vertex_list_index];
				vertex_list_index++;
				int init_x = (graph[begin_vertex].state[0]+0.0001)/0.1;
				int init_y = (graph[begin_vertex].state[1]+0.0001)/0.1;
				agent_inits.push_back(std::make_pair(init_x,init_y));
			}

			for(int task_id=0; task_id<numTasks; task_id++)
			{
				std::vector<int> agent_id_list;
				for(int agent_id=0; agent_id<numAgents; agent_id++)
					agent_id_list.push_back(agent_id);
				random_shuffle(agent_id_list.begin(), agent_id_list.end());

				std::vector<int> agents_assigned_list;
				for(int i=0; i<collabDegree; i++)
					agents_assigned_list.push_back(agent_id_list[i]);

				for(int i=0; i<agents_assigned_list.size(); i++)
				for(int j=0; j<numAgents; j++)
					if(ICTS_current_task[agents_assigned_list[i]].first!= -1 && ICTS_current_task[j].first!=-1)
						if(ICTS_current_task[agents_assigned_list[i]].first == ICTS_current_task[j].first) // same task as the one on which go is being assigned
							ICTS_terminal[j]=false;

				for(int i=0; i<agents_assigned_list.size(); i++)
					ICTS_terminal[agents_assigned_list[i]]=true;

				int start_x,start_y,goal_x,goal_y;
				std::vector<int> agents_assigned_go_costs;
				int agents_assigned_carry_cost;
				while(true)
				{

					Vertex start_vertex = vertex_list[vertex_list_index];
					vertex_list_index++;
					start_x = (graph[start_vertex].state[0]+0.0001)/0.1;
					start_y = (graph[start_vertex].state[1]+0.0001)/0.1;

					bool go_exist = true;

					agents_assigned_go_costs.clear();
					for(int i=0; i<agents_assigned_list.size(); i++)
					{
						Vertex begin_vertex;
						bool found=false;
						int carry_task_pre = ICTS_current_task[agents_assigned_list[i]].first;
						int init_x, init_y;
						if(carry_task_pre == -1)
						{
							init_x = agent_inits[agents_assigned_list[i]].first;
							init_y = agent_inits[agents_assigned_list[i]].second;
						}
						else
						{
							init_x = ICTS_start_goal[carry_task_pre].second.first;
							init_y = ICTS_start_goal[carry_task_pre].second.second;
						}
						for(int v=0; v<vertex_list.size(); v++)
							if( int((graph[vertex_list[v]].state[0]+0.0001)/0.1) == init_x
								&& int((graph[vertex_list[v]].state[1]+0.0001)/0.1) == init_y)
							{
								begin_vertex = vertex_list[v];
								found=true;
								break;
							}
						if(!found)
						{
							std::cout<<"wtf!";
							std::cin.get();
						}

						std::vector<Vertex> go_path = AStar(image, graph, begin_vertex, start_vertex);

						if(go_path.size()==0)
						{
							go_exist == false;
							break;
						}
						agents_assigned_go_costs.push_back(go_path.size()-1);
					}

					if(!go_exist)
						continue;

					Vertex goal_vertex = vertex_list[vertex_list_index];
					vertex_list_index++;
					goal_x = (graph[goal_vertex].state[0]+0.0001)/0.1;
					goal_y = (graph[goal_vertex].state[1]+0.0001)/0.1;

					std::vector<Vertex> carry_path = AStar(image, graph, start_vertex, goal_vertex);

					if(carry_path.size()==0)
						continue;

					agents_assigned_carry_cost = carry_path.size()-1;

					break;
				}

				{
					int max_goal_time = 0;
					for(int i=0; i<agents_assigned_list.size(); i++)
					{
						int carry_task_pre = ICTS_current_task[agents_assigned_list[i]].first;
						if(carry_task_pre == -1)
							max_goal_time = std::max(max_goal_time,	agents_assigned_go_costs[i]);
						else
							max_goal_time = std::max(max_goal_time,ICTS_current_task[agents_assigned_list[i]].second + 
								agents_assigned_go_costs[i]);
					}
					for(int i=0; i<agents_assigned_list.size(); i++)
					{
						int carry_task_pre = ICTS_current_task[agents_assigned_list[i]].first;
						int init_x,init_y;
						int start_time;
						if(carry_task_pre == -1)
						{
							init_x = agent_inits[agents_assigned_list[i]].first;
							init_y = agent_inits[agents_assigned_list[i]].second;
							start_time = 0;
						}
						else
						{
							init_x = ICTS_start_goal[carry_task_pre].second.first;
							init_y = ICTS_start_goal[carry_task_pre].second.second;
							start_time = ICTS_current_task[agents_assigned_list[i]].second;
						}

						ICTS_agent_list.push_back(std::vector<int>{agents_assigned_list[i]});
						
						ICTS_start_goal.push_back(std::make_pair(std::make_pair(init_x,init_y),
							std::make_pair(start_x,start_y)));
						 
						int goal_time = start_time + agents_assigned_go_costs[i];
						int slack_time = max_goal_time - goal_time;

						ICTS_task_times.push_back(std::make_pair(std::make_pair(start_time,goal_time),slack_time));
						if(carry_task_pre!=-1)
							ICTS_task_edges.push_back(std::make_pair(carry_task_pre,ICTS_task_id));
						ICTS_current_task[agents_assigned_list[i]] = std::make_pair(ICTS_task_id,max_goal_time);
						ICTS_task_id++;
					}

					ICTS_agent_list.push_back(agents_assigned_list);
					ICTS_start_goal.push_back(std::make_pair(std::make_pair(start_x,start_y),
						std::make_pair(goal_x,goal_y)));
					int start_time = max_goal_time;
					int goal_time = start_time + agents_assigned_carry_cost;
					int slack_time = 0;
					ICTS_task_times.push_back(std::make_pair(std::make_pair(start_time,goal_time),slack_time));
					for(int i=0; i<agents_assigned_list.size(); i++)
						ICTS_task_edges.push_back(std::make_pair(ICTS_current_task[agents_assigned_list[i]].first,ICTS_task_id));
					for(int i=0; i<agents_assigned_list.size(); i++)
						ICTS_current_task[agents_assigned_list[i]] = std::make_pair(ICTS_task_id,goal_time);
					ICTS_task_id++;

				}
				{
					CBS_agent_list.push_back(agents_assigned_list);

					CBS_start_goal.push_back(std::make_pair(std::make_pair(start_x,start_y),
						std::make_pair(goal_x,goal_y)));

					int start_time = 0;
					for(int i=0; i<agents_assigned_list.size(); i++)
					{
						int carry_task_pre = CBS_current_task[agents_assigned_list[i]].first;
						int go_end_time;
						if(carry_task_pre == -1)
							go_end_time = agents_assigned_go_costs[i];
						else
							go_end_time = CBS_current_task[agents_assigned_list[i]].second +
								agents_assigned_go_costs[i];
						start_time = std::max(start_time, go_end_time);
					}

					int goal_time = start_time + agents_assigned_carry_cost;

					CBS_task_times.push_back(std::make_pair(start_time,goal_time));
					for(int i=0; i<agents_assigned_list.size(); i++)
						if(CBS_current_task[agents_assigned_list[i]].first!=-1)
							CBS_task_edges.push_back(std::make_pair(CBS_current_task[agents_assigned_list[i]].first,CBS_task_id));
					for(int i=0; i<agents_assigned_list.size(); i++)
						CBS_current_task[agents_assigned_list[i]] = std::make_pair(CBS_task_id,goal_time);
					CBS_task_id++;
				}
			}

			bool validProblem = true;

			for(int agent_id=0; agent_id<numAgents; agent_id++)
				if(CBS_current_task[agent_id].first == -1)
				{
					validProblem = false;
					break;
				}

			if(validProblem)
			{
				{
					int max_total_time = 0;
					for(int agent_id=0; agent_id<numAgents; agent_id++)
						if(ICTS_terminal[agent_id])
							max_total_time = std::max(max_total_time,ICTS_current_task[agent_id].second);
					// std::cout<<"terminal agent ids: ";
					for(int agent_id=0; agent_id<numAgents; agent_id++)
					{
						if(ICTS_terminal[agent_id])
						{
							// std::cout<<agent_id<<" ";
							int task_id = ICTS_current_task[agent_id].first;
							ICTS_task_times[task_id].second = max_total_time - ICTS_current_task[agent_id].second;
						}
					}
					// std::cout<<std::endl;
				}

				{
					std::ofstream file_stream;
					file_stream.open(CBS_planning_problems_file + std::to_string(count) + ".txt");

					file_stream << std::to_string(CBS_task_id) + " ";
					file_stream << std::to_string(CBS_task_edges.size()) + "\n\n";
					file_stream << std::to_string(0.1) + "\n\n";
					file_stream << std::to_string(1) + " ";
					file_stream << std::to_string(numAgents) + "\n\n";
					for(int i=0; i<agent_inits.size(); i++)
						file_stream<<std::to_string(agent_inits[i].first) + " " + 
							std::to_string(agent_inits[i].second) + "\n";
					file_stream <<"\n";
					for(int i=0; i<CBS_task_edges.size(); i++)
						file_stream<<std::to_string(CBS_task_edges[i].first) + " " + 
							std::to_string(CBS_task_edges[i].second) + "\n";
					file_stream <<"\n";
					for(int i=0; i<CBS_start_goal.size(); i++)
					{
						file_stream << CBS_start_goal[i].first.first <<" ";
						file_stream << CBS_start_goal[i].first.second <<" ";
						file_stream << CBS_start_goal[i].second.first <<" ";
						file_stream << CBS_start_goal[i].second.second <<"\n";

						file_stream << CBS_agent_list[i].size() <<" ";
						for(int j=0; j<CBS_agent_list[i].size(); j++)
							file_stream<< CBS_agent_list[i][j]<<" ";
						file_stream <<"\n";
						file_stream << CBS_task_times[i].first <<" "<<CBS_task_times[i].second <<"\n";
						file_stream << i <<"\n\n";
					}
					file_stream.close();
				}
				{
					std::ofstream file_stream;
					file_stream.open(ICTS_planning_problems_file + std::to_string(count) + ".txt");
			
					file_stream << std::to_string(ICTS_task_id) + " ";
					file_stream << std::to_string(ICTS_task_edges.size()) + "\n\n";
					file_stream << std::to_string(0.1) + "\n\n";
					file_stream << std::to_string(1) + " ";
					file_stream << std::to_string(numAgents) + "\n\n";
					// for(int i=0; i<agent_inits.size(); i++)
					// 	file_stream<<std::to_string(agent_inits[i].first) + " " + 
					// 		std::to_string(agent_inits[i].second) + "\n";
					// file_stream <<"\n";
					for(int i=0; i<ICTS_task_edges.size(); i++)
						file_stream<<std::to_string(ICTS_task_edges[i].first) + " " + 
							std::to_string(ICTS_task_edges[i].second) + "\n";
					file_stream <<"\n";
					for(int i=0; i<ICTS_start_goal.size(); i++)
					{
						file_stream << ICTS_start_goal[i].first.first <<" ";
						file_stream << ICTS_start_goal[i].first.second <<" ";
						file_stream << ICTS_start_goal[i].second.first <<" ";
						file_stream << ICTS_start_goal[i].second.second <<"\n";
						file_stream << ICTS_agent_list[i].size() <<" ";
						for(int j=0; j<ICTS_agent_list[i].size(); j++)
							file_stream<< ICTS_agent_list[i][j]<<" ";
						file_stream <<"\n";
						file_stream << ICTS_task_times[i].first.first<<" "<<ICTS_task_times[i].first.second <<" "<<ICTS_task_times[i].second<<"\n\n";
					}
					file_stream.close();
				}

				std::cout<<"Vertex List Index: "<<vertex_list_index<<std::endl;
				break;
			}
			else
				std::cout<<"invalid :( ";
		}

	}
	return 0;
}