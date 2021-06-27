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

Vertex findVertex(Graph graph, std::vector<Vertex> vertex_list, int x, int y){
	for(int v=0; v<vertex_list.size(); v++){
		if( int((graph[vertex_list[v]].state[0]+0.0001)/0.1) == x
			&& int((graph[vertex_list[v]].state[1]+0.0001)/0.1) == y)
		{
			return vertex_list[v];
		}
	}
}

std::pair <int, int> findCoordinate(Graph graph, Vertex v){
	int x = (graph[v].state[0]+0.0001)/0.1;
	int y = (graph[v].state[1]+0.0001)/0.1;
	return std::make_pair(x, y);
}

int main(int argc, char *argv[])
{
	srand(time(NULL));
	po::options_description desc("Planning Problems Generation Options");
	desc.add_options()
			("help,h", "produce help message")
			("graph,g", po::value<std::string>()->default_value("/home/kushal/ros_ws/src/CMAPF/data/test_graphs/"), 
				"Path to graph files")
			("obstacleFile,o", po::value<std::string>()->default_value("/home/kushal/ros_ws/src/CMAPF/data/obstacles/easy"), 
				"Path to obstacles files")
			("planningProblemsFile,p",po::value<std::string>()->default_value("/home/kushal/ros_ws/src/CMAPF/data/greedy/"),
			 "Path to planning problems file")
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
	int obstacle_no=1;
	cv::Mat image = cv::imread(obstacle_file + ".png", 0);
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

		std::vector<Vertex> vertex_list;
		VertexIter vi, vi_end;
			for (boost::tie(vi, vi_end) = vertices(graph); vi != vi_end; ++vi)
				if(evaluateConfig(image,graph[*vi].state))
					vertex_list.push_back(*vi);

		random_shuffle(vertex_list.begin(), vertex_list.end());


		int cur_vertex = 0;

		// set starting points for each agent
		random_shuffle(vertex_list.begin(), vertex_list.end());
		std::vector<Vertex> agent_inits;
		
		for(int agent_id=0; agent_id<numAgents; agent_id++)
		{
			Vertex begin_vertex = vertex_list[cur_vertex];
			agent_inits.push_back(begin_vertex);
			cur_vertex++;
		}

		// set start and goal points for each task
		std::vector<Vertex> taskStarts;
		std::vector<Vertex> taskGoals;

		for(int task_no = 0; task_no <numTasks; task_no++)
		{
			Vertex start_vertex = vertex_list[cur_vertex];
			Vertex goal_vertex = vertex_list[cur_vertex+1];
			taskStarts.push_back(start_vertex);
			taskGoals.push_back(goal_vertex);
			cur_vertex+=2;
		}

		std::map <int, std::vector <int>> tasksToAgents;
		std::map <int, std::pair <int,int>> taskTimes;
		std::map <int, std::vector <int>> taskToAgentsLastTask;

		// set start time as 0 and starting position as initial posiition
		std::map <int, std::vector <Vertex>> agentPositions;
		std::map <int, std::vector <int>> agentLastTimes;
		std::map <int, int> agentLastTask;


		for (int i=0; i<numAgents; i++){
			agentPositions[i] = std::vector <Vertex> {agent_inits[i]};
			agentLastTimes[i] = std::vector <int> {0};
			agentLastTask[i] = -1;
		}

		std::vector <std::pair <int,int>> CBS_task_edges;
		std::vector <std::pair <int,int>> ICTS_task_edges;
		std::vector <std::pair <Vertex,Vertex>> ICTS_tasks;
		std::vector <std::vector<int>> ICTS_task_agents;
		std::map <int, int> ICTSLastTask;
		// greedily assign agents to tasks
		while(true){

			//find not assigned tasks
			std::vector <int> not_assigned_tasks;
			for (int i=0; i<numTasks; i++){
				if(tasksToAgents.find(i) == tasksToAgents.end()){
					not_assigned_tasks.push_back(i);
				}
			}

			//exit if all tasks assigned
			if(not_assigned_tasks.size() == 0) break;

			//find times to start node of each task
			std::map <int, std::vector <std::pair <int,int>>> taskTimeToStart;
			for (auto task: not_assigned_tasks){

				taskTimeToStart[task] = std::vector <std::pair <int,int>> {};
				Vertex goVertex = taskStarts[task];

				for (int agent=0; agent<numAgents; agent++){

					Vertex curVertex = agentPositions[agent].back();
					int curTime = agentLastTimes[agent].back();

					std::vector<Vertex> go_path = AStar(image, graph, curVertex, goVertex);
					
					taskTimeToStart[task].push_back(std::make_pair(curTime + go_path.size()-1, agent));
				}
			}

			//find task which can be completed earliest
			int best_time = 10000000;
			int best_task = -1;
			std::vector <int> best_agents;
			int best_start;

			for(auto task: not_assigned_tasks){

				//find time to complete Task
				Vertex startVertex = taskStarts[task];
				Vertex goalVertex = taskGoals[task];
				std::vector<Vertex> task_path = AStar(image, graph, startVertex, goalVertex);
				int task_time = task_path.size()-1;

				//find the closest Agents
				std::sort(taskTimeToStart[task].begin(), taskTimeToStart[task].end());
				std::cout << taskTimeToStart[task][collabDegree-1].first << std::endl;
				// std::cin.get();
				int end_time = taskTimeToStart[task][collabDegree-1].first + task_time;

				if(end_time < best_time){
					best_time = end_time;
					best_task = task;
					best_start = taskTimeToStart[task][collabDegree-1].first;
					best_agents.clear();

					for(int i=0; i<collabDegree; i++){
						int agent = taskTimeToStart[task][i].second;
						best_agents.push_back(agent);
					}
				}
			}

			//assign task to agents
			tasksToAgents[best_task] = best_agents;
			
			ICTS_tasks.push_back(std::make_pair(taskStarts[best_task], taskGoals[best_task]));
			ICTS_task_agents.push_back(best_agents);
			int cur_task = ICTS_tasks.size()-1;
			for(auto agent: best_agents){
				if(ICTSLastTask.find(agent) != ICTSLastTask.end()){
					ICTS_tasks.push_back(std::make_pair(taskGoals[agentLastTask[agent]], taskStarts[best_task]));
					ICTS_task_agents.push_back(std::vector <int> {agent});
					ICTS_task_edges.push_back(std::make_pair(ICTS_tasks.size()-1, cur_task));
					ICTS_task_edges.push_back(std::make_pair(ICTSLastTask[agent], ICTS_tasks.size()-1));
				}
				else{
					ICTS_tasks.push_back(std::make_pair(agent_inits[agent], taskStarts[best_task]));
					ICTS_task_agents.push_back(std::vector <int> {agent});
					ICTS_task_edges.push_back(std::make_pair(ICTS_tasks.size()-1, cur_task));
				}

				ICTSLastTask[agent] = cur_task;
			}

			for(auto agent: best_agents){				
				if(agentLastTask[agent] != -1){
					CBS_task_edges.push_back(std::make_pair(agentLastTask[agent], best_task));
				}
				agentLastTask[agent] = best_task;
				agentPositions[agent].push_back(taskGoals[best_task]);
				agentLastTimes[agent].push_back(best_time);
			}
			taskTimes[best_task] = std::make_pair(best_start, best_time);
		}

		std::ofstream file_stream;
		std::cout << ICTS_planning_problems_file + std::to_string(count) + ".txt" << std::endl;
		file_stream.open(ICTS_planning_problems_file + std::to_string(count) + ".txt");

		file_stream << std::to_string(ICTS_tasks.size()) + " ";
		file_stream << std::to_string(ICTS_task_edges.size()) + "\n\n";
		file_stream << std::to_string(0.1) + "\n\n";
		file_stream << std::to_string(1) + " ";
		file_stream << std::to_string(numAgents) + "\n\n";

		file_stream <<"\n";

		for(int i=0; i<ICTS_task_edges.size(); i++){
			file_stream<<std::to_string(ICTS_task_edges[i].first) + " " + 
				std::to_string(ICTS_task_edges[i].second) + "\n";
		}

		file_stream <<"\n";
		for(int i=0; i<ICTS_tasks.size(); i++)
		{
			std::pair <int, int> init = findCoordinate(graph, ICTS_tasks[i].first);
			file_stream<<std::to_string(init.first) + " " + 
				std::to_string(init.second) + " ";

			init = findCoordinate(graph, ICTS_tasks[i].second);
			file_stream<<std::to_string(init.first) + " " + 
				std::to_string(init.second) + "\n";

			file_stream << ICTS_task_agents[i].size() <<" ";
			for(int j=0; j<ICTS_task_agents[i].size(); j++)
				file_stream<< ICTS_task_agents[i][j]<<" ";
			file_stream <<"\n";
			file_stream <<"0 0 0\n\n";
		}
		file_stream.close();

		std::vector<std::pair <int,int>>::iterator ip = std::unique(CBS_task_edges.begin(), CBS_task_edges.end());
	    CBS_task_edges.resize(std::distance(CBS_task_edges.begin(), ip));

		// std::ofstream file_stream;
		std::cout << CBS_planning_problems_file + std::to_string(count) + ".txt" << std::endl;
		file_stream.open(CBS_planning_problems_file + std::to_string(count) + ".txt");

		file_stream << std::to_string(numTasks) + " ";
		file_stream << std::to_string(CBS_task_edges.size()) + "\n\n";
		file_stream << std::to_string(0.1) + "\n\n";
		file_stream << std::to_string(1) + " ";
		file_stream << std::to_string(numAgents) + "\n\n";

		for(int i=0; i<agent_inits.size(); i++){
			std::pair <int, int> init = findCoordinate(graph, agent_inits[i]);
			file_stream<<std::to_string(init.first) + " " + 
				std::to_string(init.second) + "\n";
		}
		file_stream <<"\n";
		for(int i=0; i<CBS_task_edges.size(); i++){
			file_stream<<std::to_string(CBS_task_edges[i].first) + " " + 
				std::to_string(CBS_task_edges[i].second) + "\n";
		}
		file_stream <<"\n";
		for(int task=0; task<numTasks; task++)
		{
			std::pair <int, int> init = findCoordinate(graph, taskStarts[task]);
			file_stream<<std::to_string(init.first) + " " + 
				std::to_string(init.second) + " ";

			init = findCoordinate(graph, taskGoals[task]);
			file_stream<<std::to_string(init.first) + " " + 
				std::to_string(init.second) + "\n";

			file_stream << tasksToAgents[task].size() <<" ";
			for(int j=0; j<tasksToAgents[task].size(); j++)
				file_stream<< tasksToAgents[task][j]<<" ";
			file_stream <<"\n";
			file_stream << taskTimes[task].first <<" "<<taskTimes[task].second <<"\n";
			file_stream << task <<"\n\n";
		}
		file_stream.close();
	}
	return 0;
}