#ifndef CBS_DEFINITIONS_
#define CBS_DEFINITIONS_
#include <bits/stdc++.h>
#include "BGLDefinitions.hpp"
#include "LoadGraphfromFile.hpp"
#include "time_priority_queue.hpp"

namespace PCCBS {

using namespace BGL_DEFINITIONS;

using namespace std;

struct pair_hash {
    template <class T1, class T2>
    std::size_t operator () (const std::pair<T1,T2> &p) const {
        auto h1 = std::hash<T1>{}(p.first);
        auto h2 = std::hash<T2>{}(p.second);
        return h1 ^ h2;  
    }
}; 

struct CollisionConstraint
{
	int constraint_type;  // 1 -> vertex collision_constraint 2 -> edge collision_constraint
	Vertex v;
	Vertex v1;
	Vertex v2;
	Edge e;	//edge
	int tasks_completed;
	bool in_delivery;
	int timestep; //time . for edge, time is time of target vertex

	CollisionConstraint() : constraint_type(1) {}
	CollisionConstraint(Vertex _v, int _tasks_completed, bool _in_delivery, int _t) 
		: v(_v), tasks_completed(_tasks_completed), in_delivery(_in_delivery), timestep(_t), constraint_type(1) {}
	CollisionConstraint(Edge _e, int _tasks_completed, bool _in_delivery, int _t) 
		: e(_e), tasks_completed(_tasks_completed), in_delivery(_in_delivery), timestep(_t), constraint_type(2) {}

	bool operator==(const CollisionConstraint &other) const
	{ 
		if(constraint_type!=other.constraint_type) return false;
		if(other.constraint_type==2) 
			return (timestep == other.timestep
			&& tasks_completed == other.tasks_completed && in_delivery == other.in_delivery && e==other.e);
		return (v == other.v && timestep == other.timestep
			&& tasks_completed == other.tasks_completed && in_delivery == other.in_delivery);
	}
};

struct collision_hash
{
  std::size_t operator()(const CollisionConstraint &k1) const
  {
	using namespace boost;
	using boost::hash_combine;
	size_t seed = 42;
	if(k1.constraint_type==2){
		hash_combine(seed, k1.v1);
		hash_combine(seed, k1.v2);
	}
	else{
		hash_combine(seed, k1.v);
		hash_combine(seed, k1.v);
	}
	hash_combine(seed, k1.tasks_completed);
	hash_combine(seed, k1.in_delivery);
	hash_combine(seed, k1.timestep);
	return seed;
  }
};

struct CollaborationConstraint
{
	Vertex v;
	int task_id;
	bool is_pickup; // false implies delivery
	int timestep;

	CollaborationConstraint() {}
	CollaborationConstraint(Vertex _v, int _task_id, bool _is_pickup, int _t) 
		: v(_v), task_id(_task_id), is_pickup(_is_pickup), timestep(_t){}
	bool operator==(const CollaborationConstraint &other) const
	{ 
		return (v == other.v && timestep == other.timestep
			&& task_id == other.task_id && is_pickup == other.is_pickup);
	}
};

struct allowedInterval{
	int minTime, maxTime;
	allowedInterval(int _minTime, int _maxTime):
		minTime(_minTime), maxTime(_maxTime){}
	allowedInterval():
		minTime(0), maxTime(1000000){}
};

struct Element
{
	boost::unordered_map <SearchState, allowedInterval, state_hash> nonCollabMap;
	std::vector <boost::unordered_map <CollisionConstraint, int, collision_hash>> nonCollisionMap;
	std::vector<std::vector<SearchState>> shortestPaths;

	Element(boost::unordered_map <SearchState, allowedInterval, state_hash> _nonCollabMap,
		std::vector <boost::unordered_map <CollisionConstraint, int, collision_hash>> _nonCollisionMap, 
		std::vector<std::vector<SearchState>> _shortestPaths): 
		nonCollabMap(_nonCollabMap), nonCollisionMap(_nonCollisionMap), shortestPaths(_shortestPaths) 
		{}

	inline bool operator < (const Element &b) const 
	{
		int cost = 0;
		for(int i=0; i<shortestPaths.size(); i++)
			cost = std::max(cost,shortestPaths[i].at(shortestPaths[i].size()-1).timestep);
		
		int b_cost = 0;
		for(int i=0; i<b.shortestPaths.size(); i++)
			b_cost = std::max(b_cost,b.shortestPaths[i].at(b.shortestPaths[i].size()-1).timestep);

		if(!cost || !b_cost) {
			std::cout << "UNIMPOSSIBLE\n";
			std::cin.get();
		}
    	if(cost < b_cost)
			return true;
		else
			return false;
	}
};

class CBSPriorityQueue
{
private:
	vector <Element> PQ;
	int mNumAgents;

	void min_heapify(int x)
	{
		int l=2*x;
		int r=2*x+1;
		int smallest = x;
		if(l<=(PQ.size()-1) && PQ[l]<PQ[x])
			smallest = l;
		if(r<=(PQ.size()-1) && PQ[r]<PQ[smallest])
			smallest = r;
		if(smallest!=x)
		{
			swap(PQ[smallest],PQ[x]);
			min_heapify(smallest);
		}
	}

public:
	CBSPriorityQueue(int numAgents)
	{ 
		mNumAgents = numAgents;
		reset();
	}
	void reset()
	{
		PQ.clear();
		boost::unordered_map <SearchState, allowedInterval, state_hash> v1;
		std::vector <boost::unordered_map <CollisionConstraint, int, collision_hash>> v2(mNumAgents);
		std::vector<std::vector<SearchState>> v3(mNumAgents,std::vector<SearchState>());
		Element a(v1, v2, v3);
		PQ.push_back(a);
	}
	int PQsize()
	{
		return PQ.size()-1;
	}
	int topKey()
	{
		int cost = 0;
		for(int i=0; i<PQ[1].shortestPaths.size(); i++)
			cost = std::max(cost,PQ[1].shortestPaths[i].at(PQ[1].shortestPaths[i].size()-1).timestep);
		return cost;
	}
	Element pop()
	{
		Element temp=PQ[1];
		PQ[1]=PQ[PQ.size()-1];
		PQ.erase(PQ.end()-1);
		min_heapify(1);
		return temp;
	}
	void insert(Element a)
	{
		PQ.push_back(a);
		int i=PQ.size()-1;
		while((i/2)>0)
		{
			if(PQ[i/2]<PQ[i])
				break;
			else
			{
				swap(PQ[i/2],PQ[i]);
				i=i/2;
			}
		}

	}
};

} // namespace PCCBS

#endif