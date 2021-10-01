#ifndef TPQ_
#define TPQ_

#include<bits/stdc++.h>
#include <boost/functional/hash.hpp>

#define make_search_state(a,b,c,d) std::make_pair(std::make_pair(a,b),std::make_pair(c,d))
#define search_state std::pair< std::pair<Vertex,int>, std::pair<int,bool> >
#define search_state_vertex(t) t.first.first
#define search_state_timestep(t) t.first.second
#define search_state_tasks_completed(t) t.second.first
#define search_state_in_delivery(t) t.second.second

using namespace std;

namespace PCCBS {

using namespace BGL_DEFINITIONS;

struct SearchState
{
	Vertex vertex;
	int timestep;
	int tasks_completed;

	SearchState()
		: vertex(0), timestep(-1), tasks_completed(-1){}

	SearchState(Vertex _vertex, int _timestep, int _tasks_completed)
		: vertex(_vertex), timestep(_timestep), tasks_completed(_tasks_completed) {}

	bool operator==(const SearchState &other) const
	{ 
		return (vertex == other.vertex && timestep == other.timestep
			&& tasks_completed == other.tasks_completed);
	}

};

bool compareWaypoints(std::pair<SearchState, SearchState> &i1, std::pair<SearchState,SearchState> &i2)
{
    return (i1.first.timestep < i2.first.timestep);
}

struct state_hash
{
	std::size_t operator()(const SearchState& k) const
	{
		return size_t(k.tasks_completed)+size_t(k.vertex)*16+size_t(k.timestep)*16*1024;
	}
};

size_t get_hash_state(SearchState k){
	return size_t(k.tasks_completed)+size_t(k.vertex)*16+size_t(k.timestep)*16*1024;
}

struct agent_state_pair_hash
{
  std::size_t operator()(const std::pair<int, std::pair <SearchState,SearchState>>& k1) const
  {
      using boost::hash_value;
      using boost::hash_combine;

      // Start with a hash value of 0    .
      std::size_t seed = 0;

      // Modify 'seed' by XORing and bit-shifting in
      // one member of 'SearchState' after the other:
      hash_combine(seed, hash_value(get_hash_state(k1.second.first)));
      hash_combine(seed, hash_value(get_hash_state(k1.second.second)));
      hash_combine(seed, hash_value((k1.first)));
      return seed;
  }
};


class timePriorityQueue
{
private:
	struct element
	{
		std::vector<double> keys;
		SearchState s;
		element(SearchState _s, std::vector<double> _keys): s(_s), keys(_keys) {} 
		inline bool operator < (const element &b) const 
		{
			if(keys.size() != b.keys.size())
			{
				std::cerr<<"ERROR: Key sizes are not equal!";
				std::cin.get();
			}
			for(int i=0; i<keys.size(); i++){
				if((keys[i] - b.keys[i]) < -0.01)
					return true;
				else if((keys[i] - b.keys[i]) > 0.01)
					return false;
			}
			return s.vertex < b.s.vertex;
		}
	};
	vector <element> PQ;

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
	timePriorityQueue()
	{ 
		SearchState s = SearchState();
		element a(s,std::vector<double>());
		PQ.push_back(a);
	}
	void reset()
	{
		PQ.clear();
		SearchState s = SearchState();
		element a(s,std::vector<double>());
		PQ.push_back(a);
	}
	int PQsize()
	{
		return PQ.size()-1;
	}
	std::vector<double> topKey()
	{
		return PQ[1].keys;
	}
	SearchState pop()
	{
		SearchState s=PQ[1].s;
		PQ[1]=PQ[PQ.size()-1];
		PQ.erase(PQ.end()-1);
		min_heapify(1);
		// std::cout << PQ[1].keys[0] << std::endl;
		return s;
	}
	void insert(SearchState s, std::vector<double> k)
	{
		// std::cout<<"Inserting : "<<v<<std::endl;
		element a(s,k);
		PQ.push_back(a);
		// printPQ();
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
	void remove(SearchState &s)
	{
		int i;
		for(i=1;i<PQ.size();i++)
			if(PQ[i].s == s)
				break;
		swap(PQ[i],PQ[PQ.size()-1]);
		PQ.erase(PQ.end()-1);
		// printPQ();
		min_heapify(i);
		// printPQ();
	}
	bool contains(SearchState &s)
	{
		for(int i=1;i<PQ.size();i++)
			if(PQ[i].s == s)
				break;
		return false;
	}
	void printPQ()
	{
		cout<<"Elements: "<<endl;
		for(int i=1;i<PQ.size();i++)
		{
			cout<<"( Vertex: "<<PQ[i].s.vertex<<", Timestep: "<<PQ[i].s.timestep
				<<", Tasks Completed: "<<PQ[i].s.tasks_completed
				<<", Keys: ";

			for(int j=0; j<PQ[i].keys.size(); j++)
				cout<<PQ[i].keys[j]<<" ";
			cout<<"\n";
		}
	}
};
} // namespace PCCBS

#endif 