#ifndef TPQ_
#define TPQ_


#include<bits/stdc++.h>

using namespace std;

namespace HCBS {

using namespace BGL_DEFINITIONS;

#define INF std::numeric_limits<double>::infinity()
#define EPS 0.000001 // used for double precision

class timePriorityQueue
{
private:
	struct element
	{
		std::vector<int> heuristics;
		int value;
		size_t timestep;
		element(std::vector <int> _heuristics, int _value, size_t _timestep): 
			heuristics(_heuristics), value(_value), timestep(_timestep) {} 
		inline bool operator < (const element &b) const 
		{
			return heuristics < b.heuristics;	
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
		element a(std::vector <int> {0,0,0,0,0}, 0, 0);
		PQ.push_back(a);
	}
	void reset()
	{
		PQ.clear();
		element a(std::vector <int> {0,0,0,0,0}, 0, 0);
		PQ.push_back(a);
	}
	int PQsize()
	{
		return PQ.size()-1;
	}
	std::pair<int,size_t> topKey()
	{
		int temp_val=PQ[1].value;
		size_t temp_tim = PQ[1].timestep;
		PQ[1]=PQ[PQ.size()-1];
		PQ.erase(PQ.end()-1);
		min_heapify(1);
		return std::make_pair(temp_val,temp_tim);
	}
	std::pair<int,size_t> pop()
	{
		int temp_val=PQ[1].value;
		size_t temp_tim = PQ[1].timestep;
		PQ[1]=PQ[PQ.size()-1];
		PQ.erase(PQ.end()-1);
		min_heapify(1);
		return std::make_pair(temp_val,temp_tim);
	}
	void insert(int v, size_t t, std::vector <int> heuristics)
	{
		// std::cout<<"Inserting : "<<v<<std::endl;
		element a(heuristics, v, t);
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
	void remove(int v)
	{
		int i;
		for(i=1;i<PQ.size();i++)
			if(PQ[i].value==v)
				break;
		swap(PQ[i],PQ[PQ.size()-1]);
		PQ.erase(PQ.end()-1);
		// printPQ();
		min_heapify(i);
		// printPQ();
	}
	bool contains(int v)
	{
		for(int i=1;i<PQ.size();i++)
			if(PQ[i].value==v)
				return true;
		return false;
	}
	// void printPQ()
	// {
	// 	cout<<"Elements: "<<endl;
	// 	for(int i=1;i<PQ.size();i++)
	// 		cout<<"( Value: "<<PQ[i].value<<", Timestep: "<<PQ[i].timestep<<", Key1: "<<PQ[i].key1<<", Key2: "<<PQ[i].key2<<"), ";
	// 	cout<<endl;
	// }
};

} // namespace HCBS

#endif 