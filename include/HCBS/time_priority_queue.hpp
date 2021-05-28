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
		double key1;
		double key2;
		double key3;
		double key4;
		int value;
		size_t timestep;
		element(double _key1, double _key2, double _key3, double _key4, int _value, size_t _timestep): key1(_key1), key2(_key2), key3(_key3), key4(_key4), value(_value), timestep(_timestep) {} 
		inline bool operator < (const element &b) const 
		{
			// if(key3 < b.key3) return true;
			// return false;
        	if(key1<b.key1)
				return true;
			else if(std::abs(key1-b.key1)<EPS && key2<b.key2)
				return true;
			else if(std::abs(key1-b.key1)<EPS && std::abs(key2-b.key2)<EPS && key3<b.key3)
				return true;
			else if(std::abs(key1-b.key1)<EPS && std::abs(key2-b.key2)<EPS && std::abs(key3-b.key3)<EPS && key4<b.key4)
				return true;
			else
				return false;	
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
		element a(-1,-1,-1,0, 0, 0);
		PQ.push_back(a);
	}
	void reset()
	{
		PQ.clear();
		element a(-1,-1,-1,0,0, 0);
		PQ.push_back(a);
	}
	int PQsize()
	{
		return PQ.size()-1;
	}
	pair<double,double> topKey()
	{
		return make_pair(PQ[1].key1,PQ[1].key2);
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
	void insert(int v, size_t t, double k1, double k2, double k3, double k4)
	{
		// std::cout<<"Inserting : "<<v<<std::endl;
		element a(k1,k2,k3,k4,v,t);
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
	void printPQ()
	{
		cout<<"Elements: "<<endl;
		for(int i=1;i<PQ.size();i++)
			cout<<"( Value: "<<PQ[i].value<<", Timestep: "<<PQ[i].timestep<<", Key1: "<<PQ[i].key1<<", Key2: "<<PQ[i].key2<<"), ";
		cout<<endl;
	}
};

} // namespace HCBS

#endif 