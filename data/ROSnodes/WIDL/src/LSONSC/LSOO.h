#ifndef LSOO_H
#define LSOO_H

#include "../MATv2/Mat.h"
#include <vector>


template<typename T>
class LSOO
{
	protected :	
	
	public :

	vector<Mat<T> > params;
	T bound;
	
	LSOO()
	{

	}

	LSOO(vector<Mat<T> > params)
	{
		 this->params = params;
	
	}
	
	~LSOO()
	{
	
	}
		
	virtual Mat<T> energy(const Mat<T>& X) = 0;
	
	virtual Mat<T> init() = 0;
	
	virtual Mat<T> getJacobian() = 0;
	
	virtual Mat<T> getCost() = 0;
	
	virtual Mat<T> getY() = 0;

};

#endif
