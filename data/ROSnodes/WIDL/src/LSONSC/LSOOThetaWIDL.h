#ifndef LSOOTHETAWIDL_H
#define LSOOTHETAWIDL_H

#include "LSOO.h"
#include "LSONSC.h"

template<typename T>
Mat<T> rot2D(T angle)
{
	Mat<T> r(2,2);
	r.set(cos(angle), 1,1);
	r.set(cos(angle), 2,2);
	r.set(sin(angle), 1,2);
	r.set(-sin(angle), 2,1);
	return r;
}

	
template<typename T>
class LSOOThetaWIDL : public LSOO<T>
{
	private :
	Mat<T> cost;
	Mat<T> Jacobian;
	Mat<T> y;
	
	vector<Mat<T> > epsilons;
	bool epsilonComputed;
	
	T h;
	
	public :
	
	Mat<T> R;
	Mat<T>* dP;
	vector<Mat<T> >* points1;
	vector<Mat<T> >* points2;
	Mat<int>* matches1;
	Mat<int>* matches2;
	vector<Mat<T> >* matchPinv;
	
	LSOOThetaWIDL()
	{
		Jacobian = Mat<T>((T)0, 1,1);
		epsilonComputed = false;
	}
	
	LSOOThetaWIDL(vector<Mat<T> >* points1_, vector<Mat<T> >* points2_, Mat<int>* matches1_, Mat<int>* matches2_, vector<Mat<T> >* matchPinv_, Mat<T> R_, Mat<T>* dP_) : points1(points1_), points2(points2_), matches1(matches1_), matches2(matches2_), R(R_), dP(dP_), matchPinv(matchPinv_)
	{
		Jacobian = Mat<T>((T)0, 1,1);
		epsilonComputed = false;
	}
	
	~LSOOThetaWIDL()
	{
		
	}
	
	Mat<T> energy(const Mat<T>& X)
	{
		//initialization :
		R = rot2D(X.get(1,1));
		
		//Compute Epsilons :
		if(!epsilonComputed)
			computeEpsilons();
		
		//Compute M :
		Mat<T> M((T)0,1,1);
		int n = points1->size();
		for(int k=n;k--;)
		{
			M -= transpose(epsilons[k]) * (matchPinv[0][k] * epsilons[k]);
		}
		
		//compute lnD
		T D = (T)0;
		for(int k=n;k--;)
		{
			D += log( sqrt( matchPinv[0][k].mat[0][0]*matchPinv[0][k].mat[1][1] + matchPinv[0][k].mat[1][0]*matchPinv[0][k].mat[0][1] ) );
		}
		D -= ((T)n)*log(2*PI);
		
		M.set( M.get(1,1)+D, 1,1);
		
		
		//--------------------------
		cost = M;
		y = (-M.get(1,1))*Jacobian;
		
		//---------------------------
		
		
		return M;
			
	}
	
	void computeEpsilons()
	{
		epsilons.clear();
		
		int n= points1->size();
		
		for(int k=n;k--;)
		{
#ifndef		doubleMatchOnlyEstimation				
				epsilons.insert(epsilons.begin(), ( (*points1)[k] - R* (*points2)[matches1->get(1,k+1)-1]) - *dP );
#else
				if( matches2->get( 1, (matches1->get(1,k+1)-1) +1) == k+1)
				{
					epsilons.insert(epsilons.begin(), ( (*points1)[k] - R* (*points2)[ matches1->get(1,k+1)-1]) - *dP );
				}
#endif				
		}

	}
	
	
	inline void computeJacobian(const Mat<T>& x)
    {
        int n = x.getLine();
        Mat<T> delta((T)0, n,1);        
		T pownormex = pow(norme1(x),(T)1);
		h = pow(numeric_limits<T>::epsilon(), (T)0.5)* pownormex;

        if(isnan(h))
            h = pow(numeric_limits<T>::epsilon(), (T)0.5);

        delta.set(h, 1,1);

        Mat<T> x1(x+delta);
        Mat<T> x2(x-delta);
        volatile T dx = norme2(x1-x2);
        Mat<T> temp(energy(x1) - energy(x2));
        
        for(int i=temp.getLine();i--;) 
        {
        	Jacobian.set( temp.get(i+1,1), i+1,1);
        }

        for(int i=2;i<=n;i++)
        {
            delta.set((T)0,i-1,1);
            delta.set(h,i,1);

            x1 = x+delta;
            x2 = x-delta;
            temp = (energy(x1) - energy(x2));


            for(int j=1;j<=temp.getLine();j++) 
            	Jacobian.set( (T)((float)1.0/(dx))*temp.get(j,1), j,i);
        }
    }
	
	Mat<T> init()
    {
        Mat<T> rim((T)0, 1,1);
        
        return rim;
    }
    
    Mat<T> getJacobian()
    {
    	return Jacobian;
    }
    
    Mat<T> getCost()
    {
    	return cost;
    }
    
    Mat<T> getY()
    {
    	return y;
    }

};



#endif
