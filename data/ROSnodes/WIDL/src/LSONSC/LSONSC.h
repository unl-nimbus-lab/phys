#ifndef ONSC_H
#define ONSC_H
#include "LSOO.h"
#include "../MATv2/Mat.h"
#define verbose0
#define verbose1
//#define verbose2

template<typename T>
class LSONSC
{

	protected :

	int it;
	Mat<T>* cost;
	Mat<T>* x_k;					//variables
	int k;	

	Mat<T>* Delta;					//Error terms.
	Mat<T>* grad;					//Jacobians to store.
	Mat<T>* y;					//Sum of the elemental residual to store.


	LSOO<T>* instance;		//convenient object comprising of the objective function and its parameters


	public:	

	/*Constructor
	* @param LSOO object 
	* ooInst ; object which contains the objective function ready to be used with its parameters. OUGHT TO BE IMPLEMENTED
	* init; pointer to the function which initializes and handle the variable x_k (give dimension and so on...) OUGHT TO BE IMPLEMENTED
	* @param iteration ; number of iteration to do ; TODO by default there will be a stopping criteria applied.
	**/
	LSONSC(  LSOO<T>* ooInst, int it = 1)
	{		
		this->it = it;
		this->instance = ooInst;
		
		x_k = new Mat<T>[it+1];		
		cost = new Mat<T>[it+1];
		grad = new Mat<T>[it];
		Delta = new Mat<T>[it];
		y = new Mat<T>[it+1];

		#ifdef verbose0
		cout << "LSONSC Initialization : DONE." << endl;	
		#endif

		

	}

	Mat<T> compute(int it = 1)
	{
		this->it = it;
		
		
		x_k[0] = instance->init();
		cost[0] = instance->energy(x_k[0]);
		Mat<T> variation((T)0, cost[0].getLine(), cost[0].getColumn());
		
		Mat<T> tJJ(x_k[0].getLine(),x_k[0].getLine());
		Mat<T> itJJ(x_k[0].getLine(),x_k[0].getLine());

		for(k=0;k<it;k++)
		{
			callback();
			
			
			#ifdef verbose0		
			cout << "///////////////////////////\n LSONSC : Running : iteration " << k << " COST : " << endl;
			cost[k].afficher();
			#endif			
			grad[k] = computeJ();			
			#ifdef verbose1						
			cout << "LSONSC : jacobian : computed : "<< endl;

			(grad[k]).afficher();
			#endif			

			tJJ = transpose(grad[k])*grad[k];
			itJJ = invGJ(tJJ);
			
		
			//Delta[k] = itJJ*((cost[k].get(1,1)*transpose(grad[k])));			
			Delta[k] = itJJ*((-1*computeY().get(1,1))*transpose(grad[k]));			
			#ifdef verbose1
			cout << "LSONSC : tJJ : " << endl;
			tJJ.afficher();			
			#ifdef verbose2			
			SVD<T> instanceSVD(tJJ);
			Mat<T> diag(instanceSVD.getS());
			cout << "SVD = " << endl;
			diag.afficher();
			cout << "LSONSC : itJJ : " << endl;
			itJJ.afficher();	
			cout << "LSONSC : product :" << endl;
			(tJJ*itJJ).afficher();
			#endif								
			cout << "LSONSC : Delta : computed : " << endl;
			(Delta[k]).afficher();
			#endif			
		
			x_k[k+1] = x_k[k] + Delta[k];
			
			//BOUNDING :
			for(int i=x_k[k+1].getLine();i--;)
			{
				if(fabs_(x_k[k+1].get(i+1,1)) > instance->bound)
					x_k[k+1].set( (x_k[k+1].get(i+1,1)>0? instance->bound : -instance->bound), i+1,1);
			}
			
			#ifdef verbose1		
			cout <<  "LSONSC : X_k+1 : updated : "<< endl;
			x_k[k+1].afficher();
			#endif			

			/*----------------------------------------------------------*/
			cost[k+1] = instance->energy(x_k[k+1]);
			variation = cost[k+1]-cost[k];
			#ifdef verbose1
			cout << "///////// VARIATION : " << endl;
			variation.afficher();
			#endif		        				
			/*----------------------------------------------------------*/                

		}
		
		return x_k[k+1];
	}
	
	
	~LSONSC()	
	{
		delete[] cost;
		delete[] grad;
		delete[] x_k;
		delete[] Delta;
		delete[] y;
		
	}

	Mat<T> getX(int rank = -1)	const
	{
		return x_k[ (rank < 0 ? k : (rank <= k ? rank : k) )];
	}

	Mat<T> computeJ()
	{		
		return instance->getJacobian();
	}
	
	Mat<T> computeY()
	{
		return instance->getY();
	}


	void callback()
	{
		
	}

};


#endif
