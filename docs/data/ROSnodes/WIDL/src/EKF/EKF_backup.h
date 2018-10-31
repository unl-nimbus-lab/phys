#ifndef EKF_H
#define EKF_H

#include "../MATv2/Mat.h"
#include "../RAND/rand.h"
//#include "../../PID/PIDcontroller.h"
#define default_ACC 1
#define default_DEC 1


template<typename T>
Mat<T> PIDCONTROL( Mat<T> phi_d, Mat<T> u_p, Mat<T> e_old, T dt, int behaviour = 3)
{
	Mat<T> r(phi_d-u_p);
	e_old = e_old + r;

	/*PID ANGULAR ROTATION : PERFECT*/
	Mat<T> Kp((T)/*-0.01*/-0.05, 2,1);
	Mat<T> Ki((T)50, 2,1);
	Mat<T> Kd((T)0, 2,1);
	/*-------------------------------*/
	
	if(behaviour==2)
	{
		Kp = (T)((-1000))*Kp;
		Ki = ((T)0)*Ki;
		Kd = Mat<T>((T)1,2,1);
		Kd = ((T)0)*Kd;
	
	}
	
	r = Kp%r + Ki%e_old + ((double)1.0/dt)*Kd%(r-e_old);	
//	r = dt*r+u_p;	
	
	return r;
}


template<typename T>
class EKF
{
	private :
	
	T time;
	T dt; 	/* par default : = 0.005 */
	int nbr_state;
	int nbr_ctrl;
	int nbr_obs;
	
	Mat<T> dX;	/*desired state*/
	Mat<T>* _X;	/*previous state*/
	Mat<T>* X;		/*states*/
	Mat<T> X_p;	/*predicted state*/
	Mat<T> memX_p;
	Mat<T>* X_;	/*derivated states or next states... (continuous/discrete)*/
	Mat<T>* u;		/*controls*/
	Mat<T>* z;		/*observations/measurements*/
	int phase;
	Mat<T> obs_old;
	Mat<T> eSum;
	Mat<T> e_old;
	const static T r = (T)36;  /*radius*/
        const static T l = (T)220;   /*entre-roue*/
        T elapsed_T;
        double rho_initial;
        bool rho_init;
        double Vd_max;
	
	Mat<T>* Ki;
	Mat<T>* Kp;
	Mat<T>* Kd;
	Mat<T>* Kdd;	
	
	Mat<T>* A;		/*linear relation matrix between states and derivated states.*/
	/*par default : X_ = A * X + B * u / x_i = x_i + dt * x_i_ + b_i * u_i... */
	Mat<T>* B;		/*linear relation matrix between derivated states and control.*/
	/*par default : B = 1 */
	Mat<T>* C;		/*linear relation matrix between states and observation.*/
	/*par default : C = [1 0], on observe les positions, non leurs dérivées. */

	/*Noise*/
	T std_noise;	/*par defaut : 0.0005*/
	Mat<T>* Sigma;	/*covariance matrix*/
	Mat<T>* Q;		/*process noise*/
	NormalRand* rgenQ;
	Mat<T>* R;		/*measurement noise*/
	NormalRand* rgenR;
	bool noise;
	
	/*Prediction*/
	Mat<T> K;		// Kalman Gain...
	Mat<T> Sigma_p;
	
	/*Others*/
	bool filterOn;
	Mat<T>* Identity;
	
	
	/*Extended*/
	bool extended;
	Mat<T> (*ptrMotion)(Mat<T> state, Mat<T> command, T dt);
	Mat<T> (*ptrSensor)(Mat<T> state, Mat<T> command, Mat<T> d_state, T dt);
	Mat<T> G;
	Mat<T> H;
	Mat<T> (*ptrJMotion)(Mat<T> state, Mat<T> command, T dt);
	Mat<T> (*ptrJMotionCommand)(Mat<T> state, Mat<T> command, T dt);
	Mat<T> (*ptrJSensor)(Mat<T> state, Mat<T> command, Mat<T> d_state, T dt);
	
	
	public :
	
	EKF(int nbr_state_, int nbr_ctrl_, int nbr_obs_, T dt_, T std_noise_, Mat<T> currentState, bool ext = false, bool filterOn = true, bool noise = true)
	{
		this->filterOn = filterOn;
		/*extension*/
		time = (T)0;
		extended = ext;
		ptrMotion = NULL;
		ptrSensor = NULL;
		ptrJMotion = NULL;
		ptrJMotionCommand = NULL;
		ptrJSensor = NULL;
		G = Mat<T>((T)0, nbr_state_, nbr_state_);
		H = Mat<T>((T)0, nbr_obs_, nbr_state_);		
		
		/*----------------*/
		
		dt = dt_;
		nbr_state = nbr_state_;
		nbr_ctrl = nbr_ctrl_;
		nbr_obs = nbr_obs_;
		elapsed_T = (T)0;
		rho_init = false;
		
		_X = new Mat<T>((T)0, nbr_state, (int)1);		/*previous state*/
		X = new Mat<T>(currentState); 	
		dX = *X;				/*states*/
		X_ = new Mat<T>((T)0, nbr_state, (int)1);		/*derivated states*/
		memX_p = *X_;
		u = new Mat<T>((T)0, nbr_ctrl, (int)1);			/*controls*/
		z = new Mat<T>((T)0, nbr_obs, (int)1);	
		obs_old = *z;						/*observations*/
		e_old = *z;
		eSum = *z;
		A = new Mat<T>((T)0, nbr_state, nbr_state);		/*linear relation or jacobian matrix between states and derivated states.*/
		B = new Mat<T>((T)0, nbr_state, nbr_ctrl);		/*linear relation matrix between derivated states and control.*/
		C = new Mat<T>((T)0, nbr_obs, nbr_state);		/*linear relation or jacobian matrix between states and observation.*/
	
		Ki = new Mat<T>((T)0.08, nbr_ctrl, nbr_state);
		Kp = new Mat<T>((T)0.08, nbr_ctrl, nbr_state);
		Kd = new Mat<T>((T)0.08, nbr_ctrl, nbr_state);
		Kdd = new Mat<T>((T)0.08, nbr_ctrl, nbr_state);
	
	
		std_noise = std_noise_;
		Sigma = new Mat<T>((T)0, nbr_state, nbr_state);
		Q = new Mat<T>((T)0, nbr_state, nbr_state);
		R = new Mat<T>((T)0, nbr_obs, nbr_obs/*1*/);
		this->noise = noise;
		if(noise)
		{
			rgenQ = new NormalRand(0.0,std_noise,(long)10);
			rgenR = new NormalRand(0.0,std_noise,(long)100);
		}
		//Normal Measurement noise only for now on...
	
		/*Initialize Covariance matrix as the identity matrix.*/
		for(int i=1;i<=nbr_state;i++)
		{
			Sigma->set((T)1, i,i);			
			if(noise)
				R->set((T)rgenR->dev(), i,i);
			else
				R->set( (T)std_noise, i,i);
			
			
			/*
			for(int j=1;j<=nbr_state;j++)
			{
				Sigma->set((T)1, i, j);
			
				//if(i<=nbr_obs && j==1)
				//	R->set(std_noise*std_noise, i, j);
				
			}
			*/
		}
	
		Identity = new Mat<T>(*Sigma);
		*Q = (std_noise*std_noise)*(*Identity);
		//*R = (*R);
		
		
	
	}

	~EKF()
	{
		delete _X;
		delete X;
		delete X_;
		delete u;
		delete z;
		delete A;
		delete B;
		delete C;
	
		delete Ki;
		delete Kp;
		delete Kd;
		delete Kdd;
	
		delete Sigma;
		delete Q;
		delete R;
	
		delete Identity;
	}

/*-------------------------------------------------------------*/
/*-------------------------------------------------------------*/
/*-------------------------------------------------------------*/


	int initA( Mat<T> A_)
	{
		if(A_ == *Identity)
		{
			for(int i=1;i<=(int)(nbr_state/2);i++)
			{
				A->set( dt, i, i+(int)(nbr_state/2));
			}
		
			return 1;
		}
		else
		{
			if(A_.getColumn() == nbr_state && A_.getLine() == nbr_state)
			{
				*A = A_;
				return 1;
			}
			else
			{
				cout << "ERREUR : mauvais format de matrice d'initialisation de A." << endl;
				return 0;
			}
		}
	}


	int initB( Mat<T> B_)
	{	
		if(B_.getColumn() == nbr_ctrl && B_.getLine() == nbr_state)
		{
			*B = B_;
			return 1;
		}
		else
		{
			cout << "ERREUR : mauvais format de matrice d'initialisation de B." << endl;
			return 0;
		}
	
	}
	
	
	
	int initC( Mat<T> C_)
	{	
		if(C_.getColumn() == nbr_state && C_.getLine() == nbr_obs)
		{
			*C = C_;
			return 1;
		}
		else
		{
			cout << "ERREUR : mauvais format de matrice d'initialisation de C." << endl;
			return 0;
		}
	
	}
	
	/*extension*/
	void initMotion( Mat<T> motion(Mat<T>, Mat<T>, T) )
	{
		ptrMotion = motion;
	}
	
	
	
	void initSensor( Mat<T> sensor(Mat<T>, Mat<T>, Mat<T>, T) )
	{	
		ptrSensor = sensor;	
	}
	
	void initJMotion( Mat<T> jmotion(Mat<T>, Mat<T>, T) )
	{
		ptrJMotion = jmotion;
	}
	
	void initJMotionCommand(Mat<T> jmotioncommand(Mat<T>,Mat<T>,T) )
	{
		ptrJMotionCommand = jmotioncommand;
	}
	
	
	void initJSensor( Mat<T> jsensor(Mat<T>, Mat<T>, Mat<T>, T) )
	{	
		ptrJSensor = jsensor;	
	}
	
	
/*-------------------------------------------------------------*/
/*-------------------------------------------------------------*/
/*-------------------------------------------------------------*/


	int setKi( Mat<T> Ki_)
	{
		if(Ki_.getColumn() == nbr_state && Ki_.getLine() == nbr_ctrl)
		{
			*Ki = Ki_;
			return 1;
		}
		else
		{
			cout << "ERREUR : mauvais format de vecteur d'initialisation de Ki." << endl;
			return 0;
		}
	}
	
	int setKp( Mat<T> Kp_)
	{
		if(Kp_.getColumn() == nbr_state && Kp_.getLine() == nbr_ctrl)
		{
			*Kp = Kp_;
			return 1;
		}
		else
		{
			cout << "ERREUR : mauvais format de vecteur d'initialisation de Kp." << endl;
			return 0;
		}
	}
	
	
	
	int setKd( Mat<T> Kd_)
	{
		if(Kd_.getColumn() == nbr_state && Kd_.getLine() == nbr_ctrl)
		{
			*Kd = Kd_;
			return 1;
		}
		else
		{
			cout << "ERREUR : mauvais format de vecteur d'initialisation de Kd." << endl;
			return 0;
		}

	}
	
	int setKdd( Mat<T> Kdd_)
	{
		if(Kdd_.getColumn() == nbr_state && Kdd_.getLine() == nbr_ctrl)
		{
			*Kdd = Kdd_;
			return 1;
		}
		else
		{
			cout << "ERREUR : mauvais format de vecteur d'initialisation de Kdd." << endl;
			return 0;
		}

	}
	
	
	void setdt( float dt_)
	{
		dt = dt_;
	}
	
/*-------------------------------------------------------------*/
/*-------------------------------------------------------------*/
/*-------------------------------------------------------------*/	
	
	
	Mat<T> getCommand()
	{
		return *u;
	}	
	
	Mat<T> getX()
	{
		return *X;
	}
	
	Mat<T> getXp()
	{
		return memX_p;
	}

	Mat<T> getSigma()
	{
		return *Sigma;
	}	
	
	Mat<T> getKi()
	{
		return *Ki;
	}


/*-------------------------------------------------------------*/
/*-------------------------------------------------------------*/
/*-------------------------------------------------------------*/		
	
	
	Mat<T> predictState()		/*return the computed predicted state*/
	{
		memX_p = (!extended ? (*A)*(memX_p)+(*B)*(*u) : (ptrJMotionCommand == NULL ? ptrMotion(memX_p, *u, dt) : ptrJMotion(dX,(T)(0)*(*u),dt)*(memX_p-dX) + ptrJMotionCommand(dX,(T)(0)*(*u),dt)*(*u) ) );
		
		return (!extended ? (*A)*(*X)+(*B)*(*u) : (ptrJMotionCommand == NULL ? ptrMotion(*X, *u, dt) : ptrJMotion(dX,(T)(0)*(*u),dt)*(*X-dX) + ptrJMotionCommand(dX,(T)(0)*(*u),dt)*(*u) ) );
	}
	
	
	Mat<T> predictCovariance()	/*return the predicted covariance matrix.*/
	{
		if(extended)
			G = ptrJMotion(*X, *u, dt);
				
			
		return (filterOn ? ( (!extended ? ((*A)*(*Sigma))* transpose(*A) : G*(*Sigma)*transpose(G) ) + *Q) : *Identity);
	}
	
		
	Mat<T> calculateKalmanGain()	/*return the Kalman Gain K = C*Sigma_p * (C*Sigma_p*C.T +R).inv */
	{
		if(filterOn)
		{			

			if(extended)
				H = ptrJSensor(X_p, *u, dX,dt);
			
			Mat<T> temp(invGJ( H * Sigma_p * transpose(H) + *R) );
#ifdef verbose_Jacobian				
			cout << "H :" << endl;
			H.afficher();
			cout << "sigma_p : " << endl;
			Sigma_p.afficher();
			cout << "inv GJ :" << endl;
			temp.afficher();
#endif			
			
			

			return ( !extended ? Sigma_p * transpose(*C) * invGJ( (*C) * Sigma_p * transpose(*C) + *R)  : Sigma_p * transpose(H) * temp );
		}
		else
			return *Identity;
	}
	
		
	Mat<T> correctState()		/*update X */
	{		
	
		*_X = *X;
		
		if(filterOn)
			*X = X_p + K*( (*z) - (!extended ? (*C)*X_p  : ptrSensor(X_p, *u, dX, dt) ) );
		else
			*X = X_p;
	
		return *X;
	}
	
		
	Mat<T> correctCovariance()	/*update Sigma*/
	{
		if(filterOn)
		{			
			*Sigma = (*Identity - K* (!extended ? (*C) : H) ) *Sigma_p;
		}
		else
			*Sigma = *Identity;
	
		return *Sigma;
	}
	
	
	void state_Callback()		/* Update all... */
	{
		if(noise)
		{
			for(int i=1;i<=nbr_state;i++)
			{
				R->set((T)rgenR->dev(), i,i);
			}
						
		}
			//Normal measurement noise simulation...
		
		
		time += dt;
		
		if( extended && (ptrMotion == NULL || ptrSensor == NULL || ptrJMotion == NULL || ptrJSensor == NULL) )
		{
			//~EKF();
			throw("ERREUR : les fonctions ne sont pas initialisées...");
		}		
		
		X_p = predictState();		
		Sigma_p = predictCovariance();	
		
		K = calculateKalmanGain();	

		correctState();
		correctCovariance();				 	
#ifdef K_gain_debug		
		cout << "Predicted State :" << endl;
		X_p.afficher();
		cout << "Predicted Covariance : " << endl;
		Sigma_p.afficher();
		cout << "Kalman Gain : " << endl;	
		K.afficher();
		cout << "Updated State : " << endl;
		X->afficher();
		cout << "Updated Covariance : " << endl;
		Sigma->afficher();		
#endif		

	}
	
	void measurement_Callback(Mat<T> measurements, Mat<T> dX_, bool measure = false)
	{
		if( extended && (ptrMotion == NULL || ptrSensor == NULL || ptrJMotion == NULL || ptrJSensor == NULL) )
		{
			//~EKF();
			throw("ERREUR : les fonctions ne sont pas initialisées...");
		}
		
		dX = dX_;
		
		*z = (!extended || measure ? measurements : ptrSensor(measurements,*u, dX, dt) );	
		
		if(!measure)
		{
			Mat<T> obsvar(z->getLine(),z->getColumn(), (char)1);
			//obsvar.afficher();
			*z = *z + obsvar;
			//cout << "------------Observation :" << endl;
			//z->afficher();
		}
	}
	
	void measurement_Callback(Mat<T> measurements)
	{
		if( extended && (ptrMotion == NULL || ptrSensor == NULL || ptrJMotion == NULL || ptrJSensor == NULL) )
		{
			//~EKF();
			throw("ERREUR : les fonctions ne sont pas initialisées...");
		}
		
		
		//*z = (!extended ? measurements : ptrSensor(*X,*u, dX, dt) );
		*z = (*C)*(*X);				
	}
	
	
	void setCommand(Mat<T> u)
	{
		*(this->u) = u;
	}
	
	void computeCommand( Mat<T> desiredX, T dt_, int mode)
	{
		//obs_old = obs_old + dt_*(*z);
		
		/*work on observation instead of state vector ???? */
		
		/*
		if(dt_ != (T)0 && mode != -1)
		{
			*u = (*Kp)*(desiredX - (*X));
			if(mode >= 1)
				*u = *u + (T)(dt_)*(*Ki)*(desiredX - (*X));
			if(mode >= 2)
				*u = *u + (T)((double)1.0/dt_)*(*Kd)*(desiredX - (*X));
			if(mode >= 3)
				*u = *u + (T)((double)1.0/(dt_*dt_))*(*Kdd)*(desiredX - (*X));						
		}		
		else */if( mode ==-2)
		{
			float rho = norme2(extract(desiredX-(*X), 1,1,2,1));
			float alpha = atan2( desiredX.get(2,1) - X->get(2,1), desiredX.get(1,1) - X->get(1,1));
			alpha -= X->get(3,1);
			//alpha = atan2(sin(alpha), cos(alpha));
			float beta = desiredX.get(3,1)-X->get(3,1);
			//beta = atan2( sin(beta), cos(beta));
						
			while(alpha > PI)	alpha -= PI;
			while(alpha < -PI)	alpha += PI;
			while(beta > PI)	beta -= PI;
			while(beta < -PI)	beta += PI;
			
			cout << "alpha = " << alpha << " beta = " << beta << " rho = " << rho << endl;
			
			float krho = 0;
			float kbeta = 10;
			float kalpha = 0;
			
			
			u->set( krho*rho, 1,1);
			u->set(kalpha*alpha+kbeta*beta, 2,1);
		}
		else if(mode == -3)
		{	
			T rho = desiredX.get(2,1)-X->get(2,1);
			rho /= 1+ pow(rho,2); 
			
			T G = 9.81f;
			
			cout << " rho = " << rho << endl;
			
			T krho = 0.5;			
			
			T offsetgravity = -dt*G;
			
			*u= (krho*rho+offsetgravity)*Mat<T>((T)1,4,1);
			
		}
		
	}	
	
	bool computeTrapezoidalCommandLinear( Mat<T> *phi_dT, Mat<T> phi_current, double rho, double Vd_max, double acc, double dec, int* phase)
	{
		//we begin at phi_current
		double Vd_current = (double)(r/l)*(phi_current.get(1,1)+phi_current.get(2,1));		
		double acc_T = fabs( (T)(1.0/acc)*(Vd_max - Vd_current) );
		double dec_T = fabs( (T)(1.0/dec)*(Vd_max) );
		
		
		if( acc_T*acc_T/2*acc*11.0/10 + dec_T*dec_T/2*dec*11.0/10 < fabs(rho) )
		{
		
//#ifdef verbosePID
			cout << "acc_T = " << acc_T << " dec_T = " << dec_T << endl;
			cout << "distance = " << acc_T*acc_T/2*acc*11.0/10 + dec_T*dec_T/2*dec*11.0/10 << " sur, à faire : " << fabs(rho) << endl;
			cout << "PHASE = " << *phase << endl;
			cout << "V = " << Vd_current << " / Vd_max : " << Vd_max << endl;
//#endif
			double const_T = (acc_T+dec_T)*1.0/10;
			//enough time to go to the maximum velocity value with this value of V_max and these value of acc and dec...
			if( *phase == 0 /*acceleration*/ || elapsed_T <= acc_T)
			{
				*phase = 0;				
				
				if(elapsed_T+2*dt > acc_T)
					*phase += 1;
				else
					cout << "T = " << elapsed_T << " s" << endl;
					
				phi_dT->set( acc*dt+phi_current.get(1,1), 1,1);
				phi_dT->set( acc*dt+phi_current.get(2,1), 2,1);
				
			}	
			else if( *phase == 1 || ( elapsed_T >= acc_T && elapsed_T <= acc_T+const_T) )//Vd_current <= Vd_max*11.0/10 && Vd_current >= Vd_max*9.0/10)
			{
				*phase = 1;
				if(elapsed_T+2*dt > acc_T+const_T)
					*phase += 1;
				else
					cout << "T = " << elapsed_T << " s" << endl;
					
				*phi_dT = Mat<T>((l/(2*r))*Vd_max,2,1);
				
			}
			else if( *phase == 2 || ( elapsed_T >= acc_T+const_T && elapsed_T <= acc_T+const_T+dec_T) )
			{
				*phase = 2;
				if(elapsed_T+2*dt > acc_T+const_T+dec_T)
					*phase += 1;
				else
					cout << "T = " << elapsed_T << " s" << endl;
				
				phi_dT->set( -dec*dt+phi_current.get(1,1), 1,1);
				phi_dT->set( -dec*dt+phi_current.get(2,1), 2,1);					
			
			}
			else if( *phase == 3)
			{
				*phi_dT	= Mat<T>((T)0,2,1);
			}
			
			return true;
		}
		
			//not enough time to go to the maximum velocity value...
		return false;		
	}
	
	bool computeCommandLinearG(Mat<T>* phi_dT, Mat<T> phi_current, Mat<T> phi_consigne, double Vd_max, double acc, double dec)
	{
		if( phi_consigne.get(1,1) - phi_current.get(1,1) >= (T)0)
		{
			//acceleration
			if(acc*dt > phi_consigne.get(1,1)-phi_current.get(1,1))
			{
				//increase only the difference...
				phi_dT->set(phi_consigne.get(1,1), 1,1);				
				return true;
			}
			else if( phi_consigne.get(1,1) == phi_current.get(1,1))
			{
				return true;
			}
			else
			{
				phi_dT->set( acc*dt+phi_current.get(1,1), 1,1);
				return true;
			}
		}
		else
		{
			//deceleration
			if( dec*dt > fabs_(phi_consigne.get(1,1)-phi_current.get(1,1)) )
			{
				//increase only the difference...
				phi_dT->set( phi_consigne.get(1,1),1,1);				
				return true;
			}
			else if( phi_consigne.get(1,1) == phi_current.get(1,1))
			{
				return true;
			}
			else
			{
				phi_dT->set( -dec*dt+phi_current.get(1,1), 1,1);
				return true;
			}
		}
		
		return false;
	
	
	}
	
		bool computeCommandLinearD(Mat<T>* phi_dT, Mat<T> phi_current, Mat<T> phi_consigne, double Vd_max, double acc, double dec)
	{
		if( phi_consigne.get(2,1) - phi_current.get(2,1) >= (T)0)
		{
			//acceleration
			if(acc*dt > phi_consigne.get(2,1)-phi_current.get(2,1))
			{
				//increase only the difference...
				phi_dT->set(phi_consigne.get(2,1), 2,1);				
				return true;
			}
			else if( phi_consigne.get(2,1) == phi_current.get(2,1))
			{
				return true;
			}
			else
			{
				phi_dT->set( acc*dt+phi_current.get(2,1), 2,1);
				return true;
			}
		}
		else
		{
			//deceleration
			if( dec*dt > fabs_(phi_consigne.get(2,1)-phi_current.get(2,1)) )
			{
				//increase only the difference...
				phi_dT->set( phi_consigne.get(2,1),2,1);				
				return true;
			}
			else if( phi_consigne.get(2,1) == phi_current.get(2,1))
			{
				return true;
			}
			else
			{
				phi_dT->set( -dec*dt+phi_current.get(2,1), 2,1);
				return true;
			}
		}
		
		return false;
	
	
	}
	
	Mat<T> PIDCONTROL( Mat<T> phi_d, Mat<T> u_p, T dt, int behaviour = 3)
	{
		Mat<T> r(phi_d-u_p);		
		eSum = eSum + r;
		Mat<T> eDiff(r-e_old);
		e_old = r;
		
		Mat<T> P((T)0.01, 2,1);
		Mat<T> I((T)0.00001, 2,1);
		Mat<T> D((T)0, 2,1);

		/*PID ANGULAR ROTATION : PERFECT*/
		//Mat<T> P((T)-0.01, 2,1);
		//Mat<T> I((T)50, 2,1);
		//Mat<T> D((T)0, 2,1);
		
//		Mat<T> P((T)0.01, 2,1);
//		Mat<T> I((T)-0.00001, 2,1);

//		Mat<T> P((T)1.5, 2,1);
//		Mat<T> I((T)0.1, 2,1);
		
		/*-------------------------------*/
	
		if(behaviour==2)
		{
			P = Mat<T>((T)0.8, 2,1);
			I = Mat<T>((T)1, 2,1);
			P = (T)((2))*P;
			I = ((T)1)*I;
			D = ((T)1)*D;
	
		}
		else if(behaviour ==1 || behaviour == 3)
		{
			P = Mat<T>((T)2, 2,1);
			I = Mat<T>((T)0.1, 2,1);
			D = Mat<T>((T)0,2,1);
		
		}
	
#ifdef verbose_pid		
		cout << "Error = " << endl;
		r.afficher();
		cout << " eSum = " << endl;
		eSum.afficher();
		cout << "eDiff = " << endl;
		eDiff.afficher();
#endif		
		
		r = P%r + I%eSum + /*((double)1.0/dt)*/D%eDiff;	
		
		cout << " New value COMMAND = " << endl;
		r.afficher();

	
		return r;
	}

};





template<typename T>
class EEKF
{
	private :
	
	T time;
	T dt; 	/* par default : = 0.005 */
	int nbr_state;
	int nbr_ctrl;
	int nbr_obs;
	
	Mat<T> dX;	/*desired state*/
	Mat<T>* _X;	/*previous state*/
	Mat<T>* X;		/*states*/
	Mat<T> X_p;	/*predicted state*/
	Mat<T> memX_p;
	Mat<T>* X_;	/*derivated states or next states... (continuous/discrete)*/
	Mat<T>* u;		/*controls*/
	Mat<T>* z;		/*observations/measurements*/
	
	Mat<T>* Ki;
	Mat<T>* Kp;
	Mat<T>* Kd;
	
	Mat<T>* A;		/*linear relation matrix between states and derivated states.*/
	/*par default : X_ = A * X + B * u / x_i = x_i + dt * x_i_ + b_i * u_i... */
	Mat<T>* B;		/*linear relation matrix between derivated states and control.*/
	/*par default : B = 0 */
	Mat<T>* C;		/*linear relation matrix between states and observation.*/
	/*par default : C = [1 0], on observe les positions, non leurs dérivées. */
	
	/*Mat<T>* PMotion;	//covariance matrix of the motion model
	Mat<T>* PObs;		//covariance matrix of the observation model
	//par default : P = 0;
	*/
	
	Mat<T>* innovation;
	

	/*Noise*/
	T std_noise;	/*par defaut : 0.0005*/
	Mat<T>* Sigma;	/*covariance matrix*/
	Mat<T>* Q;		/*process noise*/
	NormalRand* rgenQ;
	Mat<T>* R;		/*measurement noise*/
	NormalRand* rgenR;
	bool noise;
	
	/*Prediction*/
	Mat<T> K;		// Kalman Gain...
	Mat<T> Sigma_p;
	
	/*Others*/
	bool filterOn;
	Mat<T>* Identity;
	
	
	/*Extended*/
	bool extended;
	Mat<T> (*ptrMotion)(Mat<T> state, Mat<T> command, T dt);
	Mat<T> (*ptrSensor)(Mat<T> state, Mat<T> command, Mat<T> d_state, T dt);
	Mat<T> G;
	Mat<T> H;
	Mat<T> (*ptrJMotion)(Mat<T> state, Mat<T> command, T dt);
	Mat<T> (*ptrJMotionCommand)(Mat<T> state, Mat<T> command, T dt);
	Mat<T> (*ptrJSensor)(Mat<T> state, Mat<T> command, Mat<T> d_state, T dt);
	
	
	/*Estimation*/
	bool estimation;
	int n;	//cardinal of the sample set	
	Mat<T>* sampleval;
	
	public :
	
	EEKF(int nbr_state_, int nbr_ctrl_, int nbr_obs_, T dt_, T std_noise_, Mat<T> currentState, bool ext = false, bool filterOn = true, bool noise = true)
	{
		/*Estimation*/
		estimation = true;
		n = 100;
		sampleval = new Mat<T>[n];
		for(int i=n;i--;)	sampleval[i] = Mat<T>((T)0,nbr_obs_,1);
		//-----------------------
		
		this->filterOn = filterOn;
		/*extension*/
		time = (T)0;
		extended = ext;
		ptrMotion = NULL;
		ptrSensor = NULL;
		ptrJMotion = NULL;
		ptrJMotionCommand = NULL;
		ptrJSensor = NULL;
		G = Mat<T>((T)0, nbr_state_, nbr_state_);
		H = Mat<T>((T)0, nbr_obs_, nbr_state_);		
		
		/*----------------*/
		
		dt = dt_;
		nbr_state = nbr_state_;
		nbr_ctrl = nbr_ctrl_;
		nbr_obs = nbr_obs_;		
		
		_X = new Mat<T>((T)0, nbr_state, (int)1);		/*previous state*/
		X = new Mat<T>(currentState);
		dX = *X;				/*states*/
		X_ = new Mat<T>((T)0, nbr_state, (int)1);		/*derivated states*/
		memX_p = *X_;
		u = new Mat<T>((T)0, nbr_ctrl, (int)1);			/*controls*/
		z = new Mat<T>((T)0, nbr_obs, (int)1);	
		
		A = new Mat<T>((T)0, nbr_state, nbr_state);		/*linear relation or jacobian matrix between states and derivated states.*/
		B = new Mat<T>((T)0, nbr_state, nbr_ctrl);		/*linear relation matrix between derivated states and control.*/
		C = new Mat<T>((T)0, nbr_obs, nbr_state);		/*linear relation or jacobian matrix between states and observation.*/

		innovation = new Mat<T>((T)0, nbr_obs,1);
	
		Ki = new Mat<T>((T)0, nbr_ctrl, nbr_state);
		Kp = new Mat<T>((T)0, nbr_ctrl, nbr_state);
		Kd = new Mat<T>((T)0, nbr_ctrl, nbr_state);		
	
	
		std_noise = std_noise_;
		Sigma = new Mat<T>((T)0, nbr_state, nbr_state);
		Q = new Mat<T>((T)0, nbr_state, nbr_state);
		R = new Mat<T>((T)0, nbr_obs, nbr_obs/*1*/);
		this->noise = noise;
		if(noise)
		{
			rgenQ = new NormalRand(0.0,std_noise,(long)10);
			rgenR = new NormalRand(0.0,std_noise,(long)100);
		}
		//Normal Measurement noise only for now on...
	
		/*Initialize Covariance matrix as the identity matrix.*/
		for(int i=1;i<=nbr_state;i++)
		{
			if(noise)
				R->set((T)rgenR->dev(), i,i);
			else
				R->set( (T)std_noise, i,i);
			
			
			/*
			for(int j=1;j<=nbr_state;j++)
			{
				Sigma->set((T)1, i, j);
			
				//if(i<=nbr_obs && j==1)
				//	R->set(std_noise*std_noise, i, j);
				
			}
			*/
		}
	
		Identity = new Mat<T>((T)0,nbr_state,nbr_state);
		for(int i=nbr_state;i--;)	Identity->set((T)1, i+1,i+1);
		*Q = (std_noise*std_noise)*(*Identity);
		//*R = (*R);
		
		
	
	}

	~EEKF()
	{
		delete _X;
		delete X;
		delete X_;
		delete u;
		delete z;
		delete A;
		delete B;
		delete C;
		
		delete innovation;
		
		delete Ki;
		delete Kp;
		delete Kd;
	
		delete Sigma;
		delete Q;
		delete R;
	
		delete Identity;
		
		//Estimation
		if(estimation)
			delete[] sampleval;
	}

/*-------------------------------------------------------------*/
/*-------------------------------------------------------------*/
/*-------------------------------------------------------------*/


	int initA( const Mat<T>& A_)
	{
		if(A_ == *Identity)
		{
			for(int i=1;i<=(int)(nbr_state/2);i++)
			{
				A->set( dt, i, i+(int)(nbr_state/2));
			}
		
			return 1;
		}
		else
		{
			if(A_.getColumn() == nbr_state && A_.getLine() == nbr_state)
			{
				*A = A_;
				return 1;
			}
			else
			{
				cout << "ERREUR : mauvais format de matrice d'initialisation de A." << endl;
				return 0;
			}
		}
	}


	int initB( const Mat<T>& B_)
	{	
		if(B_.getColumn() == nbr_ctrl && B_.getLine() == nbr_state)
		{
			*B = B_;
			return 1;
		}
		else
		{
			cout << "ERREUR : mauvais format de matrice d'initialisation de B." << endl;
			return 0;
		}
	
	}
	
	
	
	int initC( const Mat<T>& C_)
	{	
		if(C_.getColumn() == nbr_state && C_.getLine() == nbr_obs)
		{
			*C = C_;
			return 1;
		}
		else
		{
			cout << "ERREUR : mauvais format de matrice d'initialisation de C." << endl;
			return 0;
		}
	
	}
	
	int initQ( const Mat<T>& Q_)
	{	
		if(Q_.getColumn() == nbr_state && Q_.getLine() == nbr_state)
		{
			*Q = Q_;
			return 1;
		}
		else
		{
			cout << "ERREUR : mauvais format de matrice d'initialisation de Q." << endl;
			return 0;
		}
	
	}
	
	
	int initR( const Mat<T>& R_)
	{	
		if(R_.getColumn() == nbr_obs && R_.getLine() == nbr_obs)
		{
			*R = R_;
			return 1;
		}
		else
		{
			cout << "ERREUR : mauvais format de matrice d'initialisation de R." << endl;
			return 0;
		}
	
	}
	
	/*extension*/
	void initMotion( Mat<T> motion(Mat<T>, Mat<T>, T) )
	{
		ptrMotion = motion;
	}
	
	
	
	void initSensor( Mat<T> sensor(Mat<T>, Mat<T>, Mat<T>, T) )
	{	
		ptrSensor = sensor;	
	}
	
	void initJMotion( Mat<T> jmotion(Mat<T>, Mat<T>, T) )
	{
		ptrJMotion = jmotion;
	}
	
	void initJMotionCommand(Mat<T> jmotioncommand(Mat<T>,Mat<T>,T) )
	{
		ptrJMotionCommand = jmotioncommand;
	}
	
	
	void initJSensor( Mat<T> jsensor(Mat<T>, Mat<T>, Mat<T>, T) )
	{	
		ptrJSensor = jsensor;	
	}
	
	
/*-------------------------------------------------------------*/
/*-------------------------------------------------------------*/
/*-------------------------------------------------------------*/


	int setKi( Mat<T> Ki_)
	{
		if(Ki_.getColumn() == nbr_state && Ki_.getLine() == nbr_ctrl)
		{
			*Ki = Ki_;
			return 1;
		}
		else
		{
			cout << "ERREUR : mauvais format de vecteur d'initialisation de Ki." << endl;
			return 0;
		}
	}
	
	int setKp( Mat<T> Kp_)
	{
		if(Kp_.getColumn() == nbr_state && Kp_.getLine() == nbr_ctrl)
		{
			*Kp = Kp_;
			return 1;
		}
		else
		{
			cout << "ERREUR : mauvais format de vecteur d'initialisation de Kp." << endl;
			return 0;
		}
	}
	
	
	
	int setKd( Mat<T> Kd_)
	{
		if(Kd_.getColumn() == nbr_state && Kd_.getLine() == nbr_ctrl)
		{
			*Kd = Kd_;
			return 1;
		}
		else
		{
			cout << "ERREUR : mauvais format de vecteur d'initialisation de Kd." << endl;
			return 0;
		}

	}	
	
	
	void setdt( float dt_)
	{
		dt = dt_;
	}
	
/*-------------------------------------------------------------*/
/*-------------------------------------------------------------*/
/*-------------------------------------------------------------*/	
	
	
	Mat<T> getCommand()
	{
		return *u;
	}	
	
	Mat<T> getX()
	{
		return *X;
	}
	
	Mat<T> getXp()
	{
		return memX_p;
	}

	Mat<T> getSigma()
	{
		return *Sigma;
	}	
	
	Mat<T> getKi()
	{
		return *Ki;
	}


/*-------------------------------------------------------------*/
/*-------------------------------------------------------------*/
/*-------------------------------------------------------------*/		
	
	
	Mat<T> predictState()		/*return the computed predicted state*/
	{
		//memX_p = (!extended ? (*A)*(memX_p)+(*B)*(*u) : (ptrJMotionCommand == NULL ? ptrMotion(memX_p, *u, dt) : ptrJMotion(dX,(T)(0)*(*u),dt)*(memX_p-dX) + ptrJMotionCommand(dX,(T)(0)*(*u),dt)*(*u) ) );
		
		if(nbr_ctrl==0)
			return (*A)*(*X);
		else
			return (!extended ? (*A)*(*X)+(*B)*(*u) : (ptrJMotionCommand == NULL ? ptrMotion(*X, *u, dt) : ptrJMotion(dX,(T)(0)*(*u),dt)*(*X-dX) + ptrJMotionCommand(dX,(T)(0)*(*u),dt)*(*u) ) );
	}
	
	
	Mat<T> predictCovariance()	/*return the predicted covariance matrix.*/
	{		
		if(extended)
			G = ptrJMotion(*X, *u, dt);
				
			
		return (filterOn ? ( (!extended ? ((*A)*(*Sigma))* transpose(*A) : G*(*Sigma)*transpose(G) ) + *Q) : *Identity);
	}
	
		
	Mat<T> calculateKalmanGain()	/*return the Kalman Gain K = C*Sigma_p * (C*Sigma_p*C.T +R).inv */
	{
		if(filterOn)
		{			

			if(extended)
			{
				H = ptrJSensor(X_p, *u, dX,dt);
			
				Mat<T> temp(invGJ( H * Sigma_p * transpose(H) + *R) );
	#ifdef verbose_Jacobian				
				cout << "H :" << endl;
				H.afficher();
				cout << "sigma_p : " << endl;
				Sigma_p.afficher();
				cout << "inv GJ :" << endl;
				temp.afficher();
	#endif			
				return Sigma_p * transpose(H) * temp;
			}	
			

			return Sigma_p * transpose(*C) * invGJ( (*C) * Sigma_p * transpose(*C) + *R) ;
		}
		else
			return *Identity;
	}
	
		
	Mat<T> correctState()		/*update X */
	{		
	
		*_X = *X;
		
		if(filterOn)
		{
			*innovation = ( (*z) - (!extended ? (*C)*X_p  : ptrSensor(X_p, *u, dX, dt) ) ); 
			*X = X_p + K*(*innovation);
		}
		else
			*X = X_p;
	
		return *X;
	}
	
		
	Mat<T> correctCovariance()	/*update Sigma*/
	{
		if(filterOn)
		{			
			*Sigma = (*Identity - K* (!extended ? (*C) : H) ) *Sigma_p;
		}
		else
			*Sigma = *Identity;
	
		return *Sigma;
	}
	
	
	void state_Callback()		/* Update all... */
	{
		clock_t timer = clock();
		
		//callbackVO();
		//callback();
		
		if(noise)
		{
			for(int i=1;i<=nbr_state;i++)
			{
				R->set((T)rgenR->dev(), i,i);
			}
						
		}
			//TODO : DELETE ..... : ..... Normal measurement noise simulation...
		
		
		time += dt;
		
		if( extended && (ptrMotion == NULL || ptrSensor == NULL || ptrJMotion == NULL || ptrJSensor == NULL) )
		{
			//~EKF();
			throw("ERREUR : les fonctions ne sont pas initialisées...");
		}		
		
		
		X_p = predictState();		
		Sigma_p = predictCovariance();	
		K = calculateKalmanGain();	

		correctState();
		correctCovariance();	
#define K_gain_debug							 	
#ifdef K_gain_debug		
		cout << "Predicted State :" << endl;
		X_p.afficher();
		cout << "Predicted Covariance : " << endl;
		Sigma_p.afficher();
		cout << "Kalman Gain : " << endl;	
		K.afficher();
		cout << "Innovation : " << endl;
		innovation->afficher();
		cout << "Updated State : " << endl;
		X->afficher();
		cout << "Updated Covariance : " << endl;
		Sigma->afficher();		
#endif		
		cout << " L'execution de l'EKF a prise : " << (float)(clock()-timer)/CLOCKS_PER_SEC << " secondes." << endl;
	}
	
	void measurement_Callback(const Mat<T>& measurements, const Mat<T>& dX_, bool measure = false)
	{
		if( extended && (ptrMotion == NULL || ptrSensor == NULL || ptrJMotion == NULL || ptrJSensor == NULL) )
		{
			//~EKF();
			throw("ERREUR : les fonctions ne sont pas initialisées...");
		}
		
		dX = dX_;
		
		*z = (!extended || measure ? measurements : ptrSensor(measurements,*u, dX, dt) );	
		
	}
	
	void measurement_Callback(const Mat<T>& measurements)
	{
		if( extended && (ptrMotion == NULL || ptrSensor == NULL || ptrJMotion == NULL || ptrJSensor == NULL) )
		{
			//~EKF();
			throw("ERREUR : les fonctions ne sont pas initialisées...");
		}
		
		
		*z = (!extended ? measurements : ptrSensor(*X,*u, dX, dt) );
		//*z = (*C)*(*X);				
	}
	
	
	void setCommand(const Mat<T>& u)
	{
		*(this->u) = u;
	}
	
	void callbackVO()
	{
		//---------------------------------------------	
		//---------------------------------------------
		//update the velocity for EKF SEMI-DENSE VO :
		float mu = 0;
		float sigmaL = 1;
		float sigmaW = 1;
		float sigma = 1;	
		NormalRand genNR(mu,sigma, (long int)1028364098765);
		
		Mat<T> increment((T)0,12,1);
		increment.set( (T)sigmaL*sigmaW*genNR.dev(), 7,1);
		increment.set( (T)sigmaL*sigmaW*genNR.dev(), 8,1);
		increment.set( (T)sigmaL*sigmaW*genNR.dev(), 9,1);
		increment.set( (T)sigmaL*genNR.dev(), 10,1);
		increment.set( (T)sigmaL*genNR.dev(), 11,1);
		increment.set( (T)sigmaL*genNR.dev(), 12,1);
		
		//*X += increment;
		
		//---------------------------------------------	
		//---------------------------------------------
		//---------------------------------------------	
		//---------------------------------------------
		// Variance estimation : var(z)
		for(int i=n;i--;)	
		{
			if(i>0)	
				sampleval[i] = sampleval[i-1];
		}
		
		sampleval[0] = extract(X,1,1,nbr_obs,1)-(*z);
		
		Mat<T> estimatedVal((T)0,nbr_obs,1);
		for(int i=n;i--;)	estimatedVal+=sampleval[i]%sampleval[i];
		estimatedVal*= (T)(1.0/(n-1));
		//unbiased estimation
		
		for(int i=nbr_obs;i--;)	Sigma->set( (T)estimatedVal.get(i+1,1),6+i+1,6+i+1);
		Sigma->afficher();
		//---------------------------------------------	
		//---------------------------------------------
	
	}
	
	void callback()
	{
		//---------------------------------------------	
		//---------------------------------------------
		//update the velocity for EKF SEMI-DENSE VO :
		float mu = 0;		
		float sigma = 1;	
		NormalRand genNR(mu,sigma, (long int)1028364098765);
		
		X->set( X->get(2,1)+genNR.dev(), 2,1);
		
		//---------------------------------------------	
		//---------------------------------------------
	}
	
	
};

#endif
