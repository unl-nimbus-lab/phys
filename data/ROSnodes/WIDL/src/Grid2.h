#include "GridCell.h"
#include "MATv2/Mat.h"
#include <vector>

#define debug_lvl1
#define LOGGRID

class Grid
{
	private :
	float precX;
	// X precision --> size of a cell in x in meters;
	float precY;
	// Y precision --> size of a cell in y in meters;
	
	int posX0Cell;
	int posY0Cell;
	//record the position in the vector vector map of the origin.
	
	int maxYCellNumber;
	//record the maximum number of cell that has been inserted in a column of the map, so far.
	vector< vector<GridCell> > grid;
	
	//----------------------------
	//----------------------------
	//----------------------------
	//		LOGGING
	int idxLOG;		
	
	public :
	
	Grid() : precX(0.1f),precY(0.1f)
	{
		grid.insert(grid.begin(), vector<GridCell>());
		grid[0].insert(grid[0].begin(), GridCell((float)0,(float)0) );
		grid[0][0].setState(GCfree);
		//necessarily the current cell occupied by the robot is free.
		posX0Cell = 0;
		posY0Cell = 0;
		//record the position in the vector vector map of the origin.
		maxYCellNumber = 1;
		
		//------------------
		//		LOGGING
		idxLOG = 0;
	}
	
	Grid(float precX_, float precY_, int idxLOG_ = 0) : precX(precX_),precY(precY_), idxLOG(idxLOG_)
	{
		grid.insert(grid.begin(), vector<GridCell>());
		grid[0].insert(grid[0].begin(), GridCell((float)0,(float)0) );
		grid[0][0].setState(GCfree);
		//necessarily the current cell occupied by the robot is free.
		posX0Cell = 0;
		posY0Cell = 0;
		//record the position in the vector vector map of the origin.
		maxYCellNumber = 1;
	}
	
	~Grid()
	{
	
	}
	
	void addGridCell(int state, float r, float theta)	//r in meters, theta in radians.
	{
		float x = r*cos(theta);
		float y = r*sin(theta);
		
		int xbi = 0;
		int ybi = 0;
		// example : x = 0.26 : precX = 0.1 : xbi = 3 ; indeed : Cell0 : -0.5--0.5;Cell1:0.5--1.5;Cell2:1.5--2.5; 	Cell3:2.5--3.5;		
		if(x>0.0f)
		{
			while( xbi*precX +precX/2 < x)
			{
				xbi++;
			}
		}
		else
		{
			while( xbi*precX -precX/2 > x)
			{
				xbi--;
			}
		}
		
		if(y>0.0f)
		{
			while( ybi*precY +precY/2 < y)
			{
				ybi++;
			}
		}
		else
		{
			while( ybi*precY -precY/2 > y)
			{
				ybi--;
			}
		}
		
		//now we are sure of the index that we want to register this point within the map, from 0 included.
		//Do we have to add cells ?
		bool addCellX = false;
		bool addCellY = false;
		
		if(xbi>0)
		{
			if(xbi > ((grid.size()-1)-posX0Cell))
				addCellX = true;
		}
		else
		{
			if( (xbi+posX0Cell) < 0)
				addCellX = true;
		}
		
		if(ybi>0)
		{
			if(ybi > (maxYCellNumber-posY0Cell))
				addCellY = true;
		}
		else
		{
			if( (ybi+posY0Cell) < 0)
				addCellY = true;
		}
		
		
		if(addCellX)
		{
			if(xbi>0)
			{
				//let's add some columns :) !!
				int nbrFill = 0;
				float coverXMax = ((grid.size()-1)-posX0Cell)*precX+precX/2;
				//then we must fill in until coverXMax > x.
				
				while( coverXMax + nbrFill*precX < x)
					nbrFill++;
					
				//now we know how many cells are to be inserted in x. Let's insert those :
				float xb = grid[grid.size()-1][0].getXb();
				for(int i=0;i<nbrFill;i++)
				{
					xb += precX;
					
					grid.insert(grid.end(), vector<GridCell>());
					//we insert those at the end since its for x positive.
					
					//and we have to put at least the number of rows that is necessary ^^":
					float yb = -posY0Cell*precY;
					//float yb = grid[grid.size()-1][grid[grid.size()-1].size()-1].
					for(int j=0;j<maxYCellNumber;j++)
					{
						
						//--------------------------------------
						//What about the state of that new Cell ? for now, let's try to put them free by default...
						int state = GCfree;
						//--------------------------------------
						
						grid[grid.size()-1].insert( grid[grid.size()-1].end(), GridCell(state,xb,yb) );
						//we insert those at the end since its for y growing from scratch.
						//and it is relevant to notice that we insert those in the last columns, since it is for x growing positive.
						
						// keep in mind that we will change the state of that last one that is inserted 
						//since it is the one which has been scanned as a hurdle and that is why we can add it to the grid.
						yb += precY;
					}	
				}
				
				//no need to update the position of the origin, it hasn't moved.
				//What about the rows now ? see below.
				
#ifdef	debug_lvl1
				cout << " X POSITIVE : nbrFil = " << nbrFill << endl;
#endif	
			}
			else
			{
				//let's add some columns :) !!
				int nbrFill = 0;
				float coverXMin = -posX0Cell*precX-precX/2;
				//then we must fill in until coverXMin < x.
				
				while( coverXMin - nbrFill*precX > x)
					nbrFill++;
					
				//now we know how many cells are to be inserted in x. Let's insert those :
				float xb = grid[0][0].getXb();
				//we begin at the first one which the lowest barycentre...
				//but we want to go lower :
				for(int i=0;i<nbrFill;i++)
				{
					xb -= precX;
					
					grid.insert(grid.begin(), vector<GridCell>());
					//we insert those at the beginning since its for x negative.
					
					//and we have to put at least the number of rows that is necessary ^^":
					float yb = -posY0Cell*precY;
					
					for(int j=0;j<maxYCellNumber;j++)
					{
						
						//--------------------------------------
						//What about the state of that new Cell ? for now, let's try to put them free by default...
						int state = GCfree;
						//--------------------------------------
						
						grid[0].insert( grid[0].end(), GridCell(state,xb,yb) );
						//we insert those at the end since its for y growing from scratch.
						//and it is relevant to notice that we insert those in the first columns, since it is for x growing negatively.
						
						// keep in mind that we will change the state of that last one that is inserted 
						//since it is the one which has been scanned as a hurdle and that is why we can add it to the grid.
						yb += precY;
					}	
				}
				//need to update the position of the origin, it has moved.
				posX0Cell+=nbrFill;
				//What about the rows now ? see below.
				
#ifdef	debug_lvl1
				cout << " X NEGATIVE : nbrFil = " << nbrFill << endl;
#endif	
			}
					
			//What about this new cell ? see at the whole end.
			
		}
		
		
		//About the rows :
		if(addCellY)
		{
			if(ybi>0)
			{
				//let's add some rows :) !!
				int nbrFill = 0;
				float coverYMax = ((maxYCellNumber-1)-posY0Cell)*precY+precY/2;
				//then we must fill in until coverYMax > y.
				
				while( coverYMax + nbrFill*precY < y)
					nbrFill++;
					
				//now we know how many cells are to be inserted in every columns. Let's insert those :
				for(int i=grid.size();i--;)
				{
					float xb = grid[i][0].getXb();
					
					//We have to insert the correct number of cells to every columns, at the end since it is for y growing positively :
					float yb = coverYMax-precY/2;
					for(int j=0;j<nbrFill;j++)
					{
						//new cells with a barycentre which extends the covering :
						yb += precY;
						//--------------------------------------
						//What about the state of that new Cell ? for now, let's try to put them free by default...
						int state = GCfree;
						//--------------------------------------
						
						grid[i].insert( grid[i].end(), GridCell(state,xb,yb) );
						//we insert those at the end since its for y growing positively, not from scratch this one.
						
						// keep in mind that we will change the state of that last one that is inserted 
						//since it is the one which has been scanned as a hurdle and that is why we can add it to the grid.
					}	
				}
				
				//no need to update the position of the origin, it hasn't moved.
				// but the number of cells on rows has had a growth :
				maxYCellNumber += nbrFill;
				//What about the rows now ? see below.
				
#ifdef	debug_lvl1
				cout << " Y POSITIVE : nbrFil = " << nbrFill << endl;
#endif	
			}
			else
			{
				//let's add some rows :) !!
				int nbrFill = 0;
				float coverYMin = (-posY0Cell)*precY-precY/2;
				//then we must fill in until coverYMin < y.
				
				while( coverYMin - nbrFill*precY > y)
					nbrFill++;
					
				//now we know how many cells are to be inserted in every columns. Let's insert those :
				for(int i=grid.size();i--;)
				{
					float xb = grid[i][0].getXb();
					
					//We have to insert the correct number of cells to every columns, at the beginning since it is for y growing negatively :
					float yb = coverYMin+precY/2;
					for(int j=0;j<nbrFill;j++)
					{
						//new cells with a barycentre which extends the covering :
						yb -= precY;
						//--------------------------------------
						//What about the state of that new Cell ? for now, let's try to put them free by default...
						int state = GCfree;
						//--------------------------------------
						
						grid[i].insert( grid[i].begin(), GridCell(state,xb,yb) );
						//we insert those at the beginning since it's for y growing negatively, not from scratch this one.
						
						// keep in mind that we will change the state of that last one that is inserted 
						//since it is the one which has been scanned as a hurdle and that is why we can add it to the grid.
					}	
				}
				
				//need to update the position of the origin, it has moved.
				posY0Cell += nbrFill;
				// and the number of cells on rows has had a growth :
				maxYCellNumber += nbrFill;
				//What about the rows now ? see below.
				
#ifdef	debug_lvl1
				cout << " Y POSITIVE : nbrFil = " << nbrFill << endl;
#endif		
			}
					
			//What about this new cell ? see at the whole end.
			
		}
		

		//--------------------------------------------------------------		
		//--------------------------------------------------------------		
		//--------------------------------------------------------------
		//SO, let us tackle, at last, this new cell,
		// now that we know that it is mapped within the grid :
		grid[xbi+posX0Cell][ybi+posY0Cell].setState(state);
		//--------------------------------------------------------------		
		//--------------------------------------------------------------		
		//--------------------------------------------------------------
	}
	
	void addGridCell( float r, float theta)	//r in meters, theta in radians.
	{
		float x = r*cos(theta);
		float y = r*sin(theta);
		
		int xbi = 0;
		int ybi = 0;
		// example : x = 0.26 : precX = 0.1 : xbi = 3 ; indeed : Cell0 : -0.5--0.5;Cell1:0.5--1.5;Cell2:1.5--2.5; 	Cell3:2.5--3.5;		
		if(x>0.0f)
		{
			while( xbi*precX +precX/2 < x)
			{
				xbi++;
			}
		}
		else
		{
			while( xbi*precX -precX/2 > x)
			{
				xbi--;
			}
		}
		
		if(y>0.0f)
		{
			while( ybi*precY +precY/2 < y)
			{
				ybi++;
			}
		}
		else
		{
			while( ybi*precY -precY/2 > y)
			{
				ybi--;
			}
		}
		
		//now we are sure of the index that we want to register this point within the map, from 0 included.
		//Do we have to add cells ?
		bool addCellX = false;
		bool addCellY = false;
		
		if(xbi>0)
		{
			if(xbi > ((grid.size()-1)-posX0Cell))
				addCellX = true;
		}
		else
		{
			if( (xbi+posX0Cell) < 0)
				addCellX = true;
		}
		
		if(ybi>0)
		{
			if(ybi > (maxYCellNumber-posY0Cell))
				addCellY = true;
		}
		else
		{
			if( (ybi+posY0Cell) < 0)
				addCellY = true;
		}
		
		
		if(addCellX)
		{
			if(xbi>0)
			{
				//let's add some columns :) !!
				int nbrFill = 0;
				float coverXMax = ((grid.size()-1)-posX0Cell)*precX+precX/2;
				//then we must fill in until coverXMax > x.
				
				while( coverXMax + nbrFill*precX < x)
					nbrFill++;
					
				//now we know how many cells are to be inserted in x. Let's insert those :
				float xb = grid[grid.size()-1][0].getXb();
				for(int i=0;i<nbrFill;i++)
				{
					xb += precX;
					
					grid.insert(grid.end(), vector<GridCell>());
					//we insert those at the end since its for x positive.
					
					//and we have to put at least the number of rows that is necessary ^^":
					float yb = -posY0Cell*precY;
					//float yb = grid[grid.size()-1][grid[grid.size()-1].size()-1].
					for(int j=0;j<maxYCellNumber;j++)
					{
						
						//--------------------------------------
						//What about the state of that new Cell ? for now, let's try to put them free by default...
						int state = GCfree;
						//--------------------------------------
						
						grid[grid.size()-1].insert( grid[grid.size()-1].end(), GridCell(state,xb,yb) );
						//we insert those at the end since its for y growing from scratch.
						//and it is relevant to notice that we insert those in the last columns, since it is for x growing positive.
						
						// keep in mind that we will change the state of that last one that is inserted 
						//since it is the one which has been scanned as a hurdle and that is why we can add it to the grid.
						yb += precY;
					}	
				}
				
				//no need to update the position of the origin, it hasn't moved.
				//What about the rows now ? see below.
				
#ifdef	debug_lvl1
				cout << " X POSITIVE : nbrFil = " << nbrFill << endl;
#endif	
			}
			else
			{
				//let's add some columns :) !!
				int nbrFill = 0;
				float coverXMin = -posX0Cell*precX-precX/2;
				//then we must fill in until coverXMin < x.
				
				while( coverXMin - nbrFill*precX > x)
					nbrFill++;
					
				//now we know how many cells are to be inserted in x. Let's insert those :
				float xb = grid[0][0].getXb();
				//we begin at the first one which the lowest barycentre...
				//but we want to go lower :
				for(int i=0;i<nbrFill;i++)
				{
					xb -= precX;
					
					grid.insert(grid.begin(), vector<GridCell>());
					//we insert those at the beginning since its for x negative.
					
					//and we have to put at least the number of rows that is necessary ^^":
					float yb = -posY0Cell*precY;
					
					for(int j=0;j<maxYCellNumber;j++)
					{
						
						//--------------------------------------
						//What about the state of that new Cell ? for now, let's try to put them free by default...
						int state = GCfree;
						//--------------------------------------
						
						grid[0].insert( grid[0].end(), GridCell(state,xb,yb) );
						//we insert those at the end since its for y growing from scratch.
						//and it is relevant to notice that we insert those in the first columns, since it is for x growing negatively.
						
						// keep in mind that we will change the state of that last one that is inserted 
						//since it is the one which has been scanned as a hurdle and that is why we can add it to the grid.
						yb += precY;
					}	
				}
				//need to update the position of the origin, it has moved.
				posX0Cell+=nbrFill;
				//What about the rows now ? see below.
				
#ifdef	debug_lvl1
				cout << " X NEGATIVE : nbrFil = " << nbrFill << endl;
#endif	
			}	
			
		}
		
		
		//About the rows :
		if(addCellY)
		{
			if(ybi>0)
			{
				//let's add some rows :) !!
				int nbrFill = 0;
				float coverYMax = ((maxYCellNumber-1)-posY0Cell)*precY+precY/2;
				//then we must fill in until coverYMax > y.
				
				while( coverYMax + nbrFill*precY < y)
					nbrFill++;
					
				//now we know how many cells are to be inserted in every columns. Let's insert those :
				for(int i=grid.size();i--;)
				{
					float xb = grid[i][0].getXb();
					
					//We have to insert the correct number of cells to every columns, at the end since it is for y growing positively :
					float yb = coverYMax-precY/2;
					for(int j=0;j<nbrFill;j++)
					{
						//new cells with a barycentre which extends the covering :
						yb += precY;
						//--------------------------------------
						//What about the state of that new Cell ? for now, let's try to put them free by default...
						int state = GCfree;
						//--------------------------------------
						grid[i].insert( grid[i].end(), GridCell(state,xb,yb) );
						//we insert those at the end since its for y growing positively, not from scratch this one.
						
						// keep in mind that we will change the state of that last one that is inserted 
						//since it is the one which has been scanned as a hurdle and that is why we can add it to the grid.
					}	
				}
				
				//no need to update the position of the origin, it hasn't moved.
				// but the number of cells on rows has had a growth :
				maxYCellNumber += nbrFill;
				//What about the rows now ? see below.
				
#ifdef	debug_lvl1
				cout << " Y POSITIVE : nbrFil = " << nbrFill << endl;
#endif	
			}
			else
			{
				//let's add some rows :) !!
				int nbrFill = 0;
				float coverYMin = (-posY0Cell)*precY-precY/2;
				//then we must fill in until coverYMin < y.
				
				while( coverYMin - nbrFill*precY > y)
					nbrFill++;
					
				//now we know how many cells are to be inserted in every columns. Let's insert those :
				for(int i=grid.size();i--;)
				{
					float xb = grid[i][0].getXb();
					
					//We have to insert the correct number of cells to every columns, at the beginning since it is for y growing negatively :
					float yb = coverYMin+precY/2;
					for(int j=0;j<nbrFill;j++)
					{
						//new cells with a barycentre which extends the covering :
						yb -= precY;
						//--------------------------------------
						//What about the state of that new Cell ? for now, let's try to put them free by default...
						int state = GCfree;
						//--------------------------------------
						grid[i].insert( grid[i].begin(), GridCell(state,xb,yb) );
						//we insert those at the beginning since it's for y growing negatively, not from scratch this one.
						
						// keep in mind that we will change the state of that last one that is inserted 
						//since it is the one which has been scanned as a hurdle and that is why we can add it to the grid.
					}	
				}
				
				//need to update the position of the origin, it has moved.
				posY0Cell += nbrFill;
				// and the number of cells on rows has had a growth :
				maxYCellNumber += nbrFill;
				//What about the rows now ? see below.
				
#ifdef	debug_lvl1
				cout << " Y POSITIVE : nbrFil = " << nbrFill << endl;
#endif		
			}
					
			
		}
		
	}
	
	void setGridCellStatePolar(int state, float r, float theta)
	{
		addGridCell(state,r,theta);
	}
	
	void setGridCellStateCart(int state, float x, float y)
	{
		addGridCell(state, sqrt(x*x+y*y), atan2(y,x));
	}
	
	void setGridCellPPolar( float occP, float r, float theta)
	{
		setGridCellPCart( occP, r*cos(theta), r*sin(theta));
	}
	
	void setGridCellPCart( float occP, float x, float y)
	{
		addGridCell(sqrt(x*x+y*y), atan2(y,x));
		
		
		int xbi = 0;
		int ybi = 0;
		
		if(x>0.0f)
		{
			while( xbi*precX +precX/2 < x)
			{
				xbi++;
			}
		}
		else
		{
			while( xbi*precX -precX/2 > x)
			{
				xbi--;
			}
		}
		
		if(y>0.0f)
		{
			while( ybi*precY +precY/2 < y)
			{
				ybi++;
			}
		}
		else
		{
			while( ybi*precY -precY/2 > y)
			{
				ybi--;
			}
		}
		
		grid[xbi+posX0Cell][ybi+posY0Cell].setOccP(occP);
		
	}
	
	
	void setGridCellLPolar( float occL, float r, float theta)
	{
		setGridCellLCart( occL, r*cos(theta), r*sin(theta));
	}
	
	void setGridCellLCart( float occL, float x, float y)
	{
		addGridCell(sqrt(x*x+y*y), atan2(y,x));
		
		int xbi = 0;
		int ybi = 0;
		
		if(x>0.0f)
		{
			while( xbi*precX +precX/2 < x)
			{
				xbi++;
			}
		}
		else
		{
			while( xbi*precX -precX/2 > x)
			{
				xbi--;
			}
		}
		
		if(y>0.0f)
		{
			while( ybi*precY +precY/2 < y)
			{
				ybi++;
			}
		}
		else
		{
			while( ybi*precY -precY/2 > y)
			{
				ybi--;
			}
		}
		
		grid[xbi+posX0Cell][ybi+posY0Cell].setOccL(occL);
	}

	inline GridCell* getCell( float x, float y)
	{
		//Verification of the data ^^""
		addGridCell(sqrt(x*x+y*y),atan2(y,x));
		
		int xbi = 0;
		int ybi = 0;
		
		if(x>0.0f)
		{
			while( xbi*precX +precX/2 < x)
			{
				xbi++;
			}
		}
		else
		{
			while( xbi*precX -precX/2 > x)
			{
				xbi--;
			}
		}
		
		if(y>0.0f)
		{
			while( ybi*precY +precY/2 < y)
			{
				ybi++;
			}
		}
		else
		{
			while( ybi*precY -precY/2 > y)
			{
				ybi--;
			}
		}
		
		
		return &(grid[xbi+posX0Cell][ybi+posY0Cell]);
		
	}

	float inverse_sensor_model(const Mat<float>& tempGlobalPose, const Mat<float>& EKFPose, const Mat<float>& EKFPoseCovar, float l0 = 0.5f)
	{
		//TODO : implementation ...
		//return 0.6f;
		float occP = 0.6f;
		return l0+log( occP/(1.0f-occP) );
	}
	
	
	void update( /*state*/const Mat<float>& EKFPose, /*CovarianceMatrix*/const Mat<float>&EKFPoseCovar, /*observations*/const vector<Mat<float> >& pointsCartesianPose, const vector<Mat<float> >& pointsPolarPose, float beta = (float)3.6e-3, float alpha = (float)0.1f)
	// those observations are the ones given in the current frame.
	{
	 	int nbrPoints = pointsCartesianPose.size();
	 	Mat<float> tempGlobalPose(2,1);
	 	Mat<float> offset( extract(EKFPose, 1,1, 2,1) );
	 	Mat<float> R2Global(rot2D( -EKFPose.get(3,1))); 
	 	
	 	float tempL;
	 	float l0 = 0.5f;
	 	
	 	for(int k=nbrPoints;k--;)
	 	{
	 		//for each points :
	 		tempGlobalPose = R2Global * (pointsCartesianPose[k]-offset);
	 		
	 		//---------------------
	 		//	L computation :
	 		//tempL = getCell(tempGlobalPose.get(1,1), tempGlobalPose.get(2,1))->getOccL() + inverse_sensor_model(tempGlobalPose, EKFPose, EKFPoseCovar, l0) - l0;
	 		tempL = getCell(tempGlobalPose.get(1,1), tempGlobalPose.get(2,1))->getOccL() + inverse_sensor_model(tempGlobalPose, EKFPose, EKFPoseCovar, l0) - l0;
	 		//---------------------
	 		
	 		setGridCellLCart( tempL,tempGlobalPose.get(1,1), tempGlobalPose.get(2,1) );
	 	} 
	 	
	 	ROS_INFO("GRID %d : %d x %d", idxLOG, grid.size(),maxYCellNumber);
#ifdef LOGGRID	
				//Gestion ecriture dans un fichier :
				stringstream spath;
				spath << "/home/kevidena/ROS/sandbox/WIDL/src/logGRID-";
				spath << idxLOG;
				spath << ".txt";
				string filepathgrid( spath.str().c_str() );
				FILE* loggrid = fopen(filepathgrid.c_str(), "w+");
				if(loggrid == NULL)
				{
					cout << "ERROR : cannot open the file LOGGRID." << endl;
					exit(1);
				}
				else
					cout << "File opened LOGGRID." << endl;
			
				//------------------------------------------
				//------------------------------------------
			
				//Ecriture  :
				for(int i=0;i<grid.size();i++)	
				{
					for(int j=0;j<grid[i].size();j++)
					{
						//if( grid[i][j].getState() == GCoccupied)
						if( true || grid[i][j].getOccP() >= 0.8)
						{
							stringstream s;
							s << grid[i][j].getXb() << " | " << grid[i][j].getYb() << " | " << i << " | " << j << " | " << grid[i][j].getOccP() << " | " << grid[i][j].getOccL() ; 
							s << endl;
							fputs( s.str().c_str(), loggrid);
						}
					}
				}
				//------------------------------------
				//Fermeture du fichier :
				if(fclose(loggrid) == EOF)
				{
					cout << "ERROR : cannot close the file LOGGRID." << endl;
					exit(1);
				}
				else
					cout << "File closed." << endl;
			
				//--------------------------------
				//--------------------------------
#endif	
	}
	
	void init( /*state*/const Mat<float>& EKFPose, /*CovarianceMatrix*/const Mat<float>&EKFPoseCovar, /*observations*/const vector<Mat<float> >& pointsCartesianPose, const vector<Mat<float> >& pointsPolarPose)
	// those observations are the ones given in the current frame.
	{
	
		//-------------------------------
		//	CLEARS THE MAP
		//----------------------------------
		for(int i=grid.size();i--;)
		{
			for(int j=maxYCellNumber+1;j--;)
			{
				grid[i][j].setState(GCfree);
			}
		}
		//except the current location of the sensor.
		grid[posX0Cell][posY0Cell].setState(GCoccupied);
		
		//-----------------------------------
		//-----------------------------------
		//-----------------------------------				
		
		
	 	int nbrPoints = pointsCartesianPose.size();
	 	Mat<float> tempGlobalPose(2,1);
	 	Mat<float> offset( extract(EKFPose, 1,1, 2,1) );
	 	Mat<float> R2Global(rot2D( -EKFPose.get(3,1))); 
	 	
	 	float tempL;
	 	float l0 = 0.5f;
	 	
	 	for(int k=nbrPoints;k--;)
	 	{
	 		//for each points :
	 		tempGlobalPose = R2Global * (pointsCartesianPose[k]-offset);
	 		
	 		//---------------------
	 		addGridCell( GCoccupied, tempGlobalPose.get(1,1), tempGlobalPose.get(2,1));
	 		//---------------------
	 	} 
	 	
	 	ROS_INFO("GRID %d : %d x %d", idxLOG, grid.size(),maxYCellNumber);
	 }
	
	
	int getHeight()
	{
		return maxYCellNumber+1;
	}
	
	int getWidth()
	{
		return grid.size();
	}
	
	float getOccP(int i, int j)
	{
		float r=0.0f;
		if(i<grid.size() && j<maxYCellNumber+1)
			r = grid[i][j].getOccP();
			
		return r;
	}
	
	float getXb(int i, int j)
	{
		float r=0.0f;
		if(i<grid.size() && j<maxYCellNumber+1)
			r = grid[i][j].getXb();
			
		return r;
	}
	
	float getYb(int i, int j)
	{
		float r=0.0f;
		if(i<grid.size() && j<maxYCellNumber+1)
			r = grid[i][j].getYb();
			
		return r;
	}
	
	Mat<int> getOrigin()
	{
		Mat<int> r(2,1);
		r.set( posX0Cell, 1,1);
		r.set( posY0Cell, 2,1);
		return r;
	}
	
	Mat<float> rot2D(float angle)
	{
		Mat<float> r(2,2);
		r.set(cos(angle), 1,1);
		r.set(cos(angle), 2,2);
		r.set(-sin(angle), 1,2);
		r.set(sin(angle), 2,1);
		return r;
	}

};
