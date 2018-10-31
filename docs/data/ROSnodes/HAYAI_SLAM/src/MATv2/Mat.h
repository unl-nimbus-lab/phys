#ifndef MAT_H
#define MAT_H

#include <iostream>
#include <limits>
#include <cstdlib>
#include <math.h>
#include <time.h>
#include <assert.h>

#ifdef OPENCV_USE
#include <opencv2/opencv.hpp>
#endif

#define PI 3.1415926535
#define maxi 10

using namespace std;

/*
#define throw(message)\
{cout << "ERROR : " << message << endl << " in file " << __FILE__ << " at line " << __LINE__ << endl;\
exit(1);}*/



template<typename T>	/*pas de point virgule en fin de ligne...*/
class Mat
{
    protected :

    /*Beware : Mat<T> : [0 ; m_line-1]x[0 ; m_column] */
        int m_line;	/*m_line = length(Mat<T>,1) +1*/
        int m_column;	/*idem...*/                

    public :
	T** mat;

        inline Mat();
        inline Mat(const Mat<T>& m);
        inline Mat(const Mat<T>& m, T oValue, int d_line, int d_column, int line, int column);	/*initialise les autres valeurs à oValue*/
        inline Mat(const Mat<T>& m, int delLine[], int cLine, int delColumn[], int cColumn);
        inline Mat(int line, int column, char mode);
        inline Mat( int line, int column);
        inline Mat( T value, int line, int column);
        void copy(const Mat<T>& m);

        Mat<T>& operator=(const Mat<T>& m);

        ~Mat();

        int getLine() const;
        int getColumn() const;

        inline T get(int line, int column) const;
        inline void set(T value, int line, int column);
        void afficher();

        void swap(int i1, int j1, int i2, int j2);
        void swapL(int i1, int i2);
        void swapC(int j1, int j2);

};


template<typename T>	/*pas de point virgule en fin de ligne...*/
T detRec(const Mat<T>& m)
{
    if(m.getLine()==2)
        return (m.get(1,1)*m.get(2,2)-m.get(2,1)*m.get(1,2));
    else if(m.getLine()==1)
        return m.get(1,1);
    else
    {
        int line = m.getLine();
        /*int column = m.getColumn();*/
        T sum = 0;

        /* developpement sur la première colonne...*/

        for(int i=1;i<=line;i++)
        {
            int tabL[1] = {i};
            int tabC[1] = {1};
            Mat<T> s_m(m, tabL, 1, tabC, 1);

            if((i+1)%2)
                sum -= m.get(i,1)*detRec(s_m);
            else
                sum += m.get(i,1)*detRec(s_m);
        }

        return sum;
    }
}


template<typename T>
T computeDeterminant(const Mat<T>& m)
{
	T det;
	
	if(m.getLine() == m.getColumn())
        {            
            det = detRec(m);
        }
        else
        {           
            det = detRec(transpose(m)*m);

            cout << "Impossible de calculer le determinant de cette matrice normalement : elle n'est pas carree." << endl;
            cout << "PSEUDOINVERSE" << endl;
        }

	return det;
}
template<typename T>
bool isnanM(Mat<T>* m);
template<typename T>
bool isnanM(Mat<T> m)
{
    bool ret = false;

    for(int i=m.getLine();i--;)
    {
        for(int j=m.getColumn();j--;)
        {
            ret = (m.get(i+1,j+1) != m.get(i+1,j+1));
            if(ret)
            {
                i=0;
                j=0;
                break;
            }
        }
    }

    return ret;
}
template<typename T>
bool operator==(const Mat<T>& a, const Mat<T>& b);
template<typename T>
bool operator>=(const Mat<T>& a, const Mat<T>& b);
template<typename T>
bool operator<=(const Mat<T>& a, const Mat<T>& b);
template<typename T>
bool operator>(const Mat<T>& a, const Mat<T>& b);
template<typename T>
bool operator<(const Mat<T>& a, const Mat<T>& b);
template<typename T>	/*pas de point virgule en fin de ligne...*/
Mat<T> operator*(const Mat<T>& a, const Mat<T>& b);
template<typename T>	/*pas de point virgule en fin de ligne...*/
Mat<T> operator*=(Mat<T>& a, const Mat<T>& b);
template<typename T>	/*pas de point virgule en fin de ligne...*/
Mat<T> product(Mat<T>* a, Mat<T>* b);
template<typename T>	/*pas de point virgule en fin de ligne...*/
inline Mat<T> operator*(const T& v, const Mat<T>& a);
template<typename T>	/*pas de point virgule en fin de ligne...*/
inline Mat<T> operator*=(Mat<T>& a, const T& v);
template<typename T>	/*pas de point virgule en fin de ligne...*/
inline Mat<T> operator+(const Mat<T>& a, const Mat<T>& b);
template<typename T>	/*pas de point virgule en fin de ligne...*/
inline Mat<T> operator+=( Mat<T>& a, const Mat<T>& b);
template<typename T>	/*pas de point virgule en fin de ligne...*/
inline Mat<T> operator-(const Mat<T>& a, const Mat<T>& b);
template<typename T>	/*pas de point virgule en fin de ligne...*/
inline Mat<T> operator-=(Mat<T>& a, const Mat<T>& b);
template<typename T>	/*pas de point virgule en fin de ligne...*/
inline Mat<T> operator%(const Mat<T>& a, const Mat<T>& b);		/*usage du modulo comme produit coefficient à coefficient*/
template<typename T>	/*pas de point virgule en fin de ligne...*/
Mat<T> operatorL(const Mat<T>& a, const Mat<T>& b);
template<typename T>	/*pas de point virgule en fin de ligne...*/
Mat<T> Line( const Mat<T>& a, int ind);
template<typename T>	/*pas de point virgule en fin de ligne...*/
Mat<T> operatorC(const Mat<T>& a, const Mat<T>& b);
template<typename T>	/*pas de point virgule en fin de ligne...*/
Mat<T> Col( const Mat<T>& a, int ind);
template<typename T>	/*pas de point virgule en fin de ligne...*/
Mat<T> Cola( const Mat<T> a, int ind);
template<typename T>	/*pas de point virgule en fin de ligne...*/
inline Mat<T> extract(const Mat<T>& m, int ib, int jb, int ie, int je);
template<typename T>	/*pas de point virgule en fin de ligne...*/
inline Mat<T> extract(Mat<T>* m, int ib, int jb, int ie, int je);
template<typename T>	/*pas de point virgule en fin de ligne...*/
inline Mat<T> transpose(const Mat<T>& a);
template<typename T>	/*pas de point virgule en fin de ligne...*/
inline void transpose(Mat<T>* a);
template<typename T>	/*pas de point virgule en fin de ligne...*/
Mat<T> inv(const Mat<T>& a);
template<typename T>	/*pas de point virgule en fin de ligne...*/
Mat<T> inverseM(const Mat<T>& a);
template<typename T>	/*pas de point virgule en fin de ligne...*/
Mat<T> sum(const Mat<T>& a);
template<typename T>	/*pas de point virgule en fin de ligne...*/
Mat<T> log(const Mat<T>& a);
template<typename T>	/*pas de point virgule en fin de ligne...*/
Mat<T> logMat(Mat<T>* a);
template<typename T>	/*pas de point virgule en fin de ligne...*/
Mat<T> expMat( Mat<T>* a);
template<typename T>	/*pas de point virgule en fin de ligne...*/
T fabs_(T a);
template<typename T>	/*pas de point virgule en fin de ligne...*/
Mat<T> absM(const Mat<T>& a);
template<typename T>	/*pas de point virgule en fin de ligne...*/
Mat<T> sqrt(const Mat<T>& a);
template<typename T>	/*pas de point virgule en fin de ligne...*/
T dist( const Mat<T>& a, const Mat<T>& b, int method = 2);
template<typename T>	/*pas de point virgule en fin de ligne...*/
T atan21( T y, T x);
template<typename T>	/*pas de point virgule en fin de ligne...*/
Mat<T> atan2(const Mat<T>& y, const Mat<T>& x);

template<typename T>	/*pas de point virgule en fin de ligne...*/
T var(Mat<T> mat);
template<typename T>	/*pas de point virgule en fin de ligne...*/
T covar(Mat<T> a, Mat<T> b);
template<typename T>	/*pas de point virgule en fin de ligne...*/
T sigmoid( const T z);
template<typename T>	/*pas de point virgule en fin de ligne...*/
Mat<T> sigmoidM(const Mat<T>& z);
template<typename T>	/*pas de point virgule en fin de ligne...*/
Mat<T> sigmoidGradM( const Mat<T>& z);
template<typename T>	/*pas de point virgule en fin de ligne...*/
Mat<T> derive(Mat<T> m, int k, int l);
template<typename T>	/*pas de point virgule en fin de ligne...*/
Mat<T> pict2mat(const FILE* file);
template<typename T>	/*pas de point virgule en fin de ligne...*/
T detRec(const Mat<T>& m);
template<typename T>
Mat<T> crossProduct(const Mat<T>& a, const Mat<T>& b);

#ifdef OPENCV_USE
/*convertion des matrice de OPENCV en Mat<T> dans le tableau tab.*/
/*renvoi le nombre de matrice cotenue finalement dans tab, */
/*correspondant au nombre de channel de mat.*/

/*
template<typename T>	//pas de point virgule en fin de ligne...
int cv2Mat(const cv::Mat mat, Mat<T>* tab);
//Mat<T> cv2Mat( cv::Mat& img);
template<typename T>	//pas de point virgule en fin de ligne...
cv::Mat Mat2CV( Mat<T>* tab);

template<typename T>	//pas de point virgule en fin de ligne...
Mat<T> cv2Mat(const cv::Mat mat);
template<typename T>	//pas de point virgule en fin de ligne...
cv::Mat Mat2CV( Mat<T> tab);
*/
cv::Mat absM(const cv::Mat& a);
inline cv::Mat sum(const cv::Mat& a);
template<typename T>	//pas de point virgule en fin de ligne...
int cv2Mat(const cv::Mat mat, Mat<T>* r, Mat<T>* g, Mat<T>* b);
template<typename T>	//pas de point virgule en fin de ligne...
int cv2MatAt(const cv::Mat mat, Mat<T>* r, Mat<T>* g, Mat<T>* b);
template<typename T>	//pas de point virgule en fin de ligne...
int cv2Matp( cv::Mat mat, Mat<T>* r, Mat<T>* g, Mat<T>* b);
template<typename T>	//pas de point virgule en fin de ligne...
Mat<T> cv2Matp( cv::Mat mat);
template<typename T>	//pas de point virgule en fin de ligne...
cv::Mat Mat2cv(Mat<T>* r, Mat<T>* g, Mat<T>* b);
template<typename T>	//pas de point virgule en fin de ligne...
cv::Mat Mat2cvVec(Mat<T> r, Mat<T> g, Mat<T> b);
template<typename T>	//pas de point virgule en fin de ligne...
cv::Mat Mat2cvAt(Mat<T> r, Mat<T> g, Mat<T> b);
template<typename T>	//pas de point virgule en fin de ligne...
cv::Mat Mat2cvp(Mat<T> r, Mat<T> g, Mat<T> b);
cv::Mat extractCV(cv::Mat im,int x, int y, int nneigh);
cv::Mat absMp(cv::Mat mat);

void afficher(cv::Mat *im, cv::Mat *im1=NULL, cv::Mat *im2=NULL, bool cont = false, float factor = 1.0);
template<typename T>
void afficherMat( Mat<T>* im, Mat<T>* im1=NULL, Mat<T>* im2=NULL, bool cont = false, float factor = 1.0);

void afficher( string s, cv::Mat* im, cv::Mat* im1=NULL, cv::Mat* im2=NULL, bool cont = false, float factor = 1.0);
template<typename T>
void afficherMat( string s, Mat<T>* im, Mat<T>* im1=NULL, Mat<T>* im2=NULL, bool cont = false, float factor = 1.0);
template<typename T>
inline void displayInverseCodedFrame( string s, Mat<T>* frame, Mat<T>* invDepthMap, bool cont = false, float factor = 1.0);
#endif


template<typename T>	/*pas de point virgule en fin de ligne...*/
Mat<T> correlation(Mat<T> m, Mat<T> k);
template<typename T>	/*pas de point virgule en fin de ligne...*/
T max(Mat<T> mat);
template<typename T>
Mat<T> idmax(Mat<T> mat);
template<typename T>
Mat<T> idmin(Mat<T> mat);
template<typename T>	/*pas de point virgule en fin de ligne...*/
T mean(Mat<T> mat);
template<typename T>	/*pas de point virgule en fin de ligne...*/
Mat<T> meanC(Mat<T> mat);
template<typename T>	/*pas de point virgule en fin de ligne...*/
Mat<T> pooling(Mat<T> m, Mat<T> k, int type = 3);
template<typename T>	/*pas de point virgule en fin de ligne...*/
Mat<T> pooling(Mat<T>*	 m, Mat<T>* k, int type = 3);
template<typename T>	/*pas de point virgule en fin de ligne...*/
Mat<T> sym(Mat<T> m);
template<typename T>	/*pas de point virgule en fin de ligne...*/
Mat<T> convolution(Mat<T> m, Mat<T> k);
template<typename T>
Mat<T> conv(Mat<T> m, Mat<T> k);
template<typename T>	/*pas de point virgule en fin de ligne...*/
inline void homogeneousNormalization( Mat<T>* X);


/*--------------------------------------------*/
/*--------------------------------------------*/
/*fin des definitions des prototypes*/
/*--------------------------------------------*/
/*--------------------------------------------*/

template<typename T>	/*pas de point virgule en fin de ligne...*/
inline Mat<T>::Mat()
{
    m_line = 1;
    m_column = 1;

    mat = new T*[m_line];

    for(int i=0;i<=m_line-1;i++)
        mat[i] = new T[m_column];

}


/*---------------------------------------------*/

template<typename T>	/*pas de point virgule en fin de ligne...*/
inline Mat<T>::Mat(const Mat<T>& m)
{
    m_line = m.getLine();
    m_column = m.getColumn();
    
    mat = new T*[m_line];

    for(int i=m_line;i--;)
    {
        mat[i] = new T[m_column];

        for(int j=m_column;j--;)
        {
            mat[i][j] = m.get(i+1,j+1);
        }
    }
   
}

/*---------------------------------------------*/

template<typename T>	/*pas de point virgule en fin de ligne...*/
inline Mat<T>::Mat(const Mat<T>& m, T oValue, int d_line, int d_column, int line, int column)
{
    m_line = line;
    m_column = column;
        
    mat = new T*[m_line];

    for(int i=m_line;i--;)
        mat[i] = new T[m_column];


    if( m.getColumn()+d_column-1<=m_column && m.getLine()+d_line-1<=m_line )
    {
        for(int i=1;i<=m_line;i++)
        {
            for(int j=1;j<=m_column;j++)
            {
                if( i>=d_line && i<=d_line+m.getLine()-1 && j>=d_column && j<=d_column+m.getColumn()-1)
                    mat[i-1][j-1] = m.get(i-d_line+1,j-d_column+1);
                else
                    mat[i-1][j-1] = oValue;
            }
        }
		
    }
    else
    {
        cerr << "Attention : les parametres specifies ne permettent pas l'initialisation de la matrice : " << endl;
        cerr << "m = " << m.getLine() << " " << m.getColumn() << endl;
        cerr << d_line << " " << d_column << endl;
        cerr << m_line << " " << m_column << endl;
    }
}


/*---------------------------------------------*/


template<typename T>	/*pas de point virgule en fin de ligne...*/
inline Mat<T>::Mat(const Mat<T>& m, int delLine[], int cLine, int delColumn[], int cColumn)
{
    m_line = m.getLine() - cLine; /*sizeof(delLine);*/
    m_column = m.getColumn() - cColumn; /*sizeof(delColumn);*/

    int m_i = 1;
    int m_j = 1;

    if((m_line > 0) && (m_column > 0))
    {
        mat = new T*[m_line];

        for(int i=0;i<=m_line-1;i++)
            mat[i] = new T[m_column];

        for(int i=1;m_i<=m_line;i++)
        {
            bool continuer = true;
            int h = (int)(sizeof(delLine)/sizeof(int));

            for(int j=0;j<=h-1;j++)
            {
                if(i==delLine[j])
                    continuer = false;
            }

            if(continuer)
            {
                m_j = 1;

                for(int j=1;m_j<=m_column;j++)
                {
                    continuer = true;
                    int w = (int)(sizeof(delColumn)/sizeof(int));

                    for(int k=0;k<=w-1;k++)
                    {
                        if(j==delColumn[k])
                            continuer = false;
                    }

                    if(continuer)
                    {
                        mat[m_i-1][m_j-1] = m.get(i,j);
                        m_j++;
                    }
                }

                m_i++;
            }
        }


    }
    else
    {
        cerr << "ERREUR : les parametres specifies ne permettent pas l'initialisation de la matrice." << endl;
    }

}



/*---------------------------------------------*/


template<typename T>	/*pas de point virgule en fin de ligne...*/
inline Mat<T>::Mat( int line, int column)
{
    assert(line>0 && column>0);
    
    mat = new T*[line];

    for(int i=line;i--;)
        mat[i] = new T[column];

    m_line = line;
    m_column = column;
}

/*---------------------------------------------*/


template<typename T>	/*pas de point virgule en fin de ligne...*/
inline Mat<T>::Mat( int line, int column, char mode)
{
    mat = new T*[line];

    for(int i=line;i--;)
        mat[i] = new T[column];

    m_line = line;
    m_column = column;

    if(mode == 1)	//random
    {        
	srand(time(NULL));
        for(int i=m_line;i--;)
            for(int j=m_column;j--;)
            {
                this->set( (T) (-1)*(rand()%(int)maxi)+(rand()%(int)maxi), i+1,j+1);
            }
    }    
    else if(mode == 2)	//gaussian zero-centered sigma = maxi
    {
	srand(time(NULL));
        for(int i=m_line;i--;)
            for(int j=m_column;j--;)	this->set( (T) (rand()%(int)maxi), i+1,j+1);
    }

}



/*---------------------------------------------*/


template<typename T>	/*pas de point virgule en fin de ligne...*/
inline Mat<T>::Mat( T value, int line, int column)
{
    mat = new T*[line];

    for(int i=line;i--;)
    {
        mat[i] = new T[column];
    }
    
    for(int i=line;i--;)
    	for(int j=column;j--;)
            mat[i][j] = value;

    m_line = line;
    m_column = column;
}


/*---------------------------------------------*/


template<typename T>	/*pas de point virgule en fin de ligne...*/
Mat<T>::~Mat()
{
    for(int i=0;i<m_line;i++)
    {
        delete[] mat[i];
        //delete[] roundoffmat[i];
    }
    delete[] mat;
    //delete roundoffmat;

}



/*---------------------------------------------*/


template<typename T>	/*pas de point virgule en fin de ligne...*/
int Mat<T>::getLine() const
{
    return m_line;
}



/*---------------------------------------------*/


template<typename T>	/*pas de point virgule en fin de ligne...*/
int Mat<T>::getColumn() const
{
    return m_column;
}



/*---------------------------------------------*/


template<typename T>	/*pas de point virgule en fin de ligne...*/
inline T Mat<T>::get(int line, int column) const
{

    if(line <= m_line && column <= m_column && line >=1 && column >=1)
        return mat[line-1][column-1];

    return 0;
}



/*---------------------------------------------*/


template<typename T>	/*pas de point virgule en fin de ligne...*/
inline void Mat<T>::set(T value, int line, int column)
{
    if(line >= 1 && line <= m_line && column >= 1 && column <= m_column)
        mat[line-1][column-1] = value;
}



/*---------------------------------------------*/


template<typename T>	/*pas de point virgule en fin de ligne...*/
void Mat<T>::afficher()
{
    cout << "Matrice :" << endl;

        for(int i=1;i<=m_line;i++)
        {
            for(int j=1;j<=m_column;j++)
            {
                cout << "  " << this->get(i,j) ;
                //cout << "  " << ( fabs_(this->get(i,j)) >= epsilon*10e-20 ? this->get(i,j) : (T)0) ;
                //cout << "  " << roundoffmat[i-1][j-1] ;
            }

            cout << endl;
        }
}



/*---------------------------------------------*/


template<typename T>	/*pas de point virgule en fin de ligne...*/
Mat<T> operator*(const Mat<T>& a, const Mat<T>& b)
{

    if(a.getColumn() != b.getLine())
    {
        cerr << "Impossible d'operer la multiplication : mauvais formats de matrices.\n" << endl;
        cerr << "Format m1 : " << a.getLine() << " x " << a.getColumn() << "\t Fromat m2 : " << b.getLine() << " x " << b.getColumn() << endl;
        exit(EXIT_FAILURE);        
        return Mat<T>(1,1);
    }

    if( a.getLine() == b.getLine() && a.getColumn() == b.getColumn())
    {
    	Mat<T> aa(a),bb(b);
	return product(&aa,&bb);
    }
    else
    {
    Mat<T> r( a.getLine(), b.getColumn());
    
#define optim
#ifdef optim    
    Mat<T> tb(transpose(b));
#else
    Mat<T> tb(b);
    transpose(&tb);
#endif
    T temp = (T) 0;
    for(int i=r.getLine();i--;)
    {
        for(int j=r.getColumn();j--;)
        {
            temp = (T)0;            
            for(int k=a.getColumn();k--;)
            {
#ifdef optim
	        temp += a.mat[i][k]*tb.mat[j][k];
#else
                temp += a.mat[i][k]*b.mat[k][j];
#endif
            }

            r.set( temp, i+1, j+1);
            //r.mat[i][j] = temp;
        }
    }


    return r;
    }
    
}


template<typename T>	/*pas de point virgule en fin de ligne...*/
Mat<T> operator*=( Mat<T>& a, const Mat<T>& b)
{

    if(a.getColumn() != b.getLine())
    {
        cerr << "Impossible d'operer la multiplication : mauvais formats de matrices.\n" << endl;
        cerr << "Format m1 : " << a.getLine() << " x " << a.getColumn() << "\t Fromat m2 : " << b.getLine() << " x " << b.getColumn() << endl;
        exit(EXIT_FAILURE);        
        return Mat<T>(1,1);
    }

    if( a.getLine() == b.getLine() && a.getColumn() == b.getColumn())
    {
    	Mat<T> aa(a),bb(b);
	a = product(&aa,&bb);
	return a;
    }
    else
    {
    Mat<T> r( a.getLine(), b.getColumn());
    
#define optim
#ifdef optim    
    Mat<T> tb(transpose(b));
#else
    Mat<T> tb(b);
    transpose(&tb);
#endif
    T temp = (T) 0;
    for(int i=r.getLine();i--;)
    {
        for(int j=r.getColumn();j--;)
        {
            temp = (T)0;            
            for(int k=a.getColumn();k--;)
            {
#ifdef optim
	        temp += a.mat[i][k]*tb.mat[j][k];
#else
                temp += a.mat[i][k]*b.mat[k][j];
#endif
            }

            r.set( temp, i+1, j+1);
            //r.mat[i][j] = temp;
        }
    }

    a = r;
    return r;
    }
    
}

template<typename T>	/*pas de point virgule en fin de ligne...*/
Mat<T> product(Mat<T>* a, Mat<T>* b)
{

    if(a->getColumn() != b->getLine())
    {
        cerr << "Impossible d'operer la multiplication : mauvais formats de matrices.\n" << endl;
        cerr << "Format m1 : " << a->getLine() << " x " << a->getColumn() << "\t Fromat m2 : " << b->getLine() << " x " << b->getColumn() << endl;
        return Mat<T>(1,1);
    }

    Mat<T> r( a->getLine(), b->getColumn());
    transpose(b);

    for(int i=r.getLine();i--;)
    {
        for(int j=r.getColumn();j--;)
        {
            T temp = (T)0;
            for(int k=a->getColumn();k--;)
            {
                //cout << "i=" << i << " j=" << j << " k=" << k << endl;
	        temp += a->mat[i][k]*b->mat[j][k];
                //temp += a->get(i+1,k+1)*b->get(j+1,k+1);
            }

            r.set( temp, i+1, j+1);
        }
    }
	
    transpose(b);

    return r;
}


/*---------------------------------------------*/


template<typename T>	/*pas de point virgule en fin de ligne...*/
inline Mat<T> operator*(const T& v, const Mat<T>& a)
{
    Mat<T> r(a.getLine(),a.getColumn());

    for(int i=r.getLine();i--;)
    {
        for(int j=r.getColumn();j--;)
        {
            r.set( (T)(v*a.get(i+1,j+1)), i+1, j+1);
        }
    }


    return r;
}



template<typename T>	/*pas de point virgule en fin de ligne...*/
inline Mat<T> operator*=(Mat<T>& a, const T& v)
{


    for(int i=a.getLine();i--;)
    {
        for(int j=a.getColumn();j--;)
        {
            a.set( v*a.get(i+1,j+1) , i+1, j+1);
        }
    }


    return a;
}
/*---------------------------------------------*/


template<typename T>	/*pas de point virgule en fin de ligne...*/
inline Mat<T> operator+(const Mat<T>& a, const Mat<T>& b)
{
    if((a.getColumn() != b.getColumn()) || (a.getLine() != b.getLine()))
    {
        cerr << "Impossible d'operer l'addition : mauvais formats de matrices.\n" << endl;
        cerr << "Format m1 : " << a.getLine() << " x " << a.getColumn() << "\t Fromat m2 : " << b.getLine() << " x " << b.getColumn() << endl;
        exit(EXIT_FAILURE);                
        return Mat<T>(1,1);
    }

    Mat<T> r( a.getLine(), b.getColumn());

    for(int i=r.getLine();i--;)
    {
        for(int j=r.getColumn();j--;)
        {
            r.set( a.get(i+1,j+1)+b.get(i+1,j+1) , i+1, j+1);
        }
    }


    return r;
}

template<typename T>	/*pas de point virgule en fin de ligne...*/
inline Mat<T> operator+=(Mat<T>& a, const Mat<T>& b)
{
    if((a.getColumn() != b.getColumn()) || (a.getLine() != b.getLine()))
    {
        cerr << "Impossible d'operer l'addition : mauvais formats de matrices.\n" << endl;
        cerr << "Format m1 : " << a.getLine() << " x " << a.getColumn() << "\t Fromat m2 : " << b.getLine() << " x " << b.getColumn() << endl;
        exit(EXIT_FAILURE);                
        return Mat<T>(1,1);
    }


    for(int i=a.getLine();i--;)
    {
        for(int j=a.getColumn();j--;)
        {
            a.set( a.get(i+1,j+1)+b.get(i+1,j+1) , i+1, j+1);
        }
    }


    return a;
}


template<typename T>	/*pas de point virgule en fin de ligne...*/
Mat<T> add(Mat<T>* a, Mat<T>* b)
{
    if((a->getColumn() != b->getColumn()) || (a->getLine() != b->getLine()))
    {
        cerr << "Impossible d'operer l'addition : mauvais formats de matrices.\n" << endl;
        cerr << "Format m1 : " << a->getLine() << " x " << a->getColumn() << "\t Fromat m2 : " << b->getLine() << " x " << b->getColumn() << endl;
        return Mat<T>(1,1);
    }

    Mat<T> r( a->getLine(), b->getColumn());

    for(int i=r.getLine();i--;)
    {
        for(int j=r.getColumn();j--;)
        {
            r.mat[i][j] = a->mat[i][j]+b.mat[i][j];
        }
    }


    return r;
}

/*---------------------------------------------*/


template<typename T>	/*pas de point virgule en fin de ligne...*/
inline Mat<T> operator-(const Mat<T>& a, const Mat<T>& b)
{
    if((a.getColumn() != b.getColumn()) || (a.getLine() != b.getLine()))
    {
        cerr << "Impossible d'operer la soustraction : mauvais formats de matrices.\n" << endl;
        cerr << "Format m1 : " << a.getLine() << " x " << a.getColumn() << "\t Fromat m2 : " << b.getLine() << " x " << b.getColumn() << endl;
        exit(EXIT_FAILURE);
        return Mat<T>(1,1);
    }

    Mat<T> r(a.getLine(), b.getColumn());

    for(int i=r.getLine();i--;)
    {
        for(int j=r.getColumn();j--;)
        {
            r.set( a.get(i+1,j+1)-b.get(i+1,j+1) , i+1, j+1);
        }
    }


    return r;
}




template<typename T>	/*pas de point virgule en fin de ligne...*/
inline Mat<T> operator-=(Mat<T>& a, const Mat<T>& b)
{
    if((a.getColumn() != b.getColumn()) || (a.getLine() != b.getLine()))
    {
        cerr << "Impossible d'operer la soustraction : mauvais formats de matrices.\n" << endl;
        cerr << "Format m1 : " << a.getLine() << " x " << a.getColumn() << "\t Fromat m2 : " << b.getLine() << " x " << b.getColumn() << endl;
        exit(EXIT_FAILURE);                
        return Mat<T>(1,1);
    }


    for(int i=a.getLine();i--;)
    {
        for(int j=a.getColumn();j--;)
        {
            a.set( a.get(i+1,j+1)-b.get(i+1,j+1) , i+1, j+1);
        }
    }


    return a;
}


/*---------------------------------------------*/


template<typename T>	/*pas de point virgule en fin de ligne...*/
inline Mat<T> operator%(const Mat<T>& a, const Mat<T>& b)
{
    if((a.getColumn() != b.getColumn()) || (a.getLine() != b.getLine()))
    {
        cerr << "Impossible d'operer la multiplication c_a_c : mauvais formats de matrices.\n" << endl;
        cerr << "Format m1 : " << a.getLine() << " x " << a.getColumn() << "\t Fromat m2 : " << b.getLine() << " x " << b.getColumn() << endl;
        exit(EXIT_FAILURE);        
        return Mat<T>((T)0,1,1);
    }

    Mat<T> r(a.getLine(), a.getColumn());

    for(int i=r.getLine();i--;)
    {
        for(int j=r.getColumn();j--;)
        {
            //r.set( a.get(i+1,j+1)*b.get(i+1,j+1), i+1, j+1);
            r.mat[i][j] =  a.mat[i][j]*b.mat[i][j];
        }
    }


    return r;
}


/*---------------------------------------------*/


template<typename T>	/*pas de point virgule en fin de ligne...*/
void Mat<T>::copy(const Mat<T>& m)
{
    m_line = m.getLine();
    m_column = m.getColumn();
    
    mat = new T*[m_line];

    for(int i=0;i<=m_line-1;i++)
    {
        mat[i] = new T[m_column];

        for(int j=0;j<=m_column-1;j++)
        {
            mat[i][j] = m.get(i+1,j+1);
        }
    }
   
}



/*---------------------------------------------*/





template<typename T>
bool operator==(const Mat<T>& a, const Mat<T>& b)
{
	bool r = true;
	bool continuer = true;
	
	if(a.getLine() == b.getLine() && a.getColumn() == b.getColumn() )
	{
		for(int i=1;i<=a.getLine();i++)
		{
			for(int j=1;j<=a.getColumn();j++)
			{
				if(a.get(i,j) != b.get(i,j))
				{
					r = false;
					continuer = false;
				}
				
				if(!continuer)
					j=a.getColumn();
			}
			
			if(!continuer)
				i=a.getLine();
		}
	}
	else
		r = false;
	
	return r;
}


/*---------------------------------------------*/




template<typename T>
bool operator<=(const Mat<T>& a, const Mat<T>& b)
{
	bool r = true;
	bool continuer = true;
	
	if(a.getLine() == b.getLine() && a.getColumn() == b.getColumn() )
	{
		for(int i=1;i<=a.getLine();i++)
		{
			for(int j=1;j<=a.getColumn();j++)
			{
				if(a.get(i,j) > b.get(i,j))
				{
					r = false;
					continuer = false;
				}
				
				if(!continuer)
					j=a.getColumn();
			}
			
			if(!continuer)
				i=a.getLine();
		}
	}
	else
		r = false;
	
	return r;
}


/*---------------------------------------------*/




template<typename T>
bool operator>=(const Mat<T>& a, const Mat<T>& b)
{
	bool r = true;
	bool continuer = true;
	
	if(a.getLine() == b.getLine() && a.getColumn() == b.getColumn() )
	{
		for(int i=1;i<=a.getLine();i++)
		{
			for(int j=1;j<=a.getColumn();j++)
			{
				if(a.get(i,j) < b.get(i,j))
				{
					r = false;
					continuer = false;
				}
				
				if(!continuer)
					j=a.getColumn();
			}
			
			if(!continuer)
				i=a.getLine();
		}
	}
	else
		r = false;
	
	return r;
}


/*---------------------------------------------*/




template<typename T>
bool operator<(const Mat<T>& a, const Mat<T>& b)
{
	bool r = true;
	bool continuer = true;
	
	if(a.getLine() == b.getLine() && a.getColumn() == b.getColumn() )
	{
		for(int i=1;i<=a.getLine();i++)
		{
			for(int j=1;j<=a.getColumn();j++)
			{
				if(a.get(i,j) >= b.get(i,j))
				{
					r = false;
					continuer = false;
				}
				
				if(!continuer)
					j=a.getColumn();
			}
			
			if(!continuer)
				i=a.getLine();
		}
	}
	else
		r = false;
	
	return r;
}


/*---------------------------------------------*/




template<typename T>
bool operator>(const Mat<T>& a, const Mat<T>& b)
{
	bool r = true;
	bool continuer = true;
	
	if(a.getLine() == b.getLine() && a.getColumn() == b.getColumn() )
	{
		for(int i=1;i<=a.getLine();i++)
		{
			for(int j=1;j<=a.getColumn();j++)
			{
				if(a.get(i,j) <= b.get(i,j))
				{
					r = false;
					continuer = false;
				}
				
				if(!continuer)
					j=a.getColumn();
			}
			
			if(!continuer)
				i=a.getLine();
		}
	}
	else
		r = false;
	
	return r;
}




/*---------------------------------------------*/




template<typename T>	/*pas de point virgule en fin de ligne...*/
Mat<T>& Mat<T>::operator=(const Mat<T>& m)
{
    if(this != &m)
    {
        if( !(m.getLine() == this->m_line && m.getColumn() == this->m_column ) )
        {
        	this->~Mat();
        
		//this->Mat<T>(m);
		this->copy(m);
		/*on ne peut pas appeller le constructeur de copie...?*/
	}
	else
	{
		for(int i=m_line;i--;)
		{
			for(int j= m_column;j--;)
			{
				this->mat[i][j] = m.mat[i][j];
			}
		}
	}
    }

    return *this;
}
/*La seconde chose à savoir est que le compilateur génère un opérateur d'affectation automatiquement si vous ne le faites pas, et que celui-ci sera suffisant dans la plupart des cas. Vous pouvez donc parfois tout simplement éviter de l'écrire. */

/*De même, imaginez que l'appel à new échoue (ce qui peut très bien arriver) : la ressource aura déjà été détruite, mais ne sera pas recréée, laissant l'objet dans un état invalide, et menant là encore à des comportements indéterminés. */




/*---------------------------------------------*/


template<typename T>	/*pas de point virgule en fin de ligne...*/
Mat<T> operatorC(const Mat<T>& a, const Mat<T>& b)
{
    if(a.getColumn()==b.getColumn())
    {
        Mat<T> r(a, (T)0, (int)1, (int)1, a.getLine()+b.getLine(), a.getColumn());

        for(int i=a.getLine()+1;i<=r.getLine();i++)
        {
            for(int j=1;j<=r.getColumn();j++)
            {
                r.set(b.get(i-a.getLine(),j),i,j);
            }
        }

        return r;
    }
    else
    {
        cout << "Erreur : impossible de concatener les deux matrices sur les colonnes." << endl;
        return Mat<T>( 1, 1);
    }

}



/*---------------------------------------------*/


template<typename T>	/*pas de point virgule en fin de ligne...*/
Mat<T> operatorL(const Mat<T>& a, const Mat<T>& b)
{
    if(a.getLine()==b.getLine())
    {
        Mat<T> r(a, (T)0, (int)1, (int)1, a.getLine(), a.getColumn()+b.getColumn());

        for(int i=1;i<=r.getLine();i++)
        {
            for(int j=a.getColumn()+1;j<=r.getColumn();j++)
            {
                r.set(b.get(i,j-a.getColumn()),i,j);
            }
        }

        return r;
    }
    else
    {
        cout << "Erreur : impossible de concatener les deux matrices sur les lignes." << endl;
        return Mat<T>(1, 1);
    }

}


/*---------------------------------------------*/


template<typename T>	/*pas de point virgule en fin de ligne...*/
Mat<T> Line( const Mat<T>& a, int ind)
{
    /*
    int* delLine = NULL;
    delLine = (int*)malloc(sizeof(int)*(a.getLine()-1));

    int delColumn[1];
    int cLine = a.getLine()-1;
    int cColumn = 0;

    int k = 0;
    for(int i=1;i<=a.getLine();i++)
    {
        if(ind != i)
        {
            delLine[k] = i;
            k++;
        }
    }

    Mat<T> r( a, delLine, cLine, delColumn, cColumn);
    */
    int m = a.getColumn();
    Mat<T> r(1, m);

    for(int i=1;i<=m;i++)	r.set( a.get(ind, i), 1,i);

    return r;
}



/*---------------------------------------------*/


template<typename T>	/*pas de point virgule en fin de ligne...*/
Mat<T> Col( const Mat<T>& a, int ind)
{
    int* delColumn = NULL;
    delColumn = (int*)malloc(sizeof(int)*(a.getColumn()-1));

    int delLine[1];
    int cColumn = a.getColumn()-1;
    int cLine = 0;

    int k = 0;
    for(int i=1;i<=a.getColumn();i++)
    {
        if(ind != i)
        {
            delColumn[k] = i;
            k++;
        }
    }

    Mat<T> r( a, delLine, cLine, delColumn, cColumn);

    return r;
}



/*---------------------------------------------*/


template<typename T>	/*pas de point virgule en fin de ligne...*/
Mat<T> Cola( const Mat<T> a, int ind)
{
    Mat<T> r(a.getLine(), 1);

    if( ind<= a.getColumn() && ind >=1)
        for(int i=1;i<=a.getLine();i++)	r.set( a.get(i,ind), i, 1);

    return r;

}



/*---------------------------------------------*/


template<typename T>	/*pas de point virgule en fin de ligne...*/
inline Mat<T> transpose(const Mat<T>& a)
{
    Mat<T> r(a.getColumn(), a.getLine());

    for(int i=a.getColumn();i--;)
    {
        for(int j=a.getLine();j--;)
        {
            r.set( a.get(j+1,i+1), i+1, j+1);
        }
    }

    return r;
}

template<typename T>	/*pas de point virgule en fin de ligne...*/
inline void transpose(Mat<T>* a)
{    
    if(a->getColumn() == a->getLine())
    {
	    for(int i=a->getColumn();i--;)
	    {
		for(int j=a->getColumn();j--;)
		{
		    std::swap( a->mat[i][j], a->mat[j][i]);
		}
	    }
    }

}



/*---------------------------------------------*/


template<typename T>	/*pas de point virgule en fin de ligne...*/
Mat<T> inv(const Mat<T>& a)
{
    Mat<T> temp(transpose(a)*a);


    //temp.computeInv();
    Mat<T> b((T)0, temp.getLine(), 1);
    gaussj(&temp, &b, 1);

    return temp*transpose(a);
}



/*---------------------------------------------*/


template<typename T>	/*pas de point virgule en fin de ligne...*/
Mat<T> sum(const Mat<T>& a)
{
    if(a.getColumn()!=1)
    {
        /*sum on columns*/
        Mat<T> r( 1, a.getLine());

        for(int j=1;j<=a.getColumn();j++)
        {
            T temp = 0;
            for(int i=1;i<=a.getLine();i++)
                temp += a.get(i,j);

            r.set(temp, 1,j);					/* [0 ; n] x [0 ; m] !!!! */
        }

        return r;
    }
    else
    {
        /*sum on the only column*/
        Mat<T> r( 1, 1);

        for(int i=1;i<=a.getLine();i++)
        {
            r.set((r.get(1,1)+a.get(i,1)), 1,1);			/* [0 ; n] x [0 ; m] !!!! */
        }

        return r;
    }
}



/*---------------------------------------------*/


template<typename T>	/*pas de point virgule en fin de ligne...*/
T sigmoid( const T z)
{
    return 1/exp(-z);
}



/*---------------------------------------------*/


template<typename T>	/*pas de point virgule en fin de ligne...*/
Mat<T> sigmoidM(const Mat<T>& z)
{
    Mat<T> r(z);


    for(int i=1;i<=z.getLine();i++)
    {
        for(int j=1;j<=z.getColumn();j++)
        {
            r.set(sigmoid(z.get(i,j)),i,j);
        }
    }

    return r;
}



/*---------------------------------------------*/


template<typename T>	/*pas de point virgule en fin de ligne...*/
Mat<T> sigmoidGradM( const Mat<T>& z)
{
    Mat<T> one((T)(1), z.getLine(), z.getColumn());
    Mat<T> r(sigmoidM(z) % (one - sigmoidM(z)));	//	sigmoid(z).*(ones(size(z))-sigmoid(z));

    return r;
}




/*---------------------------------------------*/


template<typename T>	/*pas de point virgule en fin de ligne...*/
Mat<T> log(const Mat<T>& a)
{
    Mat<T> r(a);

    for(int i=1;i<=r.getLine();i++)
    {
        for(int j=1;j<=r.getColumn();j++)
        {
            r.set( log<T>(r.get(i,j)), i,j);
        }
    }

    return r;
}



template<typename T>	/*pas de point virgule en fin de ligne...*/
Mat<T> logMat(Mat<T>* a)
{
    Mat<T> r(*a);

    for(int i=1;i<=r.getLine();i++)
    {
        for(int j=1;j<=r.getColumn();j++)
        {
            r.set( log(r.get(i,j)), i,j);
        }
    }

    return r;
}

template<typename T>	/*pas de point virgule en fin de ligne...*/
Mat<T> inverseM(const Mat<T>& a)
{
    Mat<T> r(a);

    for(int i=1;i<=r.getLine();i++)
    {
        for(int j=1;j<=r.getColumn();j++)
        {
            r.set( (T)(1.0/(r.get(i,j))), i,j);
        }
    }

    return r;
}

template<typename T>	/*pas de point virgule en fin de ligne...*/
Mat<T> expMat( Mat<T>* a)
{
    Mat<T> r(a->getLine(),a->getColumn());

    for(int i=r.getLine();i--;)
    {
        for(int j=r.getColumn();j--;)
        {
            r.mat[i][j] = (T)exp(a->mat[i][j]) ;
        }
    }

    return r;
}




/*---------------------------------------------*/


template<typename T>	/*pas de point virgule en fin de ligne...*/
Mat<T> absM(const Mat<T>& a)
{
    Mat<T> r(a.getLine(),a.getColumn());

    for(int i=r.getLine();i--;)
    {
        for(int j=r.getColumn();j--;)
        {
            r.set((T) fabs_( a.get(i+1,j+1)), i+1,j+1);
        }
    }

    return r;

}




/*---------------------------------------------*/


template<typename T>	/*pas de point virgule en fin de ligne...*/
Mat<T> sqrt(const Mat<T>& a)
{
    Mat<T> r(a);

    for(int i=1;i<=r.getLine();i++)
    {
        for(int j=1;j<=r.getColumn();j++)
        {
            r.set( sqrt(r.get(i,j)), i,j);
        }
    }

    return r;
}



/*---------------------------------------------*/


template<typename T>	/*pas de point virgule en fin de ligne...*/
T dist( const Mat<T>& a, const Mat<T>& b, int method)	/*method 2 : euclidian distance, else : L1*/
{
    T r = 0;
    Mat<T> temp = a-b;
    r= (method == 2 ? sqrt( sum( sum( temp%temp )).get(1,1) ) : sum( sum( absM(temp) ) ).get(1,1) );

    return r;

}



/*---------------------------------------------*/


template<typename T>	/*pas de point virgule en fin de ligne...*/
T atan21( T y, T x)
{
    T r = y;

    if( x!= 0)
    {
        if(x < 0)
        {
            if(y<0)
                r = -PI + atan(y/x);
            else
                r = PI/2 + atan(y/x);
        }
        else
            r = atan(y/x);
    }
    else
        r = ( y <= 0 ? -PI/2 : PI/2);

    return r;

}



/*---------------------------------------------*/


template<typename T>	/*pas de point virgule en fin de ligne...*/

Mat<T> atan2(const Mat<T>& y, const Mat<T>& x)
{
    Mat<T> r(y);

    if(x.getColumn() == y.getColumn() && x.getLine() == y.getLine())
    {
        for(int i=1;i<=r.getLine();i++)
        {
            for(int j=1;j<=r.getColumn();j++)
            {
                r.set( atan21(r.get(i,j), x.get(i,j) ), i,j);
            }
        }
    }
    else
    {
        cerr << "ERREUR : atan : les matrices ne sont pas de memes dimensions." << endl;
    }

    return r;

}


template<typename T>	/*pas de point virgule en fin de ligne...*/
T var(Mat<T> mat)
{
    T r = 0;

    if(mat.getColumn()==1 && mat.getLine() != 1)
    {
        int n = mat.getLine();
        T mu = 0;
        for(int i=1;i<=n;i++)
            mu += (T)(((T)1.0/(T)n))*mat.get(i,1);
        cout << mu << endl;

        for(int i=1;i<=n;i++)
            r += (T)(((T)1/(T)(n-1)))*(mat.get(i,1)-mu)*(mat.get(i,1)-mu);
    }

    return r;
}

template<typename T>	/*pas de point virgule en fin de ligne...*/
T covar(Mat<T> a, Mat<T> b)
{
    T r = 0;

    if((a.getColumn()==1 && a.getLine() != 1) &&(b.getColumn()==1 && b.getLine() != 1) && a.getLine() == b.getLine())
    {
        int na = a.getLine();
        T mua = 0;
        T mub = 0;
        for(int i=1;i<=na;i++)
        {
            mua += ((T)1/na)*a.get(i,1);
            mub += ((T)1/na)*b.get(i,1);
        }

        for(int i=1;i<=na;i++)
            r += (T)((T)1/(na-1))*(a.get(i,1)-mua)*(b.get(i,1)-mub);
    }

    return r;
}


/*---------------------------------------------*/


template<typename T>	/*pas de point virgule en fin de ligne...*/
Mat<T> derive(Mat<T> m, int k, int l)
{
    Mat<T> r((T)0, m.getLine(), m.getColumn());
    r.set( (T)1, k,l);

    return r;
}





template<typename T>
Mat<T> crossProduct(const Mat<T>& a, const Mat<T>& b)
{
    Mat<T> rim(a);

    if(rim.getLine() == b.getLine() && b.getLine() == 3 && b.getColumn() == rim.getColumn() && rim.getColumn() == 1)
    {
        rim.set( a.get(2,1)*b.get(3,1)-a.get(3,1)*b.get(2,1), 1,1);
        rim.set( b.get(1,1)*a.get(3,1)-b.get(3,1)*a.get(1,1), 2,1);
        rim.set( a.get(1,1)*b.get(2,1)-a.get(2,1)*b.get(1,1), 3,1);
    }

    return rim;
}



/*---------------------------------------------*/



template<typename T>	/*pas de point virgule en fin de ligne...*/
void Mat<T>::swap(int i1, int j1, int i2, int j2)
{
    T temp = this->get(i1,j1);
    this->set( this->get(i2,j2), i1, j1);
    this->set( temp, i2, j2);
}



template<typename T>	/*pas de point virgule en fin de ligne...*/
void Mat<T>::swapL(int i1, int i2)
{
    for(int j=1;j<=this->getColumn();j++)	this->swap(i1, j, i2, j);
}


template<typename T>	/*pas de point virgule en fin de ligne...*/
void Mat<T>::swapC(int j1, int j2)
{
    for(int i=1;i<=this->getLine();i++)	this->swap(i, j1, i, j2);
}


/*---------------------------------------------*/


template<typename T>	/*pas de point virgule en fin de ligne...*/
inline Mat<T> extract(const Mat<T>& m, int ib, int jb, int ie, int je)
{
    Mat<T> r(ie-ib+1, je-jb+1);

    for(int i=r.getLine();i--;)
    {
        for(int j=r.getColumn();j--;)
        {
            r.set(m.get(ib+i+1-1,jb+j+1-1), i+1, j+1);
        }
    }

    return r;
}


template<typename T>	/*pas de point virgule en fin de ligne...*/
inline Mat<T> extract(Mat<T>* m, int ib, int jb, int ie, int je)
{
    Mat<T> r(ie-ib+1, je-jb+1);

    for(int i=r.getLine();i--;)
    {
        for(int j=r.getColumn();j--;)
        {
            r.set(m->get(ib+i+1-1,jb+j+1-1), i+1, j+1);
            //r.mat[i][j] = m->mat[ib+i+1-2][jb+j+1-2];
        }
    }

    return r;
}



/*---------------------------------------------*/


template<typename T>	/*pas de point virgule en fin de ligne...*/
inline void homogeneousNormalization( Mat<T>* X)
{
    int n = X->getLine();
    int m = X->getColumn();

    for(int j=1;j<=m;j++)
    {
        T l = X->get(n,j);
        for(int i=1;i<=n;i++)	X->set( (l != 0 ? (T)((float)X->get(i,j)/l) : X->get(i,j)), i,j);
    }
}



/*---------------------------------------------*/


template<typename T>	/*pas de point virgule en fin de ligne...*/
Mat<T> correlation(Mat<T> m, Mat<T> k)
{
    Mat<T> r(m);
    int mi = m.getLine();
    int mj = m.getColumn();
    int ki = k.getLine();
    int kj = k.getColumn();

    if(mj >= kj && mi >= ki)
    {
        for(int i=1;i<=mi;i++)
        {
            for(int j=1;j<=mj;j++)
            {
                //T temp = sum(sum(k%extract(m, i-ki/2, j-kj/2, i+ki/2, j+kj/2))).get(1,1);
                T temp = 0;

                for(int ii=i-ki/2;ii<=i+ki/2;ii++)
                    for(int jj=j-kj/2;jj<=j+kj/2;jj++)
                        temp += k.get(ii-i+ki/2+1,jj-j+kj/2+1)*m.get(ii,jj);

                r.set( temp, i, j);
            }
        }
    }
    else
    {
        cout << "ERREUR : correlation impossible, l'image est plus petite que le kernel..." << endl;
    }

    return r;
}


/*---------------------------------------------*/


template<typename T>	/*pas de point virgule en fin de ligne...*/
Mat<T> conv(Mat<T> m, Mat<T> k)
{	
	int mi = m.getLine();
	int mj = m.getColumn();
	int ki = k.getLine();
	int kj = k.getColumn();
	Mat<T> r((T)0, (int)(mi/ki+1), (int)(mj/kj+1));
		
	if(mj/kj >= 1 && mi/ki >= 1)
	{
		for(int i=1;i<=mi/ki;i++)
		{
			for(int j=1;j<=mj/kj;j++)
			{
				T temp = sum(sum(k % extract(m, (i-1)*ki +1,(j-1)*kj+1, i*ki, j*kj ) ) ).get(1,1);
				r.set( temp, i, j);
			}
		}
	}
	else
	{
		cout << "ERREUR : conv impossible, l'image est plus petite que le kernel..." << endl;
	}
	
	return r;
}


/*---------------------------------------------*/


template<typename T>	/*pas de point virgule en fin de ligne...*/
T mean(Mat<T> mat)
{
    T r = 0;
    int n = mat.getLine();
    int m = mat.getColumn();

    for(int i=1;i<=n;i++)
    {
        for(int j=1;j<=m;j++)
        {
            r += mat.get(i,j);
        }
    }

    return ((T)1.0/(m*n))*r;
}




/*---------------------------------------------*/



template<typename T>	/*pas de point virgule en fin de ligne...*/
Mat<T> meanC(Mat<T> mat)
{
    int m = mat.getColumn();
    Mat<T> r(1,m);

    for(int i=1;i<=m;i++)
    {
        r.set( mean(Cola(mat, i)), 1,i);
    }

    return r;
}




/*---------------------------------------------*/


template<typename T>	/*pas de point virgule en fin de ligne...*/
T max(Mat<T> mat)
{
    T r = mat.get(1,1);
    int n = mat.getLine();
    int m = mat.getColumn();

    for(int i=1;i<=n;i++)
    {
        for(int j=1;j<=m;j++)
        {
            T temp = mat.get(i,j);
            r =  (r>= temp ? r : temp);
        }
    }

    return r;
}

template<typename T>	/*pas de point virgule en fin de ligne...*/
T maxabs(Mat<T> mat)
{
    T r = mat.get(1,1);
    int n = mat.getLine();
    int m = mat.getColumn();

    for(int i=1;i<=n;i++)
    {
        for(int j=1;j<=m;j++)
        {
            T temp = mat.get(i,j);
            r =  (r>= abs(temp) ? r : temp);
        }
    }

    return r;
}




/*---------------------------------------------*/


template<typename T>
Mat<T> idmin(Mat<T> mat)
{
	Mat<T> id((T)(1.0/numeric_limits<T>::epsilon()),3,1);	
	int n = mat.getLine();
	int m = mat.getColumn();
	
	for(int i=1;i<=n;i++)
	{
		for(int j=1;j<=m;j++)
		{
			T temp = mat.get(i,j);
			
			if(temp<id.get(3,1))
			{
				id.set((T)i, 1,1);
				id.set((T)j, 2,1);
				id.set(temp,3,1);
			}
		}
	}
	
	return id;
}

/*---------------------------------------------*/


template<typename T>
Mat<T> idmax(Mat<T> mat)
{
	Mat<T> id((T)mat.get(1,1),3,1);	
	int n = mat.getLine();
	int m = mat.getColumn();
	
	for(int i=1;i<=n;i++)
	{
		for(int j=1;j<=m;j++)
		{
			T temp = mat.get(i,j);
			if(temp > id.get(3,1))
			{
				id.set((T)i,1,1);
				id.set((T)j,1,1);
				id.set(temp,3,1);
			}
		}
	}
	
	return id;
}

/*---------------------------------------------*/

template<typename T>	/*pas de point virgule en fin de ligne...*/
/*type : 1: max 2: mean*/
/*k : kernel qui specifie en plus la taille de la convolution*/
/* en mettant une matrice remplie de 1 dans k, on obtient un pooling seul.*/
Mat<T> pooling(Mat<T> m, Mat<T> k, int type)
{    
    int mi = m.getLine();
    int mj = m.getColumn();
    int ki = k.getLine();
    int kj = k.getColumn();

    T (*ptrFonction)(Mat<T>);
    if(type == 1)
        ptrFonction = max;
    else if(type == 2)
    	ptrFonction = maxabs;
    else
        ptrFonction = mean;

    if(mj >= kj && mi >= ki)
    {
        Mat<T> r((T)0, mi/ki+1, mj/kj+1);
        Mat<T> temp(k);

        for(int i=1;i<=mi/ki;i++)
        {
            for(int j=1;j<=mj/kj;j++)
            {
                temp = extract(m, (i-1)*ki+1, (j-1)*kj+1, i*ki, j*kj);
                r.set( (*ptrFonction)( k % temp), i, j);
            }
        }

        return r;
    }
    else
    {
        cout << "ERREUR : pooling/(filtering) impossible, l'image est plus petite que le kernel..." << endl;
    }

    return m;
}




template<typename T>	/*pas de point virgule en fin de ligne...*/
/*type : 1: max 2: mean*/
/*k : kernel qui specifie en plus la taille de la convolution*/
/* en mettant une matrice remplie de 1 dans k, on obtient un pooling seul.*/
Mat<T> pooling(Mat<T>* m, Mat<T>* k, int type)
{    
    int mi = m->getLine();
    int mj = m->getColumn();
    int ki = k->getLine();
    int kj = k->getColumn();

    T (*ptrFonction)(Mat<T>);
    if(type == 1)
        ptrFonction = max;
    else if(type == 2)
    	ptrFonction = maxabs;
    else
        ptrFonction = mean;

    if(mj >= kj && mi >= ki)
    {
        Mat<T> r((T)0, mi/ki+1, mj/kj+1);
        Mat<T> temp(*k);

        for(int i=mi/ki;i--;)
        {
            for(int j=mj/kj;j--;)
            {
                //temp = extract( m, (i-1)*ki+1, (j-1)*kj+1, i*ki, j*kj);
                temp = extract( m, (i)*ki+1, (j)*kj+1, (i+1)*ki, (j+1)*kj);
                r.set( (*ptrFonction)( (*k) % temp), i+1, j+1);
            }
        }

        return r;
    }
    else
    {
        cout << "ERREUR : pooling/(filtering) impossible, l'image est plus petite que le kernel..." << endl;
    }

    return *m;
}

/*
Mat<T> sym(Mat<T> m)
{

}

*/
template<typename T>
Mat<T> convolution(Mat<T> m, Mat<T> k)
{
	int nm = m.getLine();
	int mm = m.getColumn();
	int nk = k.getLine();
	int mk = k.getColumn();
	
	int nr = nm/nk;
	int mr = mm/mk;
	Mat<T> ret((T)0, nr, mr);
	
	for(int i=1;i<=nr;i++)
	{
		for(int j=1;j<=mr;j++)
		{
			ret.set( (T)( sum( sum( correlation( extract(m, (i-1)*nk+1, (j-1)*mk+1,  i*nk, j*mk), k))) ).get(1,1) , i,j);
		}
	}
	
	return ret;

}



#ifdef OPENCV_USE


/*---------------------------------------------*/

/*
template<typename T>	//pas de point virgule en fin de ligne...
//renvoi le nombre de matrice contenue dans tab,
//correspondant au nombre de channel de mat.

int cv2Mat(const cv::Mat mat, Mat<T>* tab)
{
    int nbr_chan = 0;
    int r = 0;
    //int c = 0;

    if(tab != NULL)
    {
        nbr_chan = 3; //(int)mat.channels();
        r = mat.rows;
        //c = mat.cols;

        cout << "Conversion d'une CvMat<T> avec " << nbr_chan << " channel(s) en " << nbr_chan << " matrices de types Mat<T>." << endl;

        //tab = (Mat<T>**)malloc(sizeof(Mat<T>*)*nbr_chan);
        //tab = new Mat<T>*[nbr_chan];
        //tab = new Mat<T>(FLOAT, (T)0, r,c);



        //for(int k=0;k<=nbr_chan-1;k++)
        //{
        //	cout << "begin " << k << endl;
        //	tab[k] =  new Mat<T>(FLOAT, (T)0, r, c);
        //}


        cv::Mat_<cv::Vec3b>::const_iterator it = mat.begin<cv::Vec3b>();
        cv::Mat_<cv::Vec3b>::const_iterator itend = mat.end<cv::Vec3b>();

        int i = 0;
        int j = 0;


        for( ; it != itend; ++it)
        {
            //cout << i << "/" << r << " & " << j << "/" << c << endl;
            //tab[0]->set( (*it)[0], i+1,j+1);
            //tab[1]->set( (*it)[1], i+1,j+1);
            //tab[2]->set( (*it)[2], i+1,j+1);
            tab->set( (*it)[0], i+1,j+1);


            i++;
            if( (i+1)%r == 0 )
            {
                i=0;
                j++;

            }
        }

    }
    else
    {
        cout << "ERREUR : le tableau mis en argument n'est pas NULL. Aucune operation effectuee..." << endl;
    }

    cout << "end" << endl;
    cout << tab << endl;

    return nbr_chan;
}



//---------------------------------------------


template<typename T>	//pas de point virgule en fin de ligne...
cv::Mat Mat2CV( Mat<T>* tab)
{
    int nbr_chan = 3;
    int r = tab->getLine();
    int c = tab->getColumn();
    cv::Mat mat( r, c, CV_8UC3);

    if(tab != NULL)
    {

        cout << "Conversion d'une CvMat avec " << nbr_chan << " channel(s) en " << nbr_chan << " matrices de types Mat." << endl;
        cv::Mat_<cv::Vec3b>::const_iterator it = mat.begin<cv::Vec3b>();
        cv::Mat_<cv::Vec3b>::const_iterator itend = mat.end<cv::Vec3b>();

        int i = 0;
        int j = 0;

        for( ; it != itend; ++it)
        {
            *it= cv::Vec3b( tab->get(i+1,j+1), tab->get(i+1,j+1), tab->get(i+1,j+1));

            i++;
            if( !(i+1)%r)
            {
                i=0;
                j++;
            }
        }

    }
    else
    {
        cout << "ERREUR : le tableau mis en argument est NULL. Aucune operation effectuee..." << endl;
    }

    return mat;
}





//---------------------------
// renvoi les image ou matrice
//--------------------------



//---------------------------------------------


template<typename T>	//pas de point virgule en fin de ligne...
Mat<T> cv2Mat(const cv::Mat mat)
{
    int r = mat.rows;
    int c = mat.cols;
    Mat<T> rim( (T)0, r,c);


    //time_t timer;
    //time_t timerp;
    //time(&timer);
    //time(&timerp);
    //cv::Mat_<cv::Vec3b>::const_iterator it = mat.begin<cv::Vec3b>();
    //cv::Mat_<cv::Vec3b>::const_iterator itend = mat.end<cv::Vec3b>();

    //time(&timer);
    //cout << timer-timerp << endl;


    //int i = 0;
    //int j = 0;



    //for( ; it != itend; ++it)
    //{
    //	rim.set( (*it)[0], i+1,j+1);
    //
    //	i++;
    //	if( (i+1)%r == 0 )
    //	{
    //		i=0;
    //		j++;
    //
    //	}
    //}
    //

    for(int x=0;x<=r;x++)
    {
            for(int y=0;y<=c;y++)
            {
                    rim.set( mat.data[mat.step*y+x+1], x+1, y+1);
            }
    }

    return rim;
}




//---------------------------------------------


template<typename T>	//pas de point virgule en fin de ligne...
cv::Mat Mat2CV( Mat<T> tab)
{
    int r = tab.getLine();
    int c = tab.getColumn();
    cv::Mat mat( r, c, CV_8UC3);

    cv::Mat_<cv::Vec3b>::iterator it = mat.begin<cv::Vec3b>();
    cv::Mat_<cv::Vec3b>::const_iterator itend = mat.end<cv::Vec3b>();

    int i = 0;
    int j = 0;

    for( ; it != itend; ++it)
    {
        *it= cv::Vec3b( tab.get(i+1,j+1), tab.get(i+1,j+1), tab.get(i+1,j+1));

        i++;
        if( !(i+1)%r)
        {
            i=0;
            j++;
        }
    }
    //
    //for(int x=0;x<=r;x++)
    //{
    //		for(int y=0;y<=c;y++)
    //		{
    //				mat.data[mat.step*y+x+1] = (unsigned char)(tab.get(x+1, y+1));
    //
    //		}
    //}
    //
    return mat;
}





*/
/*
//---------------------------------------------


template<typename T>	//pas de point virgule en fin de ligne...
int CvMat2Mat(const CvMat& mat, Mat<T>** tab)
{
    int nbr_chan = 0;
    int r = 0;
    int c = 0;

    if(tab == NULL)
    {
        nbr_chan = (int)mat.channels();
        r = mat.rows;
        c = mat.cols;

        cout << "Conversion d'une CvMat avec " << nbr_chan << " channel(s) en " << nbr_chan << " matrices de types Mat." << endl;

        tab = (Mat<T>**)malloc(sizeof(Mat<T>*)*nbr_chan);

        for(int k=0;k<=nbr_chan-1;k++)
        {
            tab[k] = (Mat<T>*)malloc(sizeof(Mat<T>));
            *tab[k] = Mat<T>( r, c);;

            for(int i=1;i<=r;i++)
            {
                for(int j=1;j<=c;j++)
                {
                    tab[k]->set((T)(mat.at<T>(i-1,j-1)), i, j);
                }
            }
        }
    }
    else
    {
        cout << "ERREUR : le tableau mis en argument n'est pas NULL. Aucune operation effectuee..." << endl;
    }

    return nbr_chan;
}
*/

/*----------------------------------------------------------------------------------------*/



/*----------------------------------------------------------------------------------------*/

cv::Mat absM(const cv::Mat& a)
{
    cv::Mat r(a.rows,a.cols,CV_8UC3);

    for(int i=r.rows;i--;)
    {
        for(int j=r.cols;j--;)
        {

            r.at<char>(i,j) =  (char)fabs_( r.at<char>(i,j));
        }
    }

    return r;

}

cv::Mat absMp(cv::Mat mat)
{
    int c = mat.cols;
    int r = mat.rows;    
    cv::Mat ret(r,c,CV_8UC1);

    /*
    if(channels == 3)
        ret = cv::Mat(mat.rows,mat.cols,CV_8UC3);
        */

    uchar* p;
    uchar* pm;


    for(int i=r;i--;)
    {
        p = mat.ptr<uchar>(i);
        pm = ret.ptr<uchar>(i);

        //for(int j=c*channels;j-=channels;)
        for(int j=c;j--;)
        {
            pm[j] = p[j];

            /*
            if(channels == 3)
            {
                pm[j+1] = p[j+1];
                pm[j+2] = p[j+2];
            }
            */

        }

    }

        return ret;
}


cv::Mat absMAt(cv::Mat mat)
{
    int r = mat.rows;
    int c = mat.cols;
    cv::Mat rim(r,c,CV_8UC1);

    for(int x=r;x--;)
    {
            for(int y=c;y--;)
            {
                    rim.at<uchar>(x,y) = mat.at<uchar>(x,y);
            }
    }

    return rim;
}

inline cv::Mat sum(const cv::Mat& a)
{

    if(a.rows != 1)
    {
        /*sum on columns*/
        cv::Mat r( 1, a.cols, CV_8UC1 );

        for(int j=0;j<a.cols;j++)
        {
            char temp = 0;
            for(int i=0;i<a.rows;i++)
                temp += a.at<char>(i,j);

            r.at<char>( 0,j) = temp;
        }        

        return r;
    }
    else
    {
        /*sum on the only column*/
        cv::Mat r( 1, 1, CV_8UC1);

        for(int i=0;i<a.cols;i++)
        {
            r.at<char>(0,0) = r.at<char>(0,0) + a.at<char>(0,i);
        }


        return r;
    }
}



/*---------------------------------------------*/


template<typename T>	//pas de point virgule en fin de ligne...
int cv2Mat(const cv::Mat mat, Mat<T>* red, Mat<T>* green, Mat<T>* blue)
{
    int r = mat.rows;
    int c = mat.cols;
    *red = Mat<T>((T)0, r,c);
    *green = Mat<T>((T)0, r,c);
    *blue = Mat<T>((T)0, r,c);

    for(int x=0;x<=r;x++)
    {
            for(int y=0;y<=c;y++)
            {
                    red->set( mat.data[mat.step*y+x+0], x+1, y+1);
                    green->set( mat.data[mat.step*y+x+1], x+1, y+1);
                    blue->set( mat.data[mat.step*y+x+2], x+1, y+1);
            }
    }

    return 1;
}

template<typename T>	//pas de point virgule en fin de ligne...
int cv2MatAt(const cv::Mat mat, Mat<T>* red, Mat<T>* green, Mat<T>* blue)
{
    int r = mat.rows;
    int c = mat.cols;
    *red = Mat<T>((T)0, r,c);
    *green = Mat<T>((T)0, r,c);
    *blue = Mat<T>((T)0, r,c);

    for(int x=0;x<=r;x++)
    {
            for(int y=0;y<=c;y++)
            {
                    red->set( mat.at<cv::Vec3b>(x,y)[0], x+1, y+1);
                    green->set( mat.at<cv::Vec3b>(x,y)[1], x+1, y+1);
                    blue->set( mat.at<cv::Vec3b>(x,y)[2], x+1, y+1);
            }
    }

    return 1;
}

template<typename T>	//pas de point virgule en fin de ligne...
inline int cv2Matp( cv::Mat mat, Mat<T>* red, Mat<T>* green, Mat<T>* blue)
{
    int channels = mat.channels();
    int r = mat.rows;
    int c = mat.cols;

    *red = Mat<T>((T)0, r,c);
    *green = Mat<T>((T)0, r,c);
    *blue = Mat<T>((T)0, r,c);

    uchar* p;

    //for(int i=0;i<r;i++)
    for(int i=r;i--;)
    {
        p = mat.ptr<uchar>(i);
        //for(int j=0;j<c*channels;j+=3)
        for(int j=c*channels;j-=3;)
        {
            red->set( (T)p[j], i+1, (int)((float)(j)/channels)+1);
            //red->mat[i][(int)((float)j/channels)] = (T)p[j];
            //red->mat[i*(int)((float)j/channels)] = (T)p[j];
            green->set( (T)p[j+1], i+1, (int)((float)(j)/channels)+1);
            //green->mat[i][(int)((float)j/channels)] = (T)p[j+1];
            //green->mat[i*(int)((float)j/channels)] = (T)p[j+1];
            blue->set( (T)p[j+2], i+1, (int)((float)(j)/channels)+1);
            //blue->mat[i][(int)((float)j/channels)] = (T)p[j+2];
            //blue->mat[i*(int)((float)j/channels)] = (T)p[j+2];
        }

    }

    return 1;
}


template<typename T>	//pas de point virgule en fin de ligne...
Mat<T> cv2Matp( cv::Mat mat)
{    
    if( mat.channels() != 1)
        cv::cvtColor(mat,mat,CV_BGR2GRAY);

    int r = mat.rows;
    int c = mat.cols;

    Mat<T> rim( r,c);

    uchar* p;

    for(int i=r;i--;)
    {

        p = mat.ptr<uchar>(i);
        for(int j=c;j--;)
            rim.set((T)p[j], i+1,j+1);//rim.mat[i][j] = (T)p[j];

    }

    return rim;
}




template<typename T>	//pas de point virgule en fin de ligne...
cv::Mat Mat2cv(Mat<T> r, Mat<T> g, Mat<T> b)
{
    int rl = r.getLine();
    int rc = r.getColumn();

    int gl = g.getLine();
    int gc = g.getColumn();

    int bl = b.getLine();
    int bc = b.getColumn();

    cv::Mat rim( rl, rc, CV_8UC3);

    if(rl==gl && gl==bl && rc==gc && gc==bc)
    {
        cout << "Creating cv::Mat..." << endl;
        for(int x=0;x<=rl-1;x++)
        {
                for(int y=0;y<=rc-1;y++)
                {
                        rim.data[rim.step*y+x+0] = (uchar)r.get( x+1, y+1);
                        rim.data[rim.step*y+x+1] = (uchar)g.get( x+1, y+1);
                        rim.data[rim.step*y+x+2] = (uchar)b.get( x+1, y+1);
                }
        }
        cout << "Creation cv::Mat : DONE." << endl;
    }
    else
        rim = cv::Mat(1,1, CV_8UC3);

    return rim;

}



template<typename T>	//pas de point virgule en fin de ligne...
cv::Mat Mat2cvVec(Mat<T> r, Mat<T> g, Mat<T> b)
{
    int rl = r.getLine();
    int rc = r.getColumn();

    int gl = g.getLine();
    int gc = g.getColumn();

    int bl = b.getLine();
    int bc = b.getColumn();

    cv::Mat rim( rl, rc, CV_8UC3);

    if(rl==gl && gl==bl && rc==gc && gc==bc)
    {
        cout << "Creating cv::Mat..." << endl;
        cv::Mat_<cv::Vec3b>::const_iterator it = rim.begin<cv::Vec3b>();
        cv::Mat_<cv::Vec3b>::const_iterator itend = rim.end<cv::Vec3b>();

        int i = 0;
        int j = 0;

        for( ; it != itend; ++it)
        {
            *it= cv::Vec3b( r.get(i+1,j+1), g.get(i+1,j+1), b.get(i+1,j+1));

            i++;
            if( !(i+1)%rl)
            {
                i=0;
                j++;
            }
        }
        cout << "Creation cv::Mat : DONE." << endl;
    }
    else
        rim = cv::Mat(1,1, CV_8UC3);

    return rim;

}


template<typename T>	//pas de point virgule en fin de ligne...
cv::Mat Mat2cvAt(Mat<T> r, Mat<T> g, Mat<T> b)
{
    int rl = r.getLine();
    int rc = r.getColumn();

    int gl = g.getLine();
    int gc = g.getColumn();

    int bl = b.getLine();
    int bc = b.getColumn();

    cv::Mat rim( rl, rc, CV_8UC3);

    if(rl==gl && gl==bl && rc==gc && gc==bc)
    {
        cout << "Creating cv::Mat..." << endl;
        for(int x=0;x<=rl-1;x++)
        {
                for(int y=0;y<=rc-1;y++)
                {
                        rim.at<cv::Vec3b>(x,y) = cv::Vec3b( r.get(x+1,y+1), g.get(x+1,y+1), b.get(x+1,y+1));
                }
        }
        cout << "Creation cv::Mat : DONE." << endl;
    }
    else
        rim = cv::Mat(1,1, CV_8UC3);

    return rim;

}

template<typename T>	//pas de point virgule en fin de ligne...
inline cv::Mat Mat2cvp(Mat<T> r, Mat<T> g, Mat<T> b)
{
    int rl = r.getLine();
    int rc = r.getColumn();

    int gl = g.getLine();
    int gc = g.getColumn();

    int bl = b.getLine();
    int bc = b.getColumn();

    cv::Mat rim( rl, rc, CV_8UC3);
    uchar* p;
    int channels = 3;

    if(rl==gl && gl==bl && rc==gc && gc==bc)
    {
        //cout << "Creating cv::Mat..." << endl;
        //for(int x=0;x<rl;x++)
        for(int x=rl;x--;)
        {
                p = rim.ptr<uchar>(x);
                //for(int y=0;y<rc;y++)
                for(int y=rc;y--;)
                {
                        p[y*channels] = (uchar)r.get(x+1,y+1);//r.mat[x][y];
                        p[y*channels+1] = (uchar)g.get(x+1,y+1);//g.mat[x][y];
                        p[y*channels+2] = (uchar)b.get(x+1,y+1);//b.mat[x][y];
                }
        }
        //cout << "Creation cv::Mat : DONE." << endl;
    }
    else
        rim = cv::Mat(1,1, CV_8UC3);

    return rim;

}

/*************************************************/

cv::Mat extractCV(cv::Mat im,int x, int y, int nneigh)
{    

    cv::Mat rim( (nneigh >=1 ? nneigh : 1), (nneigh >=1 ? nneigh : 1), im.type());
    uchar *p, *pi;
    int channels = im.channels();

    for(int xi=nneigh;xi--;)
    {
        if(x-nneigh/2 +xi > 0 && x-nneigh/2 + xi < im.rows)
        {
            p = rim.ptr<uchar>(xi);
            pi = im.ptr<uchar>(x-nneigh/2 + xi);

            for(int yi=nneigh;yi--;)
            {
                for(int c=channels;c>=1;c--)
                {
                    if(yi*channels+(c-1)<rim.cols && (y-nneigh/2+yi)*channels+(c-1)<im.cols)
                    {
                        p[yi*channels + (c-1)] = pi[(y-nneigh/2+yi)*channels + (c-1)];
                    }
                }
            }
        }
    }

    return rim;

}

void afficher( cv::Mat* im, cv::Mat* im1, cv::Mat* im2, bool cont, float factor)
{
    bool continuer = true;
   /*CAMERA */
    /*
    cv::VideoCapture cap(1);
    if(!cap.isOpened())
    {
        cerr << "Erreur : Impossible de démarrer la capture video." << endl;
        cap.open(0);
        if(!cap.isOpened())
            continuer = false;
    }
    */

    //cv::destroyAllWindows();
    cv::namedWindow("OUTPUT");

    if(im1 != NULL)
        cv::namedWindow("OUTPUT 1");
    if(im2!=NULL)
        cv::namedWindow("OUTPUT 2");

    /*--------------------*/

    if(factor != 1.0)
    {
        cv::resize(*im,*im,cv::Size(0,0),factor,factor);
        if(im1!=NULL)
            cv::resize(*im1,*im1,cv::Size(0,0),factor,factor);
        if(im2!=NULL)
            cv::resize(*im2,*im2,cv::Size(0,0),factor,factor);
    }



    while(continuer)
    {
        cv::imshow("OUTPUT", *im);

        if(im1 != NULL)
            cv::imshow("OUTPUT 1", *im1);
        if(im2!=NULL)
            cv::imshow("OUTPUT 2", *im2);


        if(cv::waitKey(30) >= 0)
            continuer = false;
        if(cont)
            continuer = false;
    }

    //cv::destroyAllWindows();
    //cap.release();
    if(factor != 1.0)
    {
        cv::resize(*im,*im,cv::Size(0,0),1.0/factor,1.0/factor);
    }

    /*--------------------------*/
}


template<typename T>
void afficherMat( Mat<T>* im, Mat<T>* im1, Mat<T>* im2, bool cont, float factor)
{
    if(im != NULL)
    {
        cv::Mat imcv(Mat2cvp(*im,*im,*im));

        if(im1 != NULL)
        {
            cv::Mat im1cv( Mat2cvp(*im1,*im1,*im1));

            if(im2 != NULL)
            {
                cv::Mat im2cv(Mat2cvp(*im2,*im2,*im2));

                afficher(&imcv,&im1cv,&im2cv, cont, factor);
            }
            else
                afficher(&imcv,&im1cv,NULL, cont, factor);
        }
        else
            afficher(&imcv,NULL,NULL, cont, factor);
    }
}

template<typename T>
void displayInverseCodedFrame( string s, Mat<T>* frame, Mat<T>* invDepthMap, bool cont, float factor)
{
    if(invDepthMap != NULL)
    {
    	T idmean = (T)mean(*invDepthMap);
    	
        cv::Mat imcv( Mat2cvp( *frame, *frame, *frame));
        
        
        for(int i=invDepthMap->getLine();i--;)
        {

            for(int j=invDepthMap->getColumn();j--;)
            {
		T id = invDepthMap->get(i+1,j+1)/idmean;
                if( id != 0.0 )
                {
                	cv::Scalar color(0,0,0);
                	if(id<(T)0)
                		id = -id;
 
                	if(id >= (T)0)
		        {
				// rainbow between 0 and 4
				float r = (0-id) * 255 / 1.0; if(r < 0) r = -r;
				float g = (1-id) * 255 / 1.0; if(g < 0) g = -g;
				float b = (2-id) * 255 / 1.0; if(b < 0) b = -b;

				uchar rc = r < 0 ? 0 : (r > 255 ? 255 : r);
				uchar gc = g < 0 ? 0 : (g > 255 ? 255 : g);
				uchar bc = b < 0 ? 0 : (b > 255 ? 255 : b);
				
				color = cv::Scalar(255-rc, 255-gc, 255-bc);
			}
                	
                	cv::line(imcv,
                                 cv::Point(j,i),
                                 cv::Point(j,i),
                                 color);
                }
            }
            
        }

        afficher(s,&imcv,NULL,NULL, cont, factor);        
    }
}


void afficher( string s, cv::Mat* im, cv::Mat* im1, cv::Mat* im2, bool cont, float factor)
{
    bool continuer = true;
   /*CAMERA */
    /*
    cv::VideoCapture cap(1);
    if(!cap.isOpened())
    {
        cerr << "Erreur : Impossible de démarrer la capture video." << endl;
        cap.open(0);
        if(!cap.isOpened())
            continuer = false;
    }
    */

    //cv::destroyAllWindows();
    cv::namedWindow(s.c_str());

    if(im1 != NULL)
        cv::namedWindow( (s+string(" 1")).c_str() );
    if(im2!=NULL)
        cv::namedWindow( (s+string(" 2")).c_str() );

    /*--------------------*/

    if(factor != 1.0)
    {
        cv::resize(*im,*im,cv::Size(0,0),factor,factor);
        if(im1!=NULL)
            cv::resize(*im1,*im1,cv::Size(0,0),factor,factor);
        if(im2!=NULL)
            cv::resize(*im2,*im2,cv::Size(0,0),factor,factor);
    }



    while(continuer)
    {
        cv::imshow(s.c_str(), *im);

        if(im1 != NULL)
            cv::imshow((s+string(" 1")).c_str(), *im1);
        if(im2!=NULL)
            cv::imshow( (s+string(" 2")).c_str(), *im2);


        if(cv::waitKey(30) >= 0)
            continuer = false;
        if(cont)
            continuer = false;
    }

    //cv::destroyAllWindows();
    //cap.release();
    if(factor != 1.0)
    {
        cv::resize(*im,*im,cv::Size(0,0),1.0/factor,1.0/factor);
    }

    /*--------------------------*/
}


template<typename T>
void afficherMat( string s, Mat<T>* im, Mat<T>* im1, Mat<T>* im2, bool cont, float factor)
{
    if(im != NULL)
    {
        cv::Mat imcv(Mat2cvp(*im,*im,*im));

        if(im1 != NULL)
        {
            cv::Mat im1cv( Mat2cvp(*im1,*im1,*im1));

            if(im2 != NULL)
            {
                cv::Mat im2cv(Mat2cvp(*im2,*im2,*im2));

                afficher(s, &imcv,&im1cv,&im2cv, cont, factor);
            }
            else
                afficher( s, &imcv,&im1cv,NULL, cont, factor);
        }
        else
            afficher( s, &imcv,NULL,NULL, cont, factor);
    }
}
#endif

/*--------------------------------------------------------------------------------*/
/*--------------------------------------------------------------------------------*/
/*--------------------------------------------------------------------------------*/
/*--------------------------------------------------------------------------------*/
/*--------------------------------------------------------------------------------*/
/*--------------------------------------------------------------------------------*/
/*--------------------------------------------------------------------------------*/
/*--------------------------------------------------------------------------------*/
/*--------------------------------------------------------------------------------*/
/*--------------------------------------------------------------------------------*/
/*--------------------------------------------------------------------------------*/
/*--------------------------------------------------------------------------------*/
/*--------------------------------------------------------------------------------*/
/*--------------------------------------------------------------------------------*/
/*--------------------------------------------------------------------------------*/
/*--------------------------------------------------------------------------------*/


/*GaussJordan.cpp templated*/


template<typename T>	/*pas de point virgule en fin de ligne...*/
Mat<T> gaussTrans( Mat<T> C, int k);
template<typename T>	/*pas de point virgule en fin de ligne...*/
Mat<T> hhTrans( Mat<T> x);
template<typename T>	/*pas de point virgule en fin de ligne...*/
Mat<T> hhTransMat( Mat<T> A, int k);
template<typename T>	/*pas de point virgule en fin de ligne...*/
Mat<T> WielandHotellingTrans( Mat<T> A, Mat<T> eigv, T sigma, Mat<T> z);
template<typename T>	/*pas de point virgule en fin de ligne...*/
T RayleighQuotient(Mat<T> A, Mat<T> q);
template<typename T>	/*pas de point virgule en fin de ligne...*/
Mat<T> gaussj( Mat<T>* A_, Mat<T>* b_, int method = 1);
template<typename T>	/*pas de point virgule en fin de ligne...*/
Mat<T> Normalization(Mat<T> A, int method = 2);
template<typename T>
Mat<T> computeSmallestNonZeroEig(Mat<T> A, T* eigval = NULL);
template<typename T>	/*pas de point virgule en fin de ligne...*/
Mat<T> invGJ(const Mat<T>& mat, int method = 1);
template<typename T>	/*pas de point virgule en fin de ligne...*/
Mat<T> invGJ(Mat<T>* mat, int method = 1);
template<typename T>    /*pas de point virgule en fin de ligne...*/
Mat<T> invSVD(const Mat<T> mat);
template<typename T>
Mat<T> expMat(const Mat<T> m);
/*------------------------------------------------------------*/
/*------------------------------------------------------------*/
/*------------------------------------------------------------*/
/*---------------------------------------------*/


template<typename T>	/*pas de point virgule en fin de ligne...*/
T fabs_(T a)
{
    T r = a;

    if(r<= (T)0)
        r = -a;

    return r;
}


/*------------------------------------------------------------*/
/*------------------------------------------------------------*/
/*------------------------------------------------------------*/
/*---------------------------------------------*/



template<typename T>	/*pas de point virgule en fin de ligne...*/
/*Norme 1 naturelle*/
T norme1( Mat<T> X)
{
    T r = 0;
    int n = X.getLine();
    int m = X.getColumn();

    for(int i=1;i<=n;i++)
    {
    	for(int j=1;j<=m;j++)
	        r += fabs_(X.get(i,j));
    }
    

    return r;
}
/*------------------------------------------------------------*/
/*------------------------------------------------------------*/
/*------------------------------------------------------------*/


template<typename T>	/*pas de point virgule en fin de ligne...*/
/*Norme 2 naturelle*/
T norme2( Mat<T> X)
{
    T r = 0;
    int n = X.getLine();
    int m = X.getColumn();

    for(int i=1;i<=n;i++)
    {
    	for(int j=1;j<=m;j++)
	        r += X.get(i,j)*X.get(i,j);
    }

    r = sqrt(r);

    return r;
}
/*------------------------------------------------------------*/
/*------------------------------------------------------------*/
/*------------------------------------------------------------*/

/*---------------------------------------------*/


template<typename T>	/*pas de point virgule en fin de ligne...*/
/*Gauss transformation computing :
 * C : column matrix which corresponds to the column of the matrix that we want to zero from the k+1-th element to the end.
 * k : index of the element that is to be used as a pivot.
 */
Mat<T> gaussTrans( Mat<T> C, int k)
{
    if(k >= 1 && k<= C.getLine())
    {
        int n = C.getLine();
        Mat<T> M( 0, n,n);

        for(int i=1;i<=n;i++)	M.set((T)1, i,i);


        if(C.get(k,1) != 0)
        {
            T ipiv = 1.0/C.get(k,1);

            for(int i=k+1;i<=n;i++)	M.set( -C.get(i,1)*ipiv, i,k);
        }
        else	throw("ERREUR : gauss transformation : le pivot fourni est nul.");

        return M;
    }
    else	throw("ERREUR: mauvais parametre pour la transformation de gauss.");

    return Mat<T>( (T)0, 1, 1);
}

/*------------------------------------------------------------*/
/*------------------------------------------------------------*/
/*------------------------------------------------------------*/

/*---------------------------------------------*/


template<typename T>	/*pas de point virgule en fin de ligne...*/
Mat<T> hhTrans( Mat<T> x)
{
    int n = x.getLine();
    Mat<T> H((T)0, n,n);

    T sigma = (transpose( extract(x, 2,1, n,1))*extract(x, 2,1, n,1)).get(1,1);
    Mat<T> v( extract(x, 2,1, n,1), (T)1, 2, 1,  n, 1);
    T beta = 0;

    if(sigma != (T)0)
    {
        T mu = sqrt(x.get(1,1)*x.get(1,1)+sigma);

        if(x.get(1,1) <= (T)0)
            v.set( x.get(1,1) - mu, 1,1);
        else
            v.set( -sigma/(x.get(1,1) + mu), 1,1);


        beta = (T)(2*v.get(1,1)*v.get(1,1))/(sigma + v.get(1,1)*v.get(1,1));
        v = ((T)(1/v.get(1,1)))*v;

    }

    for(int i=1;i<=n;i++)	H.set( (T)1, i,i);

    H = H - beta*v*transpose(v);

    return H;

}


/*---------------------------------------------*/


template<typename T>	/*pas de point virgule en fin de ligne...*/
/*Householder transformation computing :
 * A : matrix whose k-th column corresponds to the column of the matrix that we want to zero from the k+1-th element to the end.
 * k : index of the element that is to be used as a pivot.
 */
Mat<T> hhTransMat( Mat<T> A, int k)
{
    int n = A.getLine();
    //int m = A.getColumn();
    Mat<T> H( hhTrans( extract( A, k,k, n, k ) ), (T)0, k, k, n, n );
    /*hhTrans renvoie une matrice de la taille n-k+1 x n-k+1 )*/

    for(int i=1; i<=k-1;i++)	H.set( (T)1, i, i);

    return H;
}



/*---------------------------------------------*/


template<typename T>	/*pas de point virgule en fin de ligne...*/
/*Wieland's hotelling transformation computing : returns the shifted matrix.
 * A : matrix whose  eigv associated eigenvalue will be shifted with regards to sigma : eigvValue := eigvValue - sigma.
 * eigv : eigenvectors whose assiocated eigenvalue will be shifted.
 * sigma : linear shifting value.
 * z : vectors which have an influence of the shifting of the right eigenvectors : rEigv := rEigv - gamma*eigv / gamma = sigma*(z.H*eigv)/(sigma - (eigvValue - rEigvValue)).
 */
Mat<T> WielandHotellingTrans( Mat<T> A, Mat<T> eigv, T sigma, Mat<T> z)
{
	Mat<T> r(A);
	
	if(eigv.getLine() == z.getLine() && eigv.getLine() == A.getLine() && A.getLine() == A.getColumn())
	{
		r = A - sigma*(eigv*transpose(z));
		//it should be the hermitian conjugate but for now one we only deal with real matrices so...
	}
	else
		cerr << "PROBLEM : Wieland Hotelling Transformation...." << endl;
	
	return r;
}


/*---------------------------------------------*/



template<typename T>	/*pas de point virgule en fin de ligne...*/
T RayleighQuotient(Mat<T> A, Mat<T> q)
{
	T r=0;
	
	if(A.getLine()==A.getColumn() && A.getLine()==q.getLine())
	{
		T temp = (transpose(q)*q).get(1,1);	// it should be a hermitian conjugate...
		
		if(temp != (T)0)
			r = (T) ((transpose(q)*A*q).get(1,1) / temp);
		else
			cerr << "PROBLEM : RAYLEIGHT QUOTIENT / 2" << endl;
	}
	else
		cerr << "PROBLEM : RAYLEIGHT QUOTIENT / 1" << endl;	

	return r;
}



/*------------------------------------------------------------*/
/*------------------------------------------------------------*/
/*------------------------------------------------------------*/


/*---------------------------------------------*/


template<typename T>	/*pas de point virgule en fin de ligne...*/
/*Perform Gauss Jordan Elimanation (1)full-pivoting (2)with backsubstitution : */

//#define constructInverseProduct

Mat<T> gaussj( Mat<T>* A_, Mat<T>* b_, int method)
{
    if( A_ != NULL && b_ != NULL)
    {
        Mat<T> A(*A_), b(*b_);        

        switch(method)
        {
            case 1 :
            {
            /* Full-Pivoting */
            int n, m;
            n = A.getLine();
            m = b.getColumn();
            T big, pivinv;

            /*bookkeeping */
            int irow=1, icol=1;
            Mat<T> indxc(n, 1);
            Mat<T> indxr( n,1);
            Mat<T> ipiv((T)0, n,1);

            #ifdef verbose
            cout << "matrice originale :" << endl;
            A.afficher();
            #endif

            Mat<T> I( (T)0, n,n);
            for(int i=n;i--;)
                I.set((T)1, i+1,i+1);

            for(int i=1;i<=n;i++)
            {
                big = 0.0;
                for(int j=1;j<=n;j++)
                {
                    if(ipiv.get(j,1) != (T)1)	/*si cette ligne j ne contient pas deja un pivot... */
                    {
                        for(int k=1;k<=n;k++)
                        {
                            if(ipiv.get(k,1) == 0) /* si cette ligne k ne contient pas deja un pivot..*/
                            {
                                if(fabs_(A.get(j,k)) > big)
                                {
                                    big = fabs_(A.get(j,k));
                                    irow = j;
                                    icol = k;
                                }

                            }
                        }

                    }
                }
                ipiv.set(ipiv.get(icol,1)+1, icol,1);
                /*on a bien un pivot à la icol ligne, car c'est la valeur la plus grande de */

                /*Maintenant que l'on a le pivot, il faut le mettre sur la diagonale en interchangeant les colonnes
                 * --> b doit changer de la meme manière que A ; l'inverse obtenue devra etre changee.
                 */

                indxr.set((T)irow, i,1) ;
                indxc.set((T)icol, i,1);
                if(irow != icol)
                {
                    for(int l=1;l<=n;l++)	A.swap( irow, l, icol, l);
                    for(int l=1;l<=n;l++)	I.swap( irow, l, icol, l);
                    for(int l=1;l<=m;l++)	b.swap( irow, l, icol, l);
                }

                /*on peut maintenant faire les divisions et soustractions qui vont bien. */
                if(A.get(icol,icol) == (T)0)
                {
#ifdef verbose
                    cerr << "Singular Matrix : diag element : " << i << endl;
                    A.afficher();
                    //throw("Singular Matrix...");
#endif
                    A.set( numeric_limits<T>::epsilon(), icol, icol);
                }

                pivinv = 1.0/A.get(icol,icol);
                for(int l=1;l<=n;l++)	A.set(A.get(icol,l)*pivinv, icol, l);
                for(int l=1;l<=n;l++)	I.set(I.get(icol,l)*pivinv, icol, l);
                for(int l=1;l<=m;l++)	b.set(b.get(icol,l)*pivinv, icol, l);

                for(int ll=1;ll<=n;ll++)
                {
                    if(ll != icol)
                    {
                        T dum = A.get(ll, icol);
                        /*A.set((T)0, ll, icol);*/
                        /*il semble que cette opération soit bien faite
                         * dans la boucle for qui suit.... ?
                         */
                        for(int l=1;l<=n;l++)	A.set( A.get(ll,l)-dum*A.get(icol,l), ll, l);
                        for(int l=1;l<=n;l++)	I.set( I.get(ll,l)-dum*I.get(icol,l), ll, l);
                        for(int l=1;l<=m;l++)	b.set( b.get(ll,l)-dum*b.get(icol,l), ll, l);

                    }
                }

                #ifdef verbose
                cout << "Step : " << i << endl;
                A.afficher();
                I.afficher();
                #endif

            }

/*-------------------------------------------------------------------------------------------------------------------*/
            /*unscrabling of the solution in view of the column interchanges by interchanging pairs of columns
             *in the reverse order of their permutation during the algorithm above :
             */
            /*

            for(int l=n;l>=1;l--)
            {
                //if the former indexes of the l-th pivot element were not the same
                //then permutations have ensuied and thus we have to reverse them.
                //
                if(indxr.get(l,1) != indxc.get(l,1))
                {
                    //we swap the indxr[l]-th column with the indxc[l]-th one.
                    for(int k=1;k<=n;k++)	A.swap(k, indxr.get(l,1), k, indxc.get(l,1));
                    for(int k=1;k<=n;k++)	I.swap(k, indxr.get(l,1), k, indxc.get(l,1));
                }
            }
	    */
/*------------------------------------------------------------------------------------------------------------*/
            /*------------------------------------*/
            /*construction de la matrice inverse avec un produit de matrice C*/
            /*-----------------------------------*/
            /*matrices C*/

#ifdef constructInverseProduct
	
            Mat<T>** C = NULL;
            C = new Mat<T>*[n];

            for(int i=0;i<=n-1;i++)
            {
                C[i] = new Mat<T>((T)0, n, n);

                C[i]->set( (T)1, indxr.get(i+1,1), indxc.get(i+1,1));
                C[i]->set( (T)1, indxc.get(i+1,1), indxr.get(i+1,1));

                for(int ii=1;ii<=n;ii++)
                    if(ii != indxc.get(i+1,1) && ii!=indxr.get(i+1,1))
                        C[i]->set((T)1, ii, ii);

                #ifdef verbose
                cout << "C : " << i << " : avec (i,j) pivot : (" << indxr.get(i+1,1) << "," << indxc.get(i+1,1) << ")" << endl;
                C[i]->afficher();
                #endif
            }

            Mat<T> Ai(*C[0]);
            for(int i=1;i<=n-1;i++)
                Ai = Ai*(*C[i]);

            /*-------------------------------*/

            for(int i=0;i<=n-1;i++)
                delete C[i];
            delete[] C;
#else
	     Mat<T> Ai(I);
#endif
	
            /*------------------------------------*/

            *A_ = I;
            *b_ = b;


            return Ai;

            }
            break;







            case 2 :
            /* backsubstitution : triangular decomposition */

            {
            /* Full-Pivoting */
            int n, m;
            n = A.getLine();
            m = b.getColumn();
            T big, pivinv;

            /*bookkeeping */
            int irow, icol;
            Mat<T> indxc( n, 1);
            Mat<T> indxr( n,1);
            //Mat<T> ipiv((T)0, n,1);

            #ifdef verbose
            cout << "matrice originale :" << endl;
            A.afficher();
            #endif
            Mat<T> I( (T)0, n,n);
            for(int i=1;i<=n;i++)
                I.set((T)1, i,i);

            for(int i=1;i<=n;i++)
            {
                big = 0.0;
                for(int j=1;j<=n;j++)
                {
                    //if(ipiv.get(j,1) != (T)1)	/*si cette ligne j ne contient pas deja un pivot... */
                    //{
                        //for(int k=1;k<=n;k++)
                        //{
                            //if(ipiv.get(k,1) == 0) /* si cette ligne k ne contient pas deja un pivot..*/
                            //{
                                if(fabs_(A.get(i,j)) > big)
                                {
                                    big = fabs_(A.get(i,j));
                                    irow = i;
                                    icol = j;
                                }

                            //}
                        //}

                    //}
                }
                //ipiv.set(ipiv.get(icol,1)+1, icol,1);
                /*on a bien un pivot à la irow ligne, car c'est la valeur la plus grande de la colonne i*/

                /*Maintenant que l'on a le pivot, il faut le mettre sur la diagonale en interchangeant les lignes
                 * --> b doit changer de la meme manière que A ; l'inverse obtenue devra etre changee.
                 */

                indxr.set((T)irow, i,1) ;
                indxc.set((T)icol, i,1);
                if(irow != icol)
                {
                    for(int l=1;l<=n;l++)	A.swap( irow, l, icol,l);
                    for(int l=1;l<=n;l++)	I.swap( irow, l, icol,l);
                    for(int l=1;l<=m;l++)	b.swap( irow, l, icol,l);// attention a l'indice et sa limite.
                }
                /*echange des lignes et non des colonnes.*/
                /*du coup il faut faire le changement juste après de icol --> irow car le pivot est en irow,irow !*/

                /*on peut maintenant faire les divisions et soustractions qui vont bien. */
                if(A.get(irow,irow) == 0.0)
                {
                	//throw("Singular Matrix...");
                	A.set( numeric_limits<T>::epsilon(), irow, irow);
                }

                pivinv = 1.0/A.get(irow,irow);
                for(int l=1;l<=n;l++)	A.set(A.get(irow,l)*pivinv, irow, l);
                for(int l=1;l<=n;l++)	I.set(I.get(irow,l)*pivinv, irow, l);
                for(int l=1;l<=m;l++)	b.set(b.get(irow,l)*pivinv, irow, l);

                for(int ll=1;ll<=n;ll++)
                {
                    if(ll > irow)	//petit changement, != --> > de sorte que l'on ne change que les lignes en dessous de la position du pivot du moment.
                    {
                        T dum = A.get(ll, irow);
                        /*A.set((T)0, ll, icol);*/
                        /*il semble que cette opération soit bien faite
                         * dans la boucle for qui suit.... ?
                         */
                        for(int l=1;l<=n;l++)	A.set( A.get(ll,l)-dum*A.get(irow,l), ll, l);
                        for(int l=1;l<=n;l++)	I.set( I.get(ll,l)-dum*I.get(irow,l), ll, l);
                        for(int l=1;l<=m;l++)	b.set( b.get(ll,l)-dum*b.get(irow,l), ll, l);

                    }
                }

                #ifdef verbose
                cout << "Step : " << i << endl;
                A.afficher();
                I.afficher();
                #endif
                
            }


            /*------------------------------------*/
            /*construction de la matrice inverse avec un produit de matrice C*/
            /*-----------------------------------*/
            /*matrices C*/
#ifdef constructInverseProduct 
            Mat<T>** C = NULL;
            C = new Mat<T>*[n];

            for(int i=0;i<=n-1;i++)
            {
                C[i] = new Mat<T>( (T)0, n, n);

                C[i]->set( (T)1, indxr.get(i+1,1), indxc.get(i+1,1));
                C[i]->set( (T)1, indxc.get(i+1,1), indxr.get(i+1,1));

                for(int ii=1;ii<=n;ii++)
                    if(ii != indxc.get(i+1,1) && ii!=indxr.get(i+1,1))
                        C[i]->set((T)1, ii, ii);

                #ifdef verbose
                cout << "C : " << i << " : avec (i,j) pivot : (" << indxr.get(i+1,1) << "," << indxc.get(i+1,1) << ")" << endl;
                C[i]->afficher();
                #endif
            }

            Mat<T> Ai(*C[n-1]);
            for(int i=n-2;i>=0;i--)
                Ai = Ai*(*C[i]);

            /*-------------------------------*/

            for(int i=0;i<=n-1;i++)
                delete C[i];
            delete C;
#else
	    Mat<T> Ai(A);
#endif	  
            /*------------------------------------*/
            /*------------------------------------*/
            /*------------------------------------*/
            /* BACKSUBSTITUTION */
            /*------------------------------------*/
            /*------------------------------------*/
            /*------------------------------------*/


            Mat<T> X(b);

            for(int j=1;j<=m;j++)
            {
                /*pour chaque colonne de b :*/
                for(int i=n;i>=1;i--)
                {
                    /*backsubstitution*/
                    T temp = 0;
                    T inv = 1.0/A.get(i,i);
                    for(int ii=i+1;ii<=n;ii++)
                        temp += A.get(ii,i)*X.get(ii,j);
                    X.set( inv*(b.get(i,j) - temp), i,j);
                }

            }

            /*-----------------------------------*/
            /*
            //unscrabling of the solution in view of the row interchanges by interchanging pairs of rows
            // in the reverse order of their permutation during the algorithm above :


            for(int l=n;l>=1;l--)
            {
                //if the former indexes of the l-th pivot element were not the same
                //then permutations have ensuied and thus we have to reverse them.
                //
                if(indxr.get(l,1) != indxc.get(l,1))
                {
                    //we swap the indxr[l]-th line with the indxc[l]-th one.*/ /*on parle bien de ligne cette fois...
                    for(int k=1;k<=n;k++)	A.swap( indxr.get(l,1), k, indxc.get(l,1), k);
                    for(int k=1;k<=n;k++)	I.swap( indxr.get(l,1), k, indxc.get(l,1), k);
                }
            }
            */
            /*------------------------------------*/

            *A_ = A;
            *b_ = b;


            return X;

            }
            break;



            default :
            throw("Mauvaise valeur de method.");
            break;
        }


    }
    else
        throw("Pointeurs arguments non-initializes.");

    return *A_;
}

/*---------------------------------------------*/


template<typename T>	/*pas de point virgule en fin de ligne...*/
Mat<T> Normalization(Mat<T> A, int method)
{

    T (*ptrnorme)(Mat<T>) = NULL;
    ptrnorme = norme2;
    if( method == 1)
    	ptrnorme = norme1;
    	
    int n = A.getColumn();
    Mat<T> I( (T)0, n,n);
    for(int i=1;i<=n;i++)
    {
        I.set((T)1, i,i);
    }

    Mat<T>* B = new Mat<T>();

    bool init = false;
    for(int it=1;it<=n;it++)
    {
        if(ptrnorme( Cola(A,it) ) != (T)0)
        {
            if(!init)
            {
                *B = ((T)(1./(T)ptrnorme( Cola(A,it) ) )) * Cola(A,it);
                init = true;
            }
            else
            {
                *B = operatorL( *B, ((T)(1./(T)ptrnorme( Cola(A,it) ) )) * Cola(A, it) );
            }
        }
        else
        {
            if(!init)
            {
                *B = ((T)0)*Cola(A, it);
                init = true;
            }
            else
            {
                *B = operatorL( *B, ((T)0)*Cola(A, it) );
            }
        }

        /*-----------------------*/
        /*
        cout << "Step :" << it << " avec 1/norme(X) : " << 1/ptrnorme(Cola(A,it)) << endl;
        A.afficher();
        T->afficher();
        */
        /*--------------------------*/

    }

    Mat<T> r(*B);
    delete B;


    return r;
}


template<typename T>	/*pas de point virgule en fin de ligne...*/
Mat<T> invGJ(const Mat<T>& mat, int method)
{
	Mat<T> r(mat);
	Mat<T> b((T)0, r.getLine(), 1);
	gaussj(&r,&b, method);
		
	return r;
}

template<typename T>	/*pas de point virgule en fin de ligne...*/
Mat<T> invGJ( Mat<T>* mat, int method)
{
	//Mat<T> r(*mat);
	Mat<T> b((T)0, mat->getLine(), 1);
	gaussj(mat,&b, method);
		
	return *mat;
}



/*------------------------------------------------------------*/
/*------------------------------------------------------------*/
/*------------------------------------------------------------*/


/*---------------------------------------------*/


template<typename T>	/*pas de point virgule en fin de ligne...*/
class LU
{

private :

    int n;
    int nbrM;
    Mat<T>* L_;
    Mat<T>* U_;
    Mat<T>* A_;
    Mat<T>** M;

public :

    /*LU decomposition :
     * A : matrix to be decomposed.
     * L : pointer initialized at NULL that is to be allocated and filled up with the L matrix by the function.
     * U : pointer initialized at NULL that is to be allocated and filled up with the U matrix by the function.
     */
    LU(Mat<T> A, Mat<T>* L, Mat<T>* U)
    {
        A_ = new Mat<T>(A);
        n = A.getColumn();
        nbrM = (n<= A.getLine() ? n : A.getLine());
        //M = (Mat<T>**)malloc(sizeof(Mat<T>*)*(nbrM-1));
        M = new Mat<T>*[nbrM-1];
        L_ = NULL;
        U_ = NULL;

        if(L != NULL || U != NULL)	throw("ERREUR : LU decomposition : mauvais pointeurs de matrices L et/ou U.");

        //L =(Mat<T>*)malloc(sizeof(Mat<T>));
        //*L = Mat<T>( (T)0, A.getLine(), A.getLine());
        L = new Mat<T>( (T)0, A.getLine(), A.getLine());
        for(int i=1;i<=L->getLine();i++)	L->set( (T)1, i,i);
        //U = (Mat<T>*)malloc(sizeof(Mat<T>));
        //*U = Mat<T>(A);
        U = new Mat<T>(A);



        for(int i=1;i<= nbrM-1;i++)
        {
            M[i-1] = new Mat<T>(gaussTrans( Col(*U, i), i));
            *U = (*M[i-1])*(*U);

            cout << "Step " << i << " : " << endl;
            M[i-1]->afficher();
            U->afficher();

            Mat<T> invM(M[i-1]->getInv());
            *L = (*L)*invM;
        }

        cout << "L" << endl;
        L->afficher();

        cout << "U" << endl;
        U->afficher();

        cout << " Produit " << endl;
        ((*L)*(*U)).afficher();

        L_ = new Mat<T>(*L);
        U_ = new Mat<T>(*U);


    }

    /*----------------------------------*/
    /*----------------------------------*/
    /*----------------------------------*/
    /*----------------------------------*/

    /*LU decomposition :
     * A : matrix to be decomposed.
     */
    LU(Mat<T> A)
    {
        A_ = new Mat<T>(A);
        n = A.getColumn();
        nbrM = (n<= A.getLine() ? n-1 : A.getLine()-1);
        //M = (Mat<T>**)malloc(sizeof(Mat<T>*)*(nbrM-1));
        M = new Mat<T>*[nbrM-1];


        //L_ =(Mat<T>*)malloc(sizeof(Mat<T>));
        //*L_ = Mat<T>((T)0, A.getLine(), A.getLine());
        L_ = new Mat<T>( (T)0, A.getLine(), A.getLine());
        for(int i=1;i<=L_->getLine();i++)	L_->set( (T)1, i,i);
        //U_ = (Mat<T>*)malloc(sizeof(Mat<T>));
        //*U_ = Mat<T>(A);
        U_ = new Mat<T>(A);

        for(int i=1;i<=nbrM-1;i++)
        {
            M[i-1] = new Mat<T>(gaussTrans( Col(*U_, i), i));
            *U_ = (*M[i-1])*(*U_);

            //cout << "Step " << i << " : " << endl;
            //M[i-1]->afficher();
            //U_->afficher();

            Mat<T> invM(M[i-1]->getInv());
            *L_ = (*L_)*invM;
        }
    }



    /*----------------------------------*/
    /*----------------------------------*/
    /*----------------------------------*/
    /*----------------------------------*/



    ~LU()
    {
        for(int i=0; i<=nbrM-2;i++)	delete M[i];
        delete M;

        if( L_ != NULL)	delete L_;
        if( U_ != NULL)	delete U_;
        if( A_ != NULL)	delete A_;

        cout << "LU : exited cleanly." << endl;
    }



    /*----------------------------------*/
    /*----------------------------------*/
    /*----------------------------------*/
    /*----------------------------------*/

    Mat<T> getL() const
    {
        return *L_;
    }

    Mat<T> getU() const
    {
        return *U_;
    }


};




/*---------------------------------------------*/
/*---------------------------------------------*/
/*---------------------------------------------*/
/*---------------------------------------------*/


template<typename T>	/*pas de point virgule en fin de ligne...*/
class QR
{

private :

    int n;
    int nbrM;
    Mat<T>* Q_;
    Mat<T>* R_;
    Mat<T>* A_;
    Mat<T>** M;

public :

    /*QR decomposition :
     * A : matrix to be decomposed.
     * Q : pointer initialized at NULL that is to be allocated and filled up with the Q matrix by the function.
     * R : pointer initialized at NULL that is to be allocated and filled up with the R matrix by the function.
     */
    QR(Mat<T> A, Mat<T>* Q, Mat<T>* R)
    {
        A_ = new Mat<T>(A);
        n = A.getColumn();
        nbrM = (n<= A.getLine() ? n : A.getLine());
        //M = (Mat<T>**)malloc(sizeof(Mat<T>*)*(nbrM-1));
        M = new Mat<T>*[nbrM-1];
        Q_ = NULL;
        R_ = NULL;

        if(Q != NULL || R != NULL)	throw("ERREUR : QR decomposition : mauvais pointeurs de matrices L et/ou U.");

        //Q =(Mat<T>*)malloc(sizeof(Mat<T>));
        //*Q = Mat<T>((T)0, A.getLine(),A.getLine());
        Q = new Mat<T>( (T)0, A.getLine(),A.getLine());
        for(int i=1;i<=Q->getLine();i++)	Q->set( (T)1, i,i);
        //R = (Mat<T>*)malloc(sizeof(Mat<T>));
        //*R = Mat<T>(A);
        R = new Mat<T>(A);


        for(int i=1;i<=nbrM-1;i++)
        {
            M[i-1] = new Mat<T>( hhTransMat( *R, i));
            *R = (*M[i-1])*(*R);

            cout << "Step " << i << " : " << endl;
            M[i-1]->afficher();
            R->afficher();

            Mat<T> invM(transpose(*M[i-1]));//M[i-1]->getInv());
            *Q = (*Q)*invM;
        }

        cout << "Q" << endl;
        Q->afficher();

        cout << "R" << endl;
        R->afficher();

        cout << " Produit " << endl;
        ((*Q)*(*R)).afficher();

        Q_ = new Mat<T>(*Q);
        R_ = new Mat<T>(*R);


    }

    /*----------------------------------*/
    /*----------------------------------*/
    /*----------------------------------*/
    /*----------------------------------*/

    /*QR decomposition :
     * A : matrix to be decomposed.
     */
    QR(Mat<T> A)
    {
        A_ = new Mat<T>(A);
        n = A.getColumn();
        nbrM = (n<= A.getLine() ? n : A.getLine());
        M = new Mat<T>*[nbrM-1];


        Q_ = new Mat<T>( (T)0, A.getLine(), A.getLine() );
        for(int i=1;i<=Q_->getLine();i++)	Q_->set( (T)1, i,i);
        R_ = new Mat<T>(*A_);


        for(int i=1;i<=nbrM-1;i++)
        {
            M[i-1] = new Mat<T>( hhTransMat( *R_, i));
            *R_ = (*M[i-1])*(*R_);

            #ifdef verbose
            cout << "Step " << i << " : " << endl;
            M[i-1]->afficher();
            R_->afficher();
            cout << "Inversion ..." << endl;
            #endif


            //Mat<T> invM(M[i-1]->getInv());            
            *Q_ = (*Q_)*transpose(*M[i-1]);
        }
    }



    /*----------------------------------*/
    /*----------------------------------*/
    /*----------------------------------*/
    /*----------------------------------*/



    ~QR()
    {
        for(int i=0; i<=nbrM-2;i++)	delete M[i];
        delete[] M;

        if( Q_ != NULL)	delete Q_;
        if( R_ != NULL)	delete R_;
        if( A_ != NULL)	delete A_;

#ifdef verbose
        cout << "QR : exited cleanly." << endl;
#endif        
    }



    /*----------------------------------*/
    /*----------------------------------*/
    /*----------------------------------*/
    /*----------------------------------*/

    Mat<T> getQ() const
    {
        return *Q_;
    }

    Mat<T> getR() const
    {
        return *R_;
    }


};








/*---------------------------------------------*/


template<typename T>	/*pas de point virgule en fin de ligne...*/
class SVD
{

    private :

    Mat<T>* A_;
    QR<T>* qr1;
    QR<T>* qr;
    Mat<T>* U;
    Mat<T>* S;
    Mat<T>* V;

    public :

    SVD(Mat<T> A, int iteration = 100)
    {
        /* QR decomposition */
        qr1 = new QR<T>(A);
        Mat<T> Q1(qr1->getQ());
        Mat<T> R1(qr1->getR());

#ifdef verbose
        cout << "//////////////////////////////" << endl;
        cout << "Matrice Q1 : " << endl;
        Q1.afficher();
        cout << "Matrice R1 : " << endl;
        R1.afficher();
        /*-------------------*/

        cout << endl << "Decomposition QR : DONE !!!" << endl << endl;
#endif
        /* QR decomposition of R1 */
        qr = new QR<T>(transpose(R1));
        Mat<T> Q(qr->getQ());
        Mat<T> R(qr->getR());
#ifdef verbose
        cout << "Matrice Q : " << endl;
        Q.afficher();
        cout << "Matrice R : " << endl;
        R.afficher();
        /*---------------------*/

        cout << endl << "Decomposition QR de R.t : DONE !!!" << endl << endl;

        cout << "Verification : " << endl;
        cout << "Matrice A :" << endl;
        A.afficher();
        cout << "Produit : " << endl;
        (Q1*transpose(R)*transpose(Q)).afficher();
        //cout << "Verification de l'orthogonalité de Q1 : " << endl;
        //(Q1*transpose(Q1)).afficher();
        cout << "Verification de D : " << endl;
        transpose(R).afficher();
        //cout << "Verification de l'orthogonalité de Q : " << endl;
        //(Q*transpose(Q)).afficher();                
        
        /*----------------------------------------*/
        /*----------------------------------------*/
        /*----------------------------------------*/
        cout << "ASSIGNATION ..." << endl;
        R.afficher();
#endif


        Mat<T> D(transpose(R));
        QR<T>* rec = new QR<T>(D);
        Mat<T> Qr(rec->getQ());
        Mat<T> rtemp(rec->getR());
        delete rec;
        rec = new QR<T>(transpose(rtemp));
        Mat<T> tQl(transpose(rec->getQ()));
        D = transpose( rec->getR());

        Mat<T> error(D);
        int dimmax = (error.getLine() > error.getColumn() ? error.getLine() : error.getColumn());
        for(int i=1;i<= dimmax;i++)	error.set((T)0, i,i);
        T eps =  numeric_limits<T>::epsilon();

        while( iteration >= 0 && sum(sum(absM(error))).get(1,1) > eps*1e30)
        {
            //cout << "Iteration : " << iteration-- << " : error : " << sum(sum(error)).get(1,1) << " / " << eps << endl;
            delete rec;
            rec = new QR<T>(D);
            Qr = Qr * rec->getQ();
            rtemp = rec->getR();
            delete rec;
            rec = new QR<T>(transpose(rtemp));
            tQl = transpose(rec->getQ()) * tQl;
            D = transpose( rec->getR());
#ifdef verbose
            D.afficher();
#endif

            /////////////////
            error = D;
            dimmax = (error.getLine() > error.getColumn() ? error.getLine() : error.getColumn());
            for(int i=1;i<= dimmax;i++)	error.set((T)0, i,i);


        }
        delete rec;

        Qr = Q1*Qr;
        tQl = tQl*transpose(Q);

#ifdef verbose
        cout << "Methode itérative : " << endl;
        cout << "Matrice originale :" << endl;
        A.afficher();
        cout << "Produit : " << endl;
        (Qr*D*tQl).afficher();
        cout << " U :" << endl;
        Qr.afficher();
        cout << " S :" << endl;
        D.afficher();
        cout << " V :" << endl;
        transpose(tQl).afficher();
#endif        
        ///////////////////////////////////////////////////////////////
	/*
        A_ = new Mat<T>(A);
        U = new Mat<T>(Q1);
        S = new Mat<T>(transpose(R));
        V = new Mat<T>(Q);
        */
        //cout << "Initialement : D vaut :" << endl;
        //transpose(R).afficher();
        
        A_ = new Mat<T>(A);
        U = new Mat<T>(Qr);
        S = new Mat<T>(D);
        V = new Mat<T>(transpose(tQl));

        /*nettoyage */
        /*rien... that's how the cookie crumble x) !! */

    }

    ~SVD()
    {
        delete A_;
        delete qr1;
        delete qr;
        delete U;
        delete S;
        delete V;
    }

    Mat<T> getU() const
    {
        return *U;
    }

    Mat<T> getS() const
    {
        return *S;
    }

    Mat<T> getV() const
    {
        return *V;
    }

};


template<typename T>    /*pas de point virgule en fin de ligne...*/
Mat<T> invSVD(const Mat<T> mat)
{    
    SVD<T> instance(mat,1000);    
    Mat<T> Si(instance.getS());
    
    //Si.afficherM();
    for(int i=Si.getLine();i--;)
    {
    	for(int j=Si.getColumn();j--;)
    	{
    		if(i==j)
    		{
			if(Si.get(i+1,i+1) != (T)0)
			    Si.set( (T)1.0/Si.get(i+1,i+1), i+1,i+1);
			else
			{
                Si.set( (T)1.0/(numeric_limits<T>::epsilon()), i+1,i+1);
			}            
		}		
	}
    }
    
    Mat<T> ret(instance.getV());
    ret = ret * Si;
    //ret.afficherM();
    ret = ret * transpose( instance.getU() );
    return ret;
}

template<typename T>
T factoriel(T k)
{
	T res=1;
	for(int i=1;i<=k;i++)	res*=i;
	
	if(k!=(T)0)
		return res;
	
	return 1;
}

template<typename T>
Mat<T> expMat(const Mat<T> m)
{
	SVD<T> instanceSVD(m,1000);
	Mat<T> S(instanceSVD.getS());
	Mat<T> U(instanceSVD.getU());
	(transpose(U)*m*U).afficher();
	/*
	for(int i=1;i<=S.getLine();i++)
	{
		for(int j=1;j<=S.getColumn();j++)
		{
			if(i!=j)
				S.set((T)0,i,j);
			else
			{
				//S.set( (T)exp(S.get(i,j) ), i,j);
			}
		}
	}
	S.afficher();
	

	Mat<T> e((T)0,S.getLine(),S.getColumn());
	Mat<T> temp(e);
	e=e+S;
	for(int i=e.getLine();i--;)	e.set((T)1,i+1,i+1);
	for(int k=2;k<=0;k++)
	{
		temp = (T)(1.0/k)*S*temp;
		//S.afficher();
		e = e + temp;

	}
	
	*/
	return U*transpose(instanceSVD.getV()) ;//instanceSVD.getU()*S*transpose(instanceSVD.getV()); 
}


/*---------------------------------------------*/


template<typename T>	/*pas de point virgule en fin de ligne...*/
class PIt 	//Power iteration class : eigenvalues are the squared ones if the initial matrix is not a square matrix...
		// Assuming that the eigenvalues are real and different from one to another...
{

    private :

    int n;		//A square dimension.
    Mat<T> A_;		// square matrix
    Mat<T> leigvec; 	//left eigenvectors.	column stack
    Mat<T> reigvec;	//right eigenvectors.	column stack
    Mat<T> eigval;	//eigenvalues.	Column


    int iteration;	//nbr of iteration for convergance...	default = 10;
    
    public :

    PIt(Mat<T> A)
    {        
        n = A.getColumn();
        A_ = (n==A.getLine() ? A : transpose(A)*A);	//if the matrix is not square, we square it... dim : n x n.
        
        leigvec = Mat<T>((T)0, n, 1);
        reigvec = Mat<T>((T)0, n, 1);
        eigval = Mat<T>((T)0, 1,1);
        
        iteration = 100;
        
        /*Main loop*/
        Mat<T> tempPoweredMat(A_);
        Mat<T> initQ( Normalization( Mat<T>((T)1,n,1), 2));	//Euclidian normalization.
        for(int i=1;i<=n;i++)
        {
        	powerIt( tempPoweredMat, initQ, &reigvec, &leigvec, &eigval, iteration);
        	
        	tempPoweredMat = WielandHotellingTrans( tempPoweredMat, Cola(reigvec, i+1), eigval.get(i+1,1), Cola(leigvec, i+1) );
        	//watch out the offset...        	
        }
        
        /*-------------------------*/
        
        
        /*remise en forme...*/
        leigvec = extract(leigvec, 1,2, n,n+1);
        reigvec = extract(reigvec, 1,2, n,n+1);
        eigval = extract(eigval, 2,1, n+1,1);
        
    }

    ~PIt()
    {
       
    }
    
    
    /*--------------------------------------------------------------------------*/
    
    
    static void powerIt( Mat<T> tempPoweredMat, Mat<T> initQ, Mat<T>* reigvec, Mat<T>* leigvec, Mat<T>* eigval, int iteration)
    {
    	Mat<T> A(tempPoweredMat);
    	Mat<T> ltemp(transpose(tempPoweredMat));
    	Mat<T> rq(initQ);	//already normalized.
    	Mat<T> lq(initQ);
    	
    	T eigv = 0;
    	
    	/*
    	for(int i=1;i<=iteration;i++)
    	{
    		rq = tempPoweredMat*rq;
    		rq = Normalization(rq, 2);
    		
    		lq = ltemp*lq;
    		lq = Normalization(lq, 2);
    	}
    	*/
    	
    	T epsilon = (T)10e-4;
   	T lambdat = 0;
   	while( iteration-- != 0 && fabs_(eigv-lambdat) > epsilon)
   	{
   		lambdat = eigv;
    		rq = tempPoweredMat*rq;
    		rq = Normalization(rq, 2);
    		
    		lq = ltemp*lq;
    		lq = Normalization(lq, 2);
    		
    		//eigv = RayleighQuotient(A, rq);
    		eigv = norme2(A*rq)/norme2(rq);
    		
    		//cout << eigv << " et |lambdat - eigv| = " << fabs_(eigv-lambdat) << endl;
    	}
   	
   	
    	//eigv = RayleighQuotient(A, rq);
    	eigv = norme2(A*rq)/norme2(rq);
    	
    	/*ASSIGNATION*/
    	*eigval = operatorC(*eigval, Mat<T>((T)eigv, 1,1) );
    	*leigvec = operatorL(*leigvec, lq);
    	*reigvec = operatorL(*reigvec, rq);
    	    	    
    }
    
    
    /*---------------------------------------------------------------------------*/
    /*---------------------------------------------------------------------------*/
    /*---------------------------------------------------------------------------*/
    /*---------------------------------------------------------------------------*/

    Mat<T> getLeigvec() const
    {
        return leigvec;
    }

    Mat<T> getReigvec() const
    {
        return reigvec;
    }

    Mat<T> getEigval() const
    {
        return eigval;
    }

};


template<typename T>
Mat<T> computeSmallestNonZeroEig(Mat<T> A, T* eigval)
{
	PIt<T> instance(A);
	Mat<T> eigv(instance.getEigval());
	int dimin = eigv.getLine();
	int idmin = 1;
        T vmin = eigv.get(1,1);
        
        int i = 1;
        while(vmin == (T)0)
        {
	        i++;
        	vmin = eigv.get(i,1);
        }
        
        T eps = numeric_limits<T>::epsilon();

        /*on va chercher le vecteur correspondant à la plus petite valeur propre non-nulle de Ag*/
        for(int i=1;i<=dimin;i++)
        {
            T temp = eigv.get(i,i);
            idmin = ( temp <= vmin && temp >= eps*10e2 ? i : idmin);
            vmin = ( temp <= vmin && temp >= eps*10e2 ? temp : vmin);
        }
	
	if(eigval != NULL)
		*eigval = vmin;
		
	return Cola( instance.getLeigvec(), idmin);
}

/*--------------------------------------------------------------------------------*/
/*--------------------------------------------------------------------------------*/
/*--------------------------------------------------------------------------------*/
/*--------------------------------------------------------------------------------*/
/*--------------------------------------------------------------------------------*/
/*--------------------------------------------------------------------------------*/
/*--------------------------------------------------------------------------------*/
/*--------------------------------------------------------------------------------*/
/*--------------------------------------------------------------------------------*/
/*--------------------------------------------------------------------------------*/
/*--------------------------------------------------------------------------------*/
/*--------------------------------------------------------------------------------*/
/*--------------------------------------------------------------------------------*/
/*--------------------------------------------------------------------------------*/
/*--------------------------------------------------------------------------------*/
/*--------------------------------------------------------------------------------*/

/*homoproj.cpp templated*/

/*---------------------------------------------*/


template<typename T>	/*pas de point virgule en fin de ligne...*/
Mat<T> crossProductHomography( Mat<T> C1, Mat<T> C2)
{
    Mat<T> r1( (T)0, 1, 3);
    Mat<T> r2(C1.get(3,1)*C2);

    r1 = operatorL( operatorL(r1, -C1.get(3,1)*C2), C1.get(2,1)*C2);
    r2 = operatorL( operatorL(r2, Mat<T>((T)0, 1,3) ), -C1.get(1,1)*C2);

    return operatorC(r1,r2);
}



/*---------------------------------------------*/


template<typename T>	/*pas de point virgule en fin de ligne...*/
Mat<T> MiseEnFormeProjection( Mat<T> C1, Mat<T> C2)
{
    int n = C2.getLine();
    Mat<T> r1(((T)(-1))*transpose(C2));
    //Mat<T> r1(transpose(C2));
    Mat<T> r2((T)0, 1, n);

    //r1 = operatorL( operatorL( r1, Mat<T>( (T)0, 1,n) ), ((T)(-1))*C1.get(1,1)*transpose(C2));
    r1 = operatorL( operatorL( r1, Mat<T>( (T)0, 1,n) ), C1.get(1,1)*transpose(C2));    
    //r2 = operatorL( operatorL( r2, transpose(C2) ), ((T)(-1))*C1.get(2,1)*transpose(C2) );
    r2 = operatorL( operatorL( r2, ((T)(-1))*transpose(C2) ), C1.get(2,1)*transpose(C2) );

    return operatorC(r1,r2);
}




/*----------------------------------------------*/


template<typename T>
Mat<T> MiseEnFormeRetroProjection(Mat<T> C1, Mat<T> C2)
{
    int n = C2.getLine();
    Mat<T> r1(((T)(-1))*transpose(C2));
    Mat<T> r2((T)0, 1, n);
    Mat<T> r3(operatorL(r2,r2));

    r1 = operatorL( operatorL( operatorL( r1, r2 ), r2), C1.get(1,1)*transpose(C2));
    r2 = operatorL( operatorL( operatorL( r2, ((T)(-1))*transpose(C2) ), r2), C1.get(2,1)*transpose(C2) );
    r3 = operatorL( r3, operatorL( ((T)(-1))*transpose(C2), C1.get(3,1)*transpose(C2)) );

    return operatorC( operatorC(r1,r2), r3);
}

/*---------------------------------------------*/


template<typename T>	/*pas de point virgule en fin de ligne...*/
/*compute Homography/Projection between correspondances of X1 and X2 's columns 2D or 3D
 * X1 : points in the image in homogeneous coordinates  that are stacked as columns of this matrix.
 * X2 : corresponding points in the image, or in the 3D space, in homogeneous coordinate  that are stacked as columns of this matrix.
 * method : SVD of a squared matrix (2) or non-squared matrix(1).
 * Singular Value Decomposition of the system created by stacking the #(columns of X1) systems :
 * the solution is the vector associated with the smallest non-zero singular value.
 *
 *
 * The Homography matrix in homogeneous coordinates is returned : H.
 */

Mat<T> computeHomoProj(Mat<T> X1, Mat<T> X2, int method = 1)
{
    Mat<T> H( (T)0, 1,1);    
    Mat<T> (*ptrFonction)(Mat<T>,Mat<T>);

    if( X1.getColumn() == X2.getColumn() || X1.getColumn() == X2.getColumn()+1 || X1.getColumn()+1 == X2.getColumn())
    /*gestion du cas d'homographie 2D ou d'une projection 2D->3D ou 3D->2D.*/
    {
        int nbrP = X1.getColumn();

        /*en supposant que les points sont en coordonnées homogènes sur les colonnes de X1 et X2*/
        homogeneousNormalization(&X1);
        homogeneousNormalization(&X2);
        //mettre les valeurs de la dernière coordonnée a 1.
        cout << "X1 : " << endl;
        X1.afficher();
        cout << "/---------------------------------/" << endl;
        cout << "X2 : " << endl;
        X2.afficher();
        cout << "/---------------------------------/" << endl;

        /*choix du type de fonction a utiliser en fonction de ce que l'on fait.*/
        if(X1.getLine() == X2.getLine())
            ptrFonction = crossProductHomography;
        else if( (X1.getLine()+1) == X2.getLine())
            ptrFonction = MiseEnFormeProjection;
        else
            ptrFonction = MiseEnFormeRetroProjection;

        Mat<T>* Ag = NULL;
        Mat<T>** A = new Mat<T>*[nbrP];
        for(int i=0;i<= nbrP-1;i++)
        {

            /*creation des matrices liée au produit en croix xi' X Hxi = 0 */
            A[i] = new Mat<T>( (*ptrFonction)( Cola(X1, i+1), Cola(X2, i+1) ) );
            /*attention : il est impératif que les points 3D si on calcul une projection se trouve dans la matrice X2.*/

            /*column stacking*/
            if(i==0)
                Ag = new Mat<T>(*A[0]);
            else
                *Ag = operatorC( *Ag, *A[i]);
        }

        /*creation du systeme : done */
        if(Ag->getLine() > Ag->getColumn())
        {
            //*Ag = transpose(*Ag);
            //cout << "Ag possede plus de ligne que de colonne... On l\'a transposée." << endl;
        }
        cout << "Matrice Ag :" << endl;
        Ag->afficher();        
        /*------------------------------*/

        int mode = 1;
        switch(mode)
        {
            case 0 :
        {
            /*resolution de Ag*h = 0.0 : gaussj*/
            Mat<T> solb((T)10e-15, Ag->getLine(), 1);
            Mat<T> invtAA(transpose(*Ag)*(*Ag));
            gaussj(&invtAA, &solb, 1);

            /*reecriture*/
            if(X1.getLine() == X2.getLine() )
            {
                H = operatorC( operatorC( transpose(extract(solb, 1,1, 3,1)), transpose(extract(solb, 4,1, 6,1))), transpose(extract(solb, 7,1, 9,1)));
            }
            else/* dans ce cas, h possede 12 lignes et H est une matrice 3x4.*/
            {
                if( X1.getLine()+1==X2.getLine())
                    H = operatorC( operatorC( transpose(extract(solb, 1,1, 4,1)), transpose(extract(solb, 5,1, 8,1)) ), transpose(extract(solb, 9,1, 12,1)) );
                else
                    H = operatorC( operatorC( operatorC( transpose(extract(solb, 1,1, 3,1)), transpose(extract(solb, 4,1, 6,1)) ), transpose(extract(solb, 7,1, 9,1)) ), transpose(extract(solb, 10,1, 12,1)));
            }

        }
            break;



            case 1 :
            /*resolution de Ag*h = 0 : svd de Ag*/
            /*method 1: non-sqared matrix ; 2:squared matrix.*/
            SVD<T> instance((method==1? *Ag : transpose(*Ag)*(*Ag)), 100);
            Mat<T> U(instance.getU());
            Mat<T> S(instance.getS());
            Mat<T> V(instance.getV());
            /*------*/
            cout << "Matrice U :" << endl;
            U.afficher();
            cout << "Matrice S :" << endl;
            S.afficher();
            cout << "Matrice V :" << endl;
            V.afficher();

            /*------*/

            int dimin = (S.getLine() <= S.getColumn() ? S.getLine() : S.getColumn() ); /*a bit useless in the second method though.*/
            /* car rank(S) <= min(dimAColumn, dimALine). Inutile d'aller chercher plus loin sur la diagonale, on sait que les valeurs sont nulles. A moins que la matrice n'ai pas ses valeurs singulières d'ordonnées ?*/
            int idmin = dimin;
            T vmin = S.get(1,1);
            T eps = numeric_limits<T>::epsilon();

            /*on va chercher le vecteur correspondant à la plus petite valeur singulière non-nulle de Ag*/
            for(int i=1;i<=dimin;i++)
            {
                T temp = S.get(i,i);
                idmin = ( temp <= vmin /*&& temp >= eps*10e4*/ ? i : idmin);
                vmin = ( temp <= vmin /*&& temp >= eps*10e4*/ ? temp : vmin);
            }

            /*solution*/            
            Mat<T> h1( Cola(V, idmin));

            /*reecriture*/
            if(X1.getLine() == X2.getLine() )
            {
                H = operatorC( operatorC( transpose(extract(h1, 1,1, 3,1)), transpose(extract(h1, 4,1, 6,1))), transpose(extract(h1, 7,1, 9,1)));                
            }            
            else/* dans ce cas, h possede 12 lignes et H est une matrice 3x4.*/
            {
                if( X1.getLine()+1==X2.getLine())
                {
                    H = operatorC( operatorC( transpose(extract(h1, 1,1, 4,1)), transpose(extract(h1, 5,1, 8,1)) ), transpose(extract(h1, 9,1, 12,1)) );
                }
                else
                {
                    H = operatorC( operatorC( operatorC( transpose(extract(h1, 1,1, 3,1)), transpose(extract(h1, 4,1, 6,1)) ), transpose(extract(h1, 7,1, 9,1)) ), transpose(extract(h1, 10,1, 12,1)));
                }
            }

            break;
        }
        cout << "Matrice Homography :" << endl;
        cout << "H (issue de V) : " << endl;
        H.afficher();

#ifdef verbose
        cout << "Test :" << endl;
        for(int c=1;c<=X2.getColumn();c++)
        {
            (Cola(X2,c)).afficher();
            Mat<float> temp(H*extract(X2, 1,c, X2.getLine(), c));
            homogeneousNormalization(&temp);
            temp.afficher();
            (Cola(X1,c)).afficher();
        }
#endif
        /*------------------------------*/
        /*nettoyage*/
        for(int i=0;i<=nbrP-1;i++)	delete A[i];
        delete A;
        delete Ag;

    }
    else
    {
        cerr << "ERREUR : les nombres de points correspondants ne correspondent pas, ni pour une homographie, ni pour une projection." << endl;
    }


    return H;
}


/*------------------------------------------------------------*/
/*------------------------------------------------------------*/
/*------------------------------------------------------------*/
/*------------------------------------------------------------*/



#endif
