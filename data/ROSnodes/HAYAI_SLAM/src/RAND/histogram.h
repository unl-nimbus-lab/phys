#ifndef HISTOGRAM_H
#define HISTOGRAM_H
#include <iostream>
#include "Mat.h"

template<typename T>
class BHT
{
    private :

    Mat<T>* img;
    Mat<T>* histo_;
    T tresh;

    public :

    BHT(Mat<T> img_)
    {
        /*initialisation*/
        img = new Mat<T>(img_);
        histo_ = new Mat<T>( histogram(*img) );        
        /*BHT*/
        tresh = BHTreshold( *histo_);
    }

    ~BHT()
    {
        delete img;
        delete histo_;
    }

    static Mat<T> histogram( Mat<T> img_)
    {
        Mat<T> r((T)0, 256, 1);

        for(int i=1;i<=img_.getLine();i++)
        {
            for(int j=1;j<=img_.getColumn();j++)
                r.set( r.get( img_.get(i,j)+1, 1) +1 , img_.get(i,j)+1, 1);
        }

        return r;
    }

    static T BHTreshold( Mat<T> histo)
    {
        T is = (T)1;
        T ie = (T)256;
        T im = (T)128;

        T wl = getW((int)is, (int)im, histo);
        T wr = getW((int)im+1, (int)ie, histo);


        while(is < ie)
        {
#ifdef verbose
            cout << " Treshold = " << im << endl;
            cout << " wr = " << wr << " & wl = " << wl << " ; is = " << is << " & ie = " << ie << endl;
#endif
            if( wr > wl)
            {
                wr = wr - histo.get( (int)ie, 1);
                ie-=1;
                T temp = (T)(is+ie)/(T)2;

                if( temp < im)
                {
                    wr += histo.get( (int)im, 1);
                    wl -= histo.get( (int)im, 1);
                    im-=1;
                }                
                else
                {
                    wr -= histo.get( (int)im, 1);
                    wl += histo.get( (int)im, 1);
                    im+=1;
                }
            }
            else
            {
                wl = wl - histo.get((int)is,1);
                is+=1;
                T temp = (T)(is+ie)/(T)2;

                if( temp > im)
                {
                    wl += histo.get((int)(im+1),1);
                    wr -= histo.get((int)(im+1),1);
                    im+=1;
                }                
                else
                {
                    wl -= histo.get((int)(im+1),1);
                    wr += histo.get((int)(im+1),1);
                    im-=1;
                }
            }
        }        

        return im;
    }

    static T getW( int idd, int ide, Mat<T> histo)
    {                
        cout << ide-idd+1 << endl;
        Mat<T> temp((T)0, ide-idd+1, 1);        
        for(int i=1;i<=ide-idd+1;i++)
            temp.set( (T)histo.get(idd+i-1, 1), i,1);

        T r = (sum(temp).get(1,1));
        return r;
    }

    T getTresh() const
    {
        return tresh;
    }

    Mat<T> getHisto()
    {
        return *histo_;
    }

    /*
    void afficher()
    {
        int height = 300;
        Mat<T> im((T)0, height, 256);
        int mx = max(*histo_);

        for(int i=0;i<=255;i++)
        {
            int j=1;

            while( (i!=tresh ? (histo_->get(i,1)-(j-1) != 0) : (j!=256)) )
            {
                int line = (int)(((double)(j*height))/((double)mx));
                im.set( (i!=tresh ? (T)255 : (T)100), height-line+1, i+1);
                j++;
            }


        }

        cv::Mat h = Mat2cvAt(im,im,im);
        bool continuer = true;
        cv::namedWindow("Histogramme");

        cout << "Affichage de l'histogramme : " << endl;
        histo_->afficher();

        while(continuer)
        {
            cv::imshow("Histogramme", h);

            if(cv::waitKey(30) >= 0)
                continuer = false;
        }


    }
    */
};

#endif // HISTOGRAM_H
