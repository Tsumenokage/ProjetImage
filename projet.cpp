#include <iostream>
#include <cstdlib>
#include <time.h>
#include <sstream>
#include <opencv2/opencv.hpp>
#include <opencv2/nonfree/features2d.hpp>
#include <opencv2/nonfree/nonfree.hpp>
#include <opencv2/legacy/legacy.hpp>

using namespace cv;
using namespace std;

void Hough(Mat);
void Corner(Mat);
void CornerFast(Mat);
void CornerSURF(Mat);

Mat originale;
Mat imageCorner;
Mat detectionTerrain;
int threshCanny = 100;
RNG rng(12345);
int cntIndex = 0;
vector<vector<Point> > contours;
vector<Point> contourLong;
vector<Point> coins;
vector<Point> coinsBut;

void seuillageOtsu(Mat im)
{
	Mat seuille;
	Mat imG;
	cvtColor(im,imG,COLOR_BGR2GRAY);
	threshold(imG,seuille,0,255,THRESH_BINARY_INV+THRESH_OTSU);
	imshow("Otsu",seuille);
}

void detectionCouleur(Mat im)
{
	 Mat imgTh;
	 Mat imHSV;
	 int iLowH = 30;
	 int iHighH = 80;

	 int iLowS = 0; 
	 int iHighS = 255;

	 int iLowV = 0;
	 int iHighV = 255;
	 cvtColor(im,imHSV,CV_BGR2HSV);
	 inRange(imHSV, Scalar(iLowH, iLowS, iLowV), Scalar(iHighH, iHighS, iHighV), imgTh); //Threshold the image
	 
	 //Hough(im,imgTh);
	 detectionTerrain = imgTh;
	 imshow("couleur",imgTh);

}


void Hough(Mat im)
{
	
	 /************ convert a BGR ->  Grey *******************/
     Mat grey;
     cvtColor(im, grey, CV_BGR2GRAY);
     threshold( grey, grey, 175, 255, 0 );
     imshow("seuil", grey);
	
	Mat outCanny;
	Mat cdst;
	im.copyTo(cdst);
	Canny(grey, outCanny, 0, 255, 3);
	imshow("Canny",outCanny);
	Corner(outCanny);
	CornerFast(outCanny);
	CornerSURF(outCanny);
	
	
	vector<Vec4i> lines;
	HoughLinesP(outCanny, lines, 1, CV_PI/180, 50, 50, 5 );
	//vector<Vec2f> lines;
	//HoughLines(outCanny, lines, 1, CV_PI/180, 100, 0, 0 );
	/*
	  for( size_t i = 0; i < lines.size(); i++ )
  {
     float rho = lines[i][0], theta = lines[i][1];
     Point pt1, pt2;
     double a = cos(theta), b = sin(theta);
     double x0 = a*rho, y0 = b*rho;
     pt1.x = cvRound(x0 + 1000*(-b));
     pt1.y = cvRound(y0 + 1000*(a));
     pt2.x = cvRound(x0 - 1000*(-b));
     pt2.y = cvRound(y0 - 1000*(a));
     line( cdst, pt1, pt2, Scalar(0,0,255), 3, CV_AA);
  }*/
	
	
for( size_t i = 0; i < lines.size(); i++ )
{
  Vec4i l = lines[i];
  line( cdst, Point(l[0], l[1]), Point(l[2], l[3]), Scalar(0,0,255), 3, CV_AA);
}
	
	imshow("Hough",cdst);

}


void CornerFast(Mat im)
{
	Mat dst;
	vector<KeyPoint> detectP;
	originale.copyTo(dst);
	FastFeatureDetector fast(150,0);
	fast.detect(im,detectP);
	
	cout << detectP.size() << endl;
	
	for (int i = 0; i < detectP.size(); i++)
	{
		 circle( dst, detectP[i].pt, 5,  Scalar(0), 2, 8, 0 );
	}
	imshow( "coinFAST", dst );
	
}

void CornerSURF(Mat im)
{
	SURF surf(400);
	vector<KeyPoint> detectP;
	Mat dst;
	originale.copyTo(dst);
	surf.detect(im,detectP);
	
	for (int i = 0; i < detectP.size(); i++)
	{
		 circle( dst, detectP[i].pt, 5,  Scalar(0), 2, 8, 0 );
	}
	
	imshow("SURF", dst);
}

void Corner(Mat im)
{
  Mat dst;
  Mat dst_norm,dst_norm_scaled;
  dst = Mat::zeros( im.size(), CV_32FC1 );
  /// Detector parameters
  int blockSize = 2;
  int apertureSize = 3;
  double k = 0.04;
  int thresh = 100;

  /// Detecting corners
  cornerHarris( im, dst, blockSize, apertureSize, k, BORDER_DEFAULT );
  
    /// Normalizing
  normalize( dst, dst_norm, 0, 255, NORM_MINMAX, CV_32FC1, Mat() );
  convertScaleAbs( dst_norm, dst_norm_scaled );
  
    /// Drawing a circle around corners
  for( int j = 0; j < dst_norm.rows ; j++ )
     { for( int i = 0; i < dst_norm.cols; i++ )
          {
            if( (int) dst_norm.at<float>(j,i) > thresh )
              {
               circle( dst_norm_scaled, Point( i, j ), 5,  Scalar(0), 2, 8, 0 );
               coins.push_back(Point(i,j));
              }
          }
     }
  /// Showing the result
  imageCorner = dst_norm_scaled;
  imshow( "coin", dst_norm_scaled );
}

void contoursTerrain()
{
	Mat detectionTerrainCanny;
    vector<Vec4i> hierarchy;
    int perimeters = 0;
    Canny( detectionTerrain, detectionTerrainCanny, threshCanny, threshCanny*2, 3 );
    findContours( detectionTerrainCanny, contours, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE, Point(0, 0) );
  
  for(int i = 0; i <contours.size();i++)
  {
	  int tmpperimeter = arcLength(contours[i],1);
	  if(tmpperimeter > perimeters)
	  {
		  perimeters = tmpperimeter;
		  cntIndex = i;
	  }
	  
  }
    
  /// Draw contours
  Mat drawing = Mat::zeros( detectionTerrainCanny.size(), CV_8UC3 );
  Scalar color = Scalar( rng.uniform(0, 255), rng.uniform(0,255), rng.uniform(0,255) );     
  drawContours( drawing, contours, cntIndex, color, 2, 8, hierarchy, 0, Point() );
  /*
  for( int i = 0; i< contours.size(); i++ )
     {
       Scalar color = Scalar( rng.uniform(0, 255), rng.uniform(0,255), rng.uniform(0,255) );
       
       drawContours( drawing, contours, i, color, 2, 8, hierarchy, 0, Point() );
     }*/

  /// Show in a window
  namedWindow( "Contours", CV_WINDOW_AUTOSIZE );
  imshow( "Contours", drawing );
}

/*
void Distance()
{
	int distance = 999999;
	int seuilleDistance = 100;
	Point Pproche;
	Mat cornerButPic;
	originale.copyTo(cornerButPic);
	
	for (int i = 0; i < coins.size(); i++)
	{
		for (int j = 0; j < contours[cntIndex].size(); j++)
		{
			if(norm(coins[i],contours[cntIndex][j]) < distance);
			{
				Pproche = contours[cntIndex][j];
				distance = norm(coins[i],contours[cntIndex][j]);
			}
		}
		if(distance < seuilleDistance)
		{
				coinsBut.push_back(coins[i]);
		}
	}
	

     for (int i = 0; i < coinsBut.size(); i++)
	 {
		 circle( cornerButPic,coinsBut[i], 5,  Scalar(0,0,0), 2, 8, 0 );
	 }
	 
	
}*/

void process(const char* ims)
{
	Mat im = imread(ims);
	im.copyTo(originale);
	Hough(im);
	detectionCouleur(im);
	contoursTerrain();
	imshow(ims,im);
	
	
	waitKey(0);
}


void usage (const char *s)
{
	cerr << "Usage:" << s << " ims" << endl;
	exit(EXIT_FAILURE);	
}

#define param 1

int main (int argc, char* argv[])
{
	if(argc != (param+1))
		usage(argv[0]);
	process(argv[1]);
	return EXIT_SUCCESS;
		
}

