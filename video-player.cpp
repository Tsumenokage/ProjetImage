#include <iostream>
#include <cstdlib>
#include <cstdio>
#include <dirent.h>
#include <opencv2/opencv.hpp>
#include <fstream>
#include <ctime>
#include <list>

using namespace cv;
using namespace std;

float progress = 0.0;
vector<Mat> images;
unsigned int image_counter;
RNG rng(12345);
int bordure = 5;

int hasValue(Mat image) {
  int count = 0;
  if (image.cols == 0 || image.rows == 0) {
    return 0;
  }
  for(int j=0;j<image.cols;j++) {
    for(int i=0;i<image.rows;i++) {
      if( (int) image.at<uchar>(i,j) != 0)
	count ++;
    }
  }
  return count;
}

Mat getPixelMatrix(Mat image, int cols, int rows, int radius) {
  int minC, maxC, minR, maxR;
  
  if ( cols-radius < 0) minC = 0; else minC = cols-radius;
  if ( cols+radius > image.cols) maxC = image.cols; else maxC = cols+radius;

  if ( rows-radius < 0) minR = 0; else minR = rows-radius;
  if ( rows+radius > image.rows) maxR = image.rows; else maxR = rows+radius;
 
  Mat submatrix = image.colRange(minC,maxC).rowRange(minR,maxR);    
  return submatrix;
}

/* follow src3, ignore src1 */
void patchAll3(const Mat src1, const Mat src2, const Mat src3, Mat& dst) {
  int taille_patch = 10;
  src1.copyTo(dst);
  dst = Mat::zeros( src1.size(), CV_8UC1);
  for(int i=0;i<src1.rows;i++) {
    for(int j=0;j<src1.cols;j++) {
      if( (int) src3.at<uchar>(i,j) == 255) {
	Mat pix1 = getPixelMatrix(src1,j,i,taille_patch);
	Mat pix2 = getPixelMatrix(src2,j,i,taille_patch);
	Mat pix3 = getPixelMatrix(src3,j,i,taille_patch);
	if( (hasValue(pix1) == 0) && (hasValue(pix2) >100) && (hasValue(pix3) > 50)) {
	  dst.at<uchar>(i,j) = 255;
	  cout << hasValue(pix1) << " " << hasValue(pix2) << " " << hasValue(pix3) <<endl;
	} 
      }
    }
  }
}

void contoursTerrain(const Mat src, Mat &dst)
{
  int threshCanny = 100;
  vector<vector<Point> > contours;
  Mat detectionTerrainCanny;
  vector<Vec4i> hierarchy;
  int perimeters = 0;
  int cntIndex = 0;
  Mat approxM;
  /*
  Mat dilateC;
  int element_shape = MORPH_ELLIPSE;
  Mat element = getStructuringElement(element_shape, Size(11,11), Point(0, 0) );
  dilate( src, dilateC, element );
  detectionTerrainCanny = dilateC-src;
  */

  Canny( src, detectionTerrainCanny, threshCanny, threshCanny*2, 3 );
  findContours( detectionTerrainCanny, contours, hierarchy, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE, Point(0, 0) );
  
  vector<vector<Point> >hull( contours.size());
   for( unsigned int i = 0; i < contours.size(); i++ )
      {  convexHull( Mat(contours[i]), hull[i], false );}
  
     /// Draw contours + hull results
   Mat hullP = Mat::zeros( src.size(), CV_8UC3 );
   for( unsigned int i = 0; i< contours.size(); i++ )
      {
        Scalar color = Scalar( rng.uniform(0, 255), rng.uniform(0,255), rng.uniform(0,255) );
        //drawContours( hullP, contours, i, color, 1, 8, vector<Vec4i>(), 0, Point() );
        drawContours( hullP, hull, i, color, 1, 8, vector<Vec4i>(), 0, Point() );
      }
   imshow( "Hull demo", hullP );
  
  for( unsigned int i = 0; i <hull.size();i++)
    {
		  int tmpperimeter = arcLength(hull[i],1);
		  if(tmpperimeter > perimeters)
		{
		  perimeters = tmpperimeter;
		  cntIndex = i;
		}
      
    }
  
  /// Draw contours
  Mat drawing = Mat::zeros( detectionTerrainCanny.size(), CV_8UC3 );
  Scalar color = Scalar( rng.uniform(0, 255), rng.uniform(0,255), rng.uniform(0,255) );     
  drawContours( drawing, hull, cntIndex, color, 2, 8, hierarchy, 0, Point() );

  cvtColor(drawing,drawing,CV_BGR2GRAY);
  for(int i=0;i<drawing.rows;i++) {
    for(int j=0;j<drawing.cols;j++) {
      int pixel = drawing.at<uchar>(i,j);
      if(pixel > 0)
	drawing.at<uchar>(i,j) = 255;
    }
  }
  // si ligne du haut avec 255, return pas de contours
  int ok = 0;
  for(int j=50;j<drawing.cols-50;j++) {
    if( (int) drawing.at<uchar>(bordure+1,j) != 255) {
      ok = 1;
      break;
    }
  }
  if (ok == 0)
    dst = Mat::zeros( detectionTerrainCanny.size(), CV_8UC1 );
  else {
    drawing.copyTo(dst);
  }
}
void detectionCouleur(const Mat im, Mat &dst)
{
	 Mat imgTh;
	 Mat imHSV;
	 int iLowH = 22;
	 int iHighH = 60;

	 int iLowS = 50; 
	 int iHighS = 215;

	 int iLowV = 40;
	 int iHighV = 175;
	 cvtColor(im,imHSV,CV_BGR2HSV);
	 inRange(imHSV, Scalar(iLowH, iLowS, iLowV), Scalar(iHighH, iHighS, iHighV), imgTh); //Threshold the image
	 
	 //Hough(im,imgTh);
	 
	 //imshow("couleur",imgTh);
	 imgTh.copyTo(dst);

}

void init() {
  cout << "Initialization" << endl;
  image_counter = 1;
  ifstream img_list;
  string image;
  img_list.open("./images/log1/log1.txt");
  if (img_list.is_open()) {
    while ( getline (img_list,image) ) {
      Mat M = imread("./images/log1/"+image);
      images.push_back ( M );
    }
    img_list.close();
  }
  //cout << images.size() << " images found" << endl;
}

/*
 * Grass is threashold matrix
 */
bool tooMuchGrass(const Mat grass) {
  int green_pixel = 0;
  for (int i=0;i<grass.cols;i++) {
	  cout << "first "<<i<<endl;
    if ( (int) grass.at<uchar>(0+bordure,i) == 255)
      green_pixel++;
    if ( (int) grass.at<uchar>(1+bordure,i) == 255)
      green_pixel++;

    if ( (int) grass.at<uchar>(grass.rows-1-bordure,i) == 255)
      green_pixel++;
    if ( (int) grass.at<uchar>(grass.rows-2-bordure,i) == 255)
      green_pixel++;
  }
  for (int i=0;i<grass.rows;i++) {
	  cout << "sec "<< i+bordure <<" "<<grass.rows<<endl;
    if ( (int) grass.at<uchar>(i,0+bordure) == 255)
      green_pixel++;
    if ( (int) grass.at<uchar>(i,1+bordure) == 255)
      green_pixel++;
    if ( (int) grass.at<uchar>(i,grass.cols-1-bordure) == 255)
      green_pixel++;
    if ( (int) grass.at<uchar>(i,grass.cols-2-bordure) == 255)
      green_pixel++;
  }
  int max_pixel = 4 * (grass.cols-(bordure*2) + grass.rows-(bordure*2));
  cout << "green "<< ((float)green_pixel)/max_pixel << endl;
  imshow("grass",grass);
 
  if (green_pixel > 0.8 * max_pixel)
    return true;
  return false;
}


int getNextMatrix(Mat& M) {

  if (image_counter < images.size()) {
    M = images[image_counter];
    //cout << image_counter << endl;
    image_counter++;
    return 1;
  } else {
    return 0;
  }

}
void thBetween(const Mat src, Mat& dst, int min, int max) {
  src.copyTo(dst);
  for(int i=0;i<src.rows;i++) {
    for(int j=0;j<src.cols;j++) {
      int pixel = (int) src.at<uchar>(i,j);
      if( min <= pixel && max >= pixel) {
        dst.at<uchar>(i,j) = 255;
      } else {
        dst.at<uchar>(i,j) = 0;
      }
    }
  }
}

void thAnd(const Mat src1, const Mat src2, Mat &dst) {
  src1.copyTo(dst);
  for(int i=0;i<src1.rows;i++) {
    for(int j=0;j<src1.cols;j++) {
      int pixel1 = (int) src1.at<uchar>(i,j);
      int pixel2 = (int) src2.at<uchar>(i,j);
      if( pixel1 == 255 && pixel2 == 255) {
        dst.at<uchar>(i,j) = 255;
      } else {
        dst.at<uchar>(i,j) = 0;
      }
    }
  }
}

void contourBlob(Mat src, Mat originale)
{
	vector<vector<Point> > contours;
	vector<Vec4i> hierarchy;
	findContours( src, contours, hierarchy, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE, Point(0, 0) );
	vector<Point> centers;
	vector<float> radiuss;
	if(contours.size() <= 300)
	{
		for (unsigned int i = 0; i < contours.size(); i++)
		{
			Point2f center;
			float radius;
			minEnclosingCircle(contours[i],center,radius);
			centers.push_back(center);
			radiuss.push_back(radius);
			circle(originale, center, (int)radius*5, Scalar(0,0,255));
		}
		
		
	}
	
}

int zoneColor(const Mat grass, Mat& current, int color_pixel, int x, int y) {
  list<int> abs;
  list<int> ord;
  abs.push_back(x);
  ord.push_back(y);
  int size = 0;
  while (abs.size() != 0) {
    int i = abs.front();
    int j = ord.front();

    if ( (int) grass.at<uchar>(i,j) == 255 && current.at<uchar>(i,j) == 0) {
      current.at<uchar>(i,j) = color_pixel;
      size++;
      
      //haut
      if( i!=0 && (int) grass.at<uchar>(i-1,j) == 255 && current.at<uchar>(i-1,j) == 0) {
	abs.push_back(i-1);
	ord.push_back(j);
      }

      //bas
      if( i!=(grass.rows-1) && (int) grass.at<uchar>(i+1,j) == 255 && current.at<uchar>(i+1,j) == 0) {
	abs.push_back(i+1);
	ord.push_back(j);
      }

      //gauche
      if( j!=0 && (int) grass.at<uchar>(i,j-1) == 255 && current.at<uchar>(i,j-1) == 0) {
	abs.push_back(i);
	ord.push_back(j-1);
      }

      //droite
      if( j!=(grass.cols-1) && (int) grass.at<uchar>(i,j+1) == 255 && current.at<uchar>(i,j+1) == 0) {
	abs.push_back(i);
	ord.push_back(j+1);
      }

    }
    abs.pop_front();
    ord.pop_front();
  }
  return size;
}

struct cPoint {
  int x;
  int y;
};

int orientation(cPoint p, cPoint q, cPoint r) {
  int val = (q.y - p.y) * (r.x - q.x) - (q.x - p.x) * (r.y - q.y);
  if (val == 0)
    return 0;
  return (val>0) ? 1:2;
}
void jarvisSlave(cPoint points[], int n, const Mat grass) {
  cout << "begin slave" << endl;
  if(n<3) return;
  int next[n];

  // init
  for(int i=0;i<n;i++) {
    next[i] = -1;
  }
  // Find the leftmost point
  int l = 0;
  for (int i = 1; i < n; i++)
    if (points[i].x < points[l].x)
      l = i;

  // Start from leftmost point, keep moving counterclockwise
  // until reach the start point again

  cout << "begin boucle" << endl;
  int p = l, q;
  do {
    // Search for a point 'q' such that orientation(p, i, q) is
    // counterclockwise for all points 'i'
    q = (p + 1) % n;
    for (int i = 0; i < n; i++)
      if (orientation(points[p], points[i], points[q]) == 2)
	q = i;
    next[p] = q; // Add q to result as a next point of p
    p = q; // Set p as q for next iteration
  }
  while (p != l);
  cout << "end boucle" << endl;
  
  //display
  Mat jarvis_mat = Mat::zeros( grass.size(), CV_8UC1);
  for(int i =0; i<n;i++) {
    if(next[i] + -1) {
      jarvis_mat.at<uchar>(points[i].x,points[i].y) = 255;
    }
  }
  imshow("jarvis",jarvis_mat);
}
	
void jarvis(const Mat grass, int size) {
  cPoint points[size];
  int cur = 0;
  for(int i=0;i<grass.rows;i++) {
    for(int j=0;j<grass.cols;j++) {
      if ( (int) grass.at<uchar>(i,j) == 255) {
	points[cur].x = i;
	points[cur].x = j;
	cur++;
      }
    }
  }
  jarvisSlave(points,size,grass);
}


void grassProcessing(const Mat grass, Mat & dst)  {
  Mat current;
  current = Mat::zeros( grass.size(), CV_8UC1);
  int color_pixel = 1;
  int biggest_size = 0;
  int biggest_color = 1;
  for(int i=0;i<grass.rows;i++) {
    for(int j=0;j<grass.cols;j++) {
      cout << i << " hey " << j << endl;
      if ( (int) grass.at<uchar>(i,j) == 255 && (int) current.at<uchar>(i,j) == 0 ) {
	int size_color = zoneColor(grass,current, color_pixel,i,j);
	if (size_color > biggest_size) {
	  biggest_size = size_color;
	  biggest_color = color_pixel;
	}
	color_pixel++;
      }
    }
  }
  cout << "color max " << color_pixel <<endl;

  cout << "size" << grass.cols*grass.rows<< " max" << biggest_size << endl;
  imshow("current_green",current);

  Mat biggest =  Mat::zeros( grass.size(), CV_8UC1);
  for(int i=0;i<grass.rows;i++) {
    for(int j=0;j<grass.cols;j++) {
      if ( (int) current.at<uchar>(i,j) == biggest_color) {
	biggest.at<uchar>(i,j) = 255;
      }
    }
  }

  // jarvis(biggest,biggest_size);
  biggest.copyTo(dst);
  imshow("biggest_green",biggest);
}
	


void progressBar()
{
    int barWidth = 70;
	
    std::cout << "[";
    int pos = barWidth * progress;
    for (int i = 0; i < barWidth; ++i) {
        if (i < pos) std::cout << "=";
        else if (i == pos) std::cout << ">";
        else std::cout << " ";
    }
    std::cout << "] " << int(progress * 100.0) << " %\r";
    std::cout.flush();
	

std::cout << std::endl;
}

void process() {
  //int cptG = 0;
  //int cptB = 0;
  clock_t begin = clock();
  while(1)
    {
	  
      Mat image;
      int found = getNextMatrix(image);
      if (found == 0)
	break;

      
      copyMakeBorder(image,image,bordure,bordure,bordure,bordure,BORDER_CONSTANT,Scalar(0,0,0));

      /************ show HSV channels *******************/
      /** find posts **/
      Mat hsv,HS;
      cvtColor(image,hsv,CV_BGR2HSV);
      inRange(hsv, Scalar(20, 175, 0), Scalar(255, 255, 255), HS); //Threshold the image
      erode(HS,HS,cv::Mat());
      dilate(HS,HS,cv::Mat());
      /*
      Mat HSV[3];
      split(hsv,HSV);
      Mat new_h;
      Mat new_S;
      thBetween(HSV[0],new_h,23,30);
      Mat element = getStructuringElement(MORPH_ELLIPSE, Size(4,4), Point(0, 0) );
      erode(new_h,new_h,element);
      //threshold( HSV[0], HSV[0], 20, 255, 0 );
      //imshow("H",new_h);
      thBetween(HSV[1],new_S,170,255);

      Mat HS;
      thAnd(new_h,new_S,HS);
      erode(HS,HS,cv::Mat());
      dilate(HS,HS,cv::Mat());
      */
      /** find grass **/
      Mat grass;
      detectionCouleur(image,grass);
      grassProcessing(grass,grass); 
      medianBlur(grass,grass,7);
      
      //imshow("grass", grass);
      
      erode(grass,grass,cv::Mat());
      erode(grass,grass,cv::Mat());
      dilate(grass,grass,cv::Mat());
      /*
      dilate(grass,grass,cv::Mat());
      dilate(grass,grass,cv::Mat());
      */
      /*** if too much grass on the corner , no posts ***/
      if ( !tooMuchGrass(grass)) {
	//grassProcessing(grass);
      Mat countour;
      contoursTerrain(grass,countour);
      //imshow("contour", countour);
      Mat input[3];
      input[0] = HS;
      input[1] = grass;
      input[2] = countour;
      Mat output;
      merge(input,3,output);
      imshow("output",output);
		
      Mat last;
      patchAll3(grass, HS, countour, last);
      Mat element = getStructuringElement(MORPH_ELLIPSE, Size(5,5), Point(0, 0) );
      dilate(last,last,element);
      imshow("last",last);
      contourBlob(last,image);
      
      /************* HoughLines *******************/
      
      //imshow("origin",image);
      //waitKey();
      progress = (float)image_counter/(float)images.size();
      progressBar();
      }
      imshow("origin",image);
      waitKey();
    }
    clock_t end = clock();
    double elapsed_secs = double(end - begin) / CLOCKS_PER_SEC;
    
    cout << "Numbers of pictures : " << images.size() << endl;
    cout << "Computing time : " << elapsed_secs << "s" <<endl;
    cout << "Time per images : " << elapsed_secs/images.size() << "s" << endl;
    cout << "Frames per seconds : " << 1.0/(elapsed_secs/images.size()) << endl;
}

void usage (const char *s) {
  cerr << "Usage " << s << endl;
  exit(EXIT_FAILURE);
}

#define param 0
int main(int argc, char* argv[]) {
  if (argc != (param+1))
    usage(argv[0]);
  init();
  process();
  return EXIT_SUCCESS;
}
