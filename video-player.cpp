#include <iostream>
#include <cstdlib>
#include <cstdio>
#include <dirent.h>
#include <opencv2/opencv.hpp>
#include <fstream>
#include <ctime>

using namespace cv;
using namespace std;

float progress = 0.0;
vector<Mat> images;
unsigned int image_counter;
RNG rng(12345);

int hasValue(Mat image) {
  int val=0;

  if (image.cols == 0 || image.rows == 0) {
    return 0;
  }
  for(int j=0;j<image.cols;j++) {
    for(int i=0;i<image.rows;i++) {
      if( (int) image.at<uchar>(i,j) != 0)
	return 1;
    }
  }
  return 0;
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

void patchAll3(const Mat src1, const Mat src2, const Mat src3, Mat& dst) {
  int taille_patch = 1;
  src1.copyTo(dst);
  for(int i=0;i<src1.rows;i++) {
    for(int j=0;j<src1.cols;j++) {
      Mat pix1 = getPixelMatrix(src1,j,i,taille_patch);
      Mat pix2 = getPixelMatrix(src2,j,i,taille_patch);
      Mat pix3 = getPixelMatrix(src3,j,i,taille_patch);
      if( hasValue(pix1) && hasValue(pix2) && hasValue(pix3)) {
	dst.at<uchar>(i,j) = 255;
      } else {
	dst.at<uchar>(i,j) = 0;
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
  
  for(int i = 0; i <contours.size();i++)
    {
      int tmpperimeter = arcLength(contours[i],1);
      approxPolyDP(contours[i], approxM,0.02 * tmpperimeter, 1);
      if(tmpperimeter > perimeters && approxM.rows  <= 4)
	{
	  perimeters = tmpperimeter;
	  cntIndex = i;
	}
      
    }
  
  /// Draw contours
  Mat drawing = Mat::zeros( detectionTerrainCanny.size(), CV_8UC3 );
  Scalar color = Scalar( rng.uniform(0, 255), rng.uniform(0,255), rng.uniform(0,255) );     
  drawContours( drawing, contours, cntIndex, color, 2, 8, hierarchy, 0, Point() );

  cvtColor(drawing,drawing,CV_BGR2GRAY);
  for(int i=0;i<drawing.rows;i++) {
    for(int j=0;j<drawing.cols;j++) {
      int pixel = drawing.at<uchar>(i,j);
      if(pixel > 0)
	drawing.at<uchar>(i,j) = 255;
    }
  }
  drawing.copyTo(dst);
}
void detectionCouleur(const Mat im, Mat &dst)
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
	 
	 //imshow("couleur",imgTh);
	 imgTh.copyTo(dst);

}

void init() {
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
bool tooMuchGrass(Mat grass) {
  int green_pixel = 0;
  for (int i=0;i<grass.cols;i++) {
    if ( (int) grass.at<uchar>(0,i) == 255)
      green_pixel++;
    if ( (int) grass.at<uchar>(1,i) == 255)
      green_pixel++;

    if ( (int) grass.at<uchar>(grass.rows-1,i) == 255)
      green_pixel++;
    if ( (int) grass.at<uchar>(grass.rows-2,i) == 255)
      green_pixel++;
  }
  for (int i=0;i<grass.rows;i++) {
    if ( (int) grass.at<uchar>(i,0) == 255)
      green_pixel++;
    if ( (int) grass.at<uchar>(i,1) == 255)
      green_pixel++;
    if ( (int) grass.at<uchar>(i,grass.cols-1) == 255)
      green_pixel++;
    if ( (int) grass.at<uchar>(i,grass.cols-2) == 255)
      green_pixel++;
  }
  int max_pixel = 4 * (grass.cols + grass.rows);
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
	
	
	if(contours.size() <= 3)
	{
		for (int i = 0; i < contours.size(); i++)
		{
			Point2f center;
			float radius;
			minEnclosingCircle(contours[i],center,radius);
			circle(originale, center, (int)radius*5, Scalar(0,0,255));
		}
	}
	
	
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
  int cptG = 0;
  int cptB = 0;
  clock_t begin = clock();
  while(1)
    {
      Mat image;
      int found = getNextMatrix(image);
      if (found == 0)
	break;
      //copyMakeBorder(image,image,2,2,2,2,BORDER_CONSTANT,Scalar(0,0,0));
      //Hough(image);
      /************ convert a BGR ->  Grey *******************/
      Mat grey;
      cvtColor(image, grey, CV_BGR2GRAY);
      threshold( grey, grey, 120, 255, 0 );
 
      
      //imshow("seuil", grey);
      
      /************ show BGR channels *******************/

      Mat BGR[3];
      split(image,BGR);
      //imshow("B",BGR[0]);
      //imshow("G",BGR[1]);
      //imshow("R",BGR[2]);

      /************ show HSV channels *******************/
      /** find posts **/
      Mat hsv;
      cvtColor(image,hsv,CV_BGR2HSV);
      Mat HSV[3];
      split(hsv,HSV);
      Mat new_h;
      Mat new_S;
      thBetween(HSV[0],new_h,25,30);
      //threshold( HSV[0], HSV[0], 20, 255, 0 );
      //imshow("H",new_h);
      thBetween(HSV[1],new_S,180,255);

      Mat HS;
      thAnd(new_h,new_S,HS);
      erode(HS,HS,cv::Mat());
      dilate(HS,HS,cv::Mat());

      /** find grass **/
      Mat grass;
      detectionCouleur(image,grass);
      
      //imshow("grass", grass);
      erode(grass,grass,cv::Mat());
      dilate(grass,grass,cv::Mat());
      dilate(grass,grass,cv::Mat());

      /*** if too much grass on the corner , no posts ***/
      if ( !tooMuchGrass(grass)) {
      
      Mat countour;
      contoursTerrain(grass,countour);
      //imshow("contour", countour);
      Mat input[3];
      input[0] = HS;
      input[1] = grass;
      input[2] = countour;
      Mat output;
      merge(input,3,output);
      //imshow("output",output);
		
      Mat last;
      patchAll3(grass, HS, countour, last);
      Mat element = getStructuringElement(MORPH_ELLIPSE, Size(5,5), Point(0, 0) );
      dilate(last,last,element);
      //imshow("last",last);
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
