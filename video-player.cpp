#include <iostream>
#include <cstdlib>
#include <cstdio>
#include <dirent.h>
#include <opencv2/opencv.hpp>
#include <fstream>

using namespace cv;
using namespace std;

vector<Mat> images;
unsigned int image_counter;
void Hough(Mat);
void Corner(Mat);
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
  Canny( src, detectionTerrainCanny, threshCanny, threshCanny*2, 3 );
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


void Corner(Mat im)
{
  Mat dst;
  Mat dst_norm,dst_norm_scaled;
  dst = Mat::zeros( im.size(), CV_32FC1 );
  /// Detector parameters
  int blockSize = 2;
  int apertureSize = 3;
  double k = 0.04;
  int thresh = 150;

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
              }
          }
     }
  /// Showing the result
  imshow( "coin", dst_norm_scaled );
}

void Hough(Mat im) {
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
	
	
  for( size_t i = 0; i < lines.size(); i++ )  {
      Vec4i l = lines[i];
      line( cdst, Point(l[0], l[1]), Point(l[2], l[3]), Scalar(0,0,255), 3, CV_AA);
  }
	
  imshow("Hough",cdst);
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
  cout << images.size() << " images found" << endl;
}

int getNextMatrix(Mat& M) {

  if (image_counter < images.size()) {
    M = images[image_counter];
    cout << image_counter << endl;
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

void process() {
  while(1)
    {
      Mat image;
      int found = getNextMatrix(image);
      if (found == 0)
	break;

      //Hough(image);
      /************ convert a BGR ->  Grey *******************/
      Mat grey;
      cvtColor(image, grey, CV_BGR2GRAY);
      threshold( grey, grey, 120, 255, 0 );
 
      
      //      imshow("toto", grey);
      
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
      //imshow("HS",HS);   <- bonne image
      //imshow("S",new_S);
      //imshow("V",HSV[2]);

      /** find grass **/
      Mat grass;
      detectionCouleur(image,grass);
      erode(grass,grass,cv::Mat());
      dilate(grass,grass,cv::Mat());
      
      Mat countour;
      contoursTerrain(grass,countour);
      Mat input[3];
      input[0] = HS;
      input[1] = grass;
      input[2] = countour;
      Mat output;
      merge(input,3,output);
      imshow("output",output);

      Mat last;
      patchAll3(grass, HS, countour, last) ;
      imshow("last",last);

      
      /************* HoughLines *******************/
      
      imshow("origin",image);
      waitKey();
    }
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
