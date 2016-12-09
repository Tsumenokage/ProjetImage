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
      Mat hsv;
      cvtColor(image,hsv,CV_BGR2HSV);
      Mat HSV[3];
      split(hsv,HSV);
      Mat new_h;
      Mat new_S;
      thBetween(HSV[0],new_h,25,30);
      //threshold( HSV[0], HSV[0], 20, 255, 0 );
      imshow("H",new_h);
      thBetween(HSV[1],new_S,180,255);

      Mat HS;
      thAnd(new_h,new_S,HS);
      imshow("HS",HS);
      imshow("S",new_S);
      //imshow("V",HSV[2]);

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
