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

void process() {
  while(1)
    {
      Mat image;
      int found = getNextMatrix(image);
      if (found == 0)
	break;
      imshow("toto", image);
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
