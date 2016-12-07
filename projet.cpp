#include <iostream>
#include <cstdlib>
#include <time.h>
#include <sstream>
#include <opencv2/opencv.hpp>

using namespace cv;
using namespace std;


void process(const char* ims)
{
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

