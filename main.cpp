#include "./calib/stereocalib.h"

int main(int argc, char** argv) {
	int nboards = atoi(argv[1]);
	int rows = atoi(argv[2]);
	int cols = atoi(argv[3]);
	float bscale = atof(argv[4]); 

	StereoCalibrator* sc = new StereoCalibrator(nboards, rows, cols, bscale);
	return 0;
}