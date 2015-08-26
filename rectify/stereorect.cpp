#include "stereorect.h"

using namespace cv;
using namespace std;

StereoRectifier::StereoRectifier(Size s) {
	FileStorage fs1("stereocalib.xml", FileStorage::READ);
	fs1["M1"] >> M1;
    fs1["M2"] >> M2;
    fs1["D1"] >> D1;
    fs1["D2"] >> D2;
    fs1["R"] >> R;
    fs1["T"] >> T;
    fs1.release();
    img_sz = s;
}

void StereoRectifier::rectify() {
	stereoRectify(M1, D1, M2, D2, img_sz, R, T, R1, R2, P1, P2, Q);
    cout << "Stereorectification done..\n";
    cout << "Generating maps..\n";
    initUndistortRectifyMap(M1, D1, R1, P1, img_sz, CV_32FC1, map1x, map1y);
    initUndistortRectifyMap(M2, D2, R2, P2, img_sz, CV_32FC1, map2x, map2y);
    cout << "Maps generated..\n";
	char c;
	cout << "Want to add params to stereorect.xml ? (y/n): ";
	cin >> c;
	if (c=='y' || c=='Y' ) {
		FileStorage fs2("stereorect.xml", FileStorage::WRITE);
		fs2 << "R1" << R1;
    	fs2 << "R2" << R2;
    	fs2 << "P1" << P1;
    	fs2 << "P2" << P2;
    	fs2 << "Q" << Q;
        fs2 << "map1x" << map1x;
        fs2 << "map1y" << map1y;
        fs2 << "map2x" << map2x;
        fs2 << "map2y" << map2y;
    	fs2.release();
	}
}
