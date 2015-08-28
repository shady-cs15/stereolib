#include "stereorect.h"

StereoRectifier::StereoRectifier(Size s) {
	FileStorage fs1("stereocalib.xml", FileStorage::READ);
	fs1 >> "M1" >> M1;
    fs1 >> "M2" >> M2;
    fs1 >> "D1" >> D1;
    fs1 >> "D2" >> D2;
    fs1 >> "R" >> R;
    fs1 >> "T" >> T;
    fs1.release();
    img_sz = s;
}

void StereoRectifier::rectify() {
	stereoRectify(M1, D1, M2, D2, img_sz, R, T, R1, R2, P1, P2, Q);
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
    	fs2.release();
	}
}