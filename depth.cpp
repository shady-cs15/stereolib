#include "opencv2/core/core.hpp"
#include "opencv2/calib3d/calib3d.hpp"
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include "opencv2/contrib/contrib.hpp"
#include <stdio.h>
#include <iostream>

using namespace cv;
using namespace std;

int main(int argc, char* argv[])
{
    if (argc<4) {
        cout<<"Usage: ./calrect n w h\n";
        return 0;
    }

    int numBoards = atoi(argv[1]);
    int board_w = atoi(argv[2]);
    int board_h = atoi(argv[3]);

    Size board_sz = Size(board_w, board_h);
    int board_n = board_w*board_h;

    vector<vector<Point3f> > object_points;
    vector<vector<Point2f> > imagePoints1, imagePoints2;
    vector<Point2f> corners1, corners2;
    vector<Point3f> obj;
    
    for (int j=0; j<board_n; j++)
       obj.push_back(Point3f(j/board_w, j%board_w, 0.0f));
 
    Mat img1, img2, gray1, gray2;
    VideoCapture cap1(1);
    VideoCapture cap2(2); 

    if (!cap1.isOpened() || !cap2.isOpened()) {
        cout<<"Cameras are not connected properly..\n";
        return -1;
    } 

    cap1.set(CV_CAP_PROP_FRAME_WIDTH,640);
    cap1.set(CV_CAP_PROP_FRAME_HEIGHT,480);
    cap2.set(CV_CAP_PROP_FRAME_WIDTH,640);
    cap2.set(CV_CAP_PROP_FRAME_HEIGHT,480);

    int success = 0, k = 0;
    bool found1 = false, found2 = false;

    while (success < numBoards) {
        cap1 >> img1;
        cap2 >> img2;
        cvtColor(img1, gray1, CV_BGR2GRAY);
        cvtColor(img2, gray2, CV_BGR2GRAY);

        found1 = findChessboardCorners(img1, board_sz, corners1, CV_CALIB_CB_ADAPTIVE_THRESH | CV_CALIB_CB_NORMALIZE_IMAGE | CALIB_CB_FAST_CHECK | CV_CALIB_CB_FILTER_QUADS);
        found2 = findChessboardCorners(img2, board_sz, corners2, CV_CALIB_CB_ADAPTIVE_THRESH | CV_CALIB_CB_NORMALIZE_IMAGE | CALIB_CB_FAST_CHECK | CV_CALIB_CB_FILTER_QUADS);

        if (found1) {
            cornerSubPix(gray1, corners1, Size(11, 11), Size(-1, -1), TermCriteria(CV_TERMCRIT_EPS | CV_TERMCRIT_ITER, 30, 0.1));
            drawChessboardCorners(gray1, board_sz, corners1, found1);
        }

        if (found2) {
            cornerSubPix(gray2, corners2, Size(11, 11), Size(-1, -1), TermCriteria(CV_TERMCRIT_EPS | CV_TERMCRIT_ITER, 30, 0.1));
            drawChessboardCorners(gray2, board_sz, corners2, found2);
        }
        
        imshow("image1", gray1);
        imshow("image2", gray2);

        k = waitKey(10);
        if (found1 && found2) 
            k = waitKey(0);
        
        if (k == 27)
            break;
    
        if (k == ' ' && found1 !=0 && found2 != 0) {        
            imagePoints1.push_back(corners1);
            imagePoints2.push_back(corners2);
            object_points.push_back(obj);
            printf ("Corners stored\n");
            success++;

            if (success >= numBoards)
                break;
        }
    }

    destroyAllWindows();

    //End of Chessboard pattern recognition
    //Starting stereo calibration

    printf("Starting Calibration\n");
    Mat CM1 = Mat(3, 3, CV_64FC1);
    Mat CM2 = Mat(3, 3, CV_64FC1);
    Mat D1, D2;
    Mat R, T, E, F;

    stereoCalibrate(object_points, imagePoints1, imagePoints2, 
                    CM1, D1, CM2, D2, img1.size(), R, T, E, F,
                    cvTermCriteria(CV_TERMCRIT_ITER+CV_TERMCRIT_EPS, 100, 1e-5), 
                    CV_CALIB_SAME_FOCAL_LENGTH | CV_CALIB_ZERO_TANGENT_DIST);

    FileStorage fs1("stereocalib.xml", FileStorage::WRITE);
    fs1 << "CM1" << CM1;
    fs1 << "CM2" << CM2;
    fs1 << "D1" << D1;
    fs1 << "D2" << D2;
    fs1 << "R" << R;
    fs1 << "T" << T;
    fs1 << "E" << E;
    fs1 << "F" << F;

    printf("Done Calibration\n");

    //End of Calibration
    //starting Stereo rectification

    printf("Starting Rectification\n");

    Mat R1, R2, P1, P2, Q;
    stereoRectify(CM1, D1, CM2, D2, img1.size(), R, T, R1, R2, P1, P2, Q);
    fs1 << "R1" << R1;
    fs1 << "R2" << R2;
    fs1 << "P1" << P1;
    fs1 << "P2" << P2;
    fs1 << "Q" << Q;

    printf("Done Rectification\n");

    //End of rectification
    //Starting Undistortion 

    printf("Applying Undistort\n");

    Mat map1x, map1y, map2x, map2y;
    Mat imgU1, imgU2, disp, disp8;

    initUndistortRectifyMap(CM1, D1, R1, P1, img1.size(), CV_32FC1, map1x, map1y);
    initUndistortRectifyMap(CM2, D2, R2, P2, img2.size(), CV_32FC1, map2x, map2y);

    printf("Undistort complete\n");

    //End of Undistortion
    //Starting stereo correspondance search

    namedWindow("d-map",CV_WINDOW_AUTOSIZE);

    StereoBM BMState; 
    int sadwintracker = 26;
    int nod = 4;
    int prefilcaptracker = 50;
    int prefilsztracker = 5;
    int mindisptracker = 0;
    int texthtracker = 20;
    int uRatio = 0;
    int spWinSz = 0;
    int spRange = 0;

    BMState.state->preFilterSize=85;
    BMState.state->preFilterCap=47;
    BMState.state->SADWindowSize=30; 
    BMState.state->minDisparity=3; 
    BMState.state->numberOfDisparities=64; 
    BMState.state->textureThreshold=18; 
    BMState.state->uniquenessRatio=0;
    BMState.state->speckleWindowSize = 0;
    BMState.state->speckleRange = 0;

    StereoSGBM sgbm;
    sgbm.SADWindowSize = 41; 
    sgbm.numberOfDisparities = 128;
    sgbm.preFilterCap = 31; 
    sgbm.minDisparity = -64; 
    sgbm.uniquenessRatio = 15;
    sgbm.speckleWindowSize = 150;
    sgbm.speckleRange = 2;
    sgbm.disp12MaxDiff = 10;
    sgbm.fullDP = false;    
    sgbm.P2 = 2400;

    createTrackbar("SADWindowSize","d-map",&sadwintracker,100);
    createTrackbar("numberOfDisparities","d-map",&nod,20);
    createTrackbar("preFilterCap","d-map",&prefilcaptracker,63);
    createTrackbar("preFilterSize","d-map",&prefilsztracker,100);
    createTrackbar("minDisparity","d-map",&mindisptracker,63);
    createTrackbar("textureThreshold","d-map",&texthtracker,400);
    createTrackbar("uniquenessRatio","d-map",&uRatio,10);
    createTrackbar("speckleWindowSize","d-map",&spWinSz,10);
    createTrackbar("speckleRange","d-map",&spRange,10);

    while(1)
    {    
        cap1 >> img1;
        cap2 >> img2;

        remap(img1, imgU1, map1x, map1y, INTER_LINEAR, BORDER_CONSTANT, Scalar());
        remap(img2, imgU2, map2x, map2y, INTER_LINEAR, BORDER_CONSTANT, Scalar());

        cvtColor(imgU1,img1,CV_BGR2GRAY);
        cvtColor(imgU2,img2,CV_BGR2GRAY);

        if (prefilsztracker<5) prefilsztracker=5;
        BMState.state->preFilterSize=(prefilsztracker%2)?prefilsztracker:prefilsztracker-1;
        if (prefilcaptracker==0) prefilcaptracker=1;
        BMState.state->preFilterCap=prefilcaptracker;
        if (sadwintracker<5) sadwintracker = 5;
        BMState.state->SADWindowSize=(sadwintracker%2)?sadwintracker:sadwintracker-1; 
        BMState.state->minDisparity=-mindisptracker;
        if (nod ==0) nod = 1; 
        BMState.state->numberOfDisparities=16*nod; 
        BMState.state->textureThreshold=texthtracker;
        BMState.state->uniquenessRatio=uRatio;
        BMState.state->speckleWindowSize=spWinSz;
        BMState.state->speckleRange=spRange;

        /*cout<<"prefilsztracker: "<<prefilsztracker<<endl;
        cout<<"prefilcaptracker: "<<prefilcaptracker<<endl;
        cout<<"sadwintracker: "<<sadwintracker<<endl;
        cout<<"minDisparity: "<<mindisptracker<<endl;
        cout<<"nod: "<<nod<<endl;
        cout<<"texture threshold: "<<texthtracker<<endl;
        cout<<"unique ratio: "<<uRatio<<endl;
        cout<<"speckleWindowSize: "<<spWinSz<<endl;
        cout<<"speckleRange: "<<spRange<<endl;
        cout<<"\n";
        */
        
        BMState(img1,img2,disp);
        normalize(disp, disp8, 0, 255, CV_MINMAX, CV_8U);
        imshow("Frame 1",img1);
        imshow("Frame 2",img2);
        imshow("d-map",disp8);
        if (waitKey(33)==27) break;
}
    destroyAllWindows();

    //debugging
    //for displaying disparity map

    unsigned char *disparity = (unsigned char*)(disp8.data);

    int i,j,r,g,b;
    for(int i = 0;i < disp8.cols;i++){
        for(int j = 0;j < disp8.rows;j++){
            b = disparity[disp8.cols * j + i ] ;
          //  cout<<b<<" ";
        }
        //cout<<"\n";
    }
    //debugging

    return 0;
}