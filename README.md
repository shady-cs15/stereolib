# StereoVisionLibrary
Depth reconstruction using stereo vision


# Requirements
2 <b>USB Cameras</b> connected via USB.
<br>
<b>OpenCV 3.0.0</b> 
<br>
<b>PCL 1.2</b> (Optional: for stereomatch)
<br>
<b>CMake</b>

# Building
go to root directory of StereoVisionLibrary
<br>
$ cmake .
<br>
$ make -j2

# Running
<b>Calibration and Rectification: </b>
./stereocalib -n_boards -b_width -b_height -c_size
<br>
<b>-n_boards: </b>number of board positions to be taken into account eg. 20
<br>
<b>-b_width: </b>no. of chessboard corners along the width - 2
<br>
<b>-b_height: </b>no. of chessboard corners along the height -2 
<br>
<b>-c_size: </b>size of the squares in cm 
<br>
<br>
<b>Testing Epipolar Rectification: </b>
./stereotest
<br>
<br>
<b>Running Block Matcher for disparity maps: </b>
./stereomatch
<br>

#Examples 
Following are examples of disparity maps generated from stereo views
<br>
<img src="http://jderobot.org/store/chakraborty/uploads/images/disp.png"/>
<br>
<img src="http://jderobot.org/store/chakraborty/uploads/images/disp2.png" />
<br>
Demo video showing Depth reconstruction on the Karlsruhe stereo vision dataset can be found <a href="https://www.youtube.com/watch?v=dCIJjDzQBHE"> here</a>.
<img src="https://j.gifs.com/vMzxDb.gif" />
<img src="https://j.gifs.com/vJnV88.gif" />
