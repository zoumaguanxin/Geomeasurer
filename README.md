# DALKO
This a 2d laser feature detector which is more robust than falko algorithm. It detects the corner in laserscan. 
Compared to Flirt algorthm, it is more suitable to our expection.



Build Instructions

git clone https://github.com/zoumaguanxin/DALKO.git

cd DALKO

mkdir build

cd build

cmake ..

make



Dependencies

FLIRT

Refer to https://github.com/tipaldi/flirtlib

Note: for the reason that the "gui" lib of flirtlib might difficultly be installed, so you can edit its CMakeLists.txt to disable it.

After finish its installation, use the following command line to find FLIRTConfig.cmake

sudo updatedb

locate FLIRTConfig.cmake

sudo vim /usr/local/share/flirtlib/cmake/FLIRTConfig.cmake

delete "gui" because it has not been built


FALKO

Refer to https://github.com/dlr1516/falkolib

PCL 1.7

