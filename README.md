# GBSAIVD
Code Repo for GBS' AIVD Jetson Nano

VLDHeadless.cpp is the main and fastest line recognition code in the repo. All other versions and compiles spend
resources uploading to screen and using display. Additionally, there is some python running to get the Jetbot motors working.
These motor calls were tweaked and still have left lane bias.
This should be later replaced with code to interact with serial to the mega to control car motors based on the turning vector
that was calculated. The turning vector should always be straight, and when angled, this is a call to... turn.

VideoLineDetection.cpp is the "base" file, but doesn't have newer optimizations
VideoLineDetectionTest.cpp was our first test using headless styles and seeing max framerate outputs.
VideoLineDetectionHeadless.cpp is explained above.

VLDHeadless is the compiled script and should be used to run.
VLD3 (should) be the the most recent (but old) script that uploads to the screen to show input.

Use /misc/compile.sh to compile any cpp file, as it will include all required libraries and enable perms on the compiled file.
If you include another library in the cpp file, make sure to include it in the compile command (this should've worked automatically, but
Tegra linux is borked).

The Navigation script is a WIP but accesses both the magnetometer and the GPS to recieve data. Unfortunately, I ran into issues
syncing up the information calls, leading to inaccurate data.
Look into TinyGPS++ and the MPU9250 library for info on how to poll data from these devices.
This absolutely will require a 2nd arduino on board, as one arduino cannot handle the size of this script and the information
processing requires.

Anything in python files is OLD. The only thing worth checking is debugcv.py to get your build info- CUDA should be ON.
