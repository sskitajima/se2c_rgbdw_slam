- install packages
```sh
# basic dependencies
sudo apt install -y build-essential pkg-config cmake git wget curl unzip

# g2o dependencies
sudo apt install -y libatlas-base-dev libsuitesparse-dev

# OpenCV dependencies
sudo apt install -y libgtk-3-dev
sudo apt install -y ffmpeg
sudo apt install -y libavcodec-dev libavformat-dev libavutil-dev libswscale-dev libavresample-dev

# eigen dependencies
sudo apt install -y gfortran
sudo apt install libeigen3-dev

# ROS dependencies
sudo apt install -y ros-melodic-octomap-server
sudo apt install -y ros-melodic-octomap-rviz-plugins

# other dependencies
sudo apt install -y libyaml-cpp-dev libgoogle-glog-dev libgflags-dev
sudo apt install -y libglew-dev
```


- install opencv from source 
  - In the following script, source code is downloaded in `~/LIB` directory.
```sh
cd ~/LIB
wget -q https://github.com/opencv/opencv/archive/3.4.0.zip
unzip -q 3.4.0.zip
rm -rf 3.4.0.zip
cd opencv-3.4.0
mkdir -p build && cd build
cmake \
    -DCMAKE_BUILD_TYPE=Release \
    -DCMAKE_INSTALL_PREFIX=/usr/local \
    -DENABLE_CXX11=ON \
    -DBUILD_DOCS=OFF \
    -DBUILD_EXAMPLES=OFF \
    -DBUILD_JASPER=OFF \
    -DBUILD_OPENEXR=OFF \
    -DBUILD_PERF_TESTS=OFF \
    -DBUILD_TESTS=OFF \
    -DWITH_EIGEN=ON \
    -DWITH_FFMPEG=ON \
    -DWITH_OPENMP=ON \
    -DBUILD_opencv_cudacodec=OFF \
    ..
make -j8
sudo make install
```

- install DBoW2
  - In the following script, source code is downloaded in `~/LIB` directory.
```sh
cd ~/LIB
git clone https://github.com/shinsumicco/DBoW2.git
cd DBoW2
mkdir build && cd build
cmake \
    -DCMAKE_BUILD_TYPE=Release \
    -DCMAKE_INSTALL_PREFIX=/usr/local \
    ..
make -j4
sudo make install
```
- install g2o
  - In the following script, source code is downloaded in `~/LIB` directory.
```sh
cd ~/LIB
git clone https://github.com/RainerKuemmerle/g2o.git
cd g2o
git checkout 9b41a4ea5ade8e1250b9c1b279f3a9c098811b5a
mkdir build && cd build
cmake \
    -DCMAKE_BUILD_TYPE=Release \
    -DCMAKE_INSTALL_PREFIX=/usr/local \
    -DCMAKE_CXX_FLAGS=-std=c++11 \
    -DBUILD_SHARED_LIBS=ON \
    -DBUILD_UNITTESTS=OFF \
    -DG2O_USE_CHOLMOD=OFF \
    -DG2O_USE_CSPARSE=ON \
    -DG2O_USE_OPENGL=OFF \
    -DG2O_USE_OPENMP=ON \
    ..
make -j4
sudo make install
```

- build
```sh
cd ~/catkin_ws
catkin_make
```
