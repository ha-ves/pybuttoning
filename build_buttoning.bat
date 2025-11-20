colcon --log-level debug build --merge-install --symlink-install ^
  --base-paths src --install-base install_py --packages-up-to pybuttoning ^
  --cmake-args -DBUILD_TESTING=OFF -DBUILD_TESTS=OFF -DENABLE_TESTS=OFF -DWITH_TESTS=OFF ^
  -Drealsense2_DIR="C:\Users\admin\source\repos\librealsense\install_l515\lib\cmake\realsense2" ^
  -DBoost_DIR="C:\local\boost_1_74_0\lib64-msvc-14.1\cmake\Boost-1.74.0" ^
  -DEigen3_DIR="C:\Tools\Eigen3\share\eigen3\cmake" ^
  -DCMAKE_CONFIGURATION_TYPES=RelWithDebInfo -DCMAKE_BUILD_TYPE=RelWithDebInfo -DPython3_FIND_VIRTUALENV=ONLY
