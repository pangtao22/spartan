#! /bin/bash
. ~/spartan/setup/docker/entrypoint.sh
use_ros

cd ~/spartan
rm -rf build
mkdir build
cd build

cmake -DWITH_PERCEPTION:BOOL=ON -DWITH_BULLET3:BOOL=ON -DWITH_TRIMESH:BOOL=OFF -DWITH_SCHUNK_DRIVER:BOOL=ON -DWITH_ROS:BOOL=ON -DWITH_REACHABILITY_ANALYZER:BOOL=OFF ..
exit_status=$?
if [ ! $exit_status -eq 0 ]; then
  echo "Error code in CMake: " $exit_status
  exit $exit_status
fi

# That CMake build generates this environment configuration
# script, which includes some definitions the compilation requires
# to work.
# . setup_environment.sh
use_spartan

make -j8 --output-sync=target
exit_status=$?
if [ ! $exit_status -eq 0 ]; then
  echo "Error code in make: " $exit_status
  exit $exit_status
fi

# Try building *again* to ensure that re-installing various pieces doesn't
# break (see e.g. issue #159)
make -j8 -
exit_status=$?
if [ ! $exit_status -eq 0 ]; then
  echo "Error code in make: " $exit_status
  exit $exit_status
fi

# See if we can source everything OK.
# . setup_environment.sh
use_spartan

# Launch a fake X-server in the background
Xvfb :100 -ac &

# Run Spartan modules test.
DISPLAY=:100 py.test --junitxml results.xml /home/jenkins/spartan/modules/spartan/test/
exit_status=$?
if [ ! $exit_status -eq 0 ]; then
  echo "Error code when running tests: " $exit_status
  exit $exit_status
fi