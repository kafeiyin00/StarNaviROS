# StarNaviROS
To complie the project, you should do:
## 1.Setup Serial
You can execute the following command if the ros version is earlier than noetic(eg. kinetic,meldic):
  sudo apt-get install ros-<your ros version>-serial
If the ros version is noetic, then do:
  git clone https://github.com/wjwwood/serial.git
  cd serial
  cmake -DCMAKE_INSTALL_PREFIX=/opt/ros/nortic .. 
  sudo make install
##2.Compile
  cd catkin_ws/src
  catkin_make
