git --git-dir=/root/robotics/src/aau_multi_robot/.git pull origin master
git --git-dir=/root/robotics/src/mappingturtles/.git pull origin master
source /ros_entrypoint.sh;
# ln /dev/null /dev/raw1394
catkin_make -C /root/robotics
timelimit -t 180 roslaunch mappingturtles adhocbotstest.launch visualize:=false record:=true 

