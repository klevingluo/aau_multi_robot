git --git-dir=/root/robotics/aau_multi_robot/.git pull origin master
git --git-dir=/root/robotics/mappingturtles/.git pull origin master
source /ros_entrypoint.sh
catkin_make -C /root/robotics
timelimit -t 180 roslaunch mappingturtles adhocbotstest.launch visualize:=false record:=true 

