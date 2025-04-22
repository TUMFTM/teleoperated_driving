cd wsp
rm -f -r build install log # clean
colcon build # Add COLCON_IGNORE to exculde packages
