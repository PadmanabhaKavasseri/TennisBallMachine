# TBM UI
## Features so far
- Arduino pings hello 1Hz.
    - ping_hello.ino
    - monitor_app.py which uses monitor.html
- Arduino echoes user message
    - echo.ino
    - app.py which uses index.html


Current
source .venv/bin/activate  
controller_v1.py
mc_v1.ino


/*
cd /Users/padmanabha/Projects/TennisBallMachineUI/ard/mc_v1/
compile:
arduino-cli compile --fqbn arduino:avr:mega mc_v1.ino

flash:
arduino-cli upload -p /dev/cu.usbmodem14401 --fqbn arduino:avr:mega mc_v1.ino

arduino-cli lib install Servo


main code now is in controller_v2
cd controlelr_v2
python controller_v2.py


RPI Setup
1. Create Venv
```
python3 -m venv .venv
source .venv/bin/activate
pip install flask flask-socketio pyserial numpy PyYaml
```


Run Standalone without ros
```
python controller_v2.py
```

Run ros2 node 
```
python ros2_controller.py
```




ROS Arduino Node Controller

colcon build --packages-select tennis_ball_machine
source install/setup.bash
ros2 run tennis_ball_machine arduino_controller_node