import sys
import math
import os
import time

t0=time.time()

dt=0
while ((time.time()-t0)<10):
  os.system('ros2 topic pub --once /configParams std_msgs/msg/Float64MultiArray \"{data: ['+str(math.sin(time.time()-t0))+', 5.0, 0.5]}\"')
  time.sleep(0.1)
