from src.ur5_controller import URControl
import numpy as np
from copy import deepcopy
robot = URControl('192.168.0.11')
current_q = np.asarray(robot.get_q())
q_waypoints = []
for i in np.linspace(0,np.pi/4, 10):
    updated_q = deepcopy(current_q)
    updated_q[3] = updated_q[3] -i
    q_waypoints.append(updated_q.tolist())
robot.moveJ_waypoints(q_waypoints, duration=10)

