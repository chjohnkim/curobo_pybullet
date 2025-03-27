import rtde_receive
import rtde_control

class URControl:
    def __init__(self, hostname):
        self.hostname = hostname
        self.rtde_c = rtde_control.RTDEControlInterface(self.hostname)
        self.rtde_r = rtde_receive.RTDEReceiveInterface(self.hostname)

    # Getters
    def get_q(self):
        return self.rtde_r.getActualQ() 

    def get_tcp_pose(self):
        """
        Returns actual Cartesian coordinates of the tool: 
        (x,y,z,rx,ry,rz) 
        where rx, ry and rz is a rotation vector representation of the tool orientation
        """
        return self.rtde_r.getActualTCPPose()

    def get_tcp_offset(self):
        return self.rtde_c.getTCPOffset()

    # Setters
    def set_tcp_offset(self, tcp_offset):
        """
        tcp_offset [List]: A pose describing the transformation of the tcp offset.
        """
        self.rtde_c.setTcp(tcp_offset)

    def activate_teach_mode(self):
        self.rtde_c.teachMode()

    def deactivate_teach_mode(self):
        self.rtde_c.endTeachMode()
        
    def moveL(self, pose, speed, acceleration, asynchronous=False):
        """
        pose [List]: target pose
        speed [float]: tool speed [m/s]
        acceleration [float]: tool acceleration [m/s^2]
        asynchronous [bool]: a bool specifying if the move command should be asynchronous. 
                    If asynchronous is true it is possible to stop a move command using either the stopJ or stopL function. 
                    Default is false, this means the function will block until the movement has completed.
        """
        self.rtde_c.moveL(pose, speed, acceleration, asynchronous)

    def __enter__(self):
        return self
    
    def __exit__(self, exc_type, exc_value, traceback):
        if exc_type:
            print(f"An exception occurred: {exc_value}")


# First argument is the pose 6d vector followed by speed and acceleration
#rtde_c.moveL([-0.143, -0.435, 0.20, -0.001, 3.12, 0.04], 0.5, 0.3)
#rtde_c.teachMode()
#rtde_c.endTeachMode()

#rtde_c.setTcp(const std::vector<double> &tcp_offset)
#tcp_offset = rtde_c.getTCPOffset()

#rtde_r = rtde_receive.RTDEReceiveInterface("192.168.1.10")
#actual_q = rtde_r.getActualQ()
#actual_pose = rtde_r.getActualTCPPose()
#print(actual_q)