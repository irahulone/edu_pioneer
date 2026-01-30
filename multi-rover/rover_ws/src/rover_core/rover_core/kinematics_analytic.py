import numpy as np

elbow_angle_offset = np.arctan2(0.128,0.024) 
a_sqrd = 0.124**2
b_sqrd = 0.13**2
ee_link = 0.1

#def max_joint_angles(joint_angles):
    

def fkne(joint_positions):
    end_effector_pose = sum(joint_positions)
    phi = end_effector_pose + np.pi/2
    lam = sum(joint_positions[0:2]) + np.pi/2
    theta = joint_positions[0] + np.pi/2
    x = ee_link*np.sin(phi)+0.124*np.sin(lam)+0.128*np.cos(theta)+0.024*np.sin(theta)
    y = -ee_link*np.cos(phi)-0.124*np.cos(lam)+0.128*np.sin(theta)-0.024*np.cos(theta)
    return [x,y,end_effector_pose]
    #print(f"x : {self.end_effector_position[0]}, y : {self.end_effector_position[1]}, theta : {self.end_effector_pose}")
    #self.ikne(self.end_effector_position, self.end_effector_pose
def fkne_3d(joint_positions):
    xl, yl, ee_pose = fkne(joint_positions[1:4])
    theta = joint_positions[0]
    x = xl*np.sin(theta)
    y = xl*np.cos(theta)
    #print(f"3d coords : x = {x}, y = {y}, z = {yl}")
    return [x,y,yl,ee_pose]

def ikne(position, orientation):
    link_3_position = position + ee_link*np.array([np.cos(orientation+np.pi), np.sin(orientation+np.pi)])
    x3, y3 = link_3_position
    #print(f"x3 : {x3}, y3 : {y3}")
    r_sqrd = np.dot(link_3_position,link_3_position)
    r = np.sqrt(r_sqrd)
    #print(f"r : {r}")
    angle = np.array([0.0,0.0,0.0])
    angle[0] = np.arccos((a_sqrd-r_sqrd-b_sqrd)/(-2*r*0.13)) + np.arctan2(y3,x3) + np.pi/2 - elbow_angle_offset - np.pi/2
    angle[1] = (np.arccos((r_sqrd-a_sqrd-b_sqrd)/(-2*0.124*0.13)) + elbow_angle_offset) - np.pi
    angle[2] = orientation - sum(angle[0:2])
    if np.any(np.isnan(angle)):
        return None
    return angle
    #self.arm_joint_positions[1:4] = angle
    #print(f"theta1 : {self.arm_joint_positions[1]}, theta2 : {self.arm_joint_positions[2]}, theta3 : {self.arm_joint_positions[3]}")
    #self.send_arm_command()

def ikne_3d(position, orientation):
    x, y, z = position
    joint_positions = [0.0,0.0,0.0,0.0]
    joint_positions[0] = np.arctan2(x,y)
    xl = np.sqrt(x**2 + y**2)
    angles = ikne([xl,z], orientation)
    if angles is None:
        return None
    joint_positions[1:4] = angles
    return joint_positions

def joint_limits(joint_angles):
    #caps joint 0 to +- 175 degrees
    valid = True
    if np.degrees(joint_angles[0]) > 175 or np.degrees(joint_angles[0]) < -175:
        joint_angles[0] = max(np.radians(-175), min(np.radians(175), joint_angles[0]))
        valid = False
    if np.degrees(joint_angles[1]) > 110 or np.degrees(joint_angles[1]) < -100:
        joint_angles[1] = max(np.radians(-100), min(np.radians(110), joint_angles[1]))
        valid = False
    if np.degrees(joint_angles[2]) > 85 or np.degrees(joint_angles[2]) < -85:
        joint_angles[2] = max(np.radians(-85), min(np.radians(85), joint_angles[2]))
        valid = False
    if np.degrees(joint_angles[3]) > 100 or np.degrees(joint_angles[3]) < -120:
        joint_angles[3] = max(np.radians(-120), min(np.radians(100), joint_angles[3]))
        valid = False
    return joint_angles, valid

