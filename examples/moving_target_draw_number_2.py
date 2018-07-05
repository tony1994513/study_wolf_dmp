'''
Copyright (C) 2016 Travis DeWolf

This program is free software: you can redistribute it and/or modify
it under the terms of the GNU General Public License as published by
the Free Software Foundation, either version 3 of the License, or
(at your option) any later version.

This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.

You should have received a copy of the GNU General Public License
along with this program.  If not, see <http://www.gnu.org/licenses/>.
'''

import numpy as np
import matplotlib.pyplot as plt
import seaborn

import pydmps
import pydmps.dmp_discrete
import baxter_interface
import pickle
import rospy
from std_srvs.srv import Trigger,TriggerResponse
import pickle

joint_cmd_names = [
    'right_s0',
    'right_s1',
    'right_e0',
    'right_e1',
    'right_w0',
    'right_w1',
    'right_w2',
]
ending_angles = None
def callback(req):
    global ending_angles
    resp = TriggerResponse()
    resp.success = True
    ending_angles = [ 1.3138545448238568, -0.6273981422451342,-0.0279951493789088, 0.1507136124097419,  -0.024160197409195266, 2.0022284233874363, 0.4924078329112178]
    return resp

def main():
    rospy.init_node("Test_online_dmp")
    trigger = rospy.Service('/task_change_flag', Trigger, callback)
    limb_interface = baxter_interface.limb.Limb('right')
    y_des = np.loadtxt("test_1.txt")
    y_des = y_des[:,9:16]
    move_to_start_position = y_des[0]
    limb_interface.move_to_joint_positions(dict(zip(joint_cmd_names, move_to_start_position)))
    starting_angles = [limb_interface.joint_angle(joint) for joint in limb_interface.joint_names()]

    dmp = pydmps.dmp_discrete.DMPs_discrete(n_dmps=7, n_bfs=100, ay=np.ones(7)*10.0)

    dmp.imitate_path(y_des=y_des.T)
    dmp.y0 = np.array(starting_angles)
    # y_track, dy_track, ddy_track = dmp.rollout()
    dmp.reset_state()   
    global ending_angles
    ending_angles = y_des[-1]
    y_track = []
    # for t in range(dmp.timesteps):
    while True:
        y, _, _ = dmp.step()
        y_track.append(np.copy(y))
        print y
        limb_interface.set_joint_positions(dict(zip(joint_cmd_names,y)))
        rospy.sleep(0.05)
        
        dmp.goal = ending_angles
        
    print "Done"
    np.save("y_track",y_track)

if __name__ == '__main__':
    main()
