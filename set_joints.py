#!/usr/bin/env python

import rospy
from std_msgs.msg import String
from std_msgs.msg import Float64MultiArray

from std_msgs.msg import Bool
import baxter_interface
from baxter_core_msgs.msg import DigitalIOState
from baxter_core_msgs.msg import EndpointState
from baxter_core_msgs.srv import (
    SolvePositionIK,
    SolvePositionIKRequest,
)
from geometry_msgs.msg import (
    PoseStamped,
    Pose,
    Point,
    Quaternion,
)




# Joints angles for each of the 2x9 positions. m = row number, n = column number, ux = pen up, dx = pen down
dx11 = {'right_s0': 0.1438106987, 'right_s1': 0.973310808801, 'right_e0': 1.48719437216, 'right_e1': 0.974461294391, 'right_w0': -2.88234989714, 'right_w1': -0.473616567719, 'right_w2': -2.35082555475}
ux11 = {'right_s0': 0.145344679486, 'right_s1': 1.00207294854, 'right_e0': 1.57386428657, 'right_e1': 0.873602057703, 'right_w0': -3.04418487008, 'right_w1': -0.75548553717, 'right_w2': -2.03674298879}
dx12 = {'right_s0': 0.0360485484741, 'right_s1': 0.995170035004, 'right_e0': 1.4818254394, 'right_e1': 0.971393332819, 'right_w0': -3.04418487008, 'right_w1': -0.52653890484, 'right_w2': -2.16214591805}
ux12 = {'right_s0': 0.0498543755493, 'right_s1': 1.00168945334, 'right_e0': 1.51825748307, 'right_e1': 0.924223423645, 'right_w0': -3.04456836528, 'right_w1': -0.706398152014, 'right_w2': -2.05438376783}
dx13 = {'right_s0': 0.0360485484741, 'right_s1': 0.95912148653, 'right_e0': 1.47568951626, 'right_e1': 0.892393322333, 'right_w0': -3.04456836528, 'right_w1': -0.629315617511, 'right_w2': -2.09273328748}
ux13 = {'right_s0': -0.0429514620117, 'right_s1': 1.00168945334, 'right_e0': 1.4818254394, 'right_e1': 0.939563231506, 'right_w0': -3.04418487008, 'right_w1': -0.738228253326, 'right_w2': -2.05016532067}
dx21 = {'right_s0': -0.0184077694336, 'right_s1': 0.999388482166, 'right_e0': 1.33609726472, 'right_e1': 1.12900985859, 'right_w0': -3.03689846135, 'right_w1': -0.428747629724, 'right_w2': -2.26185466915}
ux21 = {'right_s0': -0.0494708803528, 'right_s1': 0.993636054218, 'right_e0': 1.35373804376, 'right_e1': 1.06113120881, 'right_w0': -3.04418487008, 'right_w1': -0.692208829742, 'right_w2': -2.08467988835}
dx22 = {'right_s0': -0.141893222717, 'right_s1': 1.00168945334, 'right_e0': 1.28240793721, 'right_e1': 1.16045646471, 'right_w0': -3.03574797576, 'right_w1': -0.459427245447, 'right_w2': -2.27144204907}
ux22 = {'right_s0': -0.134223318787, 'right_s1':0.990184597449, 'right_e0': 1.31500502891, 'right_e1': 1.07647101667, 'right_w0': -3.04380137488, 'right_w1': -0.724805921448, 'right_w2': -2.08621386914}
dx23 = {'right_s0': -0.208621386914, 'right_s1': 1.00130595815, 'right_e0': 1.26745162454, 'right_e1': 1.07685451187, 'right_w0': -3.04456836528, 'right_w1': -0.645805910962, 'right_w2': -2.23692748138}
ux23 = {'right_s0': -0.245053430585, 'right_s1': 0.992869063824, 'right_e0': 1.2613157014, 'right_e1': 1.09717975728, 'right_w0': -3.04456836528, 'right_w1': -0.770441849835, 'right_w2': -2.08851484032}
dx31 = {'right_s0': -0.172189343243, 'right_s1': 0.993252559021, 'right_e0': 1.17617976777, 'right_e1': 1.21759724899, 'right_w0': -3.04418487008, 'right_w1': -0.490490356366, 'right_w2': -2.22082068312}
ux31 = {'right_s0': -0.212456338879, 'right_s1': 1.00207294854, 'right_e0': 1.16314093109, 'right_e1': 1.24444191275, 'right_w0': -3.04380137488, 'right_w1': -0.559135996545, 'right_w2': -2.15984494688}
dx32 = {'right_s0': -0.266145666394, 'right_s1': 0.99670401579, 'right_e0': 1.13783024811, 'right_e1': 1.16505840707, 'right_w0': -3.04418487008, 'right_w1': -0.633917559869, 'right_w2': -2.22273815911}
ux32 = {'right_s0': -0.322902955481, 'right_s1': 0.997854501379, 'right_e0': 1.11251956514, 'right_e1': 1.22296618174, 'right_w0': -3.04456836528, 'right_w1': -0.672650574719, 'right_w2': -2.21430126478}
dx33 = {'right_s0': -0.410723355487, 'right_s1': 0.985966150287, 'right_e0': 1.04962635291, 'right_e1': 1.2386894848, 'right_w0': -3.04495186047, 'right_w1': -0.608606876898, 'right_w2': -2.31746147265}
ux33 = {'right_s0': -0.445237923175, 'right_s1': 0.995170035004, 'right_e0': 1.06611664636, 'right_e1': 1.25134482629, 'right_w0': -3.04418487008, 'right_w1': -0.704480676031, 'right_w2': -2.20049543771}



a = [ux31, dx31, dx11, dx13, dx33, ux33, ux23, dx23, dx21, ux21]
b = [ux11, dx11, dx13, dx33, dx31, ux31, ux12, dx12, dx32, ux32, ux23, dx23, dx22, ux22]
c = [ux13, dx13, dx11, dx31, dx33, ux33]
d = [ux11, dx11, dx13, dx33, dx31, ux31, ux12, dx12, dx32, ux32]
e = [ux13, dx13, dx11, dx31, dx33, ux33, ux22, dx22, dx21, ux21]
f = [ux13, dx13, dx11, dx31, ux31, ux22, dx22, dx21, ux21]
g = [ux13, dx13, dx11, dx31, dx33, dx23, dx22, ux22]
h = [ux13, dx13, dx33, ux33, ux11, dx11, dx31, ux31, ux23, dx23, dx21, ux21]
i = [ux13, dx13, dx11, ux11, ux12, dx12, dx32, ux32, ux33, dx33, dx31, ux31]
j = [ux13, dx13, dx33, dx31, dx21, ux21]
k = [ux13, dx13, dx22, ux22, ux33, dx33, dx22, dx21, ux21, ux11, dx11, dx31, ux31]
l = [ux11, dx11, dx31, dx33, ux33]
m = [ux31, dx31, dx11, dx22, dx13, dx33, ux33]
n = [ux31, dx31, dx11, dx33, dx13, ux13]
o = [ux31, dx31, dx11, dx13, dx33, dx31, ux31]
p = [ux31, dx31, dx11, dx13, dx23, dx21, ux21]
q = [ux33, dx33, dx31, dx11, dx13, dx33, dx22, ux22]
r = [ux31, dx31, dx11, dx13, dx23, dx21, ux21, ux22, dx22, dx33, ux33]
s = [ux13, dx13, dx11, dx21, dx23, dx33, dx31, ux31]
t = [ux11, dx11, dx13, ux13, ux12, dx12, dx32, ux32]
u = [ux11, dx11, dx31, dx33, dx13, ux13]
v = [ux11, dx11, dx31, dx13, ux13]
w = [ux11, dx11, dx31, dx22, dx33, dx13, ux13]
x = [ux11, dx11, dx33, ux33, ux13, dx13, dx31, ux31]
y = [ux11, dx11, dx22, ux22, ux13, dx13, dx22, dx32, ux32]
z = [ux11, dx11, dx13, dx31, dx33, ux33]



def set_joints(data):

	alphabet = {'a': a , 'b': b, 'c': c, 'd': d, 'e': e, 'f':f, 'g': g, 'h': h, 'i': i, 'j': j, 'k': k, 'l': l, 'm': m,'n': 			n,'o': o,'p': p, 'q': q, 'r': r, 's': s, 't': t, 'u': u,'v': v, 'w': w, 'x': x, 'y': y, 'z': z}

	arm = baxter_interface.Limb('right')
	angles = arm.joint_angles()

#Joint 		(Degrees) Min limit 		Max limit 		Range 		(Radians) Min limit 		Max limit 		Range

#S0 			-97.494 		+97.494 		194.998 		-1.7016 		+1.7016 		3.4033 
#E1 			-2.864 			+150 			153 			-0.05 			+2.618 			2.67 
#W1 			-90 			+120 			210 			-1.5707 		+2.094 			3.6647 	

	#[-0.06,-0.025,-0.007,-0.125,0.155,-0.250,0.290]
	#[-0.108,-0.0315,-0.0115,-0.125,0.155,-0.250,0.195]
	delta_right = [-0.066,-0.04,-0.0,-0.0,0.0,-0.01,0.0]
	


	#11 'right_s0': 0.1438106987, 'right_s1': 0.973310808801, 'right_e0': 1.48719437216, 'right_e1': 0.974461294391, 'right_w0': -2.88234989714, 'right_w1': -0.473616567719, 'right_w2': -2.35082555475
	#14 -0.0820679720581,0.998237996576,1.47990796342,0.848674869928,-3.04456836528,-0.724422426251,-2.05975270058

	#33 right_s0': -0.410723355487, 'right_s1': 0.985966150287, 'right_e0': 1.04962635291, 'right_e1': 1.2386894848, 'right_w0': -3.04495186047, 'right_w1': -0.608606876898, 'right_w2': -2.31746147265
	#34 -0.518869000909,1.0174127564,1.06113120881,1.08490791099,-3.04456836528,-0.827199138922,-2.12264591281

		     #  s0 ,   s1 ,   w0,     w1,    w2 ,   e0 ,   e1
	#delta_right = [-0.053,  -0.0,  -0.0,  -0.05,   0.0,  -0.02,  0.0]
	


	word = data.data
	letter_count = 1
	# go through each letter in the word
	for letter in word:
		# pass the list of positions according to the letter passed
		list_pos =  alphabet[letter]
		# move the arm to each position on the board to write the letter
		for position in list_pos:
			arm.move_to_joint_positions(position)
			arm.move_to_joint_positions(position)
			arm.move_to_joint_positions(position)
			arm.move_to_joint_positions(position)
			arm.move_to_joint_positions(position)
			arm.move_to_joint_positions(position)
			arm.move_to_joint_positions(position)
			arm.move_to_joint_positions(position)
			arm.move_to_joint_positions(position)
			arm.move_to_joint_positions(position)
			# each joint position is modified to change translate the joint by delta*letter_count
			key_count = 0
			for key in position.keys():
            			position[key] = position[key] + ((delta_right[key_count])*2)*letter_count
				key_count += 1
		letter_count += 1


input_sub = rospy.Subscriber('/face_rec/user_input',String,set_joints)



if __name__ == '__main__':
    	rospy.init_node('joint', anonymous=True)
	rospy.spin()



