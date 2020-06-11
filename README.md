# Description:

This library is an implementation of the FABRIK method(Forward And Backward Reaching Inverse kinematic) 
for a 3D manipulator in the Python programming language. and is released under the MIT software license 
and is under developement. It can simulate the Robot Manipulator behavior in reaching a target with consideration of 
the joints constraints.
NOTE: This library is under developement so there is some update in future.


# Usage:
	You can change "Main" file for your usage,
	First: Determine target position and orientation(in quaternion)
	Second: Add specification for Base bone : All specification description is in the code
		NOTE: it's joint type can be "GLOBAL_HINGE" or "GLOBAL ROTOR" 
			or in case of FRANKA robot it is "twist_only" the last means it only rotate around itself.
	Third: Add consecutive bones to the chain with specification:
		 bone-length: length of the bone
		 bone-orientation: the orientation of the bone in quaternion
		 joint type: can be BALL(rotor-constraints) or local-hinge or twist-only
		 constraints and axises of rotation
	 
	Then Solve the chain
	
	
# OutPut:
	Out-put is a diagram showing the final state of the manipulator( it will show after closing the initial diagram)
	For FRANKA(with 7 joints) there is an array of joints degree can be seen as an output.
# References:
	The FABRIK algorithm is explained in the following research paper:
	Aristidou, A., & Lasenby, J. (2011). FABRIK: a fast, iterative solver for the inverse kinematics problem. Graphical Models, 		73(5), 243-260.

	The version implemented in Java can be found here:
	 Caliko: An inverse kinematics software library implementation of the FABRIK algorithm. Journal of Open Research Software, 4(1).

	You can watch a short video outlining the setup and functionality of the Caliko library here:
	[https://www.youtube.com/watch?v=wEtp4P2ucYk](https://www.youtube.com/watch?v=wEtp4P2ucYk)
