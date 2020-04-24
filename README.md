This library is an implementation of the FABRIK inverse kinematics (IK) algorithm in the Python programming language, 
and is released under the MIT software license and is under developement

Usage:
	Right now you can change "Main" file for yourself, I solved for a 3 consecutive bone with Global Hinges With Reference Axis Constraints 
	It can also solve for rotor and local and global hinges. 
	the out put is the angle of each joint
	
	in the Main file there is four step for entry the data:
	Step1 : Specify the target position which this IK solve the chain for that
	
	Step2: Specify base bone of the chain

	Step3: define base bone specification:
     joint type,
     constraints and axises
     joint type may vary between
     BALL
     GLOBAL_HINGE
     LOCAL_HINGE
	 
	 Step4: adding consecutive bone to the chain 
	 Note:number of bones are excluding base bone!


OutPut:
	as an out put is a text file ("angles.txt" ) which computed joint angles are printed on there.

References:
	The FABRIK algorithm is explained in the following research paper:
	Aristidou, A., & Lasenby, J. (2011). FABRIK: a fast, iterative solver for the inverse kinematics problem. Graphical Models, 73(5), 243-260.

	The version implemented in Java can be found here:
	 Caliko: An inverse kinematics software library implementation of the FABRIK algorithm. Journal of Open Research Software, 4(1).

	You can watch a short video outlining the setup and functionality of the Caliko library here:
	[https://www.youtube.com/watch?v=wEtp4P2ucYk](https://www.youtube.com/watch?v=wEtp4P2ucYk)
