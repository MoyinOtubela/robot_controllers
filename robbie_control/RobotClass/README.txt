README for v2_robot class

%%%%%%%%%% CONTAINS THE FOLLOWING FILES
%% CLASS FILES
v2_robot.m		:	file containing the v2_robot class

%% CONFIG FILES
robot_config2.m		:	definition of physical robot params
initial_config.m	:	contains def of initial DH params
animate_config.m	: 	contains defn of physocal robot params (used for animate.m)

%% MATLAB FUNCTIONS
DH.m			: 	function for calculating DH transformation
plotCircle3D.m		: 	function for plotting circles in 3D

% DATA LOGGING
IK_xls.xls		: 	contains angle data for IK algorithm. 

%% DEBUGGING/VISUALISATION FILES
animate.m		: 	shows simplified IK transition using angles in IK_xls.xls
fk_test.m		: 	simple code file for visualising FK - used to evaluate IK results


%% FUNCTIONALITY DEMO FILES
class_test.m		:	sample demo code

%%% BASIC DESCRIPTION OF v2_robot class

1) The following are defined as members of the class:
	* all limb masses
	* DH parameters (a, alpha, d, theta) for each joint coordinate frame - see working notes for assignment of coordinate systems
	* Transformation matrices for each DH matrix (A1, A2,..., A19)
	* array for storing the co-ordinates of each joint 
	
2) The following methods are defined:
	* constructor	
		-> reads joint parameters from file [robot_config2.m]
		-> sets up DH parameters for each joint from file [initial_config.m] 	{sets all DH parameter members}
		-> calls method to set up transformation matrices [setupDH()]		{sets up all transformation matrix members}
		-> calls method to calculate joint positions [updateJoints()]		{sets up members for each joint position }

	* function A = DH(obj, a, alpha, d, theta)
		-> returns the DH transformation matrix for the parameters passed as arguements

	* function obj = setupDH(obj)
		-> initialises the DH transformation matrix members

        * function obj = updateJoints(obj)
		-> updates the position of each joint based on the current transformation matrices (nb: loads existing transformation matrices)

	* function ff = ikine(obj, desired_ee)	
		-> calculates the required shank/knee/hip joint angles for a given ee position
		-> desired ee position passed as arguement
		-> How algorithm works:
    			a) error between current ee position and desired ee position computed
			b) calculate positions of knee, hip, shoulder (shoulder is considered ee). calculate distance between ee and hip/knee respectively
			c) set loop counter, convergence tolerence, and step. The step is the factor to which the algorithm updates theta
			d) while the counter is less than threshold [I know it should monitor threshold... doesnt really affect things though...]
				e) Calculate Jacobian and Pseudo Jacobian. Note: The Jacobian in this instance is a (6xn) matrix where n=#joints
				f) Calculate the amount by which to adjust the theta values (scaled by step)
				g) calls method to update the FK to incorporate these modified angles [IK_Update()]. Performs step b) again
				h) Calculate magnitude of updated error. If error is less than tolerence, exit loop. 
			i) when loop is exited, save angle values in a excel spreadsheet called 'IK_xlsx.xlsx'
