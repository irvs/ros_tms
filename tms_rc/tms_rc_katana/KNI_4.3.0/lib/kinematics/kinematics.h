/**************************************************************************
* kinematics.h -
* Kinematics Interface kinematics class for Katana4XX
* Copyright (C) 2007-2008 Neuronics AG
* PKE/UKE 2007, JHA 2008
**************************************************************************/
#ifndef _KINEMATICS_H_
#define _KINEMATICS_H_


#include "roboop/source/robot.h"
#include "roboop/source/utils.h"
#include "roboop/source/quaternion.h"

#include "AnalyticalGuess/include/kinematics6M180.h"
#include "AnalyticalGuess/include/kinematics6M90G.h"
#include "AnalyticalGuess/include/kinematics6M90T.h"

#include "katana_common.h"

#include <vector>
#include <string>


#define mPi 3.14159265358979323846

#define EDOM 33 
#define ERANGE 34 

#define KINLIB_VERSION_MAJOR 1
#define KINLIB_VERSION_MINOR 3
#define KINLIB_VERSION_REVISION 0

///// CUSTOMIZED ATAN FUNCTION ////////////////////////////////////////

template<typename _T> inline _T atan1(_T in1, _T in2) {

	if(in1==0.0 && in2 == 0.0)
		return 0.0;

	if(in1==0.0)
		return (in2<0) ? mPi*3.0/2.0 : mPi/2.0;

	if(in1<0.0)
		return atan(in2/in1)+mPi;

	if( (in1>0.0) && (in2<0.0) )
		return atan(in2/in1)+2.0*mPi;

	return atan(in2/in1);
}

///// CONSTANTS ///////////////////////////////////////////////////////

enum katana_type {
K_6M90A_F=0, // Katana 6M90A with Flange
K_6M90A_G=1, // Katana 6M90A with angle Gripper
K_6M180=2, // Katana 6M180 (link length with Flange)
K_6M90B_F=3, // Katana 6M90B with Flange
K_6M90B_G=4 // Katana 6M90B with angle Gripper
};

// multiply lengths to give them more weight relative to the angles in the
// inverse kinematics approximation of roboop --JHA
// [[in analyticalGuess the weight is more equalized, because it is calculated
// in mm (range typacally +/- 200-400) and degree (range typacally +/- 180),
// in roboop SI units are used: m (range typically +/- 0.2-0.4) and radian
// (range typically +/- 3.0). multiplying all lengths by about 10.0 for
// roboop calculations brings back a good precision in position.]]
const double LENGTH_MULTIPLIER = 10.0;

const int MaxDof = 10;

const int Dof_90 = 6;
const int Dof_180 = 5;

///////////////////////////////////////////////////////////
//The matrices for the different Katana HW configurations:
// 1 	sigma 		joint type (revolute=0, prismatic=1)
// 2 	theta 		Denavit-Hartenberg parameter
// 3 	d 			Denavit-Hartenberg parameter
// 4 	a 			Denavit-Hartenberg parameter
// 5 	alpha 		Denavit-Hartenberg parameter
// 6 	theta_{min} minimum value of joint variable
// 7 	theta_{max} maximum value of joint variable
// 8 	theta_{off} joint offset
// 9 	m 			mass of the link
// 10 	c_x 		center of mass along axis $x$
// 11 	c_y 		center of mass along axis $y$
// 12 	c_z 		center of mass along axis $z$
// 13 	I_{xx}		element $xx$ of the inertia tensor matrix
// 14 	I_{xy}		element $xy$ of the inertia tensor matrix
// 15 	I_{xz}		element $xz$ of the inertia tensor matrix
// 16 	I_{yy}		element $yy$ of the inertia tensor matrix
// 17 	I_{yz}		element $yz$ of the inertia tensor matrix
// 18 	I_{zz}		element $zz$ of the inertia tensor matrix
// 19 	I_m 		motor rotor inertia
// 20 	Gr 			motor gear ratio
// 21 	B 			motor viscous friction coefficient
// 22 	C_f 		motor Coulomb friction coefficient
// 23 	immobile 	flag for the kinematics and inverse kinematics
//   	  				(if true joint is locked, if false joint is free)

// NewMat Real elements used in Katana6Mxxx_data[] are defined as double
const Real Katana6M90A_F_data[] =
{0, 0, 0, 0, 0, -3.025529, 3.013311, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
0, 0, 0, 0, mPi/2.0, -0.274889, 2.168572, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
0, 0, 0, 0.19, 0, -2.221804, 2.141519, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
0, 0, 0, 0.139, 0, -3.551745, 0.462512, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
0, 0, 0.1473, 0, -mPi/2.0, -4.546583, 1.422443, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
0, 0, 0.036, 0, mPi/2.0, -2.085668, 3.656465, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};

const Real Katana6M90A_G_data[] =
{0, 0, 0, 0, 0, -3.025529, 3.013311, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
0, 0, 0, 0, mPi/2.0, -0.274889, 2.168572, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
0, 0, 0, 0.19, 0, -2.221804, 2.141519, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
0, 0, 0, 0.139, 0, -3.551745, 0.462512, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
0, 0, 0.1473, 0, -mPi/2.0, -4.546583, 1.422443, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
0, mPi/2.0, 0.1505, 0, mPi/2.0, -3.141593, 3.141593, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1};

const Real Katana6M180_data[] =
{0, 0, 0, 0, 0, -3.025529, 3.013311, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
0, 0, 0, 0, mPi/2.0, -0.274889, 2.168572, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
0, 0, 0, 0.19, 0, -2.221804, 2.141519, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
0, 0, 0, 0.139, 0, -3.551745, 0.462512, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
0, 0, 0.1473+0.041, 0, -mPi/2.0, -1.40499, 4.564036, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};

const Real Katana6M90B_F_data[] =
{0, 0, 0, 0, 0, -3.025529, 3.013311, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
0, 0, 0, 0, mPi/2.0, -0.274889, 2.168572, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
0, 0, 0, 0.19, 0, -2.221804, 2.141519, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
0, 0, 0, 0.139, 0, -3.551745, 0.462512, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
0, 0, 0.1473, 0, -mPi/2.0, -1.40499, 4.564036, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
0, 0, 0.036, 0, mPi/2.0, -2.160718, 3.721042, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};

const Real Katana6M90B_G_data[] =
{0, 0, 0, 0, 0, -3.025529, 3.013311, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
0, 0, 0, 0, mPi/2.0, -0.274889, 2.168572, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
0, 0, 0, 0.19, 0, -2.221804, 2.141519, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
0, 0, 0, 0.139, 0, -3.551745, 0.462512, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
0, 0, 0.1473, 0, -mPi/2.0, -1.40499, 4.564036, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
0, mPi/2.0, 0.1505, 0, mPi/2.0, -3.141593, 3.141593, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1};

// angle offset data
const double Angle_offset_90A_F[] = {-3.025529, 2.168572, -2.221804, 0.462512, 1.422443, 3.656465};
const double Angle_offset_90A_G[] = {-3.025529, 2.168572, -2.221804, 0.462512, 1.422443, 1.570796}; //[5] immobile 1.570796
const double Angle_offset_180[] = {-3.025529, 2.168572, -2.221804, 0.462512, 4.564036};
const double Angle_offset_90B_F[] = {-3.025529, 2.168572, -2.221804, 0.462512, 4.564036, 3.656465};
const double Angle_offset_90B_G[] = {-3.025529, 2.168572, -2.221804, 0.462512, 4.564036, 1.570796}; //[5] immobile 1.570796


// angle range data
const double Angle_range_90A_F[] = {6.03884, 2.443461, 4.363323, 4.014257, 5.969026, 5.742133};
const double Angle_range_90A_G[] = {6.03884, 2.443461, 4.363323, 4.014257, 5.969026, 0.0};
const double Angle_range_180[] = {6.03884, 2.443461, 4.363323, 4.014257, 5.969026};
const double Angle_range_90B_F[] = {6.03884, 2.443461, 4.363323, 4.014257, 5.969026, 5.742133};
const double Angle_range_90B_G[] = {6.03884, 2.443461, 4.363323, 4.014257, 5.969026, 0.0};

// link length data
const double Link_length_90A_F[] = {0.19, 0.139, 0.1473, 0.036};
const double Link_length_90A_G[] = {0.19, 0.139, 0.1473, 0.1505};
const double Link_length_180[] = {0.19, 0.139, 0.1473, 0.041}; // for anglegripper: 0.1555
const double Link_length_90B_F[] = {0.19, 0.139, 0.1473, 0.036};
const double Link_length_90B_G[] = {0.19, 0.139, 0.1473, 0.1505};

//! Encoder per cycle
const int Encoder_per_cycle[] = {51200, 94976, 47488, 51200, 51200, 51200};
//! Encoder offset
const int Encoder_offset[] = {31000, -31000, -31000, 31000, 31000, 31000};
//! Rotation direction
const int Rotation_direction[] = {-1, -1, 1, 1, 1, 1};


////////////////////////////////////////////////////////////////////////////
/*! \brief Kinematics class using the kinematics lib
 *
 */
class KinematicsLib{
private:

	///// VARIABLES ///////////////////////////////////////////////////////

	//! Robot Type
	int _type;
	//! Matrices for mDH, angle range and immobile data
	bool _matrixInit;
	Matrix _data;
	//! Degree of freedom
	int _dof;
	//! Degree of mobility
	int _dom;
	//! Encoder per cycle
	int _epc[MaxDof];
	//! Encoder offset
	int _encoderOffset[MaxDof];
	//! Rotation direction
	int _rotDir[MaxDof];
	//! Angle offset
	bool _angOffInit;
	double _angleOffset[MaxDof];
	//! Angle range
	bool _angRanInit;
	double _angleRange[MaxDof];
	//! Angle min
	double _angleMin[MaxDof];
	//! Angle max
	double _angleMax[MaxDof];
	//! Link length (for defined types)
	double _linkLength[4];
	//! Roboop robot class
	mRobot _robot;
	//! Last joint is immobile (eg. K_6M90A_G, K_6M90B_G), _dom = _dof - 1
	int _immobile;
	double _thetaimmobile;
	//! Analytical guess
	AnaGuess::Kinematics* _anaGuess;
	//! Kinematic initialized
	bool _initialized;
	//! TCP offset
	double _tcpOffset[4];

	///// FUNCTIONS ///////////////////////////////////////////////////////
	
	int sign(int value);
	int sign(double value);
	#ifdef WIN32
	double round( double x);
	#endif // ifdef WIN32
	int initializeMembers();
	int setAngleMinMax();
	int initDofMat(int dof);
	int angleArrMDH2vecK4D(const double arr[], std::vector<double>* vec);

	int invKin(std::vector<double> pose, std::vector<double> prev,
			std::vector<double>& angle);
	int invKin_bisec(std::vector<double> pose, std::vector<double> prev,
			std::vector<double>& conf, int maxBisection);
	int anaGuess(std::vector<double> pose, std::vector<double> prev,
			std::vector<double>& angle);
	bool checkConfig(std::vector<double> config, std::vector<double> pose,
			double tol = 0.00001);

public:

	///// CONSTRUCTOR / DESTRUCTOR ////////////////////////////////////////
	
	//!Constructor
	KinematicsLib();
	//!Constructor with robot type 0 (6M90A_F), 1 (6M90A_G), 2 (K_6M180),
	// 3 (K_6M90B_F) or 4 (K_6M90B_G)
	// sets default parameters and initializes kinematics.
	KinematicsLib(int type);
	//!Destructor
	~KinematicsLib();	

	///// SETTERS /////////////////////////////////////////////////////////
	
	/*! \brief This sets the robot type
	 *
	 * Setting robot type includes setting mDH parameters, immobile flag, degree of
	 * freedom (dof), degree of mobility (dom) and angles.
	 *
	 * type			dof	dom
	 * K_6M90A_F	6	6
	 * K_6M90A_G	6	5
	 * K_6M180 		5	5
	 * K_6M90B_F	6	6
	 * K_6M90B_G	6	5
	 *
	 * @param type	0: K_6M90A_F, 1: K_6M90A_G, 2: K_6M180, 3: K_6M90B_F, 4: K_6M90B_G
	 * @return		1 if successful, -1 if failed
	 */
	int setType(int type);

	/*! \brief This sets the modified Denavit-Hartenberg parameters
	 *
	 * Transformation from previous to current link: move a along (previous) x-axis,
	 * rotate alpha about (previous) x-axis, move d along (current) z-axis, rotate
	 * theta about (current) z-axis. Setting the mDH parameters includes setting dof,
	 * adjusting immobile flag (all free) and dom (= dof).
	 *
	 * The length of the vectors need to be the same.
	 *
	 * @param theta		angle about z-axis
	 * @param d			distance along x-axis
	 * @param a			distance previous to current along previous x-axis
	 * @param alpha		angle about previous x-axis
	 * @param typeNr	0: K_6M90A_F, 1: K_6M90A_G, 2: K_6M180, 3: K_6M90B_F, 4: K_6M90B_G, -1 for other
	 * @return			1 if successful, -1 if failed
	 */
	int setMDH(std::vector<double> theta, std::vector<double> d,
			std::vector<double> a, std::vector<double> alpha, int typeNr = -1);

	/*! \brief This sets the link length parameters (part of mDH-parameters)
	 *
	 * Setting link length parameters requires the type to be set to
	 * 0 (K_6M90A_F), 1 (K_6M90A_G), 2 (K_6M180), 3 (K_6M90B_F) or 4 (K_6M90B_G). This is
	 * done using the setType or setModifiedDH function that set the mDH
	 * parameters so they can be adjusted. The length of the vector has to be
	 * four.
	 *
	 * @param links	length of the links in m
	 * @return		1 if successful, -1 if failed
	 */
	int setLinkLen(std::vector<double> links);

	/*! \brief This sets the immobile flag of the last joint
	 *
	 * Flag for the last joint if it is free (0) or locked (1). Setting the
	 * immobile flag includes adjusting dom. Type or mDH parameters have to be set.
	 *
	 * @param immobile	last joint immobile flag: 0 for free, 1 for locked
	 *						(immobile)
	 * @return			1 if successful, -1 if failed
	 */
	int setImmob(int immobile);

	/*! \brief This sets the encoders per cycle
	 *
	 * Number of encoders in a full cycle of each joint. Length of epc needs to be
	 * equal to DOM set with Type or mDH parameters.
	 *
	 * @param epc	encoders pec cycle
	 * @return		1 if successful, -1 if failed
	 */
	int setEPC(std::vector<int> epc);

	/*! \brief This sets the encoder offsets
	 *
	 * The encoder values at the calibration stops. Length of encOffset needs to be
	 * equal to DOM set with Type or mDH parameters.
	 *
	 * @param encOffset	encoder values at calibration stops
	 * @return			1 if successful, -1 if failed
	 */
	int setEncOff(std::vector<int> encOffset);

	/*! \brief This sets the rotation direction
	 *
	 * The rotation direction of the joints: +1 if encoders and angles (mDH
	 * convention) grow in same direction, -1 if encoders and angles grow in opposit
	 * direction. Length of rotDir needs to be equal to DOM set with Type or mDH
	 * parameters.
	 *
	 * @param rotDir	rotation direction of the joints
	 * @return		1 if successful, -1 if failed
	 */
	int setRotDir(std::vector<int> rotDir);

	/*! \brief This sets the angle offsets
	 *
	 * The angles at the calibration stops in mDH convention! Length of angleOffset
	 * needs to be equal to DOM set with Type or mDH parameters.
	 *
	 * @param angleOffset	angle values at calibration stops in mDH
	 * @return				1 if successful, -1 if failed
	 */
	int setAngOff(std::vector<double> angleOffset);

	/*! \brief This sets the angle range
	 *
	 * The angle range from angle offset to angle stop in mDH convention (negative if
	 * angleOffset > angleStop)! Length of angleRange needs to be equal to DOM set
	 * with Type or mDH parameters.
	 *
	 * @param angleRange	angle range from angle offset to angle stop
	 * @return			1 if successful, -1 if failed
	 */
	int setAngRan(std::vector<double> angleRange);

	/*! \brief This sets the tcp offset
	 *
	 * Offset from the flange to the effective tcp.
	 * 
	 * @param tcpOffset	(x, y, z, psi) offset in m and rad respectively where psi
	 *						is rotation about x-axis of tool coordinate system
	 * @return			1 if successful, -1 if failed
	 */
	int setTcpOff(std::vector<double> tcpOffset);

	///// GETTERS /////////////////////////////////////////////////////////

	/*! \brief Get the robot type
	 *
	 * Get the robot type
	 *
	 * @return	type: 0 for K_6M90A_F, 1 for K_6M90A_G, 2 for K_6M180,
	 *					3 for K_6M90B_F, 4 for K_6M90B_G,
	 *					-1 for other or not set yet
	 */
	int getType();

	/*! \brief Get the maximum degree of freedom
	 *
	 * Get the maximum degree of freedom (maximum size of the vectors)
	 *
	 * @return	maximum DOF
	 */
	int getMaxDOF();

	/*! \brief Get the degree of freedom
	 *
	 * Get the degree of freedom
	 *
	 * @return	DOF (-1 if not set)
	 */
	int getDOF();

	/*! \brief Get the degree of mobility
	 *
	 * Get the degree of mobility
	 *
	 * @return	DOM (-1 if not set)
	 */
	int getDOM();

	/*! \brief Get the modified Denavit-Hartenberg parameters
	 *
	 * @param theta	vector to write in angle about z-axis
	 * @param d		vector to write in distance along x-axis
	 * @param a		vector to write in distance previous to current along previous
	 *					x-axis
	 * @param alpha	vector to write in angle about previous x-axis
	 * @return		1 if successful, -1 if failed
	 */
	int getMDH(std::vector<double>& theta, std::vector<double>& d,
			std::vector<double>& a, std::vector<double>& alpha);

	/*! \brief Get the immobile flag of the last joint
	 *
	 * @return	immobile flag: 0 if mobile, 1 if immobile (-1 if not set)
	 */
	int getImmob();

	/*! \brief Get the encoders per cycle
	 *
	 * @param epc	vector to write in encoders pec cycle
	 * @return		1 if successful, -1 if failed
	 */
	int getEPC(std::vector<int>& epc);

	/*! \brief Get the encoder offsets
	 *
	 * @param encoderOffset		vector to write in encoder values at calibration stops
	 * @return					1 if successful, -1 if failed
	 */
	int getEncOff(std::vector<int>& encOffset);

	/*! \brief Get the rotation direction
	 *
	 * @param rotationDirection		vector to write in rotation direction of the joints
	 * @return						1 if successful, -1 if failed
	 */
	int getRotDir(std::vector<int>& rotDir);

	/*! \brief Get the angle offsets
	 *
	 * @param angleOffset	vector to write in angle values at calibration stops
	 * @return				1 if successful, -1 if failed
	 */
	int getAngOff(std::vector<double>& angleOffset);

	/*! \brief Get the angle range
	 *
	 * @param angleRange	vector to write in angle range
	 * @return				1 if successful, -1 if failed
	 */
	int getAngRan(std::vector<double>& angleRange);
	
	/*! \brief Get angle stop
	 * 
	 * @param angleStop		vector to write angle stop
	 * @return				1 if successful, -1 if failed
	 */
	int getAngStop(std::vector<double>& angleStop);
	
	/*! \brief Get angle min
	 * 
	 * @param angleMin		vector to write angle min
	 * @return				1 if successful, -1 if failed
	 */
	int getAngMin(std::vector<double>& angleMin);
	
	/*! \brief Get angle max
	 * 
	 * @param angleMax		vector to write angle max
	 * @return				1 if successful, -1 if failed
	 */
	int getAngMax(std::vector<double>& angleMax);

	/*! \brief Get the tcp offset
	 *
	 * @param tcpOffset		vector to write in (x, y, z, psi) offset
	 * @return				1 if successful, -1 if failed
	 */
	int getTcpOff(std::vector<double>& tcpOffset);

	/*! \brief Get the version number of the library
	 *
	 * @param version		vector to write in version (major, minor, revision)
	 * @return				1 if successful, -1 if failed
	 */
	int getVersion(std::vector<int>& version);

	///// INITIALIZATION //////////////////////////////////////////////////

	/*! \brief This initializes the kinematics
	 *
	 * Initialize the kinematics. Parameters mDH, dof, angle offset and angle range
	 * (SetType or SetMDH and SetAngleOffset and SetAngleRange) have to be set
	 * prior to initialization of the kinematics!
	 *
	 * @return	1 if successful, -1 if failed
	 */
	int init();
	
	///// ANGLE DEFINITION CONVERSION /////////////////////////////////////
	
	/*! \brief This converts angles from Katana4D to mDH convention
	*
	* Length of angleK4D needs to be equal to DOM set with Type or mDH parameters.
	*
	* @param angleK4D	angle in K4D convention
	* @param angleMDH	angle in mDH convention
	* @return			1 if successful, -1 if failed
	*/
	int K4D2mDHAng(std::vector<double> angleK4D, std::vector<double>& angleMDH);

	/*! \brief This converts angles from mDH to Katana4D convention
	*
	* Length of angleMDH needs to be equal to DOM set with Type or mDH parameters.
	*
	* @param angleMDH	angle in mDH convention
	* @param angleK4D	angle in K4D convention
	* @return			1 if successful, -1 if failed
	*/
	int mDH2K4DAng(std::vector<double> angleMDH, std::vector<double>& angleK4D);

	///// ENCODER <-> ANGLE CONVERSION ////////////////////////////////////
	
	/*! \brief Converts encoders to angles
	 * 
	 * @param encoders	encoder positions of joints
	 * @param angles	resulting joint angles
	 * @return			1 if successful, -1 if failed
	 */
	int enc2rad(std::vector<int> encoders, std::vector<double>& angles);
	
	/*! \brief Converts angles to encoders
	 * 
	 * @param angles	joint angles
	 * @param encoders	resulting encoder positions of joints
	 * @return			1 if successful, -1 if failed
	 */
	int rad2enc(std::vector<double> angles, std::vector<int>& encoders);
	
	///// KINEMATICS //////////////////////////////////////////////////////

	/*! \brief Calculates the direct kinematics
	 * 
	 * @param angles	robot configuration (joint angles)
	 * @param pose		resulting pose of the TCP [x,	 y, z, phi, theta, psi]
	 * @return			1 if successful, -1 if failed
	 */
	int directKinematics(std::vector<double> angles, std::vector<double>& pose);
	
	/*! \brief Calculates the inverse kinematics
	 * 
	 * @param pose		pose of the TCP [x, y, z, phi, theta, psi]
	 * @param prev		previous angles (starting configuration)
	 * @param angles	resulting robot configuration (joint angles)
	 * @return			1 if successful, -1 if failed
	 */
	int inverseKinematics(std::vector<double> pose, std::vector<double> prev,
			std::vector<double>& angles, int maxBisection = 0);
	
};

#endif //_KINEMATICS_H_
