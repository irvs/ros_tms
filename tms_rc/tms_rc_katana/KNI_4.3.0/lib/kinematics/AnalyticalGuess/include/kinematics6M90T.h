/***************************************************************************
 *   Copyright (C) 2008 by Neuronics AG                                    *
 *   support@neuronics.ch                                                  *
 ***************************************************************************/

/**
 * analytical kinematics implementation for Katana6M90T.
 */

#ifndef _KINEMATICS6M90T_H_
#define _KINEMATICS6M90T_H_
//std:
#include <string>
#include <queue>
#include <vector>
#include <map>
//kinematics:
#include "kinematics.h"
#include "KatanaKinematicsDecisionAlgorithms.h"
#include "MathHelperFunctions.h"


namespace AnaGuess {

////////////////////////////////////////////////////////////////////////////
/*! \brief Implements the kinematics for the Katana6M90T
 *
 * This class implemets the abstract kinematics interface for the Katana6M90T
 */
class Kinematics6M90T : public Kinematics {
private:
	//!Number of motors of the robot
	int mNumberOfMotors;
	//!Number of segments of the robot
	int mNumberOfSegments;
	//!Effector segment lengths vector [m]
	std::vector<double> mSegmentLength;
	//!Angle offset vector [rad]
	std::vector<double> mAngleOffset;
	//!Angle stop vector [rad]
	std::vector<double> mAngleStop;
	//!Encoders per cycle vector
	std::vector<int> mEncodersPerCycle;
	//!Encoder offset vector
	std::vector<int> mEncoderOffset;
	//!Rotation direction vector [1|-1]
	std::vector<int> mRotationDirection;
	//!Initialized flag
	bool mIsInitialized;

	//!structs, type and constants used in inverse kinematics calculation
	struct position {
		double x;
		double y;
		double z;
	};
	struct angles_calc {
		double theta1;
		double theta2;
		double theta3;
		double theta4;
		double theta5;
		double theta6;
		double theta234;
		double b1;
		double b2;
		double costh3;
	};
	typedef std::vector<angles_calc> angles_container;
	static const int cNrOfPossibleSolutions = 8;

	//!initialization routine
	bool initialize();

	//!helper functions
	void IK_theta234theta5(angles_calc& angle, const position &p_gr) const;
	bool GripperTest(const position &p_gr, const angles_calc &angle) const;
	void IK_b1b2costh3_6MS(angles_calc &angle, const position &p) const;
	double findFirstEqualAngle(const std::vector<double>& v1, const std::vector<double>& v2) const;
	void thetacomp(angles_calc &angle, const position &p_m, const std::vector<double>& pose) const;
	bool PositionTest6MS(const double& theta1, const double& theta2, const double& theta3,
		const double& theta234, const position &p) const;
	bool angledef(angles_calc &a) const;
	bool AnglePositionTest(const angles_calc &a) const;

protected:

public:
	//!Constructor
	Kinematics6M90T();

	//!Destructor
	~Kinematics6M90T();

	//!get link length
	std::vector<double> getLinkLength();
	//!get encoders per cycle
	std::vector<int> getEpc();
	//!get encoder offset
	std::vector<int> getEncOff();
	//!get direction
	std::vector<int> getDir();
	//!get angle offset
	std::vector<double> getAngOff();
	//!get angle stop
	std::vector<double> getAngStop();
	//!get angle range
	std::vector<double> getAngRange();
	//!get angle min
	std::vector<double> getAngMin();
	//!get angle max
	std::vector<double> getAngMax();
	//!set link length
	bool setLinkLength(const std::vector<double> aLengths);
	//!set angle offset
	bool setAngOff(const std::vector<double> aAngOff);
	//!set angle stop
	bool setAngStop(const std::vector<double> aAngStop);

	//!calculates the angles corresponding to the given encoders
	//!@param aAngles	empty vector to store the angles
	//!@param aEncoders	the encoder vector
	//!@return true if no error occurred, false on error
	bool enc2rad(std::vector<double>& aAngles, const std::vector<int> aEncoders);

	//!calculates the encoders corresponding to the given angles
	//!@param aEncoders	empty vector to store the encoders
	//!@param aAngles	the angle vector
	//!@return true if no error occurred, false on error
	bool rad2enc(std::vector<int>& aEncoders, const std::vector<double> aAngles);

	//!calculates the direct kinematics
	//!@param aPosition	empty vector to store the position
	//!@param aAngles	the angle vector
	//!@return true if no error occurred, false on error
	bool directKinematics(std::vector<double>& aPosition, const std::vector<double> aAngles);

	//!caltulates the inverse kinematics
	//!@param aAngles	empty vector to store the angles
	//!@param aPosition	the position vector
	//!@param aStartingAngles	starting angle vector to find the best (nearest) solution
	//!@throws NoSolutionException	if no solutios exists
	//!@return true if no error occurred, false on error
	bool inverseKinematics(std::vector<double>& aAngles, const std::vector<double> aPosition, const std::vector<double> aStartingAngles);
};
////////////////////////////////////////////////////////////////////////////

} // namespace

#endif //_KINEMATICS6M90T_H_
