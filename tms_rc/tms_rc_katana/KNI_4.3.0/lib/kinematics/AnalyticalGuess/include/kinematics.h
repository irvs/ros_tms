/***************************************************************************
 *   Copyright (C) 2008 by Neuronics AG                                    *
 *   support@neuronics.ch                                                  *
 ***************************************************************************/

/**
 * analytical kinematics interface.
 */

#ifndef _ANAGUESS_KINEMATICS_H_
#define _ANAGUESS_KINEMATICS_H_

//std:
#include <string>
#include <queue>
#include <vector>
#include <map>
#include <algorithm>

#include "exception.h"

namespace AnaGuess {

const double cTolerance = 0.0001;

/// No solution found for the given cartesian coordinates.
/// \note error_number=-10
class NoSolutionException : public Exception {
public:
    NoSolutionException() throw():
	Exception("No solution found", -10) {}

};


////////////////////////////////////////////////////////////////////////////
/*! \brief Base Class for the kinematics implementations
 *
 * This class is pure virtual.
 * The different robot kinematics implement this interface
 */
class Kinematics {

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
	//!initializes the kinematic
	virtual bool initialize() = 0;


public:
	//!Virtual destructor
	virtual ~Kinematics(){}

	//!get link length
	virtual std::vector<double> getLinkLength() = 0;
	//!get encoders per cycle
	virtual std::vector<int> getEpc() = 0;
	//!get encoder offset
	virtual std::vector<int> getEncOff() = 0;
	//!get direction
	virtual std::vector<int> getDir() = 0;
	//!get angle offset
	virtual std::vector<double> getAngOff() = 0;
	//!get angle stop
	virtual std::vector<double> getAngStop() = 0;
	//!get angle range
	virtual std::vector<double> getAngRange() = 0;
	//!get angle min
	virtual std::vector<double> getAngMin() = 0;
	//!get angle max
	virtual std::vector<double> getAngMax() = 0;
	//!set link length
	virtual bool setLinkLength(const std::vector<double> aLengths) = 0;
	//!set angle offset
	virtual bool setAngOff(const std::vector<double> aAngOff) = 0;
	//!set angle stop
	virtual bool setAngStop(const std::vector<double> aAngStop) = 0;

	//!calculates the angles corresponding to the given encoders
	//!@param aAngles	empty vector to store the angles
	//!@param aEncoders	the encoder vector
	//!@return true if no error occurred, false on error
	virtual bool enc2rad(std::vector<double>& aAngles, const std::vector<int> aEncoders) = 0;

	//!calculates the encoders corresponding to the given angles
	//!@param aEncoders	empty vector to store the encoders
	//!@param aAngles	the angle vector
	//!@return true if no error occurred, false on error
	virtual bool rad2enc(std::vector<int>& aEncoders, const std::vector<double> aAngles) = 0;

	//!calculates the direct kinematics
	//!@param aPosition	empty vector to store the position
	//!@param aAngles	the angle vector
	//!@return true if no error occurred, false on error
	virtual bool directKinematics(std::vector<double>& aPosition, const std::vector<double> aAngles) = 0;

	//!caltulates the inverse kinematics
	//!@param aAngles	empty vector to store the angles
	//!@param aPosition	the position vector
	//!@param aStartingAngles	starting angle vector to find the best (nearest) solution
	//!@throws NoSolutionException	if no solutios exists
	//!@return true if no error occurred, false on error
	virtual bool inverseKinematics(std::vector<double>& aAngles, const std::vector<double> aPosition, const std::vector<double> aStartingAngles) = 0;
};
////////////////////////////////////////////////////////////////////////////

} // namespace

#endif //_ANAGUESS_KINEMATICS_H_
