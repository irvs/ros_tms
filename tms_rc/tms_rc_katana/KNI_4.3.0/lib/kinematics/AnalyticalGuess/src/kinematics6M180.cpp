/***************************************************************************
 *   Copyright (C) 2008 by Neuronics AG                                    *
 *   support@neuronics.ch                                                  *
 ***************************************************************************/

#include <kinematics6M180.h>
//#include <iostream>
namespace AnaGuess {

////////////////////////////////////////////////////////////////////////////
Kinematics6M180::Kinematics6M180() {
	initialize();
}
////////////////////////////////////////////////////////////////////////////
Kinematics6M180::~Kinematics6M180() {
}

//!get link length
std::vector<double> Kinematics6M180::getLinkLength() {
	std::vector<double> result(mSegmentLength);
	return result;
}
//!get encoders per cycle
std::vector<int> Kinematics6M180::getEpc() {
	std::vector<int> result(mEncodersPerCycle);
	return result;
}
//!get encoder offset
std::vector<int> Kinematics6M180::getEncOff() {
	std::vector<int> result(mEncoderOffset);
	return result;
}
//!get direction
std::vector<int> Kinematics6M180::getDir() {
	std::vector<int> result(mRotationDirection);
	return result;
}
//!get angle offset
std::vector<double> Kinematics6M180::getAngOff() {
	std::vector<double> result(mAngleOffset);
	return result;
}
//!get angle stop
std::vector<double> Kinematics6M180::getAngStop() {
	std::vector<double> result(mAngleStop);
	return result;
}
//!get angle range
std::vector<double> Kinematics6M180::getAngRange() {
	std::vector<double> result;
	double diff;
	for (int i = 0; i < 6; i++) {
		diff = mAngleStop[i] - mAngleOffset[i];
		if (diff < 0) {
			result.push_back(-diff);
		} else {
			result.push_back(diff);
		}
	}
	return result;
}
//!get angle min
std::vector<double> Kinematics6M180::getAngMin() {
	std::vector<double> result;
	for (int i = 0; i < 6; i++) {
		if (mAngleStop[i] < mAngleOffset[i]) {
			result.push_back(mAngleStop[i]);
		} else {
			result.push_back(mAngleOffset[i]);
		}
	}
	return result;
}
//!get angle max
std::vector<double> Kinematics6M180::getAngMax() {
	std::vector<double> result;
	for (int i = 0; i < 6; i++) {
		if (mAngleStop[i] < mAngleOffset[i]) {
			result.push_back(mAngleOffset[i]);
		} else {
			result.push_back(mAngleStop[i]);
		}
	}
	return result;
}

//!set link length
bool Kinematics6M180::setLinkLength(const std::vector<double> aLengths) {
	if ((int) aLengths.size() != mNumberOfSegments) {
		return false;
	}

	for (int i = 0; i < mNumberOfSegments; ++i) {
		mSegmentLength[i] = aLengths.at(i);
	}

	return true;
}
//!set angle offset
bool Kinematics6M180::setAngOff(const std::vector<double> aAngOff) {
	if ((int) aAngOff.size() != mNumberOfMotors) {
		return false;
	}

	for (int i = 0; i < mNumberOfMotors; ++i) {
		mAngleOffset[i] = aAngOff.at(i);
	}

	return true;
}
//!set angle stop
bool Kinematics6M180::setAngStop(const std::vector<double> aAngStop) {
	if ((int) aAngStop.size() != mNumberOfMotors) {
		return false;
	}

	for (int i = 0; i < mNumberOfMotors; ++i) {
		mAngleStop[i] = aAngStop.at(i);
	}

	return true;
}

//////////////////////////////////////////////////////////////////////////
bool Kinematics6M180::enc2rad(std::vector<double>& aAngles, const std::vector<int> aEncoders) {
	for(int i = 0; i < 6; ++i) {
		aAngles[i] = MHF::enc2rad(aEncoders[i], mAngleOffset[i], mEncodersPerCycle[i], mEncoderOffset[i], mRotationDirection[i]);
	}
	return true;
}
//////////////////////////////////////////////////////////////////////////
bool Kinematics6M180::rad2enc(std::vector<int>& aEncoders, const std::vector<double> aAngles) {
	for(int i = 0; i < 6; ++i ) {
		aEncoders[i] = MHF::rad2enc(aAngles[i], mAngleOffset[i], mEncodersPerCycle[i], mEncoderOffset[i], mRotationDirection[i]);
	}
	return true;
}

////////////////////////////////////////////////////////////////////////////
bool Kinematics6M180::directKinematics(std::vector<double>& aPosition, const std::vector<double> aAngles) {
	if(!mIsInitialized) {
		initialize();
	}

	// copy aAngles to currentAngles
	std::vector<double> currentAngles(6);
	for(int i = 0; i < 6; ++i) {
		currentAngles[i] = aAngles[i];
	}

	// adjust angles (needs refactoring)
	currentAngles[1] = currentAngles[1] - MHF_PI/2.0;
	currentAngles[2] = currentAngles[2] - MHF_PI;
	currentAngles[3] = MHF_PI - currentAngles[3];
	currentAngles[4] = -currentAngles[4];

	double factor;
	double r13, r23, r31, r32;

	std::vector<double> pose(6);

	std::vector<double> cx(currentAngles.size()), sx(currentAngles.size());
	std::vector<double>::iterator cx_iter, sx_iter;

	std::vector<double> angle = currentAngles;

	angle[2] = angle[1]+angle[2];
	angle[3] = angle[2]+angle[3];

	cx_iter = cx.begin();
	sx_iter = sx.begin();
	std::transform(angle.begin(), angle.end(), sx_iter, MHF::unary_precalc_sin<double>() );
	std::transform(angle.begin(), angle.end(), cx_iter, MHF::unary_precalc_cos<double>() );

	factor = (mSegmentLength[0]*sx[1]+mSegmentLength[1]*sx[2]+(mSegmentLength[2]+mSegmentLength[3])*sx[3]);
	// x = px (compare homogenous transformation matrix)
	pose[0] = cx[0]*factor;

	// y = py (compare homogenous transformation matrix)
	pose[1] = sx[0]*factor;

	// z = pz (compare homogenous transformation matrix)
	pose[2] = mSegmentLength[0]*cx[1]+mSegmentLength[1]*cx[2]+(mSegmentLength[2]+mSegmentLength[3])*cx[3];

	// phi = atan2(r13/-r23) (compare homogenous transformation matrix)
	r13 = cx[0]*sx[3];
	r23 = sx[0]*sx[3];
	pose[3] = atan2(r13,-r23);

	// theta = acos(r33) (compare homogenous transformation matrix)
	pose[4] = acos(cx[3]);

	// psi = atan2(r31/r32)  (compare homogenous transformation matrix)
	r31 = sx[3]*sx[4];
	r32 = sx[3]*cx[4];
	pose[5] = atan2(r31,r32);

	std::swap(aPosition, pose);
	return true;
}
////////////////////////////////////////////////////////////////////////////
bool Kinematics6M180::inverseKinematics(std::vector<double>& aAngles, const std::vector<double> aPosition,
		const std::vector<double> aStartingAngles) {
	if(!mIsInitialized) {
		initialize();
	}

	// Alle 8 Loeungen werden in einem Array angle, welches aus 8 Structs besteht, gespeichert:
	// 0-3 fuer theta1_1
	// 4-7 fuer theta1_2

	// Declarations
	position p_m;
	angles_container angle(cNrOfPossibleSolutions);
	double coeff1, coeff2, theta234;
	double costh5, sinth5, theta5[2];
	double R11, R21, R31, R32;
	double phi, theta, psi;

	// Initialization
	p_m.x = aPosition[0];
	p_m.y = aPosition[1];
	p_m.z = aPosition[2];

	// calculate theta1_1 and theta1_2
	angle[0].theta1 = MHF::atan1(aPosition[0],aPosition[1]);
	if (angle[0].theta1 > MHF_PI) {
		angle[0].theta1 = angle[0].theta1 - MHF_PI;
		if (angle[0].theta1 > (179.91/180*MHF_PI)) {
			angle[0].theta1 = angle[0].theta1 - MHF_PI;
		}
	}
	angle[4].theta1 = angle[0].theta1+MHF_PI;

	theta = aPosition[4];
	psi = aPosition[5];
	phi = MHF::atan1(p_m.x,p_m.y)+MHF_PI/2.0;
	theta234 = aPosition[4];

	R11 = cos(phi)*cos(psi)-sin(phi)*cos(theta)*sin(psi);
	R21 = sin(phi)*cos(psi)+cos(phi)*cos(theta)*sin(psi);
	R31 = sin(theta)*sin(psi);
	R32 = sin(theta)*cos(psi);

	// calculate theta5
	if(theta234==0) {
		//std::cout << "Warning: Singularity theta234=0 !" << std::endl;
		for (int i=0; i<2; ++i) {
			coeff1 = -sin(angle[i*4].theta1);
			coeff2 = -cos(angle[i*4].theta1);
			costh5 = coeff1*R11-coeff2*R21;
			sinth5 = coeff1*R21+coeff2*R11;
			theta5[i] = -MHF::findFirstEqualAngle(costh5, sinth5, cTolerance);
		}
		for (int i=0; i<cNrOfPossibleSolutions; ++i) {
			if(i<4)
				angle[i].theta5 = theta5[0];
			else
				angle[i].theta5 = theta5[1];
		}
	} else if(theta234==MHF_PI) {
		//std::cout << "Warning: Singularity theta234=PI !" << std::endl;
		for (int i=0; i<2; ++i) {
			coeff1 = -sin(angle[i*4].theta1);
			coeff2 = cos(angle[i*4].theta1);
			costh5 = coeff1*R11+coeff2*R21;
			sinth5 = -coeff1*R21+coeff2*R11;
			theta5[i] = -MHF::findFirstEqualAngle(costh5, sinth5, cTolerance);
		}
		for (int i=0; i<cNrOfPossibleSolutions; ++i) {
			if(i<4)
				angle[i].theta5 = theta5[0];
			else
				angle[i].theta5 = theta5[1];
		}
	} else {
		theta5[0] = -atan2(R31/sin(theta234),R32/sin(theta234));
		theta5[1] = -atan2(R31/sin(-theta234),R32/sin(-theta234));
		for (int i=0; i<cNrOfPossibleSolutions; ++i) {
			if(i%4==0 || i%4==1)
				angle[i].theta5 = theta5[0];
			else
				angle[i].theta5 = theta5[1];
		}
	}

	//====THETA1_1==================
	//-------THETA234_1-------------
	angle[0].theta234 = aPosition[4];
	//angle[0].theta5 = pose[5];
	IK_b1b2costh3_6M180(angle[0], p_m);

	angle[1] =angle[0];
	angle[0].theta3 =  acos(angle[0].costh3)-MHF_PI;
	thetacomp(angle[0], p_m);
	angle[1].theta3 =  -acos(angle[1].costh3)+MHF_PI;
	thetacomp(angle[1], p_m);

	//-------THETA234_2-------------
	angle[2].theta1 = angle[0].theta1;
	angle[2].theta234 = -angle[0].theta234;
	//angle[2].theta5 = angle[0].theta5;
	IK_b1b2costh3_6M180(angle[2], p_m);

	angle[3] = angle[2];
	angle[2].theta3 =  acos(angle[2].costh3)-MHF_PI;
	thetacomp(angle[2], p_m);
	angle[3].theta3 =  -acos(angle[3].costh3)+MHF_PI;
	thetacomp(angle[3], p_m);

	//====THETA1_2==================
	//-------THETA234_1-------------
	angle[4].theta234 = aPosition[4];
	//angle[4].theta5 = pose[5];
	IK_b1b2costh3_6M180(angle[4], p_m);

	angle[5] = angle[4];
	angle[4].theta3 = acos(angle[4].costh3)-MHF_PI;
	thetacomp(angle[4], p_m);
	angle[5].theta3 = -acos(angle[5].costh3)+MHF_PI;
	thetacomp(angle[5], p_m);

	//-------THETA234_2-------------
	angle[6].theta1 = angle[4].theta1;
	angle[6].theta234 = -angle[4].theta234;
	//angle[6].theta5 = angle[4].theta5;
	IK_b1b2costh3_6M180(angle[6], p_m);

	angle[7] = angle[6];
	angle[6].theta3 =  acos(angle[6].costh3)-MHF_PI;
	thetacomp(angle[6], p_m);
	angle[7].theta3 =  -acos(angle[7].costh3)+MHF_PI;
	thetacomp(angle[7], p_m);

	// delete solutions out of range (in joint space)
	for( std::vector<angles_calc>::iterator iter = angle.begin(); iter != angle.end();) {
		if( MHF::pow2(iter->costh3) <= 1.0) {
			if(!angledef(*iter))
				iter = angle.erase(iter);
			else
				++iter;
			continue;
		}
		iter = angle.erase(iter);
	}

	// check if solutions in range left
	if(angle.size() == 0) {
		throw NoSolutionException();
	}

	// store possible solution angles to std::vector<std::vector<double>>
	std::vector< std::vector<double> > PossibleTargets;
	for( std::vector<angles_calc>::iterator i = angle.begin(); i != angle.end(); ++i ) {
		std::vector<double> possangles(5);

		possangles[0] = i->theta1;
		possangles[1] = i->theta2;
		possangles[2] = i->theta3;
		possangles[3] = i->theta4;
		possangles[4] = i->theta5;

		PossibleTargets.push_back(possangles);
	}

	// choose best solution
	std::vector< std::vector<double> >::const_iterator sol = KinematicsDefaultRadMinAlgorithm()(PossibleTargets.begin(), PossibleTargets.end(), aStartingAngles.begin(), aStartingAngles.end());

	if(sol == PossibleTargets.end()) {
		throw NoSolutionException();
	}

	// copy solution to aAngles vector
	for (int i = aAngles.size(); i < 6; ++i)
		aAngles.push_back(0.0);
	std::vector<double>::iterator gripper_iter = std::copy( (*sol).begin(), (*sol).end(), aAngles.begin() );
	*gripper_iter = aStartingAngles[5]; // copy gripper-angle from current

	return true;
}
////////////////////////////////////////////////////////////////////////////
bool Kinematics6M180::initialize() {
	// NOTE: data for Katana6M180 with flange
	
	mIsInitialized = false;

	//fill in segment data
	mNumberOfSegments = 4;

	mSegmentLength.push_back(190.0);
	mSegmentLength.push_back(139.0);
	mSegmentLength.push_back(147.3);
	mSegmentLength.push_back(41.0); // for anglegripper: 155.5

	//fill in joint data
	mNumberOfMotors = 6;

	mAngleOffset.push_back(0.116064);
	mAngleStop.push_back(6.154904);
	mEncodersPerCycle.push_back(51200);
	mEncoderOffset.push_back(31000);
	mRotationDirection.push_back(1);

	mAngleOffset.push_back(2.168572);
	mAngleStop.push_back(-0.274889);
	mEncodersPerCycle.push_back(94976);
	mEncoderOffset.push_back(-31000);
	mRotationDirection.push_back(1);

	mAngleOffset.push_back(0.919789);
	mAngleStop.push_back(5.283112);
	mEncodersPerCycle.push_back(47488);
	mEncoderOffset.push_back(-31000);
	mRotationDirection.push_back(-1);

	mAngleOffset.push_back(1.108284);
	mAngleStop.push_back(5.122541);
	mEncodersPerCycle.push_back(51200);
	mEncoderOffset.push_back(31000);
	mRotationDirection.push_back(1);

	mAngleOffset.push_back(0.148353);
	mAngleStop.push_back(6.117379);
	mEncodersPerCycle.push_back(51200);
	mEncoderOffset.push_back(31000);
	mRotationDirection.push_back(1);

	mAngleOffset.push_back(-2.085668);
	mAngleStop.push_back(3.656465);
	mEncodersPerCycle.push_back(51200);
	mEncoderOffset.push_back(31000);
	mRotationDirection.push_back(1);

	mIsInitialized = true;

	return mIsInitialized;
}
////////////////////////////////////////////////////////////////////////////
void Kinematics6M180::IK_b1b2costh3_6M180(angles_calc &angle, const position &p) const {
	double d5 = mSegmentLength[2] + mSegmentLength[3];

	angle.b1 = p.x*cos(angle.theta1) + p.y*sin(angle.theta1) - d5*sin(angle.theta234);
	angle.b2 = p.z - d5*cos(angle.theta234);
	angle.costh3 = -( MHF::pow2(angle.b1) + MHF::pow2(angle.b2) - MHF::pow2(mSegmentLength[0]) - MHF::pow2(mSegmentLength[1]) ) / ( 2.0*mSegmentLength[0]*mSegmentLength[1] );

}
////////////////////////////////////////////////////////////////////////////
void Kinematics6M180::thetacomp(angles_calc &angle, const position &p_m) const {
	angle.theta2 = -MHF_PI/2.0 - ( MHF::atan0(angle.b1,angle.b2)+MHF::atan0(mSegmentLength[0]+mSegmentLength[1]*cos(angle.theta3),mSegmentLength[1]*sin(angle.theta3)) );
	angle.theta4 = angle.theta234-angle.theta2-angle.theta3;

	if(!PositionTest6M180(angle,p_m)) {
		angle.theta2 = angle.theta2+MHF_PI;
		angle.theta4 = angle.theta234-angle.theta2-angle.theta3;
	}

}
////////////////////////////////////////////////////////////////////////////
bool Kinematics6M180::PositionTest6M180(const angles_calc &a, const position &p) const {
	double temp, xm2, ym2, zm2;

	temp = mSegmentLength[0]*sin(a.theta2)+mSegmentLength[1]*sin(a.theta2+a.theta3)+(mSegmentLength[2]+mSegmentLength[3])*sin(a.theta234);
	xm2 = cos(a.theta1)*temp;
	ym2 = sin(a.theta1)*temp;
	zm2 = mSegmentLength[0]*cos(a.theta2)+mSegmentLength[1]*cos(a.theta2+a.theta3)+(mSegmentLength[2]+mSegmentLength[3])*cos(a.theta234);

	if((MHF::pow2(p.x-xm2)+MHF::pow2(p.y-ym2)+MHF::pow2(p.z-zm2))>=cTolerance)
		return false;

	return true;
}
////////////////////////////////////////////////////////////////////////////
bool Kinematics6M180::angledef(angles_calc &a) const {
	// constants here. needs refactoring:
	a.theta2=MHF::anglereduce(a.theta2+MHF_PI/2.0);
	a.theta3=MHF::anglereduce(a.theta3+MHF_PI);
	a.theta4=MHF::anglereduce(MHF_PI-a.theta4);
	a.theta5=MHF::anglereduce(a.theta5);

	if(a.theta1>mAngleStop[0]) {
		a.theta1=a.theta1-2.0*MHF_PI;
	}
	if(a.theta2>MHF_PI) {
		a.theta2=a.theta2-2.0*MHF_PI;
	}
	if(a.theta5<mAngleOffset[4]) {
		a.theta5=a.theta5+2.0*MHF_PI;
	}

	return AnglePositionTest(a);

}
////////////////////////////////////////////////////////////////////////////
bool Kinematics6M180::AnglePositionTest(const angles_calc &a) const {

	if( (a.theta1+0.0087<mAngleOffset[0])||(a.theta1>mAngleStop[0]) )
		return false;

	if( (a.theta2-0.0087>mAngleOffset[1])||(a.theta2<mAngleStop[1]) )
		return false;

	if( (a.theta3<mAngleOffset[2])||(a.theta3>mAngleStop[2]) )
		return false;

	if( (a.theta4<mAngleOffset[3])||(a.theta4>mAngleStop[3]) )
		return false;

	if( (a.theta5<mAngleOffset[4])||(a.theta5>mAngleStop[4]) )
		return false;

	return true;
}
////////////////////////////////////////////////////////////////////////////

} // namespace
