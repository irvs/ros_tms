 /**************************************************************************
* kinematics.cpp -
* Kinematics class for Katana4XX Robots using kinematics lib
* Copyright (C) 2007-2008 Neuronics AG
* PKE/UKE 2007, JHA 2008
 **************************************************************************/
 
#include "kinematics.h"

///////////////////////////////////////////////////////////////////////
KinematicsLib::KinematicsLib(){
	initializeMembers();
}
///////////////////////////////////////////////////////////////////////
KinematicsLib::KinematicsLib(int type){
	initializeMembers();
	setType(type);
	init();
}
///////////////////////////////////////////////////////////////////////
KinematicsLib::~KinematicsLib(){
	delete _anaGuess;
}
///////////////////////////////////////////////////////////////////////


int KinematicsLib::sign(int value) {
	return (value > 0) - (value < 0);
}
int KinematicsLib::sign(double value) {
	return (value > 0) - (value < 0);
}
#ifdef WIN32
double KinematicsLib::round( double x)
// Copyright (C) 2001 Tor M. Aamodt, University of Toronto
// Permisssion to use for all purposes commercial and otherwise granted.
// THIS MATERIAL IS PROVIDED "AS IS" WITHOUT WARRANTY, OR ANY CONDITION OR
// OTHER TERM OF ANY KIND INCLUDING, WITHOUT LIMITATION, ANY WARRANTY
// OF MERCHANTABILITY, SATISFACTORY QUALITY, OR FITNESS FOR A PARTICULAR
// PURPOSE.
{
   if( x > 0 ) {
       __int64 xint = (__int64) (x+0.5);
       if( xint % 2 ) {
           // then we might have an even number...
           double diff = x - (double)xint;
           if( diff == -0.5 )
               return double(xint-1);
       }
       return double(xint);
   } else {
       __int64 xint = (__int64) (x-0.5);
       if( xint % 2 ) {
           // then we might have an even number...
           double diff = x - (double)xint;
           if( diff == 0.5 )
               return double(xint+1);
       }
       return double(xint);
   }
}
#endif // ifdef WIN32
int KinematicsLib::initializeMembers() {
	_type = -1;
	_matrixInit = false;
	_dof = -1;
	_dom = -1;
	_angOffInit = false;
	_angRanInit = false;
	_immobile = 0;
	_thetaimmobile = 0.0;
	_initialized = false;
	for (int i = 0; i < 4; ++i) {
		_tcpOffset[i] = 0.0;
	}
	return 1;
}
int KinematicsLib::setAngleMinMax() {
	int dir;
	for (int i = 0; i < _dof; i++) {
		dir = sign(_encoderOffset[i]) * _rotDir[i];
		if (dir < 0) {
			_angleMin[i] = _angleOffset[i];
			_angleMax[i] = _angleOffset[i] + _angleRange[i];
		} else {
			_angleMax[i] = _angleOffset[i];
			_angleMin[i] = _angleOffset[i] - _angleRange[i];
		}
		_data(i + 1, 6) = _angleMin[i];
		_data(i + 1, 7) = _angleMax[i];
	}

	return 1;
}
int KinematicsLib::initDofMat(int dof) {
	_dof = dof;
	_dom = _dof;
	_data = Matrix(_dof, 23);
	_data = 0.0;
	_matrixInit = true;

	return 1;
}
int KinematicsLib::angleArrMDH2vecK4D(const double arr[], std::vector<double>* angleK4D) {
	if (_type < 0)
		return -1;
	std::vector<double> angleMDH;
	for (int i = 0; i < _dom; ++i) {
		angleMDH.push_back(arr[i]);
	}
	angleK4D->clear();
	mDH2K4DAng(angleMDH, *angleK4D);
	return 1;
}


///////////////////////////////////////////////////////////////////////
int KinematicsLib::setType(int type) {
	std::vector<double> angOff;
	double angStopArr[MaxDof];
	std::vector<double> angStop;
	std::vector<double> lengths;
	switch(type) {
	case K_6M90A_F: // 0
		_type = type;
		initDofMat(Dof_90);
		_data << Katana6M90A_F_data;
		for (int i = 0; i < _dof; i++) {
			_data(i + 1, 3) *= LENGTH_MULTIPLIER;
			_data(i + 1, 4) *= LENGTH_MULTIPLIER;
		}
		for (int i = 0; i < _dof; i++) {
			_epc[i] = Encoder_per_cycle[i];
			_encoderOffset[i] = Encoder_offset[i];
			_rotDir[i] = Rotation_direction[i];
			_angleOffset[i] = Angle_offset_90A_F[i];
			_angleRange[i] = Angle_range_90A_F[i];
		}
		_angOffInit = true;
		_angRanInit = true;
		setAngleMinMax();
		for (int i = 0; i < 4; i++) {
			_linkLength[i] = Link_length_90A_F[i];
		}
		// analytical guess
		_anaGuess = (AnaGuess::Kinematics*) new AnaGuess::Kinematics6M90T();
		angleArrMDH2vecK4D(_angleOffset, &angOff);
		_anaGuess->setAngOff(angOff);
		for (int i = 0; i < _dom; ++i) {
			angStopArr[i] = _angleOffset[i] - sign(_encoderOffset[i]) * _rotDir[i] *
				_angleRange[i];
		}
		angleArrMDH2vecK4D(angStopArr, &angStop);
		_anaGuess->setAngStop(angStop);
		for (int i = 0; i < 4; ++i) {
			lengths.push_back(_linkLength[i] * 1000);
		}
		_anaGuess->setLinkLength(lengths);
		break;
	case K_6M90A_G: // 1
		_type = type;
		initDofMat(Dof_90);
		_data << Katana6M90A_G_data;
		for (int i = 0; i < _dof; i++) {
			_data(i + 1, 3) *= LENGTH_MULTIPLIER;
			_data(i + 1, 4) *= LENGTH_MULTIPLIER;
		}
		_dom = Dof_90 - 1;
		_immobile = true;
		_thetaimmobile = _data(_dof, 2);
		for (int i = 0; i < _dof; i++) {
			_epc[i] = Encoder_per_cycle[i];
			_encoderOffset[i] = Encoder_offset[i];
			_rotDir[i] = Rotation_direction[i];
			_angleOffset[i] = Angle_offset_90A_G[i];
			_angleRange[i] = Angle_range_90A_G[i];
		}
		_angOffInit = true;
		_angRanInit = true;
		setAngleMinMax();
		for (int i = 0; i < 4; i++) {
			_linkLength[i] = Link_length_90A_G[i];
		}
		// analytical guess
		_anaGuess = (AnaGuess::Kinematics*) new AnaGuess::Kinematics6M90G();
		angleArrMDH2vecK4D(_angleOffset, &angOff);
		angOff.push_back(-2.150246); // immobile angle in mDH
		_anaGuess->setAngOff(angOff);
		for (int i = 0; i < _dom; ++i) {
			angStopArr[i] = _angleOffset[i] - sign(_encoderOffset[i]) * _rotDir[i] *
				_angleRange[i];
		}
		angleArrMDH2vecK4D(angStopArr, &angStop);
		angStop.push_back(3.731514); // immobile angle in mDH
		_anaGuess->setAngStop(angStop);
		for (int i = 0; i < 4; ++i) {
			lengths.push_back(_linkLength[i] * 1000);
		}
		_anaGuess->setLinkLength(lengths);
		break;
	case K_6M180: // 2
		_type = type;
		initDofMat(Dof_180);
		_data << Katana6M180_data;
		for (int i = 0; i < _dof; i++) {
			_data(i + 1, 3) *= LENGTH_MULTIPLIER;
			_data(i + 1, 4) *= LENGTH_MULTIPLIER;
		}
		for (int i = 0; i < _dof; i++) {
			_epc[i] = Encoder_per_cycle[i];
			_encoderOffset[i] = Encoder_offset[i];
			_rotDir[i] = Rotation_direction[i];
			_angleOffset[i] = Angle_offset_180[i];
			_angleRange[i] = Angle_range_180[i];
		}
		_angOffInit = true;
		_angRanInit = true;
		setAngleMinMax();
		for (int i = 0; i < 4; i++) {
			_linkLength[i] = Link_length_180[i];
		}
		// analytical guess
		_anaGuess = (AnaGuess::Kinematics*) new AnaGuess::Kinematics6M180();
		angleArrMDH2vecK4D(_angleOffset, &angOff);
		angOff.push_back(-2.150246); // angle not present in mDH
		_anaGuess->setAngOff(angOff);
		for (int i = 0; i < _dom; ++i) {
			angStopArr[i] = _angleOffset[i] - sign(_encoderOffset[i]) * _rotDir[i] *
				_angleRange[i];
		}
		angleArrMDH2vecK4D(angStopArr, &angStop);
		angStop.push_back(3.731514); // angle not present in mDH
		_anaGuess->setAngStop(angStop);
		for (int i = 0; i < 4; ++i) {
			lengths.push_back(_linkLength[i] * 1000);
		}
		_anaGuess->setLinkLength(lengths);
		break;
	case K_6M90B_F: // 3
		_type = type;
		initDofMat(Dof_90);
		_data << Katana6M90B_F_data;
		for (int i = 0; i < _dof; i++) {
			_data(i + 1, 3) *= LENGTH_MULTIPLIER;
			_data(i + 1, 4) *= LENGTH_MULTIPLIER;
		}
		for (int i = 0; i < _dof; i++) {
			_epc[i] = Encoder_per_cycle[i];
			_encoderOffset[i] = Encoder_offset[i];
			_rotDir[i] = Rotation_direction[i];
			_angleOffset[i] = Angle_offset_90B_F[i];
			_angleRange[i] = Angle_range_90B_F[i];
		}
		_angOffInit = true;
		_angRanInit = true;
		setAngleMinMax();
		for (int i = 0; i < 4; i++) {
			_linkLength[i] = Link_length_90B_F[i];
		}
		// analytical guess
		_anaGuess = (AnaGuess::Kinematics*) new AnaGuess::Kinematics6M90T();
		angleArrMDH2vecK4D(_angleOffset, &angOff);
		_anaGuess->setAngOff(angOff);
		for (int i = 0; i < _dom; ++i) {
			angStopArr[i] = _angleOffset[i] - sign(_encoderOffset[i]) * _rotDir[i] *
				_angleRange[i];
		}
		angleArrMDH2vecK4D(angStopArr, &angStop);
		_anaGuess->setAngStop(angStop);
		for (int i = 0; i < 4; ++i) {
			lengths.push_back(_linkLength[i] * 1000);
		}
		_anaGuess->setLinkLength(lengths);
		break;
	case K_6M90B_G: // 4
		_type = type;
		initDofMat(Dof_90);
		_data << Katana6M90B_G_data;
		for (int i = 0; i < _dof; i++) {
			_data(i + 1, 3) *= LENGTH_MULTIPLIER;
			_data(i + 1, 4) *= LENGTH_MULTIPLIER;
		}
		_dom = Dof_90 - 1;
		_immobile = true;
		_thetaimmobile = _data(_dof, 2);
		for (int i = 0; i < _dof; i++) {
			_epc[i] = Encoder_per_cycle[i];
			_encoderOffset[i] = Encoder_offset[i];
			_rotDir[i] = Rotation_direction[i];
			_angleOffset[i] = Angle_offset_90B_G[i];
			_angleRange[i] = Angle_range_90B_G[i];
		}
		_angOffInit = true;
		_angRanInit = true;
		setAngleMinMax();
		for (int i = 0; i < 4; i++) {
			_linkLength[i] = Link_length_90B_G[i];
		}
		// analytical guess
		_anaGuess = (AnaGuess::Kinematics*) new AnaGuess::Kinematics6M90G();
		angleArrMDH2vecK4D(_angleOffset, &angOff);
		angOff.push_back(-2.150246); // immobile angle in mDH
		_anaGuess->setAngOff(angOff);
		for (int i = 0; i < _dom; ++i) {
			angStopArr[i] = _angleOffset[i] - sign(_encoderOffset[i]) * _rotDir[i] *
				_angleRange[i];
		}
		angleArrMDH2vecK4D(angStopArr, &angStop);
		angStop.push_back(3.731514); // immobile angle in mDH
		_anaGuess->setAngStop(angStop);
		for (int i = 0; i < 4; ++i) {
			lengths.push_back(_linkLength[i] * 1000);
		}
		_anaGuess->setLinkLength(lengths);
		break;
	default:
		return -1;
	}

	return 1;
}
///////////////////////////////////////////////////////////////////////
int KinematicsLib::setMDH(std::vector<double> theta, std::vector<double> d,
		std::vector<double> a, std::vector<double> alpha, int typeNr) {
	// check vector sizes
	if (_dof == -1) {
		if ((int)theta.size() > MaxDof)
			return -1;
		initDofMat(theta.size());
	}

	if ((int)theta.size() != _dof || (int)d.size() != _dof ||
			(int)a.size() != _dof || (int)alpha.size() != _dof) {
		return -1;
	}

	if (typeNr >= 0)
		typeNr = -2;

	for (int i = 0; i < _dof; ++i) {
		_data(i + 1, 2) = theta.at(i);
		_data(i + 1, 3) = d.at(i) * LENGTH_MULTIPLIER;
		_data(i + 1, 4) = a.at(i) * LENGTH_MULTIPLIER;
		_data(i + 1, 5) = alpha.at(i);
		_data(i + 1, 23) = 0;
	}

	_dom = _dof;
	_immobile = false;
	_type = typeNr;

	return 1;
}
///////////////////////////////////////////////////////////////////////
int KinematicsLib::setLinkLen(std::vector<double> links) {
	if ((_dof == -1) || ((int)links.size() != 4))
		return -1;

	switch (_type) {
	case K_6M90A_F: // 0
	case K_6M90A_G: // 1
	case K_6M90B_F: // 3
	case K_6M90B_G: // 4
		_data(3, 4) = links.at(0) * LENGTH_MULTIPLIER;
		_data(4, 4) = links.at(1) * LENGTH_MULTIPLIER;
		_data(5, 3) = links.at(2) * LENGTH_MULTIPLIER;
		_data(6, 3) = links.at(3) * LENGTH_MULTIPLIER;
		break;
	case K_6M180: // 2
		_data(3, 4) = links.at(0) * LENGTH_MULTIPLIER;
		_data(4, 4) = links.at(1) * LENGTH_MULTIPLIER;
		_data(5, 3) = (links.at(2) + links.at(3)) * LENGTH_MULTIPLIER;
		break;
	default:
		return -1;
	}

	// store in _linkLength
	for (int i = 0; i < 4; ++i) {
		_linkLength[i] = links.at(i);
	}

	// set in AnalyticalGuess
	std::vector<double> lengths;
	for (int i = 0; i < 4; ++i) {
		lengths.push_back(_linkLength[i] * 1000);
	}
	_anaGuess->setLinkLength(lengths);

	return 1;
}
///////////////////////////////////////////////////////////////////////
int KinematicsLib::setImmob(int immob) {
	if (_dof == -1 || immob < 0 || immob > 1) {
		return -1;
	}

	_data(_dof, 23) = immob;
	_immobile = immob;
	if (immob) {
		_dom = _dof - 1;
		_thetaimmobile = _data(_dof, 2);
	} else {
		_dom = _dof;
	}

	return 1;
}
///////////////////////////////////////////////////////////////////////
int KinematicsLib::setEPC(std::vector<int> epc) {
	if ((int)epc.size() < _dom) {
		return -1;
	}

	for (int i = 0; i < _dom; ++i) {
		_epc[i] = epc.at(i);
	}

	return 1;
}
///////////////////////////////////////////////////////////////////////
int KinematicsLib::setEncOff(std::vector<int> encOffset) {
	if ((int)encOffset.size() < _dom) {
		return -1;
	}

	for (int i = 0; i < _dom; ++i) {
		_encoderOffset[i] = encOffset.at(i);
	}

	return 1;
}
///////////////////////////////////////////////////////////////////////
int KinematicsLib::setRotDir(std::vector<int> rotDir) {
	if ((int)rotDir.size() < _dom) {
		return -1;
	}

	for (int i = 0; i < _dom; ++i) {
		if (rotDir.at(i) < 0) {
			_rotDir[i] = -1;
		} else {
			_rotDir[i] = 1;
		}
	}

	return 1;
}
///////////////////////////////////////////////////////////////////////
int KinematicsLib::setAngOff(std::vector<double> angleOffset) {
	if ((int)angleOffset.size() < _dom) {
		return -1;
	}

	for (int i = 0; i < _dom; ++i) {
		_angleOffset[i] = angleOffset.at(i);
	}
	_angOffInit = true;
	if (_angRanInit)
		setAngleMinMax();

	// analytical guess
	std::vector<double> angOff;
	double angStopArr[MaxDof];
	std::vector<double> angStop;
	switch(_type) {
	case K_6M90A_F: // 0
	case K_6M90B_F: // 3
		angleArrMDH2vecK4D(_angleOffset, &angOff);
		_anaGuess->setAngOff(angOff);
		for (int i = 0; i < _dom; ++i) {
			angStopArr[i] = _angleOffset[i] - sign(_encoderOffset[i]) * _rotDir[i] *
				_angleRange[i];
		}
		angleArrMDH2vecK4D(angStopArr, &angStop);
		_anaGuess->setAngStop(angStop);
		break;
	case K_6M90A_G: // 1
	case K_6M90B_G: // 4
		angleArrMDH2vecK4D(_angleOffset, &angOff);
		angOff.push_back(-2.150246); // immobile angle in mDH
		_anaGuess->setAngOff(angOff);
		for (int i = 0; i < _dom; ++i) {
			angStopArr[i] = _angleOffset[i] - sign(_encoderOffset[i]) * _rotDir[i] *
				_angleRange[i];
		}
		angleArrMDH2vecK4D(angStopArr, &angStop);
		angStop.push_back(3.731514); // immobile angle in mDH
		_anaGuess->setAngStop(angStop);
		break;
	case K_6M180: // 2
		angleArrMDH2vecK4D(_angleOffset, &angOff);
		angOff.push_back(-2.150246); // angle not present in mDH
		_anaGuess->setAngOff(angOff);
		for (int i = 0; i < _dom; ++i) {
			angStopArr[i] = _angleOffset[i] - sign(_encoderOffset[i]) * _rotDir[i] *
				_angleRange[i];
		}
		angleArrMDH2vecK4D(angStopArr, &angStop);
		angStop.push_back(3.731514); // angle not present in mDH
		_anaGuess->setAngStop(angStop);
		break;
	default:
		break;
	}

	return 1;
}
///////////////////////////////////////////////////////////////////////
int KinematicsLib::setAngRan(std::vector<double> angleRange) {
	if ((int)angleRange.size() < _dom) {
		return -1;
	}

	for (int i = 0; i < _dom; ++i) {
		_angleRange[i] = angleRange.at(i);
	}
	_angRanInit = true;
	if (_angOffInit)
		setAngleMinMax();

	// analytical guess
	double angStopArr[MaxDof];
	std::vector<double> angStop;
	switch(_type) {
	case K_6M90A_F: // 0
	case K_6M90B_F: // 3
		for (int i = 0; i < _dom; ++i) {
			angStopArr[i] = _angleOffset[i] - sign(_encoderOffset[i]) * _rotDir[i] *
				_angleRange[i];
		}
		angleArrMDH2vecK4D(angStopArr, &angStop);
		_anaGuess->setAngStop(angStop);
		break;
	case K_6M90A_G: // 1
	case K_6M90B_G: // 4
		for (int i = 0; i < _dom; ++i) {
			angStopArr[i] = _angleOffset[i] - sign(_encoderOffset[i]) * _rotDir[i] *
				_angleRange[i];
		}
		angleArrMDH2vecK4D(angStopArr, &angStop);
		angStop.push_back(3.731514); // immobile angle in mDH
		_anaGuess->setAngStop(angStop);
		break;
	case K_6M180: // 2
		for (int i = 0; i < _dom; ++i) {
			angStopArr[i] = _angleOffset[i] - sign(_encoderOffset[i]) * _rotDir[i] *
				_angleRange[i];
		}
		angleArrMDH2vecK4D(angStopArr, &angStop);
		angStop.push_back(3.731514); // angle not present in mDH
		_anaGuess->setAngStop(angStop);
		break;
	default:
		break;
	}

	return 1;
}
///////////////////////////////////////////////////////////////////////
int KinematicsLib::setTcpOff(std::vector<double> tcpOffset) {
	if ((int)tcpOffset.size() < 4) {
		return -1;
	}

	for (int i = 0; i < 4; ++i) {
		_tcpOffset[i] = tcpOffset.at(i);
	}

	return 1;
}
///////////////////////////////////////////////////////////////////////
int KinematicsLib::getType() {
	return _type;
}
///////////////////////////////////////////////////////////////////////
int KinematicsLib::getMaxDOF() {
	return MaxDof;
}
///////////////////////////////////////////////////////////////////////
int KinematicsLib::getDOF() {
	return _dof;
}
///////////////////////////////////////////////////////////////////////
int KinematicsLib::getDOM() {
	return _dom;
}
///////////////////////////////////////////////////////////////////////
int KinematicsLib::getMDH(std::vector<double>& theta, std::vector<double>& d,
		std::vector<double>& a, std::vector<double>& alpha) {
	if (_dof == -1)
		return -1;
	theta.clear();
	d.clear();
	a.clear();
	alpha.clear();
	for (int i = 0; i < _dof; ++i) {
		theta.push_back(_data(i + 1, 2));
		d.push_back(_data(i + 1, 3) / LENGTH_MULTIPLIER);
		a.push_back(_data(i + 1, 4) / LENGTH_MULTIPLIER);
		alpha.push_back(_data(i + 1, 5));
	}

	return 1;
}
///////////////////////////////////////////////////////////////////////
int KinematicsLib::getImmob() {
	return _immobile;
}
///////////////////////////////////////////////////////////////////////
int KinematicsLib::getEPC(std::vector<int>& epc) {
	if (_dof == -1)
		return -1;
	epc.clear();
	for (int i = 0; i < _dom; ++i) {
		epc.push_back(_epc[i]);
	}

	return 1;
}
///////////////////////////////////////////////////////////////////////
int KinematicsLib::getEncOff(std::vector<int>& encOffset) {
	if (_dof == -1)
		return -1;
	encOffset.clear();
	for (int i = 0; i < _dom; ++i) {
		encOffset.push_back(_encoderOffset[i]);
	}

	return 1;
}
///////////////////////////////////////////////////////////////////////
int KinematicsLib::getRotDir(std::vector<int>& rotDir) {
	if (_dof == -1)
		return -1;
	rotDir.clear();
	for (int i = 0; i < _dom; ++i) {
		rotDir.push_back(_rotDir[i]);
	}

	return 1;
}
///////////////////////////////////////////////////////////////////////
int KinematicsLib::getAngOff(std::vector<double>& angleOffset) {
	if (_dof == -1)
		return -1;
	angleOffset.clear();
	for (int i = 0; i < _dom; ++i) {
		angleOffset.push_back(_angleOffset[i]);
	}

	return 1;
}
///////////////////////////////////////////////////////////////////////
int KinematicsLib::getAngRan(std::vector<double>& angleRange) {
	if (_dof == -1)
		return -1;
	angleRange.clear();
	for (int i = 0; i < _dom; ++i) {
		angleRange.push_back(_angleRange[i]);
	}

	return 1;
}
///////////////////////////////////////////////////////////////////////
int KinematicsLib::getAngStop(std::vector<double>& angleStop) {
	std::vector<double> angoff;
	int okcount = getAngOff(angoff);
	std::vector<int> encoff;
	okcount += getEncOff(encoff);
	std::vector<int> rotdir;
	okcount += getRotDir(rotdir);
	std::vector<double> angran;
	okcount += getAngRan(angran);
	angleStop.clear();
	for (int i = 0; i < _dom; i++) {
		angleStop.push_back(angoff.at(i) - (sign(encoff.at(i)) * rotdir.at(i))
				* (angran.at(i)));
	}
	int ok = (okcount == 4) ? 1 : 0;
	return ok;
}
///////////////////////////////////////////////////////////////////////
int KinematicsLib::getAngMin(std::vector<double>& angleMin) {
	std::vector<double> angoff;
	int okcount = getAngOff(angoff);
	std::vector<double> angstop;
	okcount += getAngStop(angstop);
	angleMin.clear();
	for (int i = 0; i < _dom; ++i) {
		angleMin.push_back(min(angoff.at(i), angstop.at(i)));
	}
	int ok = (okcount == 2) ? 1 : 0;
	return ok;
}
///////////////////////////////////////////////////////////////////////
int KinematicsLib::getAngMax(std::vector<double>& angleMax) {
	std::vector<double> angoff;
	int okcount = getAngOff(angoff);
	std::vector<double> angstop;
	okcount += getAngStop(angstop);
	angleMax.clear();
	for (int i = 0; i < _dom; ++i) {
		angleMax.push_back(max(angoff.at(i), angstop.at(i)));
	}
	int ok = (okcount == 2) ? 1 : 0;
	return ok;
}
///////////////////////////////////////////////////////////////////////
int KinematicsLib::getTcpOff(std::vector<double>& tcpOffset) {
	if (_dof == -1)
		return -1;
	tcpOffset.clear();
	for (int i = 0; i < 4; ++i) {
		tcpOffset.push_back(_tcpOffset[i]);
	}

	return 1;
}
///////////////////////////////////////////////////////////////////////
int KinematicsLib::getVersion(std::vector<int>& version) {
	version.clear();
	version.push_back(KINLIB_VERSION_MAJOR);
	version.push_back(KINLIB_VERSION_MINOR);
	version.push_back(KINLIB_VERSION_REVISION);

	return 1;
}
///////////////////////////////////////////////////////////////////////
int KinematicsLib::init() {
	if (!_matrixInit || !_angOffInit || !_angRanInit)
		return -1;

	_robot = mRobot(_data);
	_initialized = true;

	return 1;
}
///////////////////////////////////////////////////////////////////////
int KinematicsLib::K4D2mDHAng(std::vector<double> angleK4D, std::vector<double>& angleMDH) {
	if (_type == -1)
		return -1;

	if ((int)angleK4D.size() < _dom)
		return -1;

	angleMDH.clear();
	
	// for all models the same
	angleMDH.push_back(angleK4D.at(0) - mPi);
	angleMDH.push_back(angleK4D.at(1));
	angleMDH.push_back(angleK4D.at(2) - mPi);
	angleMDH.push_back(mPi / 2.0 - angleK4D.at(3));

	// model specific
	switch (_type) {
	case K_6M90A_F: // 0
	case K_6M90B_F: // 3
		angleMDH.push_back(mPi / 2.0 - angleK4D.at(4));
		angleMDH.push_back(mPi / 2.0 - angleK4D.at(5));
		break;
	case K_6M90A_G: // 1
	case K_6M90B_G: // 4
		angleMDH.push_back(mPi / 2.0 - angleK4D.at(4));
		break;
	case K_6M180: // 2
		angleMDH.push_back(-1.0 * angleK4D.at(4) + 3.0 * mPi / 2.0);
		break;
	default:
		return -1;
	}

	return 1;
}
///////////////////////////////////////////////////////////////////////
int KinematicsLib::mDH2K4DAng(std::vector<double> angleMDH, std::vector<double>& angleK4D) {
	if (_type == -1)
		return -1;

	if ((int)angleMDH.size() < _dom)
		return -1;

	angleK4D.clear();
	
	// for all models the same
	angleK4D.push_back(angleMDH.at(0) + mPi);
	angleK4D.push_back(angleMDH.at(1));
	angleK4D.push_back(angleMDH.at(2) + mPi);
	angleK4D.push_back(mPi / 2.0 - angleMDH.at(3));

	// model specific
	switch (_type) {
	case K_6M90A_F: // 0
	case K_6M90B_F: // 3
		angleK4D.push_back(mPi / 2.0 - angleMDH.at(4));
		angleK4D.push_back(mPi / 2.0 - angleMDH.at(5));
		break;
	case K_6M90A_G: // 1
	case K_6M90B_G: // 4
		angleK4D.push_back(mPi / 2.0 - angleMDH.at(4));
		break;
	case K_6M180: // 2
		angleK4D.push_back(-1.0 * angleMDH.at(4) + 3.0 * mPi / 2.0);
		break;
	default:
		return -1;
	}

	return 1;
}
///////////////////////////////////////////////////////////////////////
int KinematicsLib::enc2rad(std::vector<int> encoders,
		std::vector<double>& angles) {
	if ((int)encoders.size() < _dom)
		return -1;

	angles.clear();
	for(int i = 0; i < _dom; ++i) {
		angles.push_back(_angleOffset[i] + _rotDir[i] * (encoders.at(i) -
			_encoderOffset[i]) * 2.0 * mPi / ((double) _epc[i]));
	}

	return 1;
}
///////////////////////////////////////////////////////////////////////
int KinematicsLib::rad2enc(std::vector<double> angles,
		std::vector<int>& encoders) {
	if ((int)angles.size() < _dom)
		return -1;

	encoders.clear();
	for(int i = 0; i < _dom; ++i){
		encoders.push_back((int) round(_encoderOffset[i] + _rotDir[i] * (angles.at(i) -
			_angleOffset[i]) * _epc[i] / (2 * mPi)));
	}

	return 1;
}
///////////////////////////////////////////////////////////////////////
int KinematicsLib::directKinematics(std::vector<double> angles,
		std::vector<double>& pose) {
	if (!_initialized || (int)angles.size() < _dom)
		return -1;
	
	//Angles in ColumnVector
	ColumnVector qr = ColumnVector(_dof);
	for(int k = 0; k < _dof; ++k){
		if (k == _dom) {
			// immobile = 1: _dom == _dof - 1
			qr(k + 1) = (float) _thetaimmobile;
		} else {
			qr(k + 1) = angles.at(k);
		}
	}
	_robot.set_q(qr);

	//Calculate kinematics
	Matrix TCP_Position = _robot.kine();
	TCP_Position(1,4) /= LENGTH_MULTIPLIER;
	TCP_Position(2,4) /= LENGTH_MULTIPLIER;
	TCP_Position(3,4) /= LENGTH_MULTIPLIER;

	// adjust TCP offset
	// transformation from flange (kinematics tcp) to effective tcp (with offset)
	Matrix TCP_Offset(4, 4);
	TCP_Offset.row(1) << 1.0 << 0.0 << 0.0 << _tcpOffset[0];
	TCP_Offset.row(2) << 0.0 << cos(_tcpOffset[3]) << -sin(_tcpOffset[3]) << _tcpOffset[1];
	TCP_Offset.row(3) << 0.0 << sin(_tcpOffset[3]) << cos(_tcpOffset[3]) << _tcpOffset[2];
	TCP_Offset.row(4) << 0.0 << 0.0 << 0.0 << 1.0;
	TCP_Position = TCP_Position * TCP_Offset;

	pose.clear();
	
	//x-Position
	pose.push_back(TCP_Position(1,4));
	//y-Position
	pose.push_back(TCP_Position(2,4));
	//z-Position
	pose.push_back(TCP_Position(3,4));

	//Calculate euler angles
	ColumnVector eul = ieulzxz(TCP_Position);

	//Phi
	pose.push_back(eul(1));
	//Theta
	pose.push_back(eul(2));
	//Psi
	pose.push_back(eul(3));

	return 1;
}

///// IK HELPER FUNCTIONS /////////////////////////////////////////////

// single inverse kinematics
int KinematicsLib::invKin(std::vector<double> pose, std::vector<double> prev,
		std::vector<double>& angle) {
	if ((int)pose.size() < 6 || (int)prev.size() < _dof)
		return -1;

	// IK algorithm type to use
	const int ikalgo = 0; // based on Jacobian (faster)
	//const int ikalgo = 1; // based on derivative of T (converge more often)

	//ReturnMatrix eulzxz(const ColumnVector & a);
	ColumnVector v(3);
	v(1) = pose.at(3);
	v(2) = pose.at(4);
	v(3) = pose.at(5);
	Matrix Pos = eulzxz(v);
	Pos(1,4) = pose.at(0) * LENGTH_MULTIPLIER;
	Pos(2,4) = pose.at(1) * LENGTH_MULTIPLIER;
	Pos(3,4) = pose.at(2) * LENGTH_MULTIPLIER;

	//Set previous angles
	ColumnVector qs = ColumnVector(_dof);
	for (int j = 0; j < _dof; ++j) {
		qs(j+1) = prev.at(j);
	}
	_robot.set_q(qs);

	bool converge = false;
	ColumnVector q0 = _robot.inv_kin(Pos, ikalgo, _dof, converge);

	angle.clear();
	for (int j = 0; j < _dom; ++j) {
		angle.push_back(q0(j + 1));
	}
	if (_immobile == 1)
		angle.push_back(_thetaimmobile);

	int ok = 1;
	if (!converge)
		ok = -1;
	return ok;
}

// inverse kinematics using bisection if no solution found
int KinematicsLib::invKin_bisec(std::vector<double> pose, std::vector<double> prev,
		std::vector<double>& conf, int maxBisection) {
	if ((int)pose.size() < 6 || (int)prev.size() < _dof || maxBisection < 0)
		return -1;

	int ok = invKin(pose, prev, conf);

	// Orig 3D							pose
	// Orig JS	prev					angle
	// Bisec 3D	prev1pose	prev2pose	pose
	// Bisec JS	prev		prev2conf	conf
	if (ok < 0 && maxBisection > 0) {
		// prev1pose
		std::vector<double> prev1pose;
		directKinematics(prev, prev1pose);

		// prev2pose
		std::vector<double> prev2pose;
		for (int i = 0; i < 6; ++i)
			prev2pose.push_back(prev1pose.at(i) + pose.at(i) / 2.0);

		// prev2conf (IK on first part)
		std::vector<double> prev2conf;
		ok = inverseKinematics(prev2pose, prev, prev2conf, maxBisection-1);

		if (ok == 1) {
			// conf (IK on second part, only if first part successful)
			ok = inverseKinematics(pose, prev2conf, conf, maxBisection-1);
		}
	}

	return ok;
}

// analytical guess
int KinematicsLib::anaGuess(std::vector<double> pose, std::vector<double> prev,
		std::vector<double>& angle) {
	if (_type < 0 || (int)pose.size() < 6 || (int)prev.size() < _dof)
		return -1;

	std::vector<double> positions;
	for (int i = 0; i < 6; ++i) {
		if (i < 3)
			positions.push_back(pose.at(i) * 1000);
		else
			positions.push_back(pose.at(i));
	}

	std::vector<double> previous_angles;
	std::vector<double> prevK4D;
	mDH2K4DAng(prev, prevK4D);
	for (int i = 0; i < _dom; ++i) {
		previous_angles.push_back(prevK4D.at(i));
	}
	if (_dom == 5) {
		// give 6 previous angles to the analytical kinematics
		previous_angles.push_back(0.0);
	}

	std::vector<double> anglesK4D;

	int error;
	try{
		error = ((int) _anaGuess->inverseKinematics(anglesK4D, positions,
			previous_angles)) - 1;
		K4D2mDHAng(anglesK4D, angle);
		if (_immobile)
			angle.push_back(_thetaimmobile);
	} catch (AnaGuess::NoSolutionException nse) {
		error = -2;
	} catch (AnaGuess::Exception e) {
		error = e.errorNumber() - 2;
	}

	if (error != 0) {
		angle.clear();
		for (int i = 0; i < _dof; ++i)
			angle.push_back(prev.at(i));
	}

	int ok = (error == 0) ? 1 : -1;
	return ok;
}

bool KinematicsLib::checkConfig(std::vector<double> config, std::vector<double> pose,
		double tol) {
	std::vector<double> configpose;
	directKinematics(config, configpose);
	double posdiff = 0.0;
	for (int i = 0; i < 6; ++i) {
		posdiff += abs(pose.at(i) - configpose.at(i));
	}
	bool ok;
	if (posdiff > tol)
		ok = false;
	else
		ok = true;
	return ok;
}

///// END IK HELPER FUNCTIONS /////////////////////////////////////////

///////////////////////////////////////////////////////////////////////
int KinematicsLib::inverseKinematics(std::vector<double> pose,
		std::vector<double> prev, std::vector<double>& angles, int maxBisection) {
	if (!_initialized || (int)pose.size() < 6 || (int)prev.size() < _dom ||
			maxBisection < 0)
		return -1;

	// tolerance for configuration check (sum of 3 * meter + 3 * radian)
	double tol = 0.0001;
	
	// adjust TCP offset
	// pose to matrix
	ColumnVector v(3);
	v(1) = pose.at(3);
	v(2) = pose.at(4);
	v(3) = pose.at(5);
	Matrix Pos = eulzxz(v);
	Pos(1,4) = pose.at(0);
	Pos(2,4) = pose.at(1);
	Pos(3,4) = pose.at(2);
	// transformation from effective tcp (with offset) to flange (kinematics tcp)
	Matrix TCP_Offset(4, 4);
	TCP_Offset.row(1) << 1.0 << 0.0 << 0.0 << -_tcpOffset[0];
	TCP_Offset.row(2) << 0.0 << cos(_tcpOffset[3]) << sin(_tcpOffset[3]) << (-_tcpOffset[1] * cos(_tcpOffset[3]) - _tcpOffset[2] * sin(_tcpOffset[3]));
	TCP_Offset.row(3) << 0.0 << -sin(_tcpOffset[3]) << cos(_tcpOffset[3]) << (_tcpOffset[1] * sin(_tcpOffset[3]) - _tcpOffset[2] * cos(_tcpOffset[3]));
	TCP_Offset.row(4) << 0.0 << 0.0 << 0.0 << 1.0;
	Pos = Pos * TCP_Offset;
	// matrix to pose
	std::vector<double> flangepose;
	flangepose.push_back(Pos(1,4));
	flangepose.push_back(Pos(2,4));
	flangepose.push_back(Pos(3,4));
	ColumnVector eul = ieulzxz(Pos);
	if (_type != K_6M180) {
		flangepose.push_back(eul(1));
	} else {
		// calculate phi from X and Y with K_6M180
		// own impl. of atan w/ 2 args for K4D compatibility --JHA
		double phi_calc = atan1(Pos(1,4),Pos(2,4))+mPi/2.0;
		if (Pos(1,4)*eul(1) < 0) {
			// X and given phi different sign -> tool point inwards
			phi_calc += mPi;
		}
		// bring phi in range [-mPi,mPi)
		phi_calc = fmod(phi_calc+mPi,2.0*mPi)-mPi;
		// bring phi in range (-mPi,mPi]
		if(phi_calc==-mPi) phi_calc+=2.0*mPi;
		// store calculated phi
		flangepose.push_back(phi_calc);
	}
	flangepose.push_back(eul(2));
	flangepose.push_back(eul(3));

	// Copy and complete prev
	std::vector<double> prev1conf;
	for(int i = 0; i < _dof; ++i){
		if (i == _dom) {
			// immobile = 1: _dom == _dof - 1
			prev1conf.push_back(_thetaimmobile);
		} else {
			prev1conf.push_back(prev.at(i));
		}
	}

	std::vector<double> conf;

	// calculate inverse kinematics (roboop)
	int ok = invKin_bisec(flangepose, prev1conf, conf, maxBisection);

	// on fail try turning phi about pi (switch inward / outward) with K_6M180
	if (ok<0 && _type == K_6M180) {
		if (flangepose[3] > 0)
			flangepose[3] -= mPi;
		else
			flangepose[3] += mPi;
		ok = invKin_bisec(flangepose, prev1conf, conf, maxBisection);
	}

	// if no solution found and type is 6M robot, get analytical guess
	if (ok < 0 && _type >= 0) {
		std::vector<double> guess;
		ok = anaGuess(flangepose, prev1conf, guess);

		// if analytical guess found, calculate inverse kinematics using guess
		if (ok == 1) {
			ok = invKin_bisec(flangepose, guess, conf, maxBisection);
			// check solution
			if (checkConfig(conf, pose, tol) == false) // use effective pose!
				ok = -1; // wrong solution
		}

		// if no guess or solution found, get analytical guess of near pose
		if (ok < 0) {
			std::vector<double> nearpose;
			nearpose.push_back(flangepose.at(0) - sign(flangepose.at(0)) * 0.05);
			nearpose.push_back(flangepose.at(1) - sign(flangepose.at(1)) * 0.05);
			nearpose.push_back(flangepose.at(2) - sign(flangepose.at(2)) * 0.05);
			nearpose.push_back(flangepose.at(3) - sign(flangepose.at(3)) * 0.2);
			nearpose.push_back(flangepose.at(4) - sign(flangepose.at(4) - mPi / 2.0) * 0.2);
			nearpose.push_back(flangepose.at(5) - sign(flangepose.at(5)) * 0.2);
			ok = anaGuess(nearpose, prev1conf, guess);
			// if analytical guess found, calculate inverse kinematics using guess
			if (ok == 1) {
				ok = invKin_bisec(flangepose, guess, conf, maxBisection);
				// check solution
				if (checkConfig(conf, pose, tol) == false)
					ok = -1; // wrong solution
			}
		}
	}

	// set angle if solution found
	angles.clear();
	if (ok == 1) {
		for (int i = 0; i < _dom; ++i)
			angles.push_back(conf.at(i));
	} else {
		// set prev if no solution found
		for (int i = 0; i < _dom; ++i)
			angles.push_back(prev.at(i));
	}

	return ok;
}





