/* --------------------------------------------------------------------------------
 #
 #	libKinematics.cpp
 #  Project : Kinematics
 #	author : jhaller
 #	24.04.2008
 #
 # --------------------------------------------------------------------------------*/


#include "libKinematics.h"


extern "C"{

KinematicsLib* _kinematics;
bool LibInstantiated = false;

///// SETTERS /////////////////////////////////////////////////////////

int kin_setType(int type) {
	if (LibInstantiated == true) delete _kinematics;
	_kinematics = new KinematicsLib(type);
	LibInstantiated = true;
	return 0;
}

int kin_setMDH(FloatVector* theta, FloatVector* d, FloatVector* a,
		FloatVector* alpha, int typeNr) {
	if (LibInstantiated == true) delete _kinematics;
	_kinematics = new KinematicsLib();
	LibInstantiated = true;
	std::vector<double> thetaw, dw, aw, alphaw;
	for (int i = 0; i < theta->length; ++i) {
		thetaw.push_back(theta->data[i]);
		dw.push_back(d->data[i]);
		aw.push_back(a->data[i]);
		alphaw.push_back(alpha->data[i]);
	}
	int ok = _kinematics->setMDH(thetaw, dw, aw, alphaw, typeNr);
	int error = (ok < 0) ? -1 : 0;
	return error;
}

int kin_setLinkLen(FloatVector* links) {
	if (!LibInstantiated)
		return -1;
	std::vector<double> fw;
	for (int i = 0; i < links->length; ++i) {
		fw.push_back(links->data[i]);
	}
	int ok = _kinematics->setLinkLen(fw);
	int error = (ok < 0) ? -1 : 0;
	return error;
}

int kin_setImmob(int immobile) {
	if (!LibInstantiated)
		return -1;
	int ok = _kinematics->setImmob(immobile);
	int error = (ok < 0) ? -1 : 0;
	return error;
}

int kin_setEPC(IntVector* epc) {
	if (!LibInstantiated)
		return -1;
	std::vector<int> iw;
	for (int i = 0; i < epc->length; ++i) {
		iw.push_back(epc->data[i]);
	}
	int ok = _kinematics->setEPC(iw);
	int error = (ok < 0) ? -1 : 0;
	return error;
}

int kin_setEncOff(IntVector* encOffset) {
	if (!LibInstantiated)
		return -1;
	std::vector<int> iw;
	for (int i = 0; i < encOffset->length; ++i) {
		iw.push_back(encOffset->data[i]);
	}
	int ok = _kinematics->setEncOff(iw);
	int error = (ok < 0) ? -1 : 0;
	return error;
}

int kin_setRotDir(IntVector* rotDir) {
	if (!LibInstantiated)
		return -1;
	std::vector<int> iw;
	for (int i = 0; i < rotDir->length; ++i) {
		iw.push_back(rotDir->data[i]);
	}
	int ok = _kinematics->setRotDir(iw);
	int error = (ok < 0) ? -1 : 0;
	return error;
}

int kin_setAngOff(FloatVector* angleOffset) {
	if (!LibInstantiated)
		return -1;
	std::vector<double> fw;
	for (int i = 0; i < angleOffset->length; ++i) {
		fw.push_back(angleOffset->data[i]);
	}
	int ok = _kinematics->setAngOff(fw);
	int error = (ok < 0) ? -1 : 0;
	return error;
}

int kin_setAngRan(FloatVector* angleRange) {
	if (!LibInstantiated)
		return -1;
	// check vector size
	std::vector<double> fw;
	for (int i = 0; i < angleRange->length; ++i) {
		fw.push_back(angleRange->data[i]);
	}
	int ok = _kinematics->setAngRan(fw);
	int error = (ok < 0) ? -1 : 0;
	return error;
}

int kin_setTcpOff(FloatVector* tcpOffset) {
	if (!LibInstantiated)
		return -1;
	std::vector<double> fw;
	for (int i = 0; i < tcpOffset->length; ++i) {
		fw.push_back(tcpOffset->data[i]);
	}
	int ok = _kinematics->setTcpOff(fw);
	int error = (ok < 0) ? -1 : 0;
	return error;
}

///// GETTERS /////////////////////////////////////////////////////////

int kin_getType() {
	if (!LibInstantiated)
		return -1;
	return _kinematics->getType();
}

int kin_getMaxDOF() {
	if (!LibInstantiated)
		return -1;
	return _kinematics->getMaxDOF();
}

int kin_getDOF() {
	if (!LibInstantiated)
		return -1;
	return _kinematics->getDOF();
}

int kin_getDOM() {
	if (!LibInstantiated)
		return -1;
	return _kinematics->getDOM();
}

int kin_getMDH(FloatVector* theta, FloatVector* d,
		FloatVector* a, FloatVector* alpha) {
	if (!LibInstantiated)
		return -1;
	std::vector<double> thetaw, dw, aw, alphaw;
	int ok = _kinematics->getMDH(thetaw, dw, aw, alphaw);
	for (int i = 0; i < (int)thetaw.size(); ++i) {
		theta->data[i] = (float)thetaw.at(i);
		d->data[i] = (float)dw.at(i);
		a->data[i] = (float)aw.at(i);
		alpha->data[i] = (float)alphaw.at(i);
	}
	theta->length = (int)thetaw.size();
	d->length = (int)thetaw.size();
	a->length = (int)thetaw.size();
	alpha->length = (int)thetaw.size();
	int error = (ok < 0) ? -1 : 0;
	return error;
}

int kin_getImmob() {
	if (!LibInstantiated)
		return -1;
	return _kinematics->getImmob();
}

int kin_getEPC(IntVector* epc) {
	if (!LibInstantiated)
		return -1;
	std::vector<int> iw;
	int ok = _kinematics->getEPC(iw);
	for (int i = 0; i < (int)iw.size(); ++i) {
		epc->data[i] = iw.at(i);
	}
	epc->length = (int)iw.size();
	if ((int)iw.size() == 5) {
		epc->data[5] = 51200;
		epc->length++;
	}
	int error = (ok < 0) ? -1 : 0;
	return error;
}

int kin_getEncOff(IntVector* encOffset) {
	if (!LibInstantiated)
		return -1;
	std::vector<int> iw;
	int ok = _kinematics->getEncOff(iw);
	for (int i = 0; i < (int)iw.size(); ++i) {
		encOffset->data[i] = iw.at(i);
	}
	encOffset->length = iw.size();
	if ((int)iw.size() == 5) {
		encOffset->data[5] = 31000;
		encOffset->length++;
	}
	int error = (ok < 0) ? -1 : 0;
	return error;
}

int kin_getRotDir(IntVector* rotDir) {
	if (!LibInstantiated)
		return -1;
	std::vector<int> iw;
	int ok = _kinematics->getRotDir(iw);
	for (int i = 0; i < (int)iw.size(); ++i) {
		rotDir->data[i] = iw.at(i);
	}
	rotDir->length = iw.size();
	if ((int)iw.size() == 5) {
		rotDir->data[5] = 1;
		rotDir->length++;
	}
	int error = (ok < 0) ? -1 : 0;
	return error;
}

int kin_getAngOff(FloatVector* angleOffset) {
	if (!LibInstantiated)
		return -1;
	std::vector<double> fw;
	int ok = _kinematics->getAngOff(fw);
	for (int i = 0; i < (int)fw.size(); ++i) {
		angleOffset->data[i] = (float)fw.at(i);
	}
	angleOffset->length = fw.size();
	if ((int)fw.size() == 5) {
		angleOffset->data[5] = 0.0;
		angleOffset->length++;
	}
	int error = (ok < 0) ? -1 : 0;
	return error;
}

int kin_getAngRan(FloatVector* angleRange) {
	if (!LibInstantiated)
		return -1;
	std::vector<double> fw;
	int ok = _kinematics->getAngRan(fw);
	for (int i = 0; i < (int)fw.size(); ++i) {
		angleRange->data[i] = (float)fw.at(i);
	}
	angleRange->length = fw.size();
	if ((int)fw.size() == 5) {
		angleRange->data[5] = 0.0;
		angleRange->length++;
	}
	int error = (ok < 0) ? -1 : 0;
	return error;
}

int kin_getTcpOff(FloatVector* tcpOffset) {
	if (!LibInstantiated)
		return -1;
	std::vector<double> fw;
	int ok = _kinematics->getTcpOff(fw);
	for (int i = 0; i < (int)fw.size(); ++i) {
		tcpOffset->data[i] = (float)fw.at(i);
	}
	tcpOffset->length = fw.size();
	int error = (ok < 0) ? -1 : 0;
	return error;
}

int kin_getVersion(IntVector* version) {
	int error;
	if (LibInstantiated) {
		std::vector<int> iw;
		int ok = _kinematics->getVersion(iw);
		for (int i = 0; i < (int)iw.size(); ++i) {
			version->data[i] = iw.at(i);
		}
		version->length = iw.size();
		error = (ok < 0) ? -1 : 0;
	} else {
		version->data[0] = KINLIB_VERSION_MAJOR;
		version->data[1] = KINLIB_VERSION_MINOR;
		version->data[2] = KINLIB_VERSION_REVISION;
		version->length = 3;
		error = 0;
	}
	return error;
}

///// INITIALIZATION //////////////////////////////////////////////////

int kin_init() {
	if (!LibInstantiated)
		return -1;
	int ok = _kinematics->init();
	int error = (ok < 0) ? -1 : 0;
	return error;
}

///// CLEANUP /////////////////////////////////////////////////////////

int kin_clean() {
	if (LibInstantiated == true)
		delete _kinematics;
	LibInstantiated = false;
	return 0;
}

///// CONVERSION AND KINEMATICS ///////////////////////////////////////

int kin_K4D2mDHAng(FloatVector* angleK4D, FloatVector* angleMDH) {
	if (!LibInstantiated)
		return -1;
	std::vector<double> k4d, mdh;
	for (int i = 0; i < angleK4D->length; ++i) {
		k4d.push_back(angleK4D->data[i]);
	}
	int ok = _kinematics->K4D2mDHAng(k4d, mdh);
	for (int i = 0; i < (int)mdh.size(); ++i) {
		angleMDH->data[i] = (float)mdh.at(i);
	}
	angleMDH->length = mdh.size();
	int error = (ok < 0) ? -1 : 0;
	return error;
}

int kin_mDH2K4DAng(FloatVector* angleMDH, FloatVector* angleK4D) {
	if (!LibInstantiated)
		return -1;
	std::vector<double> mdh, k4d;
	for (int i = 0; i < angleMDH->length; ++i) {
		mdh.push_back(angleMDH->data[i]);
	}
	int ok = _kinematics->mDH2K4DAng(mdh, k4d);
	for (int i = 0; i < (int)k4d.size(); ++i) {
		angleK4D->data[i] = (float)k4d.at(i);
	}
	angleK4D->length = k4d.size();
	int error = (ok < 0) ? -1 : 0;
	return error;
}

int kin_enc2rad(IntVector* enc, FloatVector* angle) {
	if (!LibInstantiated)
		return -1;
	std::vector<int> iw;
	for (int i = 0; i < enc->length; ++i) {
		iw.push_back(enc->data[i]);
	}
	std::vector<double> fw;
	int ok = _kinematics->enc2rad(iw, fw);
	for (int i = 0; i < (int)fw.size(); ++i) {
		angle->data[i] = (float)fw.at(i);
	}
	angle->length = fw.size();
	int error = (ok < 0) ? -1 : 0;
	return error;
}

int kin_rad2enc(FloatVector* angle, IntVector* enc) {
	if (!LibInstantiated)
		return -1;
	std::vector<double> fw;
	for (int i = 0; i < angle->length; ++i) {
		fw.push_back(angle->data[i]);
	}
	std::vector<int> iw;
	int ok = _kinematics->rad2enc(fw, iw);
	for (int i = 0; i < (int)iw.size(); ++i) {
		enc->data[i] = iw.at(i);
	}
	enc->length = iw.size();
	int error = (ok < 0) ? -1 : 0;
	return error;
}

int kin_DK(FloatVector* angle, FloatVector* pose) {
	if (!LibInstantiated)
		return -1;
	std::vector<double> aw, pw;
	for (int i = 0; i < angle->length; ++i) {
		aw.push_back(angle->data[i]);
	}
	int ok = _kinematics->directKinematics(aw, pw);
	for (int i = 0; i < (int)pw.size(); ++i) {
		pose->data[i] = (float)pw.at(i);
	}
	pose->length = pw.size();
	int error = (ok < 0) ? -1 : 0;
	return error;
}

int kin_IK(FloatVector* pose, FloatVector* prev, FloatVector* angle,
		int maxBisection) {
	if (!LibInstantiated)
		return -1;
	std::vector<double> pw;
	for (int i = 0; i < pose->length; ++i) {
		pw.push_back(pose->data[i]);
	}
	std::vector<double> paw;
	for (int i = 0; i < prev->length; ++i) {
		paw.push_back(prev->data[i]);
	}
	std::vector<double> aw;
	int ok = _kinematics->inverseKinematics(pw, paw, aw, maxBisection);
	for (int i = 0; i < (int)aw.size(); ++i) {
		angle->data[i] = (float)aw.at(i);
	}
	angle->length = aw.size();
	int error = (ok < 0) ? -1 : 0;
	return error;
}

///////////////////////////////////////////////////////////////////////
} // end extern "C"



