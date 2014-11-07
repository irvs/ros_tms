#include "GrasplotPluginKKP.h"
#include "../../../Grasp/PlanBase.h"

using namespace std;
using namespace cnoid;
using namespace grasp;

#define rad2deg(x)	((x)*180.0/M_PI)
bool KKP_Arm::IK_arm(const Vector3& p, const Matrix3& R){
	double phi=0;
	VectorXd q_old(nJoints);
	for(int i=0;i<nJoints;i++){
		q_old[i] = armStandardPose[i];
	}
	return IK_arm( p,  R, phi,  q_old);
}


bool  KKP_Arm::IK_arm(const Vector3& p, const Matrix3& Rp, double phi, const VectorXd& q_old){
	
		VectorXd q(5); // KXPの各関節角

		double L0 = 0.2035; // m
		double L1 = 0.190;
		double L2 = 0.139;
		double L3 = 0.2688;
		double L = p(0)/p(1);
		
		double Length = 0.0;
		Length = sqrt((p(0) * p(0))+(p(1) * p(1)));
		printf("Length=%.2fm", Length);

		double tmp_for_calc;
		
		// theta0
		q(0) = asin(p(1) / Length);
	//	q(0) = atan2(p(0), p(1)) - M_PI/2;
		if (q(0) <= -3.044 || 2.880 <= q(0)) return false;
		printf("J1=%.2fdeg", rad2deg(q(0)));

		// theta2
		tmp_for_calc = (L1*L1)+(L2*L2)-((Length-L3)*(Length-L3))-((p(2)-L0)*(p(2)-L0));
		tmp_for_calc = tmp_for_calc/(2*L1*L2);
		double temp_q2 = acos(tmp_for_calc);
		q(2) = M_PI - temp_q2;
		if (q(2) <= -2.054 || 2.217 <= q(2)) return false;
		
		// theta1
		double cos_th2 = cos(q(2));
		double sin_th2 = sin(q(2));
		double varA = L2*sin_th2;
		double varB = L2*cos_th2 + L1;

		double numerator = varB*(Length-L3) - varA*(p(2)-L0); // 分子
		double denominator = varB*varB + varA*varA;

	//	tmp_for_calc = (1+cos(q(2)))*(Length-L3-((p(2)*sin(q(2)))/(1+cos(q(2)))));
		tmp_for_calc = numerator/denominator;
		q(1) = asin(tmp_for_calc);
		if (q(1) <= -0.593 || 1.705 <= q(1)) return false;
		printf("J2=%.2fdeg", rad2deg(q(1)));
		printf("J3=%.2fdeg", rad2deg(q(2)));
		
		// theta3
		q(3) = M_PI/2 - q(2) - q(1);
		if (q(3) <= -2.033 || 1.868 <= q(3)) return false;
		printf("J4=%.2fdeg", rad2deg(q(3)));
		
		// theta4
		q(4) = 0.0;
		if(isnan(q(0))||isnan(q(1))||isnan(q(2))||isnan(q(3))||isnan(q(4))) return false;

// set joint		
		for(int l=0; l<5; l++) arm_path->joint(l)->q() = q(l);

		arm_path->calcForwardKinematics();
		PlanBase* tc = PlanBase::instance();
		//cout<<"Rp:"<<endl<<Rp<<endl<<rpyFromRot(Rp)<<endl<<"R_:"<<endl<<R_<<endl<<rpyFromRot(R_)<<endl<<"R4:"<<endl<<arm_path->joint(4)->R<<endl<<rpyFromRot(arm_path->joint(4)->attitude())<<endl;
		tc->flush();
		return true;
}

extern "C" void* getGrasplotArm(cnoid::BodyPtr body, cnoid::Link *base, cnoid::Link *palm) 
{                                    
    return new KKP_Arm(body, base, palm);      
}
