#include "GrasplotPluginKatana.h"

using namespace std;
using namespace cnoid;
using namespace grasp;


bool Katana_Arm::IK_arm(const Vector3& p, const Matrix3& R){
	double phi=0;
	VectorXd q_old(nJoints);
	for(int i=0;i<nJoints;i++){
		q_old[i] = armStandardPose[i];
	}
	return IK_arm( p,  R, phi,  q_old);
}


bool  Katana_Arm::IK_arm(const Vector3& p, const Matrix3& Rp, double phi, const VectorXd& q_old){
cout << "WQ" << endl;
		double eps = 1.0e-20, k = 0;
		VectorXd q(5);
		Vector3 R0, R, X;
		X << 1.0, 1.0, 1.0;

		R0 = Rp*X;
		k = norm2(R0);
		R = -(Rp*X)/k;

cout << "Rp:" << Rp << endl;cout << "R:" << R << endl;		
	
		double d_bs = 0.2015;
		double d_se = 0.19;
		double a_ew = 0.0; //-0.0025;
		double d_ew = 0.139;
		double d_wt = 0.2061;//0.1883;//0.306335
		double d_tf = 0.09;
		
		Vector3 l_bs(0.0, 0.0, d_bs);
		Vector3 l_se(0.0, 0.0, d_se);
		Vector3 l_ew(0.0, 0.0, d_ew);
		Vector3 l_wt(0.0, 0.0, d_wt);
		Vector3 l_tf(0.0, 0.0, d_tf);
		

		q(0) = atan((d_tf*R(1)+p(1))/(d_tf*R(0)+p(0)));

		double C0 = cos(q(0));
		double S0 = sin(q(0));

		double A = ((R(0)+p(0))/C0) - d_wt*cos(atan((R(0)*C0+R(1)*S0)/R(2)));
		double B = d_bs - R(2) - p(2) - d_wt*sin(atan((R(0)*C0+R(1)*S0)/R(2)));

		//q(1) = atan(B/A) - acos(((d_se*d_se)-(d_ew*d_ew)+((A*A)+(B*B)))/(2*d_se*sqrt((A*A)+(B*B))));
q(1) = atan(B/A) + acos(((d_se*d_se)-(d_ew*d_ew)+((A*A)+(B*B)))/(2*d_se*sqrt((A*A)+(B*B))));
		double C1 = cos(q(1));
		double S1 = sin(q(1));

		//q(2) = m_pi - acos((d_se*d_se+d_ew*d_ew-(A*A+B*B))/(2*d_se*d_ew));
q(2) = -m_pi + acos((d_se*d_se+d_ew*d_ew-(A*A+B*B))/(2*d_se*d_ew));
		double C2 = cos(q(2));
		double S2 = sin(q(2));
		double C12 = cos(q(1)+q(2));
		double S12 = sin(q(1)+q(2));

		q(3) = asin((d_bs-d_ew*S1-d_wt*S12-p(2))/d_tf)-q(1)-q(2);

		for(int l=0; l<5; l++) arm_path->joint(l)->q = q(l);

		//adjustPA10Wrist();

		arm_path->calcForwardKinematics();

cout << "1:" << q(0) << endl;cout << "2:" << q(1) << endl;cout << "3:" << q(2) << endl;cout << "4:" << q(3) << endl;cout << "5:" << q(4) << endl;
		return true;
}

/*bool  Katana_Arm::IK_arm(const Vector3& p, const Matrix3& Rp, double phi, const VectorXd& q_old){
cout << "WQ" << endl;
		double eps = 1.0e-20, k = 0;
		VectorXd q(5);
		Vector3 R0, R, X;
		X << 1.0, 1.0, 1.0;

		R0 = Rp*X;
		k = norm2(R0);
		R = -(Rp*X)/k;

cout << "Rp:" << Rp << endl;cout << "R:" << R << endl;		
	
		double d_bs = 0.2015;
		double d_se = 0.19;
		double a_ew = 0.0; //-0.0025;
		double d_ew = 0.139;
		double d_wt = 0.2061;//0.1883;//0.306335
		double d_tf = 0.09;
		
		Vector3 l_bs(0.0, 0.0, d_bs);
		Vector3 l_se(0.0, 0.0, d_se);
		Vector3 l_ew(0.0, 0.0, d_ew);
		Vector3 l_wt(0.0, 0.0, d_wt);
		Vector3 l_tf(0.0, 0.0, d_tf);
		

		q(0) = atan((d_tf*R(1)+p(1))/(d_tf*R(0)+p(0)));

		double C0 = cos(q(0));
		double S0 = sin(q(0));

		double A = ((R(0)+p(0))/C0) - d_wt*cos(atan((R(0)*C0+R(1)*S0)/R(2)));
		double B = d_bs - R(2) - p(2) - d_wt*sin(atan((R(0)*C0+R(1)*S0)/R(2)));

		//q(1) = atan(B/A) - acos(((d_se*d_se)-(d_ew*d_ew)+((A*A)+(B*B)))/(2*d_se*sqrt((A*A)+(B*B))));
q(1) = atan(B/A) + acos(((d_se*d_se)-(d_ew*d_ew)+((A*A)+(B*B)))/(2*d_se*sqrt((A*A)+(B*B))));
		double C1 = cos(q(1));
		double S1 = sin(q(1));

		//q(2) = m_pi - acos((d_se*d_se+d_ew*d_ew-(A*A+B*B))/(2*d_se*d_ew));
q(2) = -m_pi + acos((d_se*d_se+d_ew*d_ew-(A*A+B*B))/(2*d_se*d_ew));
		double C2 = cos(q(2));
		double S2 = sin(q(2));
		double C12 = cos(q(1)+q(2));
		double S12 = sin(q(1)+q(2));

		q(3) = asin((d_bs-d_ew*S1-d_wt*S12-p(2))/d_tf)-q(1)-q(2);

		for(int l=0; l<5; l++) arm_path->joint(l)->q = q(l);

		//adjustPA10Wrist();

		arm_path->calcForwardKinematics();

cout << "1:" << q(0) << endl;cout << "2:" << q(1) << endl;cout << "3:" << q(2) << endl;cout << "4:" << q(3) << endl;cout << "5:" << q(4) << endl;
		return true;
}*/

extern "C" void* getGrasplotArm(cnoid::BodyPtr body, cnoid::Link *base, cnoid::Link *palm) 
{                                    
    return new Katana_Arm(body, base, palm);      
}
