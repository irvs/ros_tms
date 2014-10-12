#include "GrasplotPluginKatana.h"
#include <math.h>
#include "../../../Grasp/PlanBase.h"

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

		double eps = 1.0e-20;
		VectorXd q(5);
		Matrix3 Rb_0, R_30, R3_4, R7_e, R;

		R = arm_path->joint(4)->calcRfromAttitude(Rp);
		
		Rb_0 << 0.0, 1.0, 0.0, -1.0, 0.0, 0.0, 0.0, 0.0, 1.0;
		R7_e = trans(Rb_0);
			
		//Matrix3 R_( (base->attitude()*Rb_0).transpose()*R*(R7_e).transpose() );
		//Vector3  p_( (base->attitude()*Rb_0).transpose()*(p - base->p) );
		Vector3 p_(p - base->p);
		
		double d_bs = 0.2105;
		double d_se = 0.190;
		double d_ew = 0.139;
		double d_ww1 = 0.12;
		double d_w1w2 = 0.1;
		double d_w2t = 0.09;
		double d_wt = 0.32;
		
		Vector3 l_bs(0.0, 0.0, d_bs);
		Vector3 l_se(0.0, 0.0, d_se);
		Vector3 l_ew(0.0, 0.0, d_ew);
		Vector3 l_ww1(0.0, 0.0, d_ww1);
		Vector3 l_w1w2(0.0, 0.0, d_w1w2);
		Vector3 l_wt(0.0, 0.0, d_wt);
		
		Vector3 x_sw(p_ - l_bs - R*l_wt);
		Vector3 x_bw(p_ - R*l_wt);
		//Vector3 x_sw(p - R * l_wt);

		q(0) = atan2(p_(1), p_(0));
		if(isnan(q(0))) return false;
		double C1 = cos(q(0));
		double S1 = sin(q(0));

		double C3 = (dot(x_sw, x_sw) - d_se*d_se - d_ew*d_ew)/(2.0*d_se*d_ew);
		double S3 = sqrt(1-C3*C3);
		
		q(2) = atan2(S3,C3);
		if(isnan(q(2))) return false;		
				
		double A = sqrt(x_bw(0)*x_bw(0)+x_bw(1)*x_bw(1));
		double B = x_bw(2) - d_bs;
		double M = d_se + C3*d_ew;
		double N = S3*d_ew;

		q(1) = atan2(M*A-N*B, N*A+M*B);
		if(isnan(q(1))) return false;
		
		double C2 = cos(q(1));
		double S2 = sin(q(1));
		
		Vector3 x_be(C1*S2*d_se,S1*S2*d_se,d_bs+C2*d_se);
		Vector3 x_eo(p_-x_be);
		
		//double C4 = (dot(x_eo, x_eo) - d_ew*d_ew - dot(p_-x_bw,p_-x_bw))/(2.0*d_ew*d_wt);
		double C4 = (dot(x_eo, x_eo) - d_ew*d_ew - d_wt*d_wt)/(2.0*d_ew*d_wt);
		double S4 = -sqrt(1-C4*C4);
		
		q(3) = atan2(S4,C4);
		if(isnan(q(3))) return false;
		
		Vector3 t = rpyFromRot(Rp);
		q(4) = t(2);		
		if(isnan(q(4))) return false;

		for(int l=0; l<5; l++) arm_path->joint(l)->q = q(l);
cout << "1:" << q(0) << endl;cout << "2:" << q(1) << endl;cout << "3:" << q(2) << endl;cout << "4:" << q(3) << endl;cout << "5:" << q(4) << endl;
		
		//adjustPA10Wrist();

		arm_path->calcForwardKinematics();
		PlanBase* tc = PlanBase::instance();
tc->flush();
		return true;
}

/*bool  Katana_Arm::IK_arm(const Vector3& p, const Matrix3& Rp, double phi, const VectorXd& q_old){

		double eps = 1.0e-20;
		VectorXd q(5);
		Matrix3 Rb_0, R_30, R3_4, R7_e, R;

		R = arm_path->joint(4)->calcRfromAttitude(Rp);
		
		Rb_0 << 0.0, 1.0, 0.0, -1.0, 0.0, 0.0, 0.0, 0.0, 1.0;
		R7_e = trans(Rb_0);
			
		//Matrix3 R_( (base->attitude()*Rb_0).transpose()*R*(R7_e).transpose() );
		//Vector3  p_( (base->attitude()*Rb_0).transpose()*(p - base->p) );
		Vector3 p_(p - base->p);
		
		double d_bs = 0.21;
		double d_se = 0.192;
		double a_ew = 0.0; //-0.0025;
		double d_ew = 0.14;
		double d_ww1 = 0.12; //0.48->PA10_VVV
		double d_w1w2 = 0.1;
		double d_w2t = 0.09;
		double d_wt = 0.31;
		
		Vector3 l_bs(0.0, 0.0, d_bs);
		Vector3 l_se(0.0, 0.0, d_se);
		Vector3 l_ew(0.0, 0.0, d_ew);
		Vector3 l_ww1(0.0, 0.0, d_ww1);
		Vector3 l_w1w2(0.0, 0.0, d_w1w2);
		Vector3 l_wt(0.0, 0.0, d_wt);
		
		Vector3 x_sw(p_ - l_bs - R*l_wt);
		//Vector3 x_sw(p - R * l_wt);

		Vector3 b(R*p);
cout<<p_(1)<<" "<<p_(0)<<endl;
		q(0) = atan2(p_(1), p_(0));
cout<<q(0)<<endl;
		double C1 = cos(q(0));
		double S1 = sin(q(0));

		q(2) = acos((dot(x_sw, x_sw) - d_se*d_se - d_ew*d_ew)/(2.0*d_se*d_ew));
		
		double C3 = cos(q(2));
		double S3 = sin(q(2));
		
		double A = sqrt(x_sw(0)*x_sw(0)+x_sw(1)*x_sw(1));
		double B = x_sw(2) - d_bs;
		double M = d_se + C3*d_ew;
		double N = S3*d_ew;

		q(1) = atan2(M*A-N*B, M*A+N*B);

		double C23 = cos(q(1)+q(2));
		double S23 = sin(q(1)+q(2));

		q(3) = atan2(C23*(C1*x_sw(0)+S1*x_sw(1))-S23*x_sw(2), S23*(C1*x_sw(0)+S1*x_sw(1))+C23*x_sw(2));

		double C4 = cos(q(3));
		double S4 = sin(q(3));

		q(4) = atan2(b(1), C4*b(0)-S4*b(2));


		for(int l=0; l<5; l++) arm_path->joint(l)->q = q(l);
cout << "1:" << q(0) << endl;cout << "2:" << q(1) << endl;cout << "3:" << q(2) << endl;cout << "4:" << q(3) << endl;cout << "5:" << q(4) << endl;
		
		//adjustPA10Wrist();

		arm_path->calcForwardKinematics();

		return true;
}*/

extern "C" void* getGrasplotArm(cnoid::BodyPtr body, cnoid::Link *base, cnoid::Link *palm) 
{                                    
    return new Katana_Arm(body, base, palm);      
}
