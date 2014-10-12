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

		double eps = 1.0e-20;
		VectorXd q(5);
		Matrix3 Rb_0, R_30, R3_4, R7_e, R;

		R = Rp; //arm_path->joint(6)->calcRfromAttitude(Rp);
		
		//Rb_0 << 0.0, 1.0, 0.0, -1.0, 0.0, 0.0, 0.0, 0.0, 1.0;
		//R7_e = trans(Rb_0);
			
		//Matrix3 R_( (base->attitude()*Rb_0).transpose()*R*(R7_e).transpose() );
		//Vector3  p_( (base->attitude()*Rb_0).transpose()*(p - base->p) );
		
		double d_bs = 0.2015;
		double d_se = 0.19;
		//double a_ew = 0.0; //-0.0025;
		double d_ew = 0.139;
		double d_wt = 0.1883;//0.306335
		
		Vector3 l_bs(0.0, 0.0, d_bs);
		Vector3 l_se(0.0, 0.0, d_se);
		Vector3 l_ew(0.0, 0.0, d_ew);
		//Vector3 l_ww1(0.0, 0.0, d_ww1);
		//Vector3 l_w1w2(0.0, 0.0, d_w1w2);
		Vector3 l_wt(0.0, 0.0, d_wt);
		
		//Vector3 x_sw(p_ - l_bs - R_*l_wt);
		Vector3 x_sw(p - l_bs - R * l_wt);

		Vector3 b(R*p);

		q(0) = atan2(p(1), p(0));
		
		//double C1 = cos(q(0));
		//double S1 = sin(q(0));

		q(2) = acos((dot(x_sw, x_sw) - d_se*d_se - d_ew*d_ew)/(2.0*d_se*d_ew));
		if(q(2)<0.0)
				q(2)=0.0;
		double C3 = cos(q(2));
		double S3 = sin(q(2));
		/*
		double A = sqrt(x_sw(0)*x_sw(0)+x_sw(1)*x_sw(1));
		double B = x_sw(2);
		double M = d_se + C3*d_ew;
		double N = S3*d_ew;

		q(1) = atan2(M*A-N*B, N*A+M*B);
		*/

		/*Vector3 aa( S3*d_ew, d_se-C3*d_ew, 0);
		double tmp=sqrt(aa(0)*aa(0)+aa(1)*aa(1));
		double q20 = (PI/2) - atan2(aa(0)/tmp, aa(1)/tmp);
		q(1) = q20;
		*/

		Vector3 aa( S3*d_ew, -d_se-C3*d_ew, 0);
		
		double tmp=sqrt(aa(0)*aa(0)+aa(1)*aa(1));
		q(1) = acos(x_sw(2)/tmp) - atan2(aa(0)/tmp, -aa(1)/tmp);
		
		double acas = aa(0)*cos(q(1)) - aa(1)*sin(q(1)); // acas = sqrt(x_sw(0)*x_sw(0)+x_sw(1)*x_sw(1));
		tmp =  dbl(aa(2)) + dbl(acas);
		//q(0) = atan2( ( - aa(2)*x_sw(0) + acas*x_sw(1) )/tmp , ( acas*x_sw(0) + aa(2)*x_sw(1) )/tmp );
		
		double S1=sin(q(0));
		double C1=cos(q(0));
		double S2=sin(q(1));
		double C2=cos(q(1));

		double C23 = cos(q(1)+q(2));
		double S23 = sin(q(1)+q(2));

		q(3) = atan2(C23*(C1*p(0)+S1*p(1))-S23*p(2), S23*(C1*p(0)+S1*p(1))+C23*p(2));

		double C4 = cos(q(3));
		double S4 = sin(q(3));

		q(4) = atan2(b(1), C4*b(0)-S4*b(2));



		/*
		Vector3 aa( S3*d_ew, -d_se-C3*d_ew, a_ew);
		
		double tmp=sqrt(aa(0)*aa(0)+aa(1)*aa(1));
		double q20 = acos(x_sw(2)/tmp) - atan2(aa(0)/tmp, -aa(1)/tmp);
		
		double acas = aa(0)*cos(q20) - aa(1)*sin(q20);
		tmp =  dbl(aa(2)) + dbl(acas);
		double q10 = atan2( ( - aa(2)*x_sw(0) + acas*x_sw(1) )/tmp , ( acas*x_sw(0) + aa(2)*x_sw(1) )/tmp );
		
		double S10=sin(q10);
		double C10=cos(q10);
		double S20=sin(q20);
		double C20=cos(q20);
		
		R_30 << C10*C20, -C10*S20, -S10,  S10*C20, -S10*S20, C10,  -S20, -C20, 0.0;
		
		Vector3 u_sw(x_sw/norm2(x_sw));
		
		Matrix3 As(  sqew(u_sw)*R_30);
		Matrix3 Bs( -sqew(u_sw)*sqew(u_sw)*R_30 );
		Matrix3 Cs(  m33(u_sw)*R_30 );
		
		double Cp = cos(phi);
		double Sp = sin(phi);
		
		if(acos(-Bs(2,1)-Cs(2,1))*q20 > 0.0)
				q(1) = acos(   -As(2,1)*Sp - Bs(2,1)*Cp - Cs(2,1) );
		else
				q(1) = -acos(   -As(2,1)*Sp - Bs(2,1)*Cp - Cs(2,1) );
		
		double S2 = sin(q(1));
		
		if(fabs(S2) > eps){
				q(0) = atan2( (-As(1,1)*Sp - Bs(1,1)*Cp - Cs(1,1))/S2 , (-As(0,1)*Sp - Bs(0,1)*Cp - Cs(0,1) )/S2 );
				q(2) = atan2( ( As(2,2)*Sp + Bs(2,2)*Cp + Cs(2,2))/S2 , (-As(2,0)*Sp - Bs(2,0)*Cp - Cs(2,0) )/S2 );
		}
		else return false;
		
		R3_4 << C4, 0.0, S4, S4, 0.0, -C4, 0.0, 1.0, 0.0;
		
		Matrix3 Aw(trans(R3_4)*trans(As)*R_);
		Matrix3 Bw(trans(R3_4)*trans(Bs)*R_);
		Matrix3 Cw(trans(R3_4)*trans(Cs)*R_);
		
		double idx = 0.0, S6;
		for(int i=0; i<3; i++){
				
				int j=1-2*(i%2);
				
				q(5) = j*acos(   Aw(2,2)*Sp + Bw(2,2)*Cp + Cw(2,2) );
				
				S6=sin(q(5));
				
				if(fabs(S6) > eps){
						q(4) = atan2( (Aw(1,2)*Sp + Bw(1,2)*Cp + Cw(1,2))/S6 , ( Aw(0,2)*Sp + Bw(0,2)*Cp + Cw(0,2) )/S6 );
						q(6) = atan2( (Aw(2,1)*Sp + Bw(2,1)*Cp + Cw(2,1))/S6 , (-Aw(2,0)*Sp - Bw(2,0)*Cp - Cw(2,0) )/S6 );
						
						double eps=0.01;
						while( q(4)-q_old(4) >  2*m_pi-eps)	q(4) -= 2*m_pi;
						while( q(5)-q_old(5) >  2*m_pi-eps)	q(5) -= 2*m_pi;
						while( q(6)-q_old(6) >  2*m_pi-eps)	q(6) -= 2*m_pi;
						while( q(4)-q_old(4) < -2*m_pi+eps)	q(4) += 2*m_pi;
						while( q(5)-q_old(5) < -2*m_pi+eps)	q(5) += 2*m_pi;
						while( q(6)-q_old(6) < -2*m_pi+eps)	q(6) += 2*m_pi;

						if((q(4)-q_old(4))*(q(4)-q_old(4))+(q(5)-q_old(5))*(q(5)-q_old(5))+(q(6)-q_old(6))*(q(6)-q_old(6)) < idx)
						          break;
						
						idx = (q(4)-q_old(4))*(q(4)-q_old(4))+(q(5)-q_old(5))*(q(5)-q_old(5))+(q(6)-q_old(6))*(q(6)-q_old(6));
				}
		}

		if(fabs(S6)<eps)
				return false;
		*/

		for(int l=0; l<5; l++) arm_path->joint(l)->q = q(l);

		//adjustPA10Wrist();

		arm_path->calcForwardKinematics();

cout << "1:" << q(0) << endl;cout << "2:" << q(1) << endl;cout << "3:" << q(2) << endl;cout << "4:" << q(3) << endl;cout << "5:" << q(4) << endl;
		return true;
}

extern "C" void* getGrasplotArm(cnoid::BodyPtr body, cnoid::Link *base, cnoid::Link *palm) 
{                                    
    return new Katana_Arm(body, base, palm);      
}
