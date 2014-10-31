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


bool  Katana_Arm::IK_arm(const Vector3& p, const Matrix3& R0, double phi, const VectorXd& q_old){
	
	static const int MAX_IK_ITERATION =100;
	static const double LAMBDA = 0.9;

	double maxIkErrorSqr = 1.0e-6 * 1.0e-6;

	const int n = arm_path->numJoints();

	for(unsigned int i=0;i<armStandardPose.size();i++)
		arm_path->joint(i)->q	= armStandardPose[i];	

	arm_path->calcForwardKinematics();
	
	Matrix3 R(R0 );

	vector<double> qorg(n);
	for (int i = 0; i < n; ++i)
		qorg[i] = arm_path->joint(i)->q;

	MatrixXd J(6, n);
	VectorXd dq(n);
	VectorXd v(6);

	bool isConverged = false;

	for (int i = 0; i < MAX_IK_ITERATION; i++) {
		
		arm_path->calcJacobian(J);

		Vector3 dp(p - palm->p);
		Vector3 omega(palm->R * omegaFromRot( (palm->R).transpose()* R));

		double errsqr = dot(dp, dp) + dot(omega, omega);
		

		if (errsqr < maxIkErrorSqr) {
			isConverged = true;
			break;
		}
		setVector3(dp   , v, 0);
		setVector3(omega, v, 3);
		
		MatrixXd invJ;
		calcPseudoInverse(J, invJ);
#if 0

		dq = invJ * v + ( MatrixXd::Identity(n, n) - invJ * J) * calcGradient(0.0, 1.0) ;
		dq = LAMBDA*dq;
#ifdef DEBUG_MODE
		cout << "ik" << errsqr << " "<<dq.transpose() << endl;
#endif		
#else
		//sugihara method
		MatrixXd H(n,n);
		H = J.transpose()*MatrixXd::Identity(n,n)*J + (errsqr*0.1 + 0.001)*MatrixXd::Identity(n,n);
		MatrixXd invH = H.inverse();
		MatrixXd invJ2 = invH*J.transpose();

//		dq = invJ2*v + (errsqr*0.1 + 0.001)*( MatrixXd::Identity(n, n) - invJ * J) * calcGradient(0.0, 1.0);
		dq = invJ2*v + ( MatrixXd::Identity(n, n) - invJ * J) * calcGradient(0.0, 1.0);

#endif

	for (int j = 0; j < n; ++j)
			arm_path->joint(j)->q +=  dq(j);

		arm_path->calcForwardKinematics();
		
//		PlanBase::instance()->flush();
//		usleep(100000);
		
	}
	

	if (!isConverged) {
		for (int i = 0; i < n; ++i) {
			arm_path->joint(i)->q = qorg[i];
		}
		arm_path->calcInverseKinematics(p,R0);
		arm_path->calcForwardKinematics();
	}else{
		for(int i=0; i<n;i++){
			qorg[i] = arm_path->joint(i)->q;
		}
	}

	//arm_path->joint(5)->q = 0;

	
	return isConverged;
}

extern "C" void* getGrasplotArm(cnoid::BodyPtr body, cnoid::Link *base, cnoid::Link *palm) 
{                                    
    return new Katana_Arm(body, base, palm);      
}
