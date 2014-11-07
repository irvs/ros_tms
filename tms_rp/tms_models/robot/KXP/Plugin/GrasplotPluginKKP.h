#ifndef GRASPLOTPLUGIN_KKP_H
#define GRASPLOTPLUGIN_KKP_H

#include <iostream>
#include <cnoid/JointPath>
//#include <glibmm/i18n.h>
#include <cnoid/ItemManager>
#include <cnoid/BodyMotionItem>
#include <extplugin/graspPlugin/Grasp/Arm.h>
#include <extplugin/graspPlugin/Grasp/VectorMath.h>

#include <iostream>
#include <string>
#include <stdlib.h>
#include <math.h>

#define m_pi 3.141592

namespace grasp{

class KKP_Arm: public Arm
{
	public:
	
		KKP_Arm(cnoid::BodyPtr body, cnoid::Link *base, cnoid::Link *palm) : Arm(body, base, palm) {
			this->base = base;
			};
		~KKP_Arm() {	};
//		virtual bool IK_arm(const cnoid::Vector3 &p, const cnoid::Matrix33 &R);
		bool  IK_arm(const cnoid::Vector3& p, const cnoid::Matrix3& R);
		bool  IK_arm(const cnoid::Vector3& p, const cnoid::Matrix3& R, double phi, const cnoid::VectorXd& q_old= cnoid::VectorXd::Zero(7));
		
	private:
		void adjustKPWrist();
		cnoid::Link *base;
};	

}

#endif
