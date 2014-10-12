#ifndef GRASPLOTPLUGIN_Katana_H
#define GRASPLOTPLUGIN_Katana_H

#include <iostream>
#include <cnoid/JointPath.h>
#include <glibmm/i18n.h>
#include <cnoid/ItemManager.h>
#include <cnoid/BodyMotionItem.h>
#include <extplugin/graspPlugin/Grasp/Arm.h>
#include <extplugin/graspPlugin/Grasp/VectorMath.h>

#include <iostream>
#include <string>
#include <stdlib.h>

#define m_pi 3.141592


namespace grasp{

class Katana_Arm: public Arm
{
	public:
	
		Katana_Arm(cnoid::BodyPtr body, cnoid::Link *base, cnoid::Link *palm) : Arm(body, base, palm) {
			this->base = base;
			};
		~Katana_Arm() {	};
//		virtual bool IK_arm(const cnoid::Vector3 &p, const cnoid::Matrix33 &R);
		bool  IK_arm(const cnoid::Vector3& p, const cnoid::Matrix3& R);
		bool  IK_arm(const cnoid::Vector3& p, const cnoid::Matrix3& R, double phi, const cnoid::VectorXd& q_old= cnoid::VectorXd::Zero(7));
		
	private:
		void adjustKatanaWrist();
		cnoid::Link *base;
};	

}

#endif
