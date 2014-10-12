#include "WagonTrajectoryPlanner.h"

#include<stdio.h>

#include <cnoid/LinkPath>	/* modified by qtconv.rb 0th rule*/
#include <cnoid/JointPath>	/* modified by qtconv.rb 0th rule*/
#include <cnoid/TimeBar>	/* modified by qtconv.rb 0th rule*/
#include <cnoid/ItemTreeView>	/* modified by qtconv.rb 0th rule*/

#ifndef WIN32
#include <dirent.h>
#endif

#include "./extplugin/graspPlugin/PRM/PlanInterface.h"

#include "./extplugin/graspPlugin/Grasp/GraspController.h"

using namespace std;
using namespace boost;
using namespace cnoid;
using namespace grasp;

/*
bool PoseSeqItemGrasp::interpolationGrasp(){
	return false;
}
*/

bool  WagonTrajectoryPlanner::updateTrajectoryFromMotion(const BodyMotionPtr motionObject, const BodyMotionPtr motionRobot, vector<WagonMotionState>& wagon_motionSeq){

		PlanBase* gc= PlanBase::instance();
		const double frameRate = motionRobot->frameRate();
		const int numFrames = motionRobot->getNumFrames();

		BodyPtr objectBody = gc->targetObject->bodyItemObject->body();
		const int numJoints = objectBody->numJoints();
		const int numLinksToPut = objectBody->numLinks() ;
		motionObject->setDimension(numFrames, numJoints, numLinksToPut);
		motionObject->setFrameRate(frameRate);


		MultiSE3Seq& pseq = *motionObject->linkPosSeq();

		MultiValueSeq& qseqRobot = *motionRobot->jointPosSeq();
		BodyPtr robotBody = gc->bodyItemRobot()->body();

		const int numJointsRobot = robotBody->numJoints();

		Link* rootLink = objectBody->rootLink();

		// store the original state
		//Se3 orgp;
		//orgp.p = rootLink->p();
		//orgp.R = rootLink->R();

		vector<WagonMotionState>::iterator itg = wagon_motionSeq.begin();
		gc->setGraspingState( itg->graspingState );
		gc->setGraspingState2( itg->graspingState2 );
		itg++;

		gc->object()->p() = gc->objVisPos();
		gc->object()->R() = gc->objVisRot();
		WagonPlanner::instance()->setMotionState(*itg);

		for(int frame = 0; frame < numFrames; ++frame){
			if( (itg != wagon_motionSeq.end()) && ( itg->time <= ((double)frame)/frameRate) ){
				gc->setGraspingState( itg->graspingState );
				gc->setGraspingState2( itg->graspingState2 );
				WagonPlanner::instance()->changeObjectType(PlanBase::instance()->targetObject, WagonPlanner::ObjectType(itg->object_type));
				itg++;
			}


			MultiValueSeq::Frame qs = qseqRobot.frame(frame);
			MultiSE3Seq& pseqRobot= *motionRobot->linkPosSeq();

			SE3& p2 = pseqRobot.at(frame, 0);
			robotBody->link(0)->p() = p2.translation();
			robotBody->link(0)->R() = Matrix3(p2.rotation());

			for(int i=0; i < numJointsRobot; ++i){
				robotBody->joint(i)->q() = qs[i];
			}
			gc->calcForwardKinematics();

			SE3& p = pseq.at(frame, 0);
			p.set(gc->object()->p(), gc->object()->R());

		}

		// store the moved state
		rootLink->p() = gc->object()->p();
		rootLink->R() = gc->object()->R();
		objectBody->calcForwardKinematics();
		gc->flush();

		return true;
}

bool  WagonTrajectoryPlanner::updateWagonTrajectoryFromMotion(int InterObjectId, std::vector<InterObject> interObjectList, const BodyMotionPtr motionObject, const BodyMotionPtr motionRobot, vector<WagonMotionState>& wagon_motionSeq){

		PlanBase* gc= PlanBase::instance();
		const double frameRate = motionRobot->frameRate();
		const int numFrames = motionRobot->getNumFrames();

		BodyPtr objectBody = interObjectList[InterObjectId].slaveItem->body();
		const int numJoints = objectBody->numJoints();
		const int numLinksToPut = objectBody->numLinks() ;
		motionObject->setDimension(numFrames, numJoints, numLinksToPut);
		motionObject->setFrameRate(frameRate);


		MultiSE3Seq& pseq = *motionObject->linkPosSeq();

		MultiValueSeq& qseqRobot = *motionRobot->jointPosSeq();
		BodyPtr robotBody = gc->bodyItemRobot()->body();

		const int numJointsRobot = robotBody->numJoints();

		//~ Link* rootLink = objectBody->rootLink();

		// store the original state
		//Se3 orgp;
		//orgp.p = rootLink->p();
		//orgp.R = rootLink->R();

		vector<WagonMotionState>::iterator itg = wagon_motionSeq.begin();
		//~ gc->setGraspingState( itg->graspingState );
		//~ gc->setGraspingState2( itg->graspingState2 );
		itg++;

		//~ gc->object()->p = gc->objVisPos();
		//~ gc->object()->R = gc->objVisRot();
		WagonPlanner::instance()->setMotionState(*itg);

		for(int frame = 0; frame < numFrames; ++frame){
			if( (itg != wagon_motionSeq.end()) && ( itg->time <= ((double)frame)/frameRate) ){
				//~ gc->setGraspingState( itg->graspingState );
				//~ gc->setGraspingState2( itg->graspingState2 );
				if(WagonPlanner::instance()->targetWagon){
					WagonPlanner::instance()->changeWagonType(WagonPlanner::instance()->targetWagon, itg->wagon_type);
				}
				for(unsigned int i=0;i<interObjectList.size();i++){
					WagonPlanner::instance()->changeType(&interObjectList[i], InterObject::InterObjectType(itg->interObjectList_type[i]));
				}
				//~ WagonPlanner::instance()->changeType(&interObject, InterObject::InterObjectType(itg->interObjectList_type[InterObjectId]));
				itg++;
			}


			MultiValueSeq::Frame qs = qseqRobot.frame(frame);
			MultiSE3Seq& pseqRobot= *motionRobot->linkPosSeq();

			SE3& p2 = pseqRobot.at(frame, 0);
			robotBody->link(0)->p() = p2.translation();
			robotBody->link(0)->R() = Matrix3(p2.rotation()) ;

			for(int i=0; i < numJointsRobot; ++i){
				robotBody->joint(i)->q() = qs[i];
			}
			gc->calcForwardKinematics();

			SE3& p = pseq.at(frame, 0);
			p.set(objectBody->link(0)->p(), objectBody->link(0)->R());
		}

		// store the moved state
		//~ rootLink->p = gc->object()->p();
		//~ rootLink->R = gc->object()->R();
		gc->calcForwardKinematics();
		gc->flush();

		return true;
}

WagonTrajectoryPlanner::WagonTrajectoryPlanner(int id){

		PlanBase* gc = PlanBase::instance();
		static int id_ =0;
		if(id==0){
			id = id_;
			id_ ++;
		}
		stringstream name ;
		name  << "WagonTrajectorySeqItem" << id;

		poseSeqItemRobot = new PoseSeqItem();
		poseSeqItemRobot->setName(name.str());
		gc->bodyItemRobot()->addSubItem(poseSeqItemRobot);	/* modified by qtconv.rb 4th rule*/
		
		cnoid::PoseSeqItem* temp_poseSeqItemObject;

		if(gc->targetObject){
				poseSeqItemObject = new PoseSeqItem();
				poseSeqItemObject->setName(name.str());
				gc->targetObject->bodyItemObject->addSubItem(poseSeqItemObject);	/* modified by qtconv.rb 4th rule*/
		}
		poseSeqInterObject.clear();
		for(unsigned int i=0;i<gc->interObjectList.size();i++){
				temp_poseSeqItemObject = new PoseSeqItem();
				temp_poseSeqItemObject->setName(name.str());
				gc->interObjectList[i].slaveItem->addSubItem(temp_poseSeqItemObject);	/* modified by qtconv.rb 4th rule*/
				poseSeqInterObject.push_back(temp_poseSeqItemObject);
		}

}

bool WagonTrajectoryPlanner::doWagonTrajectoryPlanning() {
		ItemTreeView::mainInstance()->clearSelection();
		PlanBase* gc = PlanBase::instance();
		WagonPlanner* wp = WagonPlanner::instance();

		gc->setGraspingState(PlanBase::NOT_GRASPING);
		gc->setGraspingState2(PlanBase::NOT_GRASPING);
		WagonMotionState tempState = wp->getMotionState();
		gc->initialCollision();

		vector<WagonMotionState> inputMotionSeq;
		bool outputGraspMotionSeq = true;

		if(gc->jointSeq.size()>1){
			for(unsigned int i=0; i<gc->jointSeq.size(); i++){
				gc->setGraspingState(gc->graspingStateSeq[i]);
				gc->setGraspingState2(gc->graspingStateSeq2[i]);
				if(gc->objectContactStateSeq.size()>0) gc->setObjectContactState(gc->objectContactStateSeq[i]);
				if(gc->pathPlanDOFSeq.size()>0) gc->pathPlanDOF = gc->pathPlanDOFSeq[i];
				WagonMotionState temp = wp->getMotionState();
				temp.jointSeq = gc->jointSeq[i];
				inputMotionSeq.push_back(temp);
			}
		}
		else if ( wp->wagonMotionSeq.size() > 1){
			inputMotionSeq = wp->wagonMotionSeq;
		}
		else {
			outputGraspMotionSeq = false;
			//gc->graspMotionSeq.clear();
			WagonMotionState startMotionState, endMotionState, temp;

			if(wp->startMotionState.id > 0){
				startMotionState = wp->startMotionState;
			}
			else{
				temp = wp->getMotionState();
				for(int i=0;i<temp.jointSeq.size();i++) temp.jointSeq[i]=0;
				temp.id = 1;
				startMotionState = temp;
			}
			if(wp->endMotionState.id > 0){
				endMotionState = wp->endMotionState;
			}
			else{
				temp = wp->getMotionState();
				temp.id = 2;
				endMotionState = temp;
			}

			if( (startMotionState.pos - endMotionState.pos).norm() > 1.e-10){
				gc->setTrajectoryPlanMapDOF();
			}else{
				gc->setTrajectoryPlanDOF();
			}
			startMotionState.pathPlanDOF = gc->pathPlanDOF;
			endMotionState.pathPlanDOF = gc->pathPlanDOF;
			inputMotionSeq.push_back(startMotionState);
			inputMotionSeq.push_back(endMotionState);
		}

		gc->setGraspingState(PlanBase::NOT_GRASPING);
		gc->setGraspingState2(PlanBase::NOT_GRASPING);

		grasp::PlanInterface planner(gc->bodyItemRobot(), gc->bodyItemEnv);

		vector<VectorXd> config, config_tmp;

		bool successAll=true;

		if(inputMotionSeq.size() > 1){
			wp->wagonMotionSeq.clear();
			for(unsigned int i=0; i<inputMotionSeq.size()-1; i++){
				cout <<"PRM: "<<  i <<" th input motion" << endl;
				config_tmp.clear();

				wp->setMotionState(inputMotionSeq[i]);
				//if(gc->graspMotionSeq[i].tolerance >=0) gc->setTolerance(gc->graspMotionSeq[i].tolerance);
				VectorXd cfull(inputMotionSeq[i].jointSeq.size()+6);
				cfull << inputMotionSeq[i].jointSeq, inputMotionSeq[i].pos, inputMotionSeq[i].rpy;
				config_tmp.push_back(cfull);

				cfull << inputMotionSeq[i+1].jointSeq, inputMotionSeq[i+1].pos, inputMotionSeq[i+1].rpy;
				config_tmp.push_back(cfull);

				bool success = planner.call_planner(config_tmp, inputMotionSeq[i].pathPlanDOF);
				if(!success) successAll=false;

				planner.call_smoother(config_tmp);
				
				vector<VectorXd> config_tmp2;

				for(unsigned int j=0; j<config_tmp.size()-1; j++){
					double dyaw = fabs( config_tmp[j][gc->body()->numJoints()+5] - config_tmp[j+1][gc->body()->numJoints()+5] ) ;
					int div = dyaw/3.0;
					div +=1;
					for(int i=0;i<div;i++){
						config_tmp2.push_back( config_tmp[j+1]*i/div + config_tmp[j]*(div-i)/div);
					}
				}
				config_tmp2.push_back(config_tmp.back());

				for(unsigned int j=0; j<config_tmp2.size(); j++){
					int l=i+1;
					if(j==0) l=i;

					gc->body()->link(0)->p() = config_tmp2[j].segment<3>(gc->body()->numJoints());
					gc->body()->link(0)->R() = rotFromRpy( config_tmp2[j].segment<3>(gc->body()->numJoints()+3) );
					for(int k=0;k<gc->bodyItemRobot()->body()->numJoints();k++) gc->bodyItemRobot()->body()->joint(k)->q() = config_tmp2[j][k];
					gc->setInterLink();
					WagonMotionState temp = wp->getMotionState();
					wagon_motionSeq.push_back( temp );

					if(outputGraspMotionSeq){
						if( l < gc->motionTimeSeq.size()  )  temp.motionTime = gc->motionTimeSeq[l];
						if(j < config_tmp2.size()-1 || i==inputMotionSeq.size()-2){
							wp->wagonMotionSeq.push_back(temp);
						}
					}
				}
			}
		}
		else{
			return false;
		}

		gc->setGraspingState(PlanBase::NOT_GRASPING);
		gc->setGraspingState2(PlanBase::NOT_GRASPING);
		if(gc->targetObject){
			gc->object()->p() = gc->objVisPos();
			gc->object()->R() = gc->objVisRot();
		}

		gc->setMotionState(tempState);
		gc->bodyItemRobot()->body()->calcForwardKinematics();


		PosePtr pose = new Pose(gc->bodyItemRobot()->body()->numJoints());
		for(int i=0;i<gc->bodyItemRobot()->body()->numJoints();i++){
			pose->setJointPosition(gc->bodyItemRobot()->body()->joint(i)->jointId(), gc->bodyItemRobot()->body()->joint(i)->q());
		}

		PosePtr poseObject = new Pose(1);
		if(gc->targetObject){
			poseObject->setBaseLink(0, gc->objVisPos(), gc->objVisRot());
			poseSeqItemObject->poseSeq()->insert(poseSeqItemObject->poseSeq()->end(), 0 , poseObject);
		}
		
		PosePtr tempPose = new Pose(1);
		for(unsigned int i=0;i<gc->interObjectList.size();i++){
			tempPose->setBaseLink(0, gc->interObjectList[i].slaveItem->body()->link(0)->p(), gc->interObjectList[i].slaveItem->body()->link(0)->R());
			poseSeqInterObject[i]->poseSeq()->insert(poseSeqInterObject[i]->poseSeq()->end(), 0 , tempPose);
		}

		//~ vector<VectorXd>::iterator it = config.begin();
		double time = 0;
		for(unsigned int j=0;j<wagon_motionSeq.size();j++){
			PosePtr pose_ = new Pose(*pose);
			pose_->setBaseLink(0, wagon_motionSeq[j].pos, rotFromRpy(wagon_motionSeq[j].rpy) );
			for(int i=0;i<gc->bodyItemRobot()->body()->numJoints();i++){
				pose_->setJointPosition(i, wagon_motionSeq[j].jointSeq[i]);
			}
			wagon_motionSeq[j].time = time;
			poseSeqItemRobot->poseSeq()->insert(poseSeqItemRobot->poseSeq()->end(), time , pose_);
			time  += 1.0;
		}
		poseSeqItemRobot->updateInterpolation();
		poseSeqItemRobot->updateTrajectory();
		ItemTreeView::mainInstance()->selectItem( poseSeqItemRobot->bodyMotionItem() );

		if(gc->targetObject){
			gc->object()->p() = gc->objVisPos();
			gc->object()->R() = gc->objVisRot();
			poseSeqItemObject->updateInterpolation();
			updateTrajectoryFromMotion(poseSeqItemObject->bodyMotionItem()->motion(), poseSeqItemRobot->bodyMotionItem()->motion(),wagon_motionSeq);
			poseSeqItemObject->bodyMotionItem()->notifyUpdate();
			ItemTreeView::mainInstance()->selectItem( poseSeqItemObject->bodyMotionItem() );
		}
		
		//~ enum InterObject::InterObjectType inter_object_type;
		for(unsigned int i=0;i<gc->interObjectList.size();i++){
			//~ inter_object_type = InterObject::InterObjectType(gc->interObjectList[i].type);
			//~ if(inter_object_type == InterObject::GRASPED_OBJECT){
				poseSeqInterObject[i]->updateInterpolation();
				updateWagonTrajectoryFromMotion(i, gc->interObjectList, poseSeqInterObject[i]->bodyMotionItem()->motion(), poseSeqItemRobot->bodyMotionItem()->motion(),wagon_motionSeq);
				poseSeqInterObject[i]->bodyMotionItem()->notifyUpdate();
				ItemTreeView::mainInstance()->selectItem( poseSeqInterObject[i]->bodyMotionItem() );
			//~ }
		}

		return successAll;
}





