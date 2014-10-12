#include "GiveObjToHumanPlanner.h"
#include "HumanPlanner.h"
#include <cnoid/MessageView>

using namespace std;
using namespace cnoid;
using namespace grasp;

string dirPath = "./extplugin/graspPlugin/GiveObjToHumanPlanner/manipulabilityMAP/";

double calcDistance(double x1, double y1, double x2, double y2){
	return sqrt( ((x1-x2)*(x1-x2)) + ((y1-y2)*(y1-y2)) );
}

GiveObjToHumanPlanner::GiveObjToHumanPlanner()  : 	os (MessageView::mainInstance()->cout() )
{
	manip_MAP.clear();
}

GiveObjToHumanPlanner::~GiveObjToHumanPlanner() {

}

GiveObjToHumanPlanner* GiveObjToHumanPlanner::instance(GiveObjToHumanPlanner *gc) {
	static GiveObjToHumanPlanner* instance = (gc) ? gc : new GiveObjToHumanPlanner();
	if(gc) instance = gc;
	return instance;
}

void GiveObjToHumanPlanner::makeManipulabilityMap(char model_type, bool useWaist){
	PlanBase* tc = PlanBase::instance();
	HumanPlanner* hp = HumanPlanner::instance();
	
	double offset_palm_length = 0.0, offset_palm_thickness = 0.0, offset_palm_height = 0.0;
	BodyPtr use_body;
	Link* use_base;
	ArmPtr use_arm;
	FingerPtr use_finger;
	string w_name;
	double z_max = 0.0;
	switch(model_type){
		case 'R':
			w_name = "robot/" + tc->targetArmFinger->bodyItemRobot->name();
			offset_palm_length = 0.18;
			offset_palm_thickness = 0.04;
			offset_palm_height = 0.03;
			if(useWaist){
				tc->targetArmFinger = tc->armsList[0];
				w_name += "/withWaist";
			}
			else{
				tc->targetArmFinger = tc->armsList[2];
				w_name += "/noWaist";
			}
			use_body = tc->body();
			use_base = tc->base();
			use_arm = tc->arm();
			use_finger = tc->fingers(0);
			z_max = 1.61;
			break;
			
		case 'H':
			w_name = "human/" + hp->targetHuman->name();
			offset_palm_length = 0.06;
			offset_palm_thickness = 0.03;
			offset_palm_height = 0.02;
			if(useWaist){
				hp->targetHuman = hp->humanList[0];
				w_name += "/withWaist";
			}
			else{
				hp->targetHuman = hp->humanList[2];
				w_name += "/noWaist";
			}
			use_body = hp->body();
			use_base = hp->base();
			use_arm = hp->arm();
			use_finger = hp->targetHuman->fingers[0];
			z_max = (atof(hp->targetHuman->name().substr(6).c_str())+1.0) * 0.01;
			break;
			
		default:
			os<<"Error : model_type "<<model_type<<" is not type"<<endl;
			return;
	}
	
	int jointIndex = 0;
	vector<double> standardPose;
	standardPose.clear();
	Listing& pose = *use_body->info()->findListing("standardPose");
    if(pose.isValid()){
        const int n = std::min(pose.size(), use_body->numJoints());
        while(jointIndex < n){
            standardPose.push_back(deg2rad(pose[jointIndex].toDouble()));
            jointIndex++;
        }
    }
	
	//////// calc manipulability ////////
	double model_th = rpyFromRot(use_base->R())(2);
	
	Matrix3	y_roll, z_roll, palm_R;
	y_roll(0, 0) = 0.0; y_roll(0, 1) = 0.0; y_roll(0, 2) = -1.0;
	y_roll(1, 0) = 0.0; y_roll(1, 1) = 1.0; y_roll(1, 2) = 0.0;
	y_roll(2, 0) = 1.0; y_roll(2, 1) = 0.0; y_roll(2, 2) = 0.0;
	
	z_roll(0, 0) = cos(model_th); z_roll(0, 1) = -sin(model_th); z_roll(0, 2) = 0.0;
	z_roll(1, 0) = sin(model_th); z_roll(1, 1) =  cos(model_th); z_roll(1, 2) = 0.0;
	z_roll(2, 0) = 0.0; z_roll(2, 1) = 0.0; z_roll(2, 2) = 1.0;
	
	palm_R = z_roll * y_roll;
	
	Vector3	objPos, palmPos;
	
	FILE *fp_palm, *fp_max_manipulability_param;
	char fname_palm[256], fname_max_manipulability_param[256];
	
	string temp_name = dirPath + w_name + "/max_param.csv";
	sprintf(fname_max_manipulability_param, temp_name.c_str());
	fp_max_manipulability_param = fopen( fname_max_manipulability_param, "w" );
	
	double temp_manipulability = 0.0, max_manipulability = 0.0, total_manipulability = 0.0, temp_dangle = 0.0, min_dangle = 1000000.0;
	vector<vector<double> > param;
	param.clear();
	vector<double> temp_param;
	temp_param.clear();
	
	char z_name[128];
	for(double z=0.0;z<=z_max;z+=0.01){
		max_manipulability = 0.0;
		total_manipulability = 0.0;
		min_dangle = 1000000.0;
		param.clear();
		
		sprintf(z_name, "z%.0f", z*1000.0);
		temp_name = dirPath + w_name + "/palmPos_" + z_name + ".csv" ;
		sprintf(fname_palm, temp_name.c_str());
		//~ temp_name = dirPath + w_name + "/param_" + z_name + ".csv" ;
		//~ sprintf(fname_param, temp_name.c_str());

		fp_palm = fopen( fname_palm, "w" );
		//~ fp_param = fopen( fname_param, "w" );
		for(double x=0.0+use_base->p()(0);x<=1.0+use_base->p()(0);x+=0.01){
			for(double y=-1.0+use_base->p()(1);y<=0.50+use_base->p()(1);y+=0.01){
				temp_dangle = 0.0;
				temp_param.clear();
				
				objPos(0) = x;
				objPos(1) = y;
				objPos(2) = z;
				//~ tc->object()->p = objPos;
				palmPos(0) = x - offset_palm_length;
				palmPos(1) = y - offset_palm_thickness;
				palmPos(2) = z - offset_palm_height;
				if(use_arm->IK_arm(palmPos, palm_R)){
					//~ use_finger->joint(0)->q()=deg2rad(-25.0);
					//~ hp->flush();
					//~ sleep(0.5);
					if(!hp->isColliding())
						temp_manipulability = use_arm->Manipulability();
					else
						temp_manipulability = 0.0;
				}
				else
					temp_manipulability = 0.0;
					
				total_manipulability += temp_manipulability;
				
				fprintf( fp_palm, "%f,%f,%f\n", palmPos(0)-use_base->p()(0), palmPos(1)-use_base->p()(1), temp_manipulability);
				
				for(int k=0;k<use_body->numJoints();k++){
					temp_dangle += fabs( use_body->joint(k)->q() - standardPose[k] );
				}
				
				if((max_manipulability!=0)&&(temp_manipulability == max_manipulability)&&(temp_dangle < min_dangle)){
					min_dangle = temp_dangle;
					temp_param.push_back(objPos(0)-use_base->p()(0));
					temp_param.push_back(objPos(1)-use_base->p()(1));
					temp_param.push_back(objPos(2));
					temp_param.push_back(palmPos(0)-use_base->p()(0));
					temp_param.push_back(palmPos(1)-use_base->p()(1));
					temp_param.push_back(palmPos(2)-use_base->p()(2));
					temp_param.push_back(max_manipulability);
					for(int k=0;k<use_body->numJoints();k++){
						temp_param.push_back(rad2deg(use_body->joint(k)->q()));
					}
					param.push_back(temp_param);
				}
				if(temp_manipulability > max_manipulability){
					max_manipulability = temp_manipulability;
					min_dangle = temp_dangle;
					param.clear();
					
					temp_param.push_back(objPos(0)-use_base->p()(0));
					temp_param.push_back(objPos(1)-use_base->p()(1));
					temp_param.push_back(objPos(2));
					temp_param.push_back(palmPos(0)-use_base->p()(0));
					temp_param.push_back(palmPos(1)-use_base->p()(1));
					temp_param.push_back(palmPos(2));
					temp_param.push_back(max_manipulability);
					for(int k=0;k<use_body->numJoints();k++){
						temp_param.push_back(rad2deg(use_body->joint(k)->q()));
					}
					param.push_back(temp_param);
				}
			}
		}
		fclose( fp_palm );
		//~ fclose( fp_param );
		printf( "%sファイル書き込みが終わりました\n", fname_palm );
		//~ printf( "%sファイル書き込みが終わりました\n", fname_param );
		
		if(param.size()!=0){
			for(int n=0;n<param.size();n++){
				for(int m=0;m<param[n].size();m++){
					fprintf( fp_max_manipulability_param, "%f,", param[n][m]);
				}
				fprintf( fp_max_manipulability_param, "%f\n", total_manipulability);
			}
		}
	}
	
	fclose( fp_max_manipulability_param );
	printf( "%sファイル書き込みが終わりました\n", fname_max_manipulability_param );
	//////// end calc manipulability ////////
}

bool GiveObjToHumanPlanner::SetRobotManipulabilityMap(char robot_LR, bool robot_useWaist){	//LR_flg:0=left, 1=right
	robot_manip_MAP.resize(161);
	
	PlanBase* tc = PlanBase::instance();
	
	FILE *fp_r;
	char fname_r[256];
	string temp_name = dirPath;
	char *tp_r;
	char buff[4096];
	int CommaCount=0;
	
	for(int i=0;i<robot_manip_MAP.size();i++){
		robot_manip_MAP[i].reachable = false;
		robot_manip_MAP[i].robot_manipulability = 0.0;
		robot_manip_MAP[i].peak_manipulability = 0.0;
		robot_manip_MAP[i].total_manipulability = 0.0;
		
		robot_manip_MAP[i].obj_pos(0) = robot_manip_MAP[i].obj_pos(1) = robot_manip_MAP[i].obj_pos(2) = 0.0;
		robot_manip_MAP[i].robot_palm_pos_rel(0) = robot_manip_MAP[i].robot_palm_pos_rel(1) = robot_manip_MAP[i].robot_palm_pos_rel(2) = 0.0;
		
		robot_manip_MAP[i].dist_from_obj_to_robot = robot_manip_MAP[i].th_from_obj_to_robot = 0.0;
		
		robot_manip_MAP[i].robot_joint_angle.clear();
	}
	
	////////// read robot manipulability map //////////
	temp_name = dirPath;
	temp_name += "robot/" + tc->targetArmFinger->bodyItemRobot->name();
	if(robot_useWaist){
		temp_name += "/withWaist";
		tc->targetArmFinger = tc->armsList[0];
		if(robot_LR=='L')
			tc->targetArmFinger = tc->armsList[1];
	}
	else{
		temp_name += "/noWaist";
		tc->targetArmFinger = tc->armsList[2];
		if(robot_LR=='L')
			tc->targetArmFinger = tc->armsList[3];
	}
	temp_name += "/max_param.csv";
	sprintf(fname_r, temp_name.c_str());
	while(1){
		if((fp_r = fopen(fname_r, "r")) == NULL){
			printf("Error: file cannot open\n");
			return false;
		}
		else
			break;
	}

	unsigned int z = 0;
	while(fgets(buff, 4096, fp_r) != NULL){
		manip_map_data temp_map_data;
		CommaCount = 0;
		tp_r = strtok(buff, ",");
		while(tp_r != NULL){
			if((CommaCount>=0)&&(CommaCount<=2))
				temp_map_data.obj_pos(CommaCount) = atof(tp_r);
			else if((CommaCount>=3)&&(CommaCount<=5))
				temp_map_data.robot_palm_pos_rel(CommaCount-3) = atof(tp_r);
			else if(CommaCount == 6){
				temp_map_data.robot_manipulability = atof(tp_r);
				temp_map_data.peak_manipulability = atof(tp_r);
			}
			else if((CommaCount>=7)&&(CommaCount<7+tc->targetArmFinger->bodyItemRobot->body()->numJoints()))
				temp_map_data.robot_joint_angle.push_back(deg2rad(atof(tp_r)));
			else if(CommaCount == 7+tc->targetArmFinger->bodyItemRobot->body()->numJoints()){
				temp_map_data.total_manipulability = atof(tp_r);
				if(temp_map_data.total_manipulability!=0.0)
					temp_map_data.reachable = true;
			}
			else if(CommaCount == 7+tc->targetArmFinger->bodyItemRobot->body()->numJoints()+1)
				break;
			tp_r = strtok(NULL, ",");
			CommaCount++;
		}
		
		z = (int)round(temp_map_data.obj_pos(2)*100);
		
		robot_manip_MAP[z] = temp_map_data;
		//~ robot_manip_MAP[z].robot_manipulability = temp_map_data.robot_manipulability;
		//~ robot_manip_MAP[z].robot_palm_pos_rel = temp_map_data.robot_palm_pos_rel;
		//~ robot_manip_MAP[z].robot_joint_angle = temp_map_data.robot_joint_angle;
		if(robot_LR=='L'){
			temp_map_data.obj_pos(1) *= -1.0;
			robot_manip_MAP[z].robot_palm_pos_rel(1) *= -1.0;
			for(unsigned int i=0;i<8;i++){
				if((i==1)||(i==2)||(i==4)||(i==6)){
					temp_map_data.robot_joint_angle[2+i] *= -1.0;
					temp_map_data.robot_joint_angle[2+i+8] *= -1.0;
				}
				robot_manip_MAP[z].robot_joint_angle[2+i] = temp_map_data.robot_joint_angle[2+i+8];
				robot_manip_MAP[z].robot_joint_angle[2+i+8] = temp_map_data.robot_joint_angle[2+i];
			}
		}
		robot_manip_MAP[z].dist_from_obj_to_robot = sqrt( (temp_map_data.obj_pos(0)*temp_map_data.obj_pos(0)) + (temp_map_data.obj_pos(1)*temp_map_data.obj_pos(1)) );
		robot_manip_MAP[z].th_from_obj_to_robot = atan2( temp_map_data.obj_pos(1), temp_map_data.obj_pos(0) );
		//~ robot_manip_MAP[z].total_manipulability = temp_map_data.total_manipulability;
	}
	fclose(fp_r);
	printf( "finish read %s\n", fname_r );
	
	//~ FILE *fp_out_h;
	//~ const char	*fname_out_h = "./robot_manip_MAP_test.csv";
	//~ fp_out_h = fopen( fname_out_h, "w" );	
	//~ for(unsigned int z=0;z<robot_manip_MAP.size();z++){
		//~ fprintf( fp_out_h, "%d,%f,%f,%f,%f\n", z, robot_manip_MAP[z].obj_pos(0), robot_manip_MAP[z].obj_pos(1), robot_manip_MAP[z].dist_from_obj_to_robot, robot_manip_MAP[z].total_manipulability);
	//~ }
	//~ fclose( fp_out_h );
	//~ printf( "%sファイル書き込みが終わりました\n", fname_out_h );
	
	return true;
}

//human_pos = face_center_pos
bool GiveObjToHumanPlanner::unifyManipulabilityMap(Vector3 human_pos, double human_th, int human_height, char robot_LR, bool robot_useWaist, char human_LR, bool human_useWaist){	//LR_flg:0=left, 1=right
	manip_MAP.resize(human_height+1);
	
	PlanBase* tc = PlanBase::instance();
	HumanPlanner* hp = HumanPlanner::instance();
	
	FILE *fp_r, *fp_h;
	char fname_r[256], fname_h[256];
	string temp_name = dirPath;
	char *tp_r, *tp_h;
	char buff[4096];
	int CommaCount=0;
	double temp_z = 0.0;
	
	double human_z_offset = (double)human_height - 10.0 - human_pos(2)*100.0;//10.0:頭のてっぺんから目の高さまでの距離
	
	for(int i=0;i<manip_MAP.size();i++){
		manip_MAP[i].reachable = false;
		manip_MAP[i].robot_manipulability = 0.0;
		manip_MAP[i].human_manipulability = 0.0;
		manip_MAP[i].peak_manipulability = 0.0;
		manip_MAP[i].total_manipulability = 0.0;
		
		manip_MAP[i].obj_pos(0) = manip_MAP[i].obj_pos(1) = manip_MAP[i].obj_pos(2) = 0.0;
		manip_MAP[i].robot_palm_pos_rel(0) = manip_MAP[i].robot_palm_pos_rel(1) = manip_MAP[i].robot_palm_pos_rel(2) = 0.0;
		
		manip_MAP[i].dist_from_obj_to_robot = manip_MAP[i].th_from_obj_to_robot = 0.0;
		
		manip_MAP[i].human_joint_angle.clear();
		manip_MAP[i].robot_joint_angle.clear();
	}
	
	////////// read human manipulability map //////////
	temp_name = dirPath;
	temp_name += "human/" + hp->targetHuman->bodyItemHuman->name();
	if(human_useWaist){
		temp_name += "/withWaist";
		hp->targetHuman = hp->humanList[0];
		if(human_LR=='L')
			hp->targetHuman = hp->humanList[1];
	}
	else{
		temp_name += "/noWaist";
		hp->targetHuman = hp->humanList[2];
		if(human_LR=='L')
			hp->targetHuman = hp->humanList[3];
	}
	temp_name += "/max_param.csv";
	sprintf(fname_h, temp_name.c_str());
	while(1){
		if((fp_h = fopen(fname_h, "r")) == NULL){
			printf("Error: file cannot open\n");
			return false;
		}
		else
			break;
	}

	while(fgets(buff, 4096, fp_h) != NULL){
		manip_map_data temp_map_data;
		CommaCount = 0;
		tp_h = strtok(buff, ",");
		while(tp_h != NULL){
			if((CommaCount>=0)&&(CommaCount<=2))
				temp_map_data.obj_pos(CommaCount) = atof(tp_h);
			else if(CommaCount == 6){
				temp_map_data.human_manipulability = atof(tp_h);
				temp_map_data.peak_manipulability = atof(tp_h);
			}
			else if((CommaCount>=7)&&(CommaCount<7+hp->targetHuman->bodyItemHuman->body()->numJoints()))
				temp_map_data.human_joint_angle.push_back(deg2rad(atof(tp_h)));
			else if(CommaCount == 7+hp->targetHuman->bodyItemHuman->body()->numJoints()){
				temp_map_data.total_manipulability = atof(tp_h);
			}
			else if(CommaCount == 7+hp->targetHuman->bodyItemHuman->body()->numJoints()+1)
				break;
			tp_h = strtok(NULL, ",");
			CommaCount++;
		}
		temp_z = round(temp_map_data.obj_pos(2)*100 - human_z_offset);
		if((int)temp_z > 0){
			temp_map_data.reachable = true;
			manip_MAP[(int)temp_z] = temp_map_data;
			if(human_LR=='L'){
				temp_map_data.obj_pos(1) *= -1.0;
				manip_MAP[(int)temp_z].human_joint_angle[0] = -temp_map_data.human_joint_angle[0];
				for(unsigned int i=0;i<8;i++){
					if((i==1)||(i==2)||(i==4)||(i==6)){
						temp_map_data.human_joint_angle[2+i] *= -1.0;
						temp_map_data.human_joint_angle[2+i+8] *= -1.0;
					}
					manip_MAP[(int)temp_z].human_joint_angle[2+i] = temp_map_data.human_joint_angle[2+i+8];
					manip_MAP[(int)temp_z].human_joint_angle[2+i+8] = temp_map_data.human_joint_angle[2+i];
				}
			}
			manip_MAP[(int)temp_z].obj_pos(0) = human_pos(0) + (temp_map_data.obj_pos(0)*cos(human_th) - temp_map_data.obj_pos(1)*sin(human_th));
			manip_MAP[(int)temp_z].obj_pos(1) = human_pos(1) + (temp_map_data.obj_pos(0)*sin(human_th) + temp_map_data.obj_pos(1)*cos(human_th));
			manip_MAP[(int)temp_z].obj_pos(2) = temp_z/100.0;
		}
	}
	fclose(fp_h);
	printf( "finish read %s\n", fname_h );
	
	////////// read robot manipulability map //////////
	temp_name = dirPath;
	temp_name += "robot/" + tc->targetArmFinger->bodyItemRobot->name();
	if(robot_useWaist){
		temp_name += "/withWaist";
		tc->targetArmFinger = tc->armsList[0];
		if(robot_LR=='L')
			tc->targetArmFinger = tc->armsList[1];
	}
	else{
		temp_name += "/noWaist";
		tc->targetArmFinger = tc->armsList[2];
		if(robot_LR=='L')
			tc->targetArmFinger = tc->armsList[3];
	}
	temp_name += "/max_param.csv";
	sprintf(fname_r, temp_name.c_str());
	while(1){
		if((fp_r = fopen(fname_r, "r")) == NULL){
			printf("Error: file cannot open\n");
			return false;
		}
		else
			break;
	}

	while(fgets(buff, 4096, fp_r) != NULL){
		manip_map_data temp_map_data;
		CommaCount = 0;
		tp_r = strtok(buff, ",");
		while(tp_r != NULL){
			if((CommaCount>=0)&&(CommaCount<=2))
				temp_map_data.obj_pos(CommaCount) = atof(tp_r);
			else if((CommaCount>=3)&&(CommaCount<=5))
				temp_map_data.robot_palm_pos_rel(CommaCount-3) = atof(tp_r);
			else if(CommaCount == 6)
				temp_map_data.robot_manipulability = atof(tp_r);
			else if((CommaCount>=7)&&(CommaCount<7+tc->targetArmFinger->bodyItemRobot->body()->numJoints()))
				temp_map_data.robot_joint_angle.push_back(deg2rad(atof(tp_r)));
			else if(CommaCount == 7+tc->targetArmFinger->bodyItemRobot->body()->numJoints()){
				temp_map_data.total_manipulability = atof(tp_r);
			}
			else if(CommaCount == 7+tc->targetArmFinger->bodyItemRobot->body()->numJoints()+1)
				break;
			tp_r = strtok(NULL, ",");
			CommaCount++;
		}
		
		temp_z = round(temp_map_data.obj_pos(2)*100.0);
		if(!manip_MAP[(int)temp_z].reachable)
			continue;
		manip_MAP[(int)temp_z].robot_manipulability = temp_map_data.robot_manipulability;
		manip_MAP[(int)temp_z].peak_manipulability += temp_map_data.robot_manipulability;
		manip_MAP[(int)temp_z].robot_palm_pos_rel = temp_map_data.robot_palm_pos_rel;
		manip_MAP[(int)temp_z].robot_joint_angle = temp_map_data.robot_joint_angle;
		if(robot_LR=='L'){
			temp_map_data.obj_pos(1) *= -1.0;
			manip_MAP[(int)temp_z].robot_palm_pos_rel(1) *= -1.0;
			for(unsigned int i=0;i<8;i++){
				if((i==1)||(i==2)||(i==4)||(i==6)){
					temp_map_data.robot_joint_angle[2+i] *= -1.0;
					temp_map_data.robot_joint_angle[2+i+8] *= -1.0;
				}
				manip_MAP[(int)temp_z].robot_joint_angle[2+i] = temp_map_data.robot_joint_angle[2+i+8];
				manip_MAP[(int)temp_z].robot_joint_angle[2+i+8] = temp_map_data.robot_joint_angle[2+i];
			}
		}
		manip_MAP[(int)temp_z].dist_from_obj_to_robot = sqrt( (temp_map_data.obj_pos(0)*temp_map_data.obj_pos(0)) + (temp_map_data.obj_pos(1)*temp_map_data.obj_pos(1)) );
		manip_MAP[(int)temp_z].th_from_obj_to_robot = atan2( temp_map_data.obj_pos(1), temp_map_data.obj_pos(0) );
		manip_MAP[(int)temp_z].total_manipulability += temp_map_data.total_manipulability;
	}
	fclose(fp_r);
	printf( "finish read %s\n", fname_r );
	
	for(unsigned int i=0;i<manip_MAP.size();i++){
		if((manip_MAP[i].human_manipulability==0.0)||(manip_MAP[i].robot_manipulability==0.0))
			manip_MAP[i].reachable = false;
	}
	
	//~ FILE *fp_out_h;
	//~ const char	*fname_out_h = "./manip_MAP_test.csv";
	//~ fp_out_h = fopen( fname_out_h, "w" );	
	//~ for(unsigned int z=0;z<manip_MAP.size();z++){
		//~ fprintf( fp_out_h, "%d,%f,%f,%f,%f\n", z, manip_MAP[z].obj_pos(0), manip_MAP[z].obj_pos(1), manip_MAP[z].dist_from_obj_to_robot, manip_MAP[z].total_manipulability);
	//~ }
	//~ fclose( fp_out_h );
	//~ printf( "%sファイル書き込みが終わりました\n", fname_out_h );
	
	return true;
}

void GiveObjToHumanPlanner::QSort_manip_data_total(vector<manip_map_data>& m_Map, int left, int right){
	int i, j;
    double pivot;
    manip_map_data m_data;

    i = left;
    j = right;

    pivot = m_Map[(left + right) / 2].total_manipulability;

    while (1) {
        while (m_Map[i].total_manipulability > pivot)
            i++;
        while (pivot > m_Map[j].total_manipulability)
            j--;
        if (i >= j)
            break;

		//swap
        m_data = m_Map[i];
		m_Map[i] = m_Map[j];
		m_Map[j] = m_data;
		
        i++;
        j--;
    }

    if (left < i - 1)
        QSort_manip_data_total(m_Map, left, i - 1);
    if (j + 1 <  right)
        QSort_manip_data_total(m_Map, j + 1, right);
}

void GiveObjToHumanPlanner::QSort_manip_data_peak(vector<manip_map_data>& m_Map, int left, int right){
	int i, j;
    double pivot;
    manip_map_data m_data;

    i = left;
    j = right;

    pivot = m_Map[(left + right) / 2].peak_manipulability;

    while (1) {
        while (m_Map[i].peak_manipulability > pivot)
            i++;
        while (pivot > m_Map[j].peak_manipulability)
            j--;
        if (i >= j)
            break;

		//swap
        m_data = m_Map[i];
		m_Map[i] = m_Map[j];
		m_Map[j] = m_data;
		
        i++;
        j--;
    }

    if (left < i - 1)
        QSort_manip_data_peak(m_Map, left, i - 1);
    if (j + 1 <  right)
        QSort_manip_data_peak(m_Map, j + 1, right);
}

void GiveObjToHumanPlanner::calcRobotPos_GiveObj(unsigned int rank, vector<vector<double> > &out_posList){
	out_posList.clear();
	
	if(manip_MAP.empty()){
		cout<<"Error <calcRobotPosAtGiveObj>: manip_MAP is not set"<<endl;
		return;
	}
	if((rank<0)||(rank>=manip_MAP.size())){
		cout<<"Error <calcRobotPosAtGiveObj>: set rank is invalid"<<endl;
		return;
	}
	
	PlanBase* tc = PlanBase::instance();
	
	QSort_manip_data_peak(manip_MAP, 0, manip_MAP.size()-1);
	
	tc->object()->p() = manip_MAP[0].obj_pos;
	
	vector<double>	BasePalmPos, BasePos;
	Vector3 PalmPos;
	BasePalmPos.resize(6);
	BasePos.resize(3);
	
	double dist = manip_MAP[0].dist_from_obj_to_robot;
	double th = manip_MAP[0].th_from_obj_to_robot;
	
	Matrix3	y_roll, z_roll, palm_R;
	y_roll(0, 0) = 0.0; y_roll(0, 1) = 0.0; y_roll(0, 2) = -1.0;
	y_roll(1, 0) = 0.0; y_roll(1, 1) = 1.0; y_roll(1, 2) = 0.0;
	y_roll(2, 0) = 1.0; y_roll(2, 1) = 0.0; y_roll(2, 2) = 0.0;
	
	for(double i=0;i<2*M_PI;i+=deg2rad(1.0)){
		BasePos[0] = tc->object()->p()(0) + dist*cos(i);
		BasePos[1] = tc->object()->p()(1) + dist*sin(i);
		BasePos[2] = i + M_PI - th;
		
		PalmPos[0] = BasePos[0] + ( manip_MAP[0].robot_palm_pos_rel(0)*cos(BasePos[2]) - manip_MAP[0].robot_palm_pos_rel(1)*sin(BasePos[2]) );
		PalmPos[1] = BasePos[1] + ( manip_MAP[0].robot_palm_pos_rel(0)*sin(BasePos[2]) + manip_MAP[0].robot_palm_pos_rel(1)*cos(BasePos[2]) );
		PalmPos[2] = manip_MAP[0].robot_palm_pos_rel(2);
		
		for(int j=0;j<3;j++){
			BasePalmPos[j] = BasePos[j];
			BasePalmPos[j+3] = PalmPos[j];
		}
		
		out_posList.push_back(BasePalmPos);
		//~ tc->body()->link(0)->p()(0) = BasePos[0];
		//~ tc->body()->link(0)->p()(1) = BasePos[1];
		//~ tc->body()->link(0)->R = rotFromRpy(0,0,BasePos[2]);
		//~ 
		//~ z_roll(0, 0) = cos(BasePos[2]); z_roll(0, 1) = -sin(BasePos[2]); z_roll(0, 2) = 0.0;
		//~ z_roll(1, 0) = sin(BasePos[2]); z_roll(1, 1) =  cos(BasePos[2]); z_roll(1, 2) = 0.0;
		//~ z_roll(2, 0) = 0.0; z_roll(2, 1) = 0.0; z_roll(2, 2) = 1.0;
	//~ 
		//~ palm_R = z_roll * y_roll;
		//~ 
		//~ tc->arm()->IK_arm(PalmPos, palm_R);
		//~ 
		//~ hp->flush();sleep(0.1);
	}
	
	//~ for(int i=0;i<hp->body()->numJoints();i++){
		//~ hp->body()->joint(i)->q() = manip_MAP[0].human_joint_angle[i];
	//~ }
	//~ hp->flush();
}

void GiveObjToHumanPlanner::calcRobotPos_GetObj(unsigned int rank, vector<vector<double> > &out_posList){
	out_posList.clear();
	
	if(robot_manip_MAP.empty()){
		cout<<"Error <calcRobotPosAtGetObj>: manip_MAP is not set"<<endl;
		return;
	}
	if((rank<0)||(rank>=robot_manip_MAP.size())){
		cout<<"Error <calcRobotPosAtGetObj>: set rank is invalid"<<endl;
		return;
	}
	
	PlanBase* tc = PlanBase::instance();
	
	vector<double>	BasePos;
	BasePos.resize(6);
	
	double dist = robot_manip_MAP[(int)(tc->object()->p()(2)*100)].dist_from_obj_to_robot;
	double th = robot_manip_MAP[(int)(tc->object()->p()(2)*100)].th_from_obj_to_robot;
	
	//~ tc->initial();
	for(double i=0;i<2*M_PI;i+=deg2rad(1.0)){
		BasePos[0] = tc->object()->p()(0) + dist*cos(i);
		BasePos[1] = tc->object()->p()(1) + dist*sin(i);
		BasePos[2] = i + M_PI - th;
		BasePos[3] = 0.0;
		BasePos[4] = 0.0;
		BasePos[5] = 0.0;
		
		out_posList.push_back(BasePos);
		//~ tc->body()->link(0)->p()(0) = BasePos[0];
		//~ tc->body()->link(0)->p()(1) = BasePos[1];
		//~ tc->body()->link(0)->R = rotFromRpy(0,0,BasePos[2]);
		//~ GraspController::instance()->loadAndSelectGraspPattern();
		//~ hp->flush();sleep(0.1);
	}
}

void GiveObjToHumanPlanner::expandRobotObjDist(Vector3 objPos, double expand_dist, vector<vector<double> > &out_posList){
	if(out_posList.empty()){
		cout<<"Error <expandRobotObjDist>: posList is not set"<<endl;
		return;
	}
	
	double temp_th;
	for(unsigned int i=0;i<out_posList.size();i++){
		temp_th = atan2(out_posList[i][1]-objPos(1), out_posList[i][0]-objPos(0));
		out_posList[i][0] += expand_dist*cos(temp_th);
		out_posList[i][1] += expand_dist*sin(temp_th);
	}
}

void GiveObjToHumanPlanner::dividePosList(unsigned int k, vector<vector<double> > posList, vector<vector<vector<double> > >& out_posList){
	if(k<0){
		cout<<"Error <dividePosList>: k < 0"<<endl;
		return;
	}
	if(k>posList.size()){
		cout<<"Error <dividePosList>: posList's size < number of divides"<<endl;
		return;
	}
	
	out_posList.resize(k);
	vector<vector<double> > tempList;
	tempList.clear();
	int j=0;
	for(unsigned int i=0;i<posList.size();i++){
		if(i<(j+1)*(posList.size()/k)){
			tempList.push_back(posList[i]);
			continue;
		}
		out_posList[j]=tempList;
		tempList.clear();
		i--;
		j++;
	}
	out_posList.push_back(tempList);
	tempList.clear();
}

void GiveObjToHumanPlanner::removeCollisionPos(vector<vector<double> > in_posList, vector<vector<double> >& out_posList){
	out_posList.clear();
	Vector3 prePos = PlanBase::instance()->body()->link(0)->p();
	Matrix3 preRot = PlanBase::instance()->body()->link(0)->R();
	
	for(unsigned int i=0;i<in_posList.size();i++){
		PlanBase::instance()->body()->link(0)->p()(0) = in_posList[i][0];
		PlanBase::instance()->body()->link(0)->p()(1) = in_posList[i][1];
		PlanBase::instance()->body()->link(0)->R() = rotFromRpy(0,0,in_posList[i][2]);
		
		PlanBase::instance()->calcForwardKinematics();
		if(HumanPlanner::instance()->isColliding())
			continue;
		else
			out_posList.push_back(in_posList[i]);
	}
	
	PlanBase::instance()->body()->link(0)->p() = prePos;
	PlanBase::instance()->body()->link(0)->R() = preRot;
	PlanBase::instance()->calcForwardKinematics();
}

void GiveObjToHumanPlanner::QSort_posList_distance(vector<vector<double> >& posList, int left, int right){
	int i, j;
    double pivot;
    vector<double> temp_pos;

    i = left;
    j = right;

    pivot = posList[(left+right)/2][6];
    while (1) {
		while(posList[i][6] < pivot)
			i++;
		while(pivot < posList[j][6])
			j--;
		
		if (i >= j)
			break;

		//swap
		temp_pos.clear();
        temp_pos = posList[i];
		posList[i] = posList[j];
		posList[j] = temp_pos;
		
        i++;
        j--;
    }
    
    if (left < i - 1)
        QSort_posList_distance(posList, left, i - 1);
    if (j + 1 <  right)
        QSort_posList_distance(posList, j + 1, right);
}

void GiveObjToHumanPlanner::calc_posList_distance(Vector3 target_pos, vector<vector<double> >& posList){
	for(unsigned int i=0;i<posList.size();i++){
		posList[i].push_back(calcDistance(target_pos(0),target_pos(1),posList[i][0],posList[i][1]));
	}
}
