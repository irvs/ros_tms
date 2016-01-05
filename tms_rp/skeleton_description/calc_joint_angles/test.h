#ifndef _CALC_JOINT_ANGLES_H_
#define _CALC_JOINT_ANGLES_H_

#include <math.h>

#include <vector>
#include <map>
#include <Eigen/Eigen>

#include <tms_msg_ss/Skeleton.h>

typedef enum {
  SB, //SpineBase,
  SM, //SpineMid,
  NE, //Neck,
  HE, //Head,
  SL, //ShoulderLeft,
  EL, //ElbowLeft,
  WL, //WristLeft,
  HaL, //HandLeft,
  SR, //ShoulderRight,
  ER, //ElbowRight,
  WR, //WristRight,
  HaR, //HandRight,
  HL, //HipLeft,
  KL, //KneeLeft,
  AL, //AnkleLeft,
  FL, //FootLeft,
  HR, //HipRight,
  KR, //KneeRight,
  AR, //AnkleRight,
  FR, //FootRight,
  SS, //SpineShoulder,
  HTL, //HandTipLeft,
  TL, //ThumbLeft,
  HTR, //HandTipRight,
  TR, //ThumbRight,
  JOINT_NUM
} JOINT_NAME;

const char* kJointName[] = {
  //"WAIST_JOINT1",
  //"WAIST_JOINT2",
  "R_ARM_JOINT1", // right arm flexion and extension
  "R_ARM_JOINT2", // right arm abduction and adduction
  "R_ARM_JOINT3", // right arm lateral and medial rotation
  "R_ARM_JOINT4", // right elbow flexion and extension
  "R_ARM_JOINT5",
  "R_ARM_JOINT6",
  "R_ARM_JOINT7",
  //"R_ARM_JOINT8",
  "L_ARM_JOINT1", // left arm flexion and extension
  "L_ARM_JOINT2", // left arm abduction and adduction
  "L_ARM_JOINT3", // left arm lateral and medial rotation
  "L_ARM_JOINT4", // left elbow flexion and extension
  "L_ARM_JOINT5",
  "L_ARM_JOINT6",
  "L_ARM_JOINT7",
  //"L_ARM_JOINT8",
  "NECK_JOINT0",
  "NECK_JOINT1",
  "NECK_JOINT2",
  "R_LEG_JOINT1", // right hip flexion and extension
  "R_LEG_JOINT2", // right hip abduction and adduction
  "R_LEG_JOINT3", // right hip lateral and medial rotation
  "R_LEG_JOINT4", // right knee flexion and extension
  "R_LEG_JOINT5",
  "R_LEG_JOINT6",
  //"R_LEG_JOINT7",
  "L_LEG_JOINT1", // left hip flexion and extension
  "L_LEG_JOINT2", // left hip abduction and adduction
  "L_LEG_JOINT3", // left hip lateral and medial rotation
  "L_LEG_JOINT4", // left knee flexion and extension
  "L_LEG_JOINT5",
  "L_LEG_JOINT6"
  //"L_LEG_JOINT7",
};

double 	Teta_SL, Phi_SL, Teta_SL2, gramma, Teta_EL, Teta_SR, Phi_SR, Teta_SR2, Teta_ER, Teta_HL, Phi_HL, Teta_HL2, Teta_KL, Teta_HR, Phi_HR, Teta_HR2, Teta_KR;
	
const int kModel01BaseLink = SM;

// Keep consistency with kJointName
const int kJointDoF = 29;

template <class T>
int calcForModel01(
    const tms_msg_ss::Skeleton &in,
    Eigen::Matrix<T, 3, 1> &position,
    Eigen::Quaternion<T> &rotation,
    std::map<std::string, T> &out)
{
  if (in.user_id < 0)
  {
    // Set data to disappear from environment
    position = Eigen::Matrix<T, 3, 1>(0.0, 0.0, 0.0);
    rotation = Eigen::Quaternion<T>(1.0, 0.0, 0.0, 0.0);
    for (int j=0; j < kJointDoF; j++)
    {
      out[kJointName[j]] = 0.0;
    }
    return -1;
  }
  else
  {
    std::vector<Eigen::Matrix<T,3,1> > j;
    j.resize(JOINT_NUM);
    for (int i=0; i<JOINT_NUM; i++)
    {
      j[i] = Eigen::Matrix<T, 3, 1>(
          in.position[i].x,
          in.position[i].y,
          in.position[i].z);
    }
    // Calculation of position.
    position = Eigen::Matrix<T,3,1>(
        j[SM][0],
        j[SM][1],
        j[SM][2]);

    // Calculation of rotation.
    Eigen::Matrix<T, 3, 1> x,y,z;
    x = (j[SB]-j[SM]).normalized();
    y = ((j[SS]-j[SL]).cross(x)).normalized();
    z = (x.cross(y));
    Eigen::Matrix<T, 3, 3> mat();
    mat <<
      x[0], y[0], z[0],
      x[1], y[1], z[1],
      x[2], y[2], z[2];

    rotation = mat;

    // Avoiding Error : I don't know why this error happens.
    if ( isnan(rotation.x()) )
    {
	// It happens when gets same j[SpinMid], j[SpineBase], j[HipRight] and j[HipLeft]
    	// pump comment: the cross product is zero thus y-axis and x-axis become zero
      rotation = Eigen::Quaternion<T>(1.0, 0.0, 0.0, 0.0);
    }

    // Initialize
    for (int i=0; i < kJointDoF; i++)
    {
      out[kJointName[i]] = 0.0;
    }

    // Calculation of joint angles.
    Eigen::Matrix<T, 3, 1> vec;
    Eigen::Quaternion<T> rot[2];
    // -*- Invalid calculation -*- 
    //if (1)//req.skeleton.confidence[SpineMid] == 2 &&
    //    //req.skeleton.confidence[SpineBase] == 2 &&
    //    //req.skeleton.confidence[SpineShoulder] == 2 &&
    //    //req.skeleton.confidence[ShoulderLeft] == 2 &&
    //    //req.skeleton.confidence[ShoulderRight] == 2)
    //{
    //  // 0:WAIST_JOINT1
    //  z = (j[SpineMid]-j[SpineBase]).normalized();
    //  x = ((j[HipLeft]-j[HipRight]).cross(j[SpineMid]-j[SpineBase])).normalized();
    //  y = (j[HipLeft]-j[HipRight]).normalized();
    //  vec = (j[SpineShoulder]-j[SpineMid]).cross(j[ShoulderLeft]-j[ShoulderRight]);
    //  out[kJointName[0]] = atan2(y.dot(vec),x.dot(vec));
    //  rot[0] = Eigen::AngleAxis<T>(out[kJointName[0]],x.cross(y));
    //  // 1:WAIST_JOINT2
    //  x = rot[0] * (j[SpineMid]-j[SpineBase]).normalized();
    //  y = rot[0] * ((j[SpineMid]-j[SpineBase]).cross(j[HipRight]-j[HipLeft])).normalized();
    //  vec = j[SpineShoulder]-j[SpineMid];
    //  out[kJointName[1]] = atan2(y.dot(vec),x.dot(vec));
    //}

Eigen::Matrix<T, 3, 1> x_w, y_w, z_w, x_SM, y_SM, z_SM, acf, tcs;

Eigen::Matrix<T, 4, 1> t1, temp_vec, arm_L, acf_SL2, forearm_L, arm_R, forearm_R, thigh_L, tcs_HL2, shank_L, thigh_R, tcs_HR2, shank_R;

Eigen::Matrix<T, 4, 4> T_WtoSM, T_SMtoSL, T_SLtoSL2, T_SL2toEL, T_SMtoSR, T_SRtoSR2, T_SR2toER, T_SMtoHL, T_HLtoHL2, T_HL2toKL, T_SMtoHR, T_HRtoHR2, T_HR2toKR;
    
	// world coordinate to ref. frame at SM (Spine Mid)
	x_w << 1, 0, 0;
	y_w << 0, 1, 0;
	z_w << 0, 0, 1;
	
	// ref. Frame at SM point
	x_SM = (j[SB]-j[SM]).normalized();
	y_SM = ((j[SS]-j[SL]).cross(x)).normalized();
	z_SM = x_SM.cross(y_SM);

	// transformation matrix from world coordinate to ref. Frame at SM point
	T_WtoSM << 
		x_w.dot(x_SM), y_w.dot(x_SM), z_w.dot(x_SM), -j[SM][0],
		x_w.dot(y_SM), y_w.dot(y_SM), z_w.dot(y_SM), -j[SM][1],
		x_w.dot(z_SM), y_w.dot(z_SM), z_w.dot(z_SM), -j[SM][2],
                	    0,	           0,	          0,         1;
	// Left Arm
    if (in.confidence[SS] == 2 &&
        in.confidence[SL] == 2 &&
        in.confidence[SM] == 2 &&
        in.confidence[EL] == 2)
    {
	//// Calculation arm_L flexion/extention, Teta_SL and arm_L abductioin/adduction, Phi_SL
	// transformation matrix from SM point to SL point (translation only, t1)
	temp_vec << 
		j[SM][0]-j[SL][0],
		j[SM][1]-j[SL][1],
		j[SM][2]-j[SL][2],
				1; 
	t1 = T_WtoSM*temp_vec;
	T_SMtoSL << 
		1, 0, 0, t1[0],
		0, 1, 0, t1[1],
        	0, 0, 1, t1[2],
		0, 0, 0,     1;
	arm_L = T_SMtoSL*T_WtoSM*(j[EL]-j[SL]);

	// Arm Flexion and Extension, Teta_SL
	Teta_SL = atan2(arm_L[1],arm_L[0]);
	out["L_ARM_JOINT1"] = Teta_SL;

	// Arm Abduction and Adduction, Phi_SL
	Phi_SL = atan2(arm_L[2],sqrt(arm_L[0]^2+arm_L[1]^2));
	out["L_ARM_JOINT2"] = Phi_SL;

	//// Calculation arm_L lateral/Medial rotation, Teta_SL2
	// transformation matrix from SL to SL2 (rotate around z_SL and y_SL by Teta_SL and Phi_SL)
	T_SLtoSL2 <<
	 cos(Teta_SL)*cos(Phi_SL), -sin(Teta_SL), cos(Teta_SL)*sin(Phi_SL), 0,
	 sin(Teta_SL)*cos(Phi_SL),  cos(Teta_SL), sin(Teta_SL)*sin(Phi_SL), 0,
		    		0,             0,              cos(Phi_SL), 0,
                    		0,             0,                        0, 1;
 
	// arm cross forearm, acf
	acf = (j[SL]-j[EL]).cross((j[WL]-j[EL]));
	
	// arm cross forearm in SL2 frame, acf_SL2
	temp_vec << acf[0], acf[1], acf[2], 1;
	acf_SL2 = T_SLtoSL2*T_SMtoSL*T_WtoSM*temp_vec;
	Teta_SL2 = atan2(acf_SL2[1],acf_SL2[2]);
	out["L_ARM_JOINT3"] = Teta_SL2;
    }
	
	// Left Elbow
    if (in.confidence[SL] == 2 &&
        in.confidence[EL] == 2 &&
        in.confidence[WL] == 2)
    {
	//// Calculation elbow flexion/extension, Teta_EL
	// transformation matrix from SL2 to EL (rotate around x_SL2 axis by 90-Teta_SL2 and translate from SL to EL point)
	gramma = 90-Teta_SL2;
	temp_vec << 
		j[SL][0]-j[EL][0],
		j[SL][1]-j[EL][1],
		j[SL][2]-j[EL][2],
				1;
	arm_L = T_SLtoSL2*T_SMtoSL*T_WtoSM*temp_vec;
	
	T_SL2toEL << 
		1,           0,            0, arm_L[0],
		0, cos(gramma), -sin(gramma), arm_L[1],
	        0, sin(gramma),  cos(gramma), arm_L[2],
	      	0,           0,            0, 	     1;
	
	temp_vec <<
		j[WL][0]-j[EL][0],
		j[WL][1]-j[EL][1],
		j[WL][2]-j[EL][2],
				1;

	forearm_L = T_SL2toEL*T_SLtoSL2*T_SMtoSL*T_WtoSM*temp_vec;
	Teta_EL = atan2(forearm_L[1],forearm_L[0]);
	out["L_ARM_JOINT4"] = Teta_EL;
    }

	//// Right Arm
    if (in.confidence[SS] == 2 &&
        in.confidence[SR] == 2 &&
        in.confidence[SM] == 2 &&
        in.confidence[ER] == 2)
    {		
	//// Calculation arm_R flexion/extention, Teta_SR and arm_R abductioin/adduction, Phi_SR
	// transformation matrix from SM point to SR point (translation only, t1)
	temp_vec <<
		j[SM][0]-j[SR][0],
		j[SM][1]-j[SR][1],
		j[SM][2]-j[SR][2],
				1;
	t1 = T_WtoSM*temp_vec;
	T_SMtoSR <<
		1, 0, 0, t1[0],
		0, 1, 0, t1[1],
		0, 0, 1, t1[2],
		0, 0, 0,     1;

	temp_vec <<
		j[ER][0]-j[SR][0],
		j[ER][1]-j[SR][1],
		j[ER][2]-j[SR][2],
				1;
	
	arm_R = T_SMtoSR*T_WtoSM*temp_vec;

	// Arm Flexion and Extension, Teta_SR
	Teta_SR = atan2(arm_R[1],arm_R[0]);
	out["R_ARM_JOINT1"] = Teta_SR;

	// Arm Abduction and Adduction, Phi_SR
	Phi_SR = atan2(arm_R[2],sqrt(arm_R[0]^2+arm_R[1]^2));
	out["R_ARM_JOINT2"] = Phi_SR;

	//// Calculation arm_R lateral/Medial rotation, Teta_SR2
	// transformation matrix from SR to SR2 (rotate around z_SR and y_SR by Teta_SR and Phi_SR)
	T_SRtoSR2 <<
	 cos(Teta_SR)*cos(Phi_SR), -sin(Teta_SR), cos(Teta_SR)*sin(Phi_SR), 0,
	 sin(Teta_SR)*cos(Phi_SR),  cos(Teta_SR), sin(Teta_SR)*sin(Phi_SR), 0,
	             		0,             0,    	       cos(Phi_SR), 0,
	             		0,             0,           	         0, 1;

	// arm cross forearm, acf
	acf = (j[SR]-j[ER]).cross((j[WR]-j[ER]));
	temp_vec << acf[0], acf[1], acf[2], 1;
	// arm cross forearm in SR2 frame, acf_SR2
	acf_SR2 = T_SRtoSR2*T_SMtoSR*T_WtoSM*temp_vec;
	Teta_SR2 = atan2(acf_SR2[1],acf_SR2[2]);
	out["R_ARM_JOINT3"] = Teta_SR2;	
    }
	
	// Right Elbow
    if (in.confidence[SR] == 2 &&
        in.confidence[ER] == 2 &&
        in.confidence[WR] == 2)
    {
	//// Calculation elbow flexion/extension, Teta_ER
	// transformation matrix from SR2 to ER (rotate around x_SR2 axis by 90-Teta_SR2 and translate from SR to ER point)
	gramma = 90-Teta_SR2;
	temp_vec <<
		j[SR][0]-j[ER][0],
		j[SR][1]-j[ER][1],
		j[SR][2]-j[ER][2],
				1;		
	
	arm_R = T_SRtoSR2*T_SMtoSR*T_WtoSM*temp_vec;
	T_SR2toER <<
		1,     	     0,            0, arm_R[0],
		0, cos(gramma), -sin(gramma), arm_R[1],
		0, sin(gramma),  cos(gramma), arm_R[2],
		0,     	     0,            0,        1;
	
	temp_vec <<
		j[WR][0]-j[ER][0],
		j[WR][1]-j[ER][1],
		j[WR][2]-j[ER][2],
				1;

	forearm_R = T_SR2toER*T_SRtoSR2*T_SMtoSR*T_WtoSM*temp_vec;
	Teta_ER = atan2(forearm_R[1],forearm_R[0]);
	out["R_ARM_JOINT4"] = Teta_ER;	
    }

	//// Left Leg
    if (in.confidence[SB] == 2 &&
        in.confidence[HL] == 2 &&
        in.confidence[SM] == 2 &&
        in.confidence[KL] == 2)
    {
	//// Calculation leg_L flexion/extention, Teta_HL and leg_L abductioin/adduction, Phi_HL
	// transformation matrix from SM point to HL point (translation only, t1)
	temp_vec <<
		j[SM][0]-j[HL][0],
		j[SM][1]-j[HL][1],
		j[SM][2]-j[HL][2],
				1;		
	t1 = T_WtoSM*temp_vec;
	T_SMtoHL << 
		1, 0, 0, t1[0],
		0, 1, 0, t1[1],
		0, 0, 1, t1[2],
		0, 0, 0,     1;
	
	temp_vec <<
		j[KL][0]-j[HL][0],
		j[KL][1]-j[HL][1],
		j[KL][2]-j[HL][2],
				1;

	thigh_L = T_SMtoHL*T_WtoSM*temp_vec;
	
	// Leg Flexion and Extension, Teta_HL
	Teta_HL = atan2(thigh_L[1],thigh_L[0]);
	out["L_LEG_JOINT1"] = Teta_HL;	
	// Leg Abduction and Adduction, Phi_HL
	Phi_HL = atan2(thigh_L[2],sqrt(thigh_L[0]^2+thigh_L[1]^2));
	out["L_LEG_JOINT2"] = Phi_HL;
	
	//// Calculation leg_L lateral/Medial rotation, Teta_HL2
	//transformation matrix from HL to HL2 (rotate around z_HL and y_HL by Teta_HL and Phi_HL)
	T_HLtoHL2 <<
	 cos(Teta_HL)*cos(Phi_HL), -sin(Teta_HL), cos(Teta_HL)*sin(Phi_HL), 0,
         sin(Teta_HL)*cos(Phi_HL),  cos(Teta_HL), sin(Teta_HL)*sin(Phi_HL), 0,
				0,	       0,	       cos(Phi_HL), 0,
				0,             0,           	         0, 1;
	
	// thigh cross shank, tcs
	tcs = (j[HL]-j[KL]).cross((j[AL]-j[KL]));

	// thigh cross shank in HL2 frame, tcs_HL2
	temp_vec <<t tcs[0], tcs[1], tcs[2], 1;
	tcs_HL2 = T_HLtoHL2*T_SMtoHL*T_WtoSM*temp_vec;
	Teta_HL2 = atan2(tcs_HL2[1],tcs_HL2[2]);
	out["L_LEG_JOINT3"] = Teta_HL2;
    }
	
	// Left Knee
    if (in.confidence[HL] == 2 &&
        in.confidence[KL] == 2 &&
        in.confidence[AL] == 2)
    {
	//// Calculation knee flexion/extension, Teta_KL
	// transformation matrix from HL2 to KL (rotate around x_HL2 axis by 90-Teta_HL2 and translate from SL to EL point)
	gramma = 90-Teta_HL2;
	temp_vec <<
		j[HL][0]-j[KL][0],
		j[HL][1]-j[KL][1],
		j[HL][2]-j[KL][2],
				1;
	thigh_L = T_HLtoHL2*T_SMtoHL*T_WtoSM*temp_vec;
	
	T_HL2toKL <<
		1,           0,            0, thigh_L[0],
		0, cos(gramma), -sin(gramma), thigh_L[1],
		0, sin(gramma),  cos(gramma), thigh_L[2],
		0,           0,            0, 	       1;
	
	temp_vec <<
		[j[AL][0]-j[KL][0],
		[j[AL][1]-j[KL][1],
		[j[AL][2]-j[KL][2],
				 1;

	shank_L = T_HL2toKL*T_HLtoHL2*T_SMtoHL*T_WtoSM*temp_vec;
	Teta_KL = atan2(shank_L[1],shank_L[0]);
	out["L_LEG_JOINT4"] = Teta_KL;
    }
	
	//// Right Leg
    if (in.confidence[SB] == 2 &&
        in.confidence[HR] == 2 &&
        in.confidence[SM] == 2 &&
        in.confidence[KR] == 2)
    {		
	//// Calculation leg_R flexion/extention, Teta_HR and leg_R abductioin/adduction, Phi_HR
	// transformation matrix from SM point to HR point (translation only, t1)
	temp_vec <<
		j[SM][0]-j[HR][0],
		j[SM][1]-j[HR][1],
		j[SM][2]-j[HR][2],
				1;
	t1 = T_WtoSM*temp_vec;
	T_SMtoHR <<
		1, 0, 0, t1[0],
		0, 1, 0, t1[1],
		0, 0, 1, t1[2],
		0, 0, 0,     1;
	
	temp_vec <<
		j[KR][0]-j[HR][0],
		j[KR][1]-j[HR][1],
		j[KR][2]-j[HR][2],
				1;

	thigh_R = T_SMtoHR*T_WtoSM*temp_vec;
	
	// Leg Flexion and Extension, Teta_HR
	Teta_HR = atan2(thigh_R[1],thigh_R[0]);
	out["R_LEG_JOINT1"] = Teta_HR;	

	// Leg Abduction and Adduction, Phi_HR
	Phi_HR = atan2(leg_R[2],sqrt(leg_R[0]^2+leg_R[1]^2));
	out["R_LEG_JOINT2"] = Phi_HR;	
	
	//// Calculation leg_R lateral/Medial rotation, Teta_HR2
	// transformation matrix from HR to HR2 (rotate around z_HR and y_HR by Teta_HR and Phi_HR)
	
	T_HRtoHR2 <<
	 cos(Teta_HR)*cos(Phi_HR), -sin(Teta_HR), cos(Teta_HR)*sin(Phi_HR), 0,
         sin(Teta_HR)*cos(Phi_HR),  cos(Teta_HR), sin(Teta_HR)*sin(Phi_HR), 0,
				0,             0,  	       cos(Phi_SR), 0,
                     		0,             0,    		         0, 1;

	// thigh cross shank, tcs
	tcs = (j[HR]-j[KR]).cross((j[AR]-j[KR]));
	temp_vec << tcs[0], tcs[1], tcs[2], 1;
	
	// thigh cross shank in HR2 frame, tcs_HR2
	tcs_HR2 = T_HRtoHR2*T_SMtoHR*T_WtoSM*temp_vec;
	Teta_HR2 = atan2(tcs_HR2[1],tcs_HR2[2]);
	out["R_LEG_JOINT3"] = Teta_HR2;	
    }
	// Right Knee
    if (in.confidence[HR] == 2 &&
        in.confidence[KR] == 2 &&
        in.confidence[AR] == 2)
    {
	//// Calculation knee flexion/extension, Teta_KR
	// transformation matrix from HR2 to KR (rotate around x_HR2 axis by 90-Teta_HR2 and translate from HR to KR point)
	gramma = 90-Teta_HR2;
	temp_vec <<
		j[HR][0]-j[KR][0],
		j[HR][1]-j[KR][1],
		j[HR][2]-j[KR][2],
				1;

	thigh_R = T_HRtoHR2*T_SMtoHR*T_WtoSM*temp_vec;
	T_HR2toKR <<
		1,           0,            0, thigh_R[0],
             	0, cos(gramma), -sin(gramma), thigh_R[1],
		0, sin(gramma),  cos(gramma), thigh_R[2],
		0,           0,            0, 	       1;

	temp_vec <<
		j[AR][0]-j[KR][0],
		j[AR][1]-j[KR][1],
		j[AR][2]-j[KR][2],
				1;

	shank_R = T_HR2toKR*T_HRtoHR2*T_SMtoHR*T_WtoSM*temp_vec;
	Teta_KR = atan2(shank_R[1],shank_R[0]);
	out["R_LEG_JOINT4"] = Teta_KR;	
    }

    for (int i=0; i < kJointDoF; i++)
    {
      out[kJointName[i]] = (isnan(out[kJointName[i]]) ? 0.0 : out[kJointName[i]]);
    }
  }

  return 0;
}

#endif //_CALC_JOINT_ANGLES_H_
