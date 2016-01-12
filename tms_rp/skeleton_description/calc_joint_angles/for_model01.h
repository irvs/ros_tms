#ifndef _CALC_JOINT_ANGLES_H_
#define _CALC_JOINT_ANGLES_H_

#include <math.h>

#include <vector>
#include <map>
#include <Eigen/Eigen>

#include <tms_msg_ss/Skeleton.h>

typedef enum {
  SpineBase,
  SpineMid,
  Neck,
  Head,
  ShoulderLeft,
  ElbowLeft,
  WristLeft,
  HandLeft,
  ShoulderRight,
  ElbowRight,
  WaistRight, // WaistRight
  HandRight,
  HipLeft,
  KneeLeft,
  AnkleLeft,
  FootLeft,
  HipRight,
  KneeRight,
  AnkleRight,
  FootRight,
  SpineShoulder,
  HandTipLeft,
  ThumbLeft,
  HandTipRight,
  ThumbRight,
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
  "NECK_JOINTARM0",
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

  
const int kModel01BaseLink = SpineMid;

// Keep consistency with kJointName
const int kJointDoF = 29;

// -----------------------------------------------------------------------------

// -----------------------------------------------------------------------------
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
        j[SpineMid][0],
        j[SpineMid][1],
        j[SpineMid][2]);

    // Calculation of rotation.
    Eigen::Matrix<T, 3, 1> x,y,z;
    x = (j[SpineBase]-j[SpineMid]).normalized();
    y = ((j[SpineShoulder]-j[ShoulderLeft]).cross(x)).normalized();
    z = (x.cross(y));
    Eigen::Matrix<T, 3, 3> mat;
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
   

    Eigen::Matrix<T, 3, 1> x_w, y_w, z_w, x_SM, y_SM, z_SM, L_acf, R_acf, L_tcs, R_tcs, temp_vec3;
    Eigen::Matrix<T, 3, 3> rot_WtoSM, rot_SMtoSL, rot_SLtoSL2, rot_SL2toEL, rot_SMtoSR, rot_SRtoSR2, rot_SR2toER, rot_SMtoHL, rot_HLtoHL2, rot_HL2toKL, rot_SMtoHR, rot_HRtoHR2, rot_HR2toKR;
    Eigen::Matrix<T, 4, 4> T_WtoSM, T_SMtoSL, T_SLtoSL2, T_SL2toEL, T_SMtoSR, T_SRtoSR2, T_SR2toER, T_SMtoHL, T_HLtoHL2, T_HL2toKL, T_SMtoHR, T_HRtoHR2, T_HR2toKR;
    T Teta_SL, Phi_SL, Teta_SL2, Teta_EL, Teta_SR, Phi_SR, Teta_SR2, Teta_ER, Teta_HL, Phi_HL, Teta_HL2, Teta_KL, Teta_HR, Phi_HR, Teta_HR2, Teta_KR;

    // world coordinate to ref. frame at SM (Spine Mid)
    x_w = Eigen::Matrix<T, 3, 1>::UnitX();
    y_w = Eigen::Matrix<T, 3, 1>::UnitY();
    z_w = Eigen::Matrix<T, 3, 1>::UnitZ();

    // ref. Frame at SM point
    x_SM = (j[SpineBase]-j[SpineMid]).normalized();
    y_SM = ((j[SpineShoulder]-j[ShoulderLeft]).cross(x_SM)).normalized();
    z_SM = x_SM.cross(y_SM);

    rot_WtoSM << 
      x_w.dot(x_SM), y_w.dot(x_SM), z_w.dot(x_SM),
      x_w.dot(y_SM), y_w.dot(y_SM), z_w.dot(y_SM),
      x_w.dot(z_SM), y_w.dot(z_SM), z_w.dot(z_SM);

    temp_vec3 = rot_WtoSM*(-j[SpineMid]);

    T_WtoSM << rot_WtoSM, temp_vec3, Eigen::Matrix<T,1,3>::Zero(), 1;

    // Left Arm
    if (in.confidence[SpineShoulder] == 2 &&
        in.confidence[ShoulderLeft] == 2 &&
        in.confidence[SpineMid] == 2 &&
        in.confidence[ElbowLeft] == 2)
    {
      //// Calculation arm_L flexion/extention, Teta_SL and arm_L abductioin/adduction, Phi_SL
      // transformation matrix from SM point to SL point (translation only)
      rot_SMtoSL.setIdentity();
      temp_vec3 = rot_SMtoSL*rot_WtoSM*(j[SpineMid]-j[ShoulderLeft]);
      T_SMtoSL << rot_SMtoSL, temp_vec3, Eigen::Matrix<T,1,3>::Zero(), 1;

      // Arm Flexion and Extension, Teta_SL
      temp_vec3 = rot_SMtoSL*rot_WtoSM*(j[ElbowLeft]-j[ShoulderLeft]);
      Teta_SL = atan2(temp_vec3(1), temp_vec3(0));
      out["L_ARM_JOINT1"] = Teta_SL;
      ROS_INFO("L-ARM Flex/Exten: %f\n", Teta_SL*180/M_PI);

      // Arm Abduction and Adduction, Phi_SL
      Phi_SL = atan2(-temp_vec3(2),sqrt(pow(temp_vec3(0), 2.0)+pow(temp_vec3(1), 2.0)));
      out["L_ARM_JOINT2"] = Phi_SL;
      ROS_INFO("L-ARM Abduct/Adduct: %f\n", Phi_SL*180/M_PI);

      //// Calculation arm_L lateral/Medial rotation, Teta_SL2
      // transformation matrix from SL to SL2 (rotate around z_SL and y_SL by Teta_SL and Phi_SL)
      rot_SLtoSL2 = Eigen::AngleAxis<T>(-Phi_SL, Eigen::Matrix<T,3,1>::UnitY())
			* Eigen::AngleAxis<T>(-Teta_SL, Eigen::Matrix<T,3,1>::UnitZ());
      T_SLtoSL2 <<
        rot_SLtoSL2, Eigen::Matrix<T, 3, 1>::Zero(),
        Eigen::Matrix<T, 1, 3>::Zero(), 1.0;

      // Left arm cross forearm, L_acf
      L_acf = (j[ElbowLeft]-j[ShoulderLeft]).cross((j[WristLeft]-j[ShoulderLeft]));
      ROS_INFO ("L_acf: %f %f %f\n", L_acf(0), L_acf(1), L_acf(2));

//      if (abs(L_acf(0)) > 0.000001 ||
//	  abs(L_acf(1)) > 0.000001 || 	         
//	  abs(L_acf(2)) > 0.000001   )

      if (L_acf != Eigen::Matrix<T, 3, 1>::Zero())
      {
	// arm cross forearm in SL2 frame, L_acf
        L_acf = rot_SLtoSL2*rot_SMtoSL*rot_WtoSM*L_acf;
        Teta_SL2 = atan2(L_acf(2),L_acf(1)) - M_PI/2.0;
        out["L_ARM_JOINT3"] = Teta_SL2;
        ROS_INFO("L-ARM Lateral/Medial: %f, (%f)\n", Teta_SL2*180/M_PI, L_acf[0]);
      }
      else
      {
        Teta_SL2 = 0;
        out["L_ARM_JOINT3"] = Teta_SL2;
        ROS_INFO("L-ARM Lateral/Medial (zero cross): %f\n", Teta_SL2);
      }
    }

    // Left Elbow
    if (in.confidence[ShoulderLeft] == 2 &&
        in.confidence[ElbowLeft] == 2 &&
        in.confidence[WristLeft] == 2)
    {
      //// Calculation elbow flexion/extension, Teta_EL
      // transformation matrix from SL2 to EL
      // (rotate around x_SL2 axis by Teta_SL2 and translate from SL to EL point)

      temp_vec3 = rot_SLtoSL2*rot_SMtoSL*rot_WtoSM*(j[ShoulderLeft]-j[ElbowLeft]);
      rot_SL2toEL = Eigen::AngleAxis<T>(-Teta_SL2, Eigen::Matrix<T,3,1>::UnitX());
      T_SL2toEL <<
        rot_SL2toEL, temp_vec3,
        Eigen::Matrix<T,1,3>::Zero(), 1.0;

      temp_vec3 = rot_SL2toEL * rot_SLtoSL2 * rot_SMtoSL * rot_WtoSM * (j[WristLeft]-j[ElbowLeft]);
//      std::cout << temp_vec3 << std::endl;

      Teta_EL = atan2(temp_vec3(1),temp_vec3(0));
      out["L_ARM_JOINT4"] = Teta_EL;
      ROS_INFO("L-Elbow: %f\n", Teta_EL*180/M_PI);
    }

    // Right Arm
    if (in.confidence[SpineShoulder] == 2 &&
        in.confidence[ShoulderRight] == 2 &&
        in.confidence[SpineMid] == 2 &&
        in.confidence[ElbowRight] == 2)
    {
      //// Calculation arm_R flexion/extention, Teta_SR and arm_R abductioin/adduction, Phi_SR
      // transformation matrix from SM point to SR point (translation only)
      rot_SMtoSR.setIdentity();
      temp_vec3 = rot_SMtoSR*rot_WtoSM*(j[SpineMid]-j[ShoulderRight]);
      T_SMtoSR << rot_SMtoSL, temp_vec3, Eigen::Matrix<T,1,3>::Zero(), 1;

      // Arm Flexion and Extension, Teta_SR
      temp_vec3 = rot_SMtoSR*rot_WtoSM*(j[ElbowRight]-j[ShoulderRight]);
      Teta_SR = atan2(temp_vec3(1), temp_vec3(0));
      out["R_ARM_JOINT1"] = Teta_SR;
      ROS_INFO("R-ARM Flex/Exten: %f\n", Teta_SR*180/M_PI);

      // Arm Abduction and Adduction, Phi_SR
      Phi_SR = atan2(-temp_vec3(2),sqrt(pow(temp_vec3(0), 2.0)+pow(temp_vec3(1), 2.0)));
      out["R_ARM_JOINT2"] = Phi_SR;
      ROS_INFO("R-ARM Abduct/Adduct: %f\n", Phi_SR*180/M_PI);

      //// Calculation arm_R lateral/Medial rotation, Teta_SR2
      // transformation matrix from SR to SR2 (rotate around z_SR and y_SR by Teta_SR and Phi_SR)
      rot_SRtoSR2 = Eigen::AngleAxis<T>(-Phi_SR, Eigen::Matrix<T,3,1>::UnitY())
			* Eigen::AngleAxis<T>(-Teta_SR, Eigen::Matrix<T,3,1>::UnitZ());
      T_SRtoSR2 <<
        rot_SRtoSR2, Eigen::Matrix<T, 3, 1>::Zero(),
        Eigen::Matrix<T, 1, 3>::Zero(), 1.0;

      // Right arm cross forearm, R_acf
      R_acf = (j[ElbowRight]-j[ShoulderRight]).cross((j[WaistRight]-j[ShoulderRight]));
      ROS_INFO ("R_acf: %f %f %f\n", R_acf(0), R_acf(1), R_acf(2));

//      if (abs(R_acf(0)) > 0.000001 ||
//	  abs(R_acf(1)) > 0.000001 || 	         
//	  abs(R_acf(2)) > 0.000001   )

      if (R_acf != Eigen::Matrix<T, 3, 1>::Zero())
      {
	// arm cross forearm in SR2 frame, R_acf
        R_acf = rot_SRtoSR2*rot_SMtoSR*rot_WtoSM*R_acf;
        Teta_SR2 = atan2(R_acf(2),R_acf(1)) - M_PI/2.0;
        out["R_ARM_JOINT3"] = Teta_SR2;
        ROS_INFO("R-ARM Lateral/Medial: %f, (%f)\n", Teta_SR2*180/M_PI, R_acf[0]);
      }
      else
      {
        Teta_SR2 = 0;
        out["R_ARM_JOINT3"] = Teta_SR2;
        ROS_INFO("R-ARM Lateral/Medial (zero cross): %f\n", Teta_SR2);
      }
    }

    // Right Elbow
    if (in.confidence[ShoulderRight] == 2 &&
        in.confidence[ElbowRight] == 2 &&
        in.confidence[WaistRight] == 2)
    {
      //// Calculation elbow flexion/extension, Teta_ER
      // transformation matrix from SR2 to ER
      // (rotate around x_SR2 axis by Teta_SR2 and translate from SR to ER point)

      temp_vec3 = rot_SRtoSR2*rot_SMtoSR*rot_WtoSM*(j[ShoulderRight]-j[ElbowRight]);
      rot_SR2toER = Eigen::AngleAxis<T>(-Teta_SR2, Eigen::Matrix<T,3,1>::UnitX());
      T_SR2toER <<
        rot_SR2toER, temp_vec3,
        Eigen::Matrix<T,1,3>::Zero(), 1.0;

      temp_vec3 = rot_SR2toER * rot_SRtoSR2 * rot_SMtoSR * rot_WtoSM * (j[WaistRight]-j[ElbowRight]);
//      std::cout << temp_vec3 << std::endl;

      Teta_ER = atan2(temp_vec3(1),temp_vec3(0));
      out["R_ARM_JOINT4"] = Teta_ER;
      ROS_INFO("R-Elbow: %f\n", Teta_ER*180/M_PI);
    }


    // Left Leg
    if (in.confidence[SpineBase] == 2 &&
        in.confidence[HipLeft] == 2 &&
        in.confidence[SpineMid] == 2 &&
        in.confidence[KneeLeft] == 2)
    {
      //// Calculation Leg_L flexion/extention, Teta_HL and Leg_L abductioin/adduction, Phi_HL
      // transformation matrix from SM point to SL point (translation only)
      rot_SMtoHL.setIdentity();
      temp_vec3 = rot_SMtoHL*rot_WtoSM*(j[SpineMid]-j[HipLeft]);
      T_SMtoHL << rot_SMtoHL, temp_vec3, Eigen::Matrix<T,1,3>::Zero(), 1;

      // Left Leg Flexion and Extension, Teta_HL
      temp_vec3 = rot_SMtoHL*rot_WtoSM*(j[KneeLeft]-j[HipLeft]);
      Teta_HL = atan2(temp_vec3(1), temp_vec3(0));
      out["L_LEG_JOINT1"] = Teta_HL;
      ROS_INFO("L-LEG Flex/Exten: %f\n", Teta_HL*180/M_PI);

      // Left Leg Abduction and Adduction, Phi_HL
      Phi_HL = atan2(-temp_vec3(2),sqrt(pow(temp_vec3(0), 2.0)+pow(temp_vec3(1), 2.0)));
      out["L_LEG_JOINT2"] = Phi_HL;
      ROS_INFO("L-LEG Abduct/Adduct: %f\n", Phi_HL*180/M_PI);

      //// Calculation leg_L lateral/Medial rotation, Teta_HL2
      // transformation matrix from HL to HL2 (rotate around z_HL and y_HL by Teta_HL and Phi_HL)
      rot_HLtoHL2 = Eigen::AngleAxis<T>(-Phi_HL, Eigen::Matrix<T,3,1>::UnitY())
			* Eigen::AngleAxis<T>(-Teta_HL, Eigen::Matrix<T,3,1>::UnitZ());
      T_HLtoHL2 <<
        rot_HLtoHL2, Eigen::Matrix<T, 3, 1>::Zero(),
        Eigen::Matrix<T, 1, 3>::Zero(), 1.0;

      // Left thigh cross shank, L_tcs
      L_tcs = (j[KneeLeft]-j[HipLeft]).cross((j[AnkleLeft]-j[HipLeft]));
      ROS_INFO ("L_tcs: %f %f %f\n", L_tcs(0), L_tcs(1), L_tcs(2));

//      if (abs(L_tcs(0)) > 0.000001 ||
//	  abs(L_tcs(1)) > 0.000001 || 	         
//	  abs(L_tcs(2)) > 0.000001   )

      if (L_tcs != Eigen::Matrix<T, 3, 1>::Zero())
      {
	// thigh cross shank in HL2 frame, L_tcs
        L_tcs = rot_HLtoHL2*rot_SMtoHL*rot_WtoSM*L_tcs;
        Teta_HL2 = atan2(L_tcs(2),L_tcs(1)) - M_PI/2.0;
        out["L_LEG_JOINT3"] = Teta_HL2;
        ROS_INFO("L-LEG Lateral/Medial: %f, (%f)\n", Teta_HL2*180/M_PI, L_tcs(0));
      }
      else
      {
        Teta_HL2 = 0;
        out["L_LEG_JOINT3"] = Teta_HL2;
        ROS_INFO("L-LEG Lateral/Medial (zero cross): %f\n", Teta_HL2);
      }
    }

    // Left Knee
    if (in.confidence[HipLeft] == 2 &&
        in.confidence[KneeLeft] == 2 &&
        in.confidence[AnkleLeft] == 2)
    {
      //// Calculation knee flexion/extension, Teta_KL
      // transformation matrix from HL2 to KL
      // (rotate around x_HL2 axis by Teta_HL2 and translate from HL to KL point)

      temp_vec3 = rot_HLtoHL2*rot_SMtoHL*rot_WtoSM*(j[HipLeft]-j[KneeLeft]);
      rot_HL2toKL = Eigen::AngleAxis<T>(-Teta_HL2, Eigen::Matrix<T,3,1>::UnitX());
      T_HL2toKL <<
        rot_HL2toKL, temp_vec3,
        Eigen::Matrix<T,1,3>::Zero(), 1.0;

      temp_vec3 = rot_HL2toKL * rot_HLtoHL2 * rot_SMtoHL * rot_WtoSM * (j[AnkleLeft]-j[KneeLeft]);
//      std::cout << temp_vec3 << std::endl;

      Teta_KL = atan2(temp_vec3(1),temp_vec3(0));
      out["L_LEG_JOINT4"] = Teta_KL;
      ROS_INFO("L-KNEE: %f\n", Teta_KL*180/M_PI);
    }

    // Right Leg
    if (in.confidence[SpineBase] == 2 &&
        in.confidence[HipRight] == 2 &&
        in.confidence[SpineMid] == 2 &&
        in.confidence[KneeRight] == 2)
    {
      //// Calculation leg_R flexion/extention, Teta_HR and leg_R abductioin/adduction, Phi_HR
      // transformation matrix from SM point to SR point (translation only)
      rot_SMtoHR.setIdentity();
      temp_vec3 = rot_SMtoHR*rot_WtoSM*(j[SpineMid]-j[HipRight]);
      T_SMtoHR << rot_SMtoHL, temp_vec3, Eigen::Matrix<T,1,3>::Zero(), 1;

      // Right Leg Flexion and Extension, Teta_HR
      temp_vec3 = rot_SMtoHR*rot_WtoSM*(j[KneeRight]-j[HipRight]);
      Teta_HR = atan2(temp_vec3(1), temp_vec3(0));
      out["R_LEG_JOINT1"] = Teta_HR;
      ROS_INFO("R-LEG Flex/Exten: %f\n", Teta_HR*180/M_PI);

      // Right Leg Abduction and Adduction, Phi_HR
      Phi_HR = atan2(-temp_vec3(2),sqrt(pow(temp_vec3(0), 2.0)+pow(temp_vec3(1), 2.0)));
      out["R_LEG_JOINT2"] = Phi_HR;
      ROS_INFO("R-LEG Abduct/Adduct: %f\n", Phi_HR*180/M_PI);

      //// Calculation leg_R lateral/Medial rotation, Teta_HR2
      // transformation matrix from HR to HR2 (rotate around z_HR and y_HR by Teta_HR and Phi_HR)
      rot_HRtoHR2 = Eigen::AngleAxis<T>(-Phi_HR, Eigen::Matrix<T,3,1>::UnitY())
			* Eigen::AngleAxis<T>(-Teta_HR, Eigen::Matrix<T,3,1>::UnitZ());
      T_HRtoHR2 <<
        rot_HRtoHR2, Eigen::Matrix<T, 3, 1>::Zero(),
        Eigen::Matrix<T, 1, 3>::Zero(), 1.0;

      // Right thigh cross shank, R_tcs
      R_tcs = (j[KneeRight]-j[HipRight]).cross((j[AnkleRight]-j[HipRight]));
      ROS_INFO ("R_tcs: %f %f %f\n", R_tcs(0), R_tcs(1), R_tcs(2));

//      if (abs(R_tcs(0)) > 0.000001 ||
//	  abs(R_tcs(1)) > 0.000001 || 	         
//	  abs(R_tcs(2)) > 0.000001   )

      if (R_tcs != Eigen::Matrix<T, 3, 1>::Zero())
      {
	// thigh cross shank in HR2 frame, R_tcs
        R_tcs = rot_HRtoHR2*rot_SMtoHR*rot_WtoSM*R_tcs;
        Teta_HR2 = atan2(R_tcs(2),R_tcs(1)) - M_PI/2.0;
        out["R_LEG_JOINT3"] = Teta_HR2;
        ROS_INFO("R-LEG Lateral/Medial: %f, (%f)\n", Teta_HR2*180/M_PI, R_tcs[0]);
      }
      else
      {
        Teta_HR2 = 0;
        out["R_LEG_JOINT3"] = Teta_HR2;
        ROS_INFO("R-LEG Lateral/Medial (zero cross): %f\n", Teta_HR2);
      }
    }

    // Right Knee
    if (in.confidence[HipRight] == 2 &&
        in.confidence[KneeRight] == 2 &&
        in.confidence[AnkleRight] == 2)
    {
      //// Calculation knee flexion/extension, Teta_KR
      // transformation matrix from HR2 to KR
      // (rotate around x_HR2 axis by Teta_HR2 and translate from HR to KR point)

      temp_vec3 = rot_HRtoHR2*rot_SMtoHR*rot_WtoSM*(j[HipRight]-j[KneeRight]);
      rot_HR2toKR = Eigen::AngleAxis<T>(-Teta_HR2, Eigen::Matrix<T,3,1>::UnitX());
      T_HR2toKR <<
        rot_HR2toKR, temp_vec3,
        Eigen::Matrix<T,1,3>::Zero(), 1.0;

      temp_vec3 = rot_HR2toKR * rot_HRtoHR2 * rot_SMtoHR * rot_WtoSM * (j[AnkleRight]-j[KneeRight]);
//      std::cout << temp_vec3 << std::endl;

      Teta_KR = atan2(temp_vec3(1),temp_vec3(0));
      out["R_LEG_JOINT4"] = Teta_KR;
      ROS_INFO("R-Knee: %f\n", Teta_KR*180/M_PI);
    }

    for (int i=0; i < kJointDoF; i++)
    {
      out[kJointName[i]] = (isnan(out[kJointName[i]]) ? 0.0 : out[kJointName[i]]);
    }
  }

  return 0;
}

#endif //_CALC_JOINT_ANGLES_H_
