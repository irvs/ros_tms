#include "gnugraph.h"
#include "quaternion.h"
#include "robot.h"
#include "utils.h"

#ifdef use_namespace
using namespace ROBOOP;
#endif
void homogen_demo(void);

int main(void)
{
   cout << "=====================================================\n";
   cout << " Katana6M180  \n";
   cout << " DEMO program \n";
   cout << "=====================================================\n";
   cout << "\n";

   homogen_demo();
   //kinematics_demo();
   //dynamics_demo();

   return(0);
}
// 1 	sigma 		joint type (revolute=0, prismatic=1)
// 2 	theta 		Denavit-Hartenberg parameter
// 3 	d 		Denavit-Hartenberg parameter
// 4 	a 		Denavit-Hartenberg parameter
// 5 	alpha 		Denavit-Hartenberg parameter
// 6 	theta_{min} 	minimum value of joint variable
// 7 	theta_{max} 	maximum value of joint variable
// 8 	theta_{off} 	joint offset
// 9 	m 		mass of the link
// 10 	c_x 		center of mass along axis $x$
// 11 	c_y 		center of mass along axis $y$
// 12 	c_z 		center of mass along axis $z$
// 13 	I_{xx}		element $xx$ of the inertia tensor matrix
// 14 	I_{xy}		element $xy$ of the inertia tensor matrix
// 15 	I_{xz}		element $xz$ of the inertia tensor matrix
// 16 	I_{yy}		element $yy$ of the inertia tensor matrix
// 17 	I_{yz}		element $yz$ of the inertia tensor matrix
// 18 	I_{zz}		element $zz$ of the inertia tensor matrix
// 19 	I_m 		motor rotor inertia
// 20 	Gr 		motor gear ratio
// 21 	B 		motor viscous friction coefficient
// 22 	C_f 		motor Coulomb friction coefficient
// 23 	immobile 	flag for the kinematics and inverse kinematics
//   	  		(if true joint is locked, if false joint is free)

//  const Real Katana180_data[] =
//    {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
//     0, 0, 0, 0, M_PI/2.0, -31000, 94976-31000, -31000, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
//     0, 0, 0, 0.19, 0, -31000, 47488-31000, -31000, 0.926,0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
//     0, 0, 0, 0.139,0 , 31000, 51200+31000, 31000, 0.745,0.69, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
//     0, 0, 0.1473, 0, -M_PI/2.0, 31000, 31000+51200, 31000, 0.366,0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
//     0, 0, 0.166, 0, 0, 31000, 31000+51200, 31000, 0.181,0, 0, 0.08, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};

const Real Katana180_data[] =
  {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
   0, 0, 0, 0, M_PI/2.0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
   0, 0, 0, 0.19, 0, 0, 0, 0, 0.926,0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
   0, 0, 0, 0.139, 0, 0, 0, 0, 0.745,0.69, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
   0, 0, 0.1473, 0, -M_PI/2.0, 0, 0, 0, 0.366,0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
   0, 0, 0.166, 0, 0, 0, 0, 0,0.181,0, 0, 0.08, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};

void homogen_demo(void)
{
   Matrix initrob(6,23), Tobj;
   ColumnVector qs, qr;
   int dof = 0;

   cout << "\n";
   cout << "=====================================================\n";
   cout << "Katana 6M180 kinematics\n";
   cout << "=====================================================\n";
   initrob << Katana180_data;
   mRobot robot = mRobot(initrob);
   dof = robot.get_dof();

   cout << "Robot D-H parameters\n";
   cout << "   type     theta      d        a      alpha\n";
   cout << setw(7) << setprecision(3) << initrob.SubMatrix(1,dof,1,5);
   cout << "\n";

   cout << "DOF = " << dof;
   cout << "\n";

   cout << "Robot joints variables\n";
   cout << setw(7) << setprecision(3) << robot.get_q();
   cout << "\n";

   cout << "Robot position\n";
   cout << setw(7) << setprecision(3) << robot.kine(); //Gibt homogene Trafomatrix aus
   cout << "\n";

   qr = ColumnVector(dof);
   qr(1)=M_PI/2.0;
   qr(2)=0.0;
   qr(3)=0.0;
   qr(4)=0.0;
   qr(5)=0.0;
   robot.set_q(qr);
   cout << "Robot joints variables\n";
   cout << setw(7) << setprecision(3) << robot.get_q();
   cout << "\n";

   cout << "Robot Pose\n";
   Matrix Pos=robot.kine();
   cout << setw(7) << setprecision(3) << Pos;
   cout << "\n";

   cout << "x,y,z coordinates\n";
   cout << Pos(1,4) << "\n" << Pos(2,4) << "\n" << Pos(3,4) << "\n";
   cout << "\n";

   Matrix Xobj = robot.kine(); //Aktuelle Position
   qs = ColumnVector(dof);
   qs = M_PI/16.0;
   //robot.set_q(qs); // Ändert Position
   cout << "Robot inverse kinematics\n";
   cout << "  q start  q final  q real\n";
   cout << setw(7) << setprecision(3) << (qs | robot.inv_kin(Xobj) | qr); //Jetztige Winkel | IK der vorherigen Postion | Vorherige Winkel
   cout << "\n";
   cout << "\n";

}


