#ifndef HEXAPOD_KINEMATICS_H_
#define HEXAPOD_KINEMATICS_H_


//Body Inverse Kinematics
extern short           PosX;            //Input position of the feet X
extern short           PosZ;            //Input position of the feet Z
extern short           PosY;            //Input position of the feet Y
//long          TotalX;            //Total X distance between the center of the body and the feet
//long          TotalZ;            //Total Z distance between the center of the body and the feet
extern long            BodyFKPosX;        //Output Position X of feet with Rotation
extern long            BodyFKPosY;        //Output Position Y of feet with Rotation
extern long            BodyFKPosZ;        //Output Position Z of feet with Rotation
// New with zentas stuff
extern short           BodyRotOffsetX;    //Input X offset value to adjust centerpoint of rotation
extern short           BodyRotOffsetY;    //Input Y offset value to adjust centerpoint of rotation
extern short           BodyRotOffsetZ;    //Input Z offset value to adjust centerpoint of rotation

//[Balance]
extern long            TotalTransX;
extern long            TotalTransZ;
extern long            TotalTransY;
extern long            TotalYBal1;
extern long            TotalXBal1;
extern long            TotalZBal1;


//Leg Inverse Kinematics
extern long            IKFeetPosX;        //Input position of the Feet X
extern long            IKFeetPosY;        //Input position of the Feet Y
extern long            IKFeetPosZ;        //Input Position of the Feet Z
extern boolean         IKSolution;        //Output true if the solution is possible
extern boolean         IKSolutionWarning;    //Output true if the solution is NEARLY possible
extern boolean         IKSolutionError;    //Output true if the solution is NOT possible

//(BODY INVERSE KINEMATICS) 
//BodyRotX         - Global Input pitch of the body 
//BodyRotY         - Global Input rotation of the body 
//BodyRotZ         - Global Input roll of the body 
//RotationY         - Input Rotation for the gait 
//PosX            - Input position of the feet X 
//PosZ            - Input position of the feet Z 
//SinB                  - Sin buffer for BodyRotX
//CosB               - Cos buffer for BodyRotX
//SinG                  - Sin buffer for BodyRotZ
//CosG               - Cos buffer for BodyRotZ
//BodyFKPosX         - Output Position X of feet with Rotation 
//BodyFKPosY         - Output Position Y of feet with Rotation 
//BodyFKPosZ         - Output Position Z of feet with Rotation
void BodyFK(short PosX, short PosZ, short PosY, short RotationY, byte BodyIKLeg);



//--------------------------------------------------------------------
//[LEG INVERSE KINEMATICS] Calculates the angles of the coxa, femur and tibia for the given position of the feet
//IKFeetPosX            - Input position of the Feet X
//IKFeetPosY            - Input position of the Feet Y
//IKFeetPosZ            - Input Position of the Feet Z
//IKSolution            - Output true if the solution is possible
//IKSolutionWarning     - Output true if the solution is NEARLY possible
//IKSolutionError    - Output true if the solution is NOT possible
//FemurAngle1           - Output Angle of Femur in degrees
//TibiaAngle1           - Output Angle of Tibia in degrees
//CoxaAngle1            - Output Angle of Coxa in degrees
//--------------------------------------------------------------------
void LegIK(short IKFeetPosX, short IKFeetPosY, short IKFeetPosZ, byte LegIKLegNr);


#endif
