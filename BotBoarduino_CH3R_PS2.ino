
//=============================================================================
//Project Lynxmotion Phoenix
//Description: Phoenix software
//Software version: V2.0
//Date: 29-10-2009
//Programmer: Jeroen Janssen [aka Xan]
//         Kurt Eckhardt(KurtE) converted to C and Arduino
//
// This version of the Phoenix code was ported over to the Arduino Environement
// and is specifically configured for the Lynxmotion BotBoarduino 
//
//NEW IN V2.1 (2013-05-17)
//   - setup() made more generic by replacing exact pin-n° by PS2_CMD
//NEW IN V2.X
//=============================================================================
//
//KNOWN BUGS:
//    - Lots ;)
//
//=============================================================================
// Header Files
//=============================================================================

#if ARDUINO>99
#include <Arduino.h>
#else
#endif

#include <PS2X_lib.h>
#include <pins_arduino.h>
#include <SoftwareSerial.h>        
#include "Hex_globals.h"
#include "mathfunc.h"
#include "kinematics.h"
#include "hexLegAngles.h"
#include "hexPhysicalDimensions.h"
#include "globalVariables.h"

#define BalanceDivFactor 6    //;Other values than 6 can be used, testing...CAUTION!! At your own risk ;)




//Build tables for Leg configuration like I/O and MIN/imax values to easy access values using a FOR loop
//Constants are still defined as single values in the cfg file to make it easy to read/configure




#ifdef c4DOF
  #ifdef cRRTarsHornOffset1   // per leg configuration
    static const short cTarsHornOffset1[] PROGMEM = {cRRTarsHornOffset1,  cRMTarsHornOffset1,  cRFTarsHornOffset1,  cLRTarsHornOffset1,  cLMTarsHornOffset1,  cLFTarsHornOffset1};
    #define CTARSHORNOFFSET1(LEGI) ((short)pgm_read_word(&cTarsHornOffset1[LEGI]))
    #else   // Fixed per leg, if not defined 0
    #ifndef cTarsHornOffset1
      #define cTarsHornOffset1  0
    #endif
    #define CTARSHORNOFFSET1(LEGI)  cTarsHornOffset1
  #endif
#endif








//Start positions for the leg
const short cInitPosX[] PROGMEM = {cRRInitPosX, cRMInitPosX, cRFInitPosX, cLRInitPosX, cLMInitPosX, cLFInitPosX};
const short cInitPosY[] PROGMEM = {cRRInitPosY, cRMInitPosY, cRFInitPosY, cLRInitPosY, cLMInitPosY, cLFInitPosY};
const short cInitPosZ[] PROGMEM = {cRRInitPosZ, cRMInitPosZ, cRFInitPosZ, cLRInitPosZ, cLMInitPosZ, cLFInitPosZ};


// Define some globals for debug information
boolean g_fShowDebugPrompt;
boolean g_fDebugOutput = true;


//--------------------------------------------------------------------
//[REMOTE]                 
#define cTravelDeadZone         4    //The deadzone for the analog input from the remote
//====================================================================


//--------------------------------------------------------------------
//[POSITIONS SINGLE LEG CONTROL]

short           LegPosX[6];    //Actual X Posion of the Leg
short           LegPosY[6];    //Actual Y Posion of the Leg
short           LegPosZ[6];    //Actual Z Posion of the Leg
//--------------------------------------------------------------------
//[INPUTS]

//--------------------------------------------------------------------
//[GP PLAYER]
//--------------------------------------------------------------------
//[OUTPUTS]
boolean         LedA;    //Red
boolean         LedB;    //Green
boolean         LedC;    //Orange
boolean         Eyes;    //Eyes output
//--------------------------------------------------------------------
//[VARIABLES]
byte            Index;                    //Index universal used
byte            LegIndex;                //Index used for leg Index Number






//--------------------------------------------------------------------
//[TIMING]
unsigned long   lTimerStart;    //Start time of the calculation cycles
unsigned long   lTimerEnd;        //End time of the calculation cycles
byte            CycleTime;        //Total Cycle time

word            ServoMoveTime;        //Time for servo updates
word            PrevServoMoveTime;    //Previous time for the servo updates

//--------------------------------------------------------------------
//[GLOABAL]
//--------------------------------------------------------------------

// Define our global Input Control State object
INCONTROLSTATE   g_InControlState;      // This is our global Input control state object...

// Define our ServoWriter class
ServoDriver  g_ServoDriver;      // our global servo driver class

boolean         g_fLowVoltageShutdown;    // If set the bot shuts down because the input voltage is to low
word            Voltage;


//--boolean         g_InControlState.fHexOn;            //Switch to turn on Phoenix
//--boolean         g_InControlState.fPrev_HexOn;        //Previous loop state 
//--------------------------------------------------------------------


//[Single Leg Control]
byte            PrevSelectedLeg;
boolean         AllDown;

//[gait]

short		NomGaitSpeed;		//Nominal speed of the gait
short           TLDivFactor;         //Number of steps that a leg is on the floor while walking
short           NrLiftedPos;         //Number of positions that a single leg is lifted [1-3]
byte            LiftDivFactor;       //Normaly: 2, when NrLiftedPos=5: 4

boolean         HalfLiftHeigth;      //If TRUE the outer positions of the ligted legs will be half height    

boolean         TravelRequest;        //Temp to check if the gait is in motion
byte            StepsInGait;         //Number of steps in gait

boolean         LastLeg;             //TRUE when the current leg is the last leg of the sequence
byte            GaitStep;            //Actual Gait step

byte            GaitLegNr[6];        //Init position of the leg

byte            GaitLegNrIn;         //Input Number of the leg

long            GaitPosX[6];         //Array containing Relative X position corresponding to the Gait
long            GaitPosY[6];         //Array containing Relative Y position corresponding to the Gait
long            GaitPosZ[6];         //Array containing Relative Z position corresponding to the Gait
long            GaitRotY[6];         //Array containing Relative Y rotation corresponding to the Gait


boolean         fWalking;            //  True if the robot are walking
boolean         fContinueWalking;    // should we continue to walk?



//=============================================================================
// Function prototypes
//=============================================================================
extern void GaitSelect(void);
extern void  WriteLEDOutputs(void);    
extern void SingleLegControl(void);
void MandibleControl(void);
extern void GaitSeq(void);
extern void BalanceBody(void);
extern void CheckAngles();
void StartUpdateServos();
boolean TerminalMonitor(void);

extern void    PrintSystemStuff(void);            // Try to see why we fault...

         
extern void BalCalcOneLeg (short PosX, short PosZ, short PosY, byte BalLegNr);
extern void Gait (byte GaitCurrentLegNr);


//--------------------------------------------------------------------------
// SETUP: the main arduino setup function.
//--------------------------------------------------------------------------
void setup(){
      
    int error;

    g_fShowDebugPrompt = true;
    g_fDebugOutput = false;
#ifdef DBGSerial    
    DBGSerial.begin(57600);
#endif
    // Init our ServoDriver
    g_ServoDriver.Init();
    
    pinMode(PS2_CMD, INPUT);
    if(!digitalRead(PS2_CMD))
      g_ServoDriver.SSCForwarder();

   //Checks to see if our Servo Driver support a GP Player
//    DBGSerial.write("Program Start\n\r");
    // debug stuff
    delay(10);


    //Turning off all the leds
    LedA = 0;
    LedB = 0;
    LedC = 0;
    Eyes = 0;
         
    //Tars Init Positions
    for (LegIndex= 0; LegIndex <= 5; LegIndex++ )
    {
        LegPosX[LegIndex] = (short)pgm_read_word(&cInitPosX[LegIndex]);    //Set start positions for each leg
        LegPosY[LegIndex] = (short)pgm_read_word(&cInitPosY[LegIndex]);
        LegPosZ[LegIndex] = (short)pgm_read_word(&cInitPosZ[LegIndex]);  
    }
    
    //Single leg control. Make sure no leg is selected
    g_InControlState.SelectedLeg = 255; // No Leg selected
    PrevSelectedLeg = 255;
    
    //Body Positions
    g_InControlState.BodyPos.x = 0;
    g_InControlState.BodyPos.y = 0;
    g_InControlState.BodyPos.z = 0;
        
    //Body Rotations
    g_InControlState.BodyRot1.x = 0;
    g_InControlState.BodyRot1.y = 0;
    g_InControlState.BodyRot1.z = 0;
    BodyRotOffsetX = 0;
    BodyRotOffsetY = 0;        //Input Y offset value to adjust centerpoint of rotation
    BodyRotOffsetZ = 0;
    
        
    //Gait
    g_InControlState.GaitType = 1;  // 0; Devon wanted 
    g_InControlState.BalanceMode = 0;
    g_InControlState.LegLiftHeight = 50;
    GaitStep = 1;
    GaitSelect();
    
    g_InputController.Init();
    
    // Servo Driver
    ServoMoveTime = 150;
    g_InControlState.fHexOn = 0;
    g_fLowVoltageShutdown = false;
}

    
//-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
// Loop: the main arduino main Loop function
//-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
void loop(void)
{
    //Start time
    lTimerStart = millis(); 
    //Read input
    
    // Shut down if voltage is low
    CheckVoltage();
    if (!g_fLowVoltageShutdown)
        g_InputController.ControlInput();
    
    WriteLEDOutputs();
 

#ifdef OPT_GPPLAYER
    //GP Player
    g_ServoDriver.GPPlayer();
#endif

    //Single leg control
    SingleLegControl ();
            
    //Gait
    GaitSeq();
             
    //Balance calculations
    TotalTransX = 0;     //reset values used for calculation of balance
    TotalTransZ = 0;
    TotalTransY = 0;
    TotalXBal1 = 0;
    TotalYBal1 = 0;
    TotalZBal1 = 0;
    if (g_InControlState.BalanceMode) {
        for (LegIndex = 0; LegIndex <= 2; LegIndex++) {    // balance calculations for all Right legs

            BalCalcOneLeg (-LegPosX[LegIndex]+GaitPosX[LegIndex], 
                        LegPosZ[LegIndex]+GaitPosZ[LegIndex], 
                        (LegPosY[LegIndex]-(short)pgm_read_word(&cInitPosY[LegIndex]))+GaitPosY[LegIndex], LegIndex);
        }

        for (LegIndex = 3; LegIndex <= 5; LegIndex++) {    // balance calculations for all Right legs
            BalCalcOneLeg(LegPosX[LegIndex]+GaitPosX[LegIndex], 
                        LegPosZ[LegIndex]+GaitPosZ[LegIndex], 
                        (LegPosY[LegIndex]-(short)pgm_read_word(&cInitPosY[LegIndex]))+GaitPosY[LegIndex], LegIndex);
        }
        BalanceBody();
    }

          
 //Reset IKsolution indicators 
     IKSolution = 0 ;
     IKSolutionWarning = 0; 
     IKSolutionError = 0 ;
            
     //Do IK for all Right legs
     for (LegIndex = 0; LegIndex <=2; LegIndex++) {    
        BodyFK(-LegPosX[LegIndex]+g_InControlState.BodyPos.x+GaitPosX[LegIndex] - TotalTransX,
                LegPosZ[LegIndex]+g_InControlState.BodyPos.z+GaitPosZ[LegIndex] - TotalTransZ,
                LegPosY[LegIndex]+g_InControlState.BodyPos.y+GaitPosY[LegIndex] - TotalTransY,
                GaitRotY[LegIndex], LegIndex);
                               
        LegIK (LegPosX[LegIndex]-g_InControlState.BodyPos.x+BodyFKPosX-(GaitPosX[LegIndex] - TotalTransX), 
                LegPosY[LegIndex]+g_InControlState.BodyPos.y-BodyFKPosY+GaitPosY[LegIndex] - TotalTransY,
                LegPosZ[LegIndex]+g_InControlState.BodyPos.z-BodyFKPosZ+GaitPosZ[LegIndex] - TotalTransZ, LegIndex);
    }
          
    //Do IK for all Left legs  
    for (LegIndex = 3; LegIndex <=5; LegIndex++) {
        BodyFK(LegPosX[LegIndex]-g_InControlState.BodyPos.x+GaitPosX[LegIndex] - TotalTransX,
                LegPosZ[LegIndex]+g_InControlState.BodyPos.z+GaitPosZ[LegIndex] - TotalTransZ,
                LegPosY[LegIndex]+g_InControlState.BodyPos.y+GaitPosY[LegIndex] - TotalTransY,
                GaitRotY[LegIndex], LegIndex);
        LegIK (LegPosX[LegIndex]+g_InControlState.BodyPos.x-BodyFKPosX+GaitPosX[LegIndex] - TotalTransX,
                LegPosY[LegIndex]+g_InControlState.BodyPos.y-BodyFKPosY+GaitPosY[LegIndex] - TotalTransY,
                LegPosZ[LegIndex]+g_InControlState.BodyPos.z-BodyFKPosZ+GaitPosZ[LegIndex] - TotalTransZ, LegIndex);
    }

    //Check mechanical limits
    CheckAngles();
                
    //Write IK errors to leds
    LedC = IKSolutionWarning;
    LedA = IKSolutionError;
            
    //Drive Servos
    if (g_InControlState.fHexOn) {
        if (g_InControlState.fHexOn && !g_InControlState.fPrev_HexOn) {
            MSound(SOUND_PIN, 3, 60, 2000, 80, 2250, 100, 2500);
#ifdef USEXBEE
            XBeePlaySounds(3, 60, 2000, 80, 2250, 100, 2500);
#endif            

            Eyes = 1;
        }
        
        //Calculate Servo Move time
        if ((abs(g_InControlState.TravelLength.x)>cTravelDeadZone) || (abs(g_InControlState.TravelLength.z)>cTravelDeadZone) ||
                (abs(g_InControlState.TravelLength.y*2)>cTravelDeadZone)) {         
            ServoMoveTime = NomGaitSpeed + (g_InControlState.InputTimeDelay*2) + g_InControlState.SpeedControl;
                
            //Add aditional delay when Balance mode is on
            if (g_InControlState.BalanceMode)
                ServoMoveTime = ServoMoveTime + 100;
        } else //Movement speed excl. Walking
            ServoMoveTime = 200 + g_InControlState.SpeedControl;
        
        // note we broke up the servo driver into start/commit that way we can output all of the servo information
        // before we wait and only have the termination information to output after the wait.  That way we hopefully
        // be more accurate with our timings...
        StartUpdateServos();
        
        // See if we need to sync our processor with the servo driver while walking to ensure the prev is completed before sending the next one
                
        fContinueWalking = false;
            
        // Finding any incident of GaitPos/Rot <>0:
        for (LegIndex = 0; LegIndex <= 5; LegIndex++) {
            if ( (GaitPosX[LegIndex] > 2) || (GaitPosX[LegIndex] < -2)
                    || (GaitPosY[LegIndex] > 2) || (GaitPosY[LegIndex] < -2)
                    || (GaitPosZ[LegIndex] > 2) || (GaitPosZ[LegIndex] < -2)
                    || (GaitRotY[LegIndex] > 2) || (GaitRotY[LegIndex] < -2) )    {
                fContinueWalking = true;
                break;
            }
        }
        if (fWalking || fContinueWalking) {
            word  wDelayTime;
            fWalking = fContinueWalking;
                  
            //Get endtime and calculate wait time
            lTimerEnd = millis();
            if (lTimerEnd > lTimerStart)
                CycleTime = lTimerEnd-lTimerStart;
            else
                CycleTime = 0xffffffffL - lTimerEnd + lTimerStart + 1;
            
            // if it is less, use the last cycle time...
            //Wait for previous commands to be completed while walking
            wDelayTime = (min(max ((PrevServoMoveTime - CycleTime), 1), NomGaitSpeed));
            delay (wDelayTime); 
        }
        
    } else {
        //Turn the bot off
        if (g_InControlState.fPrev_HexOn || (AllDown= 0)) {
            ServoMoveTime = 600;
            StartUpdateServos();
            g_ServoDriver.CommitServoDriver(ServoMoveTime);
            MSound(SOUND_PIN, 3, 100, 2500, 80, 2250, 60, 2000);
#ifdef USEXBEE            
            XBeePlaySounds(3, 100, 2500, 80, 2250, 60, 2000);
#endif            
            delay(600);
        } else {
            g_ServoDriver.FreeServos();
            Eyes = 0;
        }
            // We also have a simple debug monitor that allows us to 
        // check things. call it here..
#ifdef OPT_TERMINAL_MONITOR  
        if (TerminalMonitor())
            return;           
#endif
        delay(20);  // give a pause between times we call if nothing is happening
    }

    // Xan said Needed to be here...
    g_ServoDriver.CommitServoDriver(ServoMoveTime);
    PrevServoMoveTime = ServoMoveTime;

    //Store previous g_InControlState.fHexOn State
    if (g_InControlState.fHexOn)
        g_InControlState.fPrev_HexOn = 1;
    else
        g_InControlState.fPrev_HexOn = 0;
}


//-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
// StartUpdateServos
//-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
void StartUpdateServos()
{        
    byte    LegIndex;

    // First call off to the init...
    g_ServoDriver.BeginServoUpdate();    // Start the update 

    for (LegIndex = 0; LegIndex <= 5; LegIndex++) {
#ifdef c4DOF
        g_ServoDriver.OutputServoInfoForLeg(LegIndex, CoxaAngle1[LegIndex], FemurAngle1[LegIndex], TibiaAngle1[LegIndex], sTarsAngle1[LegIndex]);
#else
        g_ServoDriver.OutputServoInfoForLeg(LegIndex, CoxaAngle1[LegIndex], FemurAngle1[LegIndex], TibiaAngle1[LegIndex]);
#endif      
    }
    
    g_ServoDriver.OutputServoInfoForMandibles(g_InControlState.ManPos.x, g_InControlState.ManPos.y, g_InControlState.ManPos.z);
}




//-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
//[WriteLEDOutputs] Updates the state of the leds
//-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
void WriteLEDOutputs(void)
{
 #ifdef cEyesPin
    digitalWrite(cEyesPin, Eyes);
#endif        
}



//-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
//[CHECK VOLTAGE]
//Reads the input voltage and shuts down the bot when the power drops
//-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
byte s_bLVBeepCnt;
bool CheckVoltage() {
#ifdef  cVoltagePin   
#ifdef cTurnOffVol
    Voltage = analogRead(cVoltagePin); // Battery voltage
    Voltage = ((long)Voltage*1955)/1000;

    if (!g_fLowVoltageShutdown) {
        if ((Voltage < cTurnOffVol) || (Voltage >= 1999)) {
#ifdef DBGSerial          
            DBGSerial.println("Voltage went low, turn off robot");
#endif            
	     //Turn off
	    g_InControlState.BodyPos.x = 0;
	    g_InControlState.BodyPos.y = 0;
	    g_InControlState.BodyPos.z = 0;
	    g_InControlState.BodyRot1.x = 0;
	    g_InControlState.BodyRot1.y = 0;
	    g_InControlState.BodyRot1.z = 0;
	    g_InControlState.TravelLength.x = 0;
	    g_InControlState.TravelLength.z = 0;
	    g_InControlState.TravelLength.y = 0;
	    g_InControlState.SelectedLeg = 255;
	    g_fLowVoltageShutdown = 1;
            s_bLVBeepCnt = 0;    // how many times we beeped...
	    g_InControlState.fHexOn = false;
	}
#ifdef cTurnOnVol
    } else if ((Voltage > cTurnOnVol) && (Voltage < 1999)) {
#ifdef DBGSerial
            DBGSerial.println("Voltage restored");
#endif          
            g_fLowVoltageShutdown = 0;
            
#endif      
    } else {
        if (s_bLVBeepCnt < 5) {
          s_bLVBeepCnt++;
          MSound(SOUND_PIN, 1, 45, 2000);
        }
        delay(2000);
    }
#endif	
#endif
  return g_fLowVoltageShutdown;
}
 
 
 
//-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
//[SINGLE LEG CONTROL]
//-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
void SingleLegControl(void)
{
  // TODO: use one or the other, or both
  MandibleControl();
  return;

  //Check if all legs are down
    AllDown = (LegPosY[cRF]==(short)pgm_read_word(&cInitPosY[cRF])) && 
              (LegPosY[cRM]==(short)pgm_read_word(&cInitPosY[cRM])) && 
              (LegPosY[cRR]==(short)pgm_read_word(&cInitPosY[cRR])) && 
              (LegPosY[cLR]==(short)pgm_read_word(&cInitPosY[cLR])) && 
              (LegPosY[cLM]==(short)pgm_read_word(&cInitPosY[cLM])) && 
              (LegPosY[cLF]==(short)pgm_read_word(&cInitPosY[cLF]));

    if (g_InControlState.SelectedLeg<=5) {
        if (g_InControlState.SelectedLeg!=PrevSelectedLeg) {
            if (AllDown) { //Lift leg a bit when it got selected
                LegPosY[g_InControlState.SelectedLeg] = (short)pgm_read_word(&cInitPosY[g_InControlState.SelectedLeg])-20;
        
                //Store current status
                 PrevSelectedLeg = g_InControlState.SelectedLeg;
            } else {//Return prev leg back to the init position
                LegPosX[PrevSelectedLeg] = (short)pgm_read_word(&cInitPosX[PrevSelectedLeg]);
                LegPosY[PrevSelectedLeg] = (short)pgm_read_word(&cInitPosY[PrevSelectedLeg]);
                LegPosZ[PrevSelectedLeg] = (short)pgm_read_word(&cInitPosZ[PrevSelectedLeg]);
            }
        } else if (!g_InControlState.fSLHold) {
            LegPosY[g_InControlState.SelectedLeg] = LegPosY[g_InControlState.SelectedLeg]+g_InControlState.SLLeg.y;
            LegPosX[g_InControlState.SelectedLeg] = (short)pgm_read_word(&cInitPosX[g_InControlState.SelectedLeg])+g_InControlState.SLLeg.x;
            LegPosZ[g_InControlState.SelectedLeg] = (short)pgm_read_word(&cInitPosZ[g_InControlState.SelectedLeg])+g_InControlState.SLLeg.z;     
        }
    } else {//All legs to init position
        if (!AllDown) {
            for(LegIndex = 0; LegIndex <= 5;LegIndex++) {
                LegPosX[LegIndex] = (short)pgm_read_word(&cInitPosX[LegIndex]);
                LegPosY[LegIndex] = (short)pgm_read_word(&cInitPosY[LegIndex]);
                LegPosZ[LegIndex] = (short)pgm_read_word(&cInitPosZ[LegIndex]);
            }
        } 
        if (PrevSelectedLeg!=255)
           PrevSelectedLeg = 255;
    }
}

//-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
//[SINGLE LEG CONTROL]
//-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
void MandibleControl(void)
{
  LegPosY[g_InControlState.SelectedLeg] = LegPosY[g_InControlState.SelectedLeg]+g_InControlState.SLLeg.y;
  LegPosX[g_InControlState.SelectedLeg] = (short)pgm_read_word(&cInitPosX[g_InControlState.SelectedLeg])+g_InControlState.SLLeg.x;
  LegPosZ[g_InControlState.SelectedLeg] = (short)pgm_read_word(&cInitPosZ[g_InControlState.SelectedLeg])+g_InControlState.SLLeg.z;     
}



//-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
// [GAITSELECT]
//-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
void GaitSelect(void)
{
    //Gait selector
    switch (g_InControlState.GaitType)  {
        case 0:
            //Ripple Gait 12 steps
            GaitLegNr[cLR] = 1;
            GaitLegNr[cRF] = 3;
            GaitLegNr[cLM] = 5;
            GaitLegNr[cRR] = 7;
            GaitLegNr[cLF] = 9;
            GaitLegNr[cRM] = 11;
            
            NrLiftedPos = 3;
            HalfLiftHeigth = 3;
            TLDivFactor = 8;      
            StepsInGait = 12;    
            NomGaitSpeed = 70;
            break;
        case 1:
            //Tripod 8 steps
            GaitLegNr[cLR] = 5;
            GaitLegNr[cRF] = 1;
            GaitLegNr[cLM] = 1;
            GaitLegNr[cRR] = 1;
            GaitLegNr[cLF] = 5;
            GaitLegNr[cRM] = 5;
                
            NrLiftedPos = 3;
            HalfLiftHeigth = 3;
            TLDivFactor = 4;
            StepsInGait = 8; 
            NomGaitSpeed = 70;
            break;
        case 2:
            //Triple Tripod 12 step
            GaitLegNr[cRF] = 3;
            GaitLegNr[cLM] = 4;
            GaitLegNr[cRR] = 5;
            GaitLegNr[cLF] = 9;
            GaitLegNr[cRM] = 10;
            GaitLegNr[cLR] = 11;
                
            NrLiftedPos = 3;
            HalfLiftHeigth = 3;
            TLDivFactor = 8;
            StepsInGait = 12; 
            NomGaitSpeed = 60;
            break;
        case 3:
            // Triple Tripod 16 steps, use 5 lifted positions
            GaitLegNr[cRF] = 4;
            GaitLegNr[cLM] = 5;
            GaitLegNr[cRR] = 6;
            GaitLegNr[cLF] = 12;
            GaitLegNr[cRM] = 13;
            GaitLegNr[cLR] = 14;
                
            NrLiftedPos = 5;
            HalfLiftHeigth = 1;
            TLDivFactor = 10;
            StepsInGait = 16; 
            NomGaitSpeed = 60;
            break;
        case 4:
            //Wave 24 steps
            GaitLegNr[cLR] = 1;
            GaitLegNr[cRF] = 21;
            GaitLegNr[cLM] = 5;
            
            GaitLegNr[cRR] = 13;
            GaitLegNr[cLF] = 9;
            GaitLegNr[cRM] = 17;
                
            NrLiftedPos = 3;
            HalfLiftHeigth = 3;
            TLDivFactor = 20;      
            StepsInGait = 24;        
            NomGaitSpeed = 70;
            break;
    }
}    

//--------------------------------------------------------------------
//[GAIT Sequence]
void GaitSeq(void)
{
    //Check if the Gait is in motion
    TravelRequest = ((abs(g_InControlState.TravelLength.x)>cTravelDeadZone) || (abs(g_InControlState.TravelLength.z)>cTravelDeadZone) || (abs(g_InControlState.TravelLength.y)>cTravelDeadZone));
    if (NrLiftedPos == 5)
  	LiftDivFactor = 4;    
    else  
 	LiftDivFactor = 2;

   //Calculate Gait sequence
    LastLeg = 0;
    for (LegIndex = 0; LegIndex <= 5; LegIndex++) { // for all legs
        if (LegIndex == 5) // last leg
            LastLeg = 1 ;
    
        Gait(LegIndex);
    }    // next leg
}


//--------------------------------------------------------------------
//[GAIT]
void Gait (byte GaitCurrentLegNr)
{

    
    //Clear values under the cTravelDeadZone
    if (!TravelRequest) {    
        g_InControlState.TravelLength.x=0;
        g_InControlState.TravelLength.z=0;
        g_InControlState.TravelLength.y=0;
    }
    //Leg middle up position
    //Gait in motion														  									Gait NOT in motion, return to home position
    if ((TravelRequest && (NrLiftedPos==1 || NrLiftedPos==3 || NrLiftedPos==5) && 
            GaitStep==GaitLegNr[GaitCurrentLegNr]) || (!TravelRequest && GaitStep==GaitLegNr[GaitCurrentLegNr] && ((abs(GaitPosX[GaitCurrentLegNr])>2) || 
                (abs(GaitPosZ[GaitCurrentLegNr])>2) || (abs(GaitRotY[GaitCurrentLegNr])>2)))) { //Up
        GaitPosX[GaitCurrentLegNr] = 0;
        GaitPosY[GaitCurrentLegNr] = -g_InControlState.LegLiftHeight;
        GaitPosZ[GaitCurrentLegNr] = 0;
        GaitRotY[GaitCurrentLegNr] = 0;
    }
    //Optional Half heigth Rear (2, 3, 5 lifted positions)
    else if (((NrLiftedPos==2 && GaitStep==GaitLegNr[GaitCurrentLegNr]) || (NrLiftedPos>=3 && 
            (GaitStep==GaitLegNr[GaitCurrentLegNr]-1 || GaitStep==GaitLegNr[GaitCurrentLegNr]+(StepsInGait-1))))
            && TravelRequest) {
        GaitPosX[GaitCurrentLegNr] = -g_InControlState.TravelLength.x/LiftDivFactor;
        GaitPosY[GaitCurrentLegNr] = -3*g_InControlState.LegLiftHeight/(3+HalfLiftHeigth);     //Easier to shift between div factor: /1 (3/3), /2 (3/6) and 3/4
        GaitPosZ[GaitCurrentLegNr] = -g_InControlState.TravelLength.z/LiftDivFactor;
        GaitRotY[GaitCurrentLegNr] = -g_InControlState.TravelLength.y/LiftDivFactor;
    }    
  	  
    // Optional Half heigth front (2, 3, 5 lifted positions)
    else if ((NrLiftedPos>=2) && (GaitStep==GaitLegNr[GaitCurrentLegNr]+1 || GaitStep==GaitLegNr[GaitCurrentLegNr]-(StepsInGait-1)) && TravelRequest) {
        GaitPosX[GaitCurrentLegNr] = g_InControlState.TravelLength.x/LiftDivFactor;
        GaitPosY[GaitCurrentLegNr] = -3*g_InControlState.LegLiftHeight/(3+HalfLiftHeigth); // Easier to shift between div factor: /1 (3/3), /2 (3/6) and 3/4
        GaitPosZ[GaitCurrentLegNr] = g_InControlState.TravelLength.z/LiftDivFactor;
        GaitRotY[GaitCurrentLegNr] = g_InControlState.TravelLength.y/LiftDivFactor;
    }

    //Optional Half heigth Rear 5 LiftedPos (5 lifted positions)
    else if (((NrLiftedPos==5 && (GaitStep==GaitLegNr[GaitCurrentLegNr]-2 ))) && TravelRequest) {
	GaitPosX[GaitCurrentLegNr] = -g_InControlState.TravelLength.x/2;
        GaitPosY[GaitCurrentLegNr] = -g_InControlState.LegLiftHeight/2;
        GaitPosZ[GaitCurrentLegNr] = -g_InControlState.TravelLength.z/2;
        GaitRotY[GaitCurrentLegNr] = -g_InControlState.TravelLength.y/2;
     }  		

    //Optional Half heigth Front 5 LiftedPos (5 lifted positions)
    else if ((NrLiftedPos==5) && (GaitStep==GaitLegNr[GaitCurrentLegNr]+2 || GaitStep==GaitLegNr[GaitCurrentLegNr]-(StepsInGait-2)) && TravelRequest) {
        GaitPosX[GaitCurrentLegNr] = g_InControlState.TravelLength.x/2;
        GaitPosY[GaitCurrentLegNr] = -g_InControlState.LegLiftHeight/2;
        GaitPosZ[GaitCurrentLegNr] = g_InControlState.TravelLength.z/2;
        GaitRotY[GaitCurrentLegNr] = g_InControlState.TravelLength.y/2;
    }

  //Leg front down position
  else if ((GaitStep==GaitLegNr[GaitCurrentLegNr]+NrLiftedPos || GaitStep==GaitLegNr[GaitCurrentLegNr]-(StepsInGait-NrLiftedPos))
            && GaitPosY[GaitCurrentLegNr]<0) {
        GaitPosX[GaitCurrentLegNr] = g_InControlState.TravelLength.x/2;
        GaitPosZ[GaitCurrentLegNr] = g_InControlState.TravelLength.z/2;
        GaitRotY[GaitCurrentLegNr] = g_InControlState.TravelLength.y/2;      	
        GaitPosY[GaitCurrentLegNr] = 0;	//Only move leg down at once if terrain adaption is turned off
    }

    //Move body forward      
    else {
        GaitPosX[GaitCurrentLegNr] = GaitPosX[GaitCurrentLegNr] - (g_InControlState.TravelLength.x/TLDivFactor);
        GaitPosY[GaitCurrentLegNr] = 0; 
        GaitPosZ[GaitCurrentLegNr] = GaitPosZ[GaitCurrentLegNr] - (g_InControlState.TravelLength.z/TLDivFactor);
        GaitRotY[GaitCurrentLegNr] = GaitRotY[GaitCurrentLegNr] - (g_InControlState.TravelLength.y/TLDivFactor);
    }
   

    //Advance to the next step
    if (LastLeg)  {  //The last leg in this step
        GaitStep = GaitStep+1;
        if (GaitStep>StepsInGait)
              GaitStep = 1;
    }
}  


//-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
//[BalCalcOneLeg]
//-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
void BalCalcOneLeg (short PosX, short PosZ, short PosY, byte BalLegNr)
{
    short            CPR_X;            //Final X value for centerpoint of rotation
    short            CPR_Y;            //Final Y value for centerpoint of rotation
    short            CPR_Z;            //Final Z value for centerpoint of rotation
    long             lAtan;

    //Calculating totals from center of the body to the feet
    CPR_Z = (short)pgm_read_word(&cOffsetZ[BalLegNr]) + PosZ;
    CPR_X = (short)pgm_read_word(&cOffsetX[BalLegNr]) + PosX;
    CPR_Y = 150 + PosY;        // using the value 150 to lower the centerpoint of rotation 'g_InControlState.BodyPos.y +

    TotalTransY += (long)PosY;
    TotalTransZ += (long)CPR_Z;
    TotalTransX += (long)CPR_X;
    
    lAtan = GetATan2(CPR_X, CPR_Z);
    TotalYBal1 += (lAtan*1800) / 31415;
    
    lAtan = GetATan2 (CPR_X, CPR_Y);
    TotalZBal1 += ((lAtan*1800) / 31415) -900; //Rotate balance circle 90 deg
    
    lAtan = GetATan2 (CPR_Z, CPR_Y);
    TotalXBal1 += ((lAtan*1800) / 31415) - 900; //Rotate balance circle 90 deg

}


//-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
//[BalanceBody]
//-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
void BalanceBody(void)
{
    TotalTransZ = TotalTransZ/BalanceDivFactor ;
    TotalTransX = TotalTransX/BalanceDivFactor;
    TotalTransY = TotalTransY/BalanceDivFactor;

    if (TotalYBal1 > 0)        //Rotate balance circle by +/- 180 deg
        TotalYBal1 -=  1800;
    else
        TotalYBal1 += 1800;
    
    if (TotalZBal1 < -1800)    //Compensate for extreme balance positions that causes owerflow
        TotalZBal1 += 3600;
    
    if (TotalXBal1 < -1800)    //Compensate for extreme balance positions that causes owerflow
        TotalXBal1 += 3600;
    
    //Balance rotation
    TotalYBal1 = -TotalYBal1/BalanceDivFactor;
    TotalXBal1 = -TotalXBal1/BalanceDivFactor;
    TotalZBal1 = TotalZBal1/BalanceDivFactor;
}





  
    



//--------------------------------------------------------------------
//[CHECK ANGLES] Checks the mechanical limits of the servos
//--------------------------------------------------------------------
void CheckAngles(void)
{

    for (LegIndex = 0; LegIndex <=5; LegIndex++)
    {
        CoxaAngle1[LegIndex]  = min(max(CoxaAngle1[LegIndex], (short)pgm_read_word(&cCoxaMin1[LegIndex])), 
                    (short)pgm_read_word(&cCoxaMax1[LegIndex]));
        FemurAngle1[LegIndex] = min(max(FemurAngle1[LegIndex], (short)pgm_read_word(&cFemurMin1[LegIndex])),
                    (short)pgm_read_word(&cFemurMax1[LegIndex]));
        TibiaAngle1[LegIndex] = min(max(TibiaAngle1[LegIndex], (short)pgm_read_word(&cTibiaMin1[LegIndex])),
                    (short)pgm_read_word(&cTibiaMax1[LegIndex]));
#ifdef c4DOF
        if ((byte)pgm_read_byte(&cTarsLength[LegIndex])) {    // We allow mix of 3 and 4 DOF legs...
            TarsAngle1[LegIndex] = min(max(TarsAngle1[LegIndex], (short)pgm_read_word(&cTarsMin1[LegIndex])),
                    (short)pgm_read_word(&cTarsMax1[LegIndex]));
        }
#endif
    }
}



//--------------------------------------------------------------------
// Why are we faulting?
//--------------------------------------------------------------------


void PrintSystemStuff(void)            // Try to see why we fault...
{

}

// BUGBUG:: Move to some library...
//==============================================================================
//    SoundNoTimer - Quick and dirty tone function to try to output a frequency
//            to a speaker for some simple sounds.
//==============================================================================
void SoundNoTimer(uint8_t _pin, unsigned long duration,  unsigned int frequency)
{
#ifdef __AVR__
    volatile uint8_t *pin_port;
    volatile uint8_t pin_mask;
#else
    volatile uint32_t *pin_port;
    volatile uint16_t pin_mask;
#endif
    long toggle_count = 0;
    long lusDelayPerHalfCycle;
    
    // Set the pinMode as OUTPUT
    pinMode(_pin, OUTPUT);

    pin_port = portOutputRegister(digitalPinToPort(_pin));
    pin_mask = digitalPinToBitMask(_pin);
    
    toggle_count = 2 * frequency * duration / 1000;
    lusDelayPerHalfCycle = 1000000L/(frequency * 2);
    
    // if we are using an 8 bit timer, scan through prescalars to find the best fit
    while (toggle_count--) {
        // toggle the pin
        *pin_port ^= pin_mask;
        
        // delay a half cycle
        delayMicroseconds(lusDelayPerHalfCycle);
    }    
    *pin_port &= ~(pin_mask);  // keep pin low after stop

}

void MSound(uint8_t _pin, byte cNotes, ...)
{
    va_list ap;
    unsigned int uDur;
    unsigned int uFreq;
    va_start(ap, cNotes);

    while (cNotes > 0) {
        uDur = va_arg(ap, unsigned int);
        uFreq = va_arg(ap, unsigned int);
        SoundNoTimer(_pin, uDur, uFreq);
        cNotes--;
    }
    va_end(ap);
}

#ifdef OPT_TERMINAL_MONITOR
//==============================================================================
// TerminalMonitor - Simple background task checks to see if the user is asking
//    us to do anything, like update debug levels ore the like.
//==============================================================================
boolean TerminalMonitor(void)
{
    byte szCmdLine[5];  // currently pretty simple command lines...
    int ich;
    int ch;
    // See if we need to output a prompt.
    if (g_fShowDebugPrompt) {
        DBGSerial.println("Arduino Phoenix Monitor");
        DBGSerial.println("D - Toggle debug on or off");
#ifdef OPT_FIND_SERVO_OFFSETS
        DBGSerial.println("O - Enter Servo offset mode");
#endif        
#ifdef OPT_SSC_FORWARDER
        DBGSerial.println("S - SSC Forwarder");
#endif        
        g_fShowDebugPrompt = false;
    }
       
    // First check to see if there is any characters to process.
    if (ich = DBGSerial.available()) {
        ich = 0;
        // For now assume we receive a packet of data from serial monitor, as the user has
        // to click the send button...
        for (ich=0; ich < sizeof(szCmdLine); ich++) {
            ch = DBGSerial.read();        // get the next character
            if ((ch == -1) || ((ch >= 10) && (ch <= 15)))
                break;
             szCmdLine[ich] = ch;
        }
        szCmdLine[ich] = '\0';    // go ahead and null terminate it...
        DBGSerial.print("Serial Cmd Line:");        
        DBGSerial.write(szCmdLine, ich);
        DBGSerial.println("!!!");
        
        // So see what are command is.
        if (ich == 0) {
            g_fShowDebugPrompt = true;
        } else if ((ich == 1) && ((szCmdLine[0] == 'd') || (szCmdLine[0] == 'D'))) {
            g_fDebugOutput = !g_fDebugOutput;
            if (g_fDebugOutput) 
                DBGSerial.println("Debug is on");
            else
                DBGSerial.println("Debug is off");
#ifdef OPT_FIND_SERVO_OFFSETS
        } else if ((ich == 1) && ((szCmdLine[0] == 'o') || (szCmdLine[0] == 'O'))) {
            g_ServoDriver.FindServoOffsets();
#endif
#ifdef OPT_SSC_FORWARDER
        } else if ((ich == 1) && ((szCmdLine[0] == 's') || (szCmdLine[0] == 'S'))) {
            g_ServoDriver.SSCForwarder();
#endif
        }
        
        return true;
    }
    return false;
}
#endif

//--------------------------------------------------------------------
// SmoothControl (From Zenta) -  This function makes the body 
//            rotation and translation much smoother while walking
//--------------------------------------------------------------------
short SmoothControl (short CtrlMoveInp, short CtrlMoveOut, byte CtrlDivider)
{
    if (fWalking)
    {
        if (CtrlMoveOut < (CtrlMoveInp - 4))
              return CtrlMoveOut + abs((CtrlMoveOut - CtrlMoveInp)/CtrlDivider);
        else if (CtrlMoveOut > (CtrlMoveInp + 4))
              return CtrlMoveOut - abs((CtrlMoveOut - CtrlMoveInp)/CtrlDivider);
    }

    return CtrlMoveInp;
}


