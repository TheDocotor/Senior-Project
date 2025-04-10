/*==========================================================
; Program Name: SeniorProject.cpp
;
; Authors: Ryan Enslow & Garrett Nadauld
;
; Description:
; This program provides an auto-leveling system for the RC 2 Ellipsometer
; made by J.A. Woolam. Used in in-situ temperature testing this program
; utilizes a lateral effect sensor and laser to track the deformation of
; the sample being tested. Two Stepper motors X and Y use the data from
; the sensor to adjust the test bed accordingly.
;
; Company: Weber State University
;
;========================================================== */

#include "ClearCore.h"

// Stepper motor set up:
// Options are: ConnectorM0, ConnectorM1, ConnectorM2, or ConnectorM3.

#define motorX ConnectorM0 //motor X is connected to M0 on the clear core
#define motorY ConnectorM1 //motor Y is connected to M1 on the clear core
// This example has built-in functionality to automatically clear motor alerts, 
//  including motor shutdowns. Any uncleared alert will cancel and disallow motion.
// WARNING: enabling automatic alert handling will clear alerts immediately when 
//  encountered and return a motor to a state in which motion is allowed. Before 
//  enabling this functionality, be sure to understand this behavior and ensure 
//  your system will not enter an unsafe state. 
// To enable automatic alert handling, #define HANDLE_ALERTS (1)
// To disable automatic alert handling, #define HANDLE_ALERTS (0)
#define HANDLE_ALERTS (1)
#define inputPin ConnectorIO5
// Define the velocity and acceleration limits to be used for each move
const int32_t velocityLimit = 10000; // 10000pulses per sec
const int32_t accelerationLimit = 10000; //50000 pulses per sec^2


//ADC Set up:
// Defines the bit-depth of the ADC readings (8-bit, 10-bit, or 12-bit)
// Supported adcResolution values are 8, 10, and 12
#define adcResolution 12

int16_t leveling; // State of input switch


// Declares user-defined helper functions.
void MoveDistanceX(int32_t distance);
void MoveDistanceY(int32_t distance);
void HandleAlertsY();
void HandleAlertsX();


/*------------------------------------------------------------------------------
 * Main
 *
 *    Main loop, reads analog input of SUM, deltaX, deltaY
 *	  checks if laser position is within x and y tolerance
 *    calls move motor if out of tolerance, alternates starting with x and y
 *    adjusts every 0.75 second.
 *    
 *
 * Parameters:
 *    None
 *
 * Returns: 
 *    None
 -----------------------------------------------------------------------------*/
 int main() {

    // Set the resolution of the ADC.
    AdcMgr.AdcResolution(adcResolution);
	inputPin.Mode(Connector::INPUT_DIGITAL);
	
	//Motor config
	MotorMgr.MotorInputClocking(MotorManager::CLOCK_RATE_NORMAL);
	MotorMgr.MotorModeSet(MotorManager::MOTOR_ALL, Connector::CPM_MODE_STEP_AND_DIR);
	motorX.HlfbMode(MotorDriver::HLFB_MODE_HAS_BIPOLAR_PWM);
	motorY.HlfbMode(MotorDriver::HLFB_MODE_HAS_BIPOLAR_PWM);
	motorX.HlfbCarrier(MotorDriver::HLFB_CARRIER_482_HZ);
	motorY.HlfbCarrier(MotorDriver::HLFB_CARRIER_482_HZ);
	motorX.VelMax(velocityLimit);
	motorY.VelMax(velocityLimit);
	motorX.AccelMax(accelerationLimit);
	motorY.AccelMax(accelerationLimit);
	motorX.EnableRequest(false);
	motorY.EnableRequest(false);
	

    double inputVoltageSUM, inputVoltageY, inputVoltageX, SumX, SumY, SumSum, voltageX, voltageY, voltageSum = 0.0; //tracks voltages
    bool LevelFlag, ledState = false;	//Used to set level position of first iteration of loop
    double LevelX, LevelY, Xpos, Ypos = 0.0; //used to track the desired voltages when leveling is activated
	double delay = 75; //Sets the amount of time in milliseconds before the next sample
	int num_samples = 10; //Sets the number of samples taken for the average
	int count = 0; //takes 10 samples then computes the average
	const double Xtol = 1.5E-2; // X axis tolerance
	const double Ytol = 1.5E-2; // Y axis tolerance
	const double deltaY = 2E-4; // Steps for smallest delta voltage
	const double deltaX = 2E-4; // Steps for smallest delta voltage
	
    int16_t adcSUM, adcY, adcX = 0; //adc read values
 
    while (true) {
		
		//update the leveling switch state
		leveling = inputPin.State(); 
		
        // Read the analog input (A-9 through A-12 may be configured as analog inputs).
		
        adcSUM = ConnectorA12.State();
        // Convert the reading to a voltage.
        voltageSum = 10.0 * adcSUM / ((1 << adcResolution) - 1);
        
        adcY = ConnectorA10.State();
        // Convert the reading to a voltage.
        voltageY = 10.0 * adcY / ((1 << adcResolution) - 1);
        
        adcX = ConnectorA11.State();
        // Convert the reading to a voltage.
        voltageX = 10.0 * adcX / ((1 << adcResolution) - 1);
		
		//Collect 10 voltage samples for Sum, X, and Y
		SumX += voltageX;
		SumY += voltageY;
		SumSum += voltageSum;
		
		if(count == num_samples)
		{
			//Compute the average for each voltage and set sum back to zero
			inputVoltageX = SumX/num_samples;
			inputVoltageY = SumY/num_samples;
			inputVoltageSUM =  SumSum/num_samples;
			SumY = 0;
			SumX = 0;
			SumSum = 0;
			count = 0;

			if (leveling) 
			{	//Once switch has been set to the on position the bed is level, enter automated leveling state
				
				//enable motors when leveling, this will disable manual adjustments and turn on motors.
				motorX.EnableRequest(true);
				motorY.EnableRequest(true);
				
				Xpos = inputVoltageX;	//New laser position for X
				Ypos = inputVoltageY;	//New laser position for Y
				
				if(inputVoltageSUM < 2.5) //Check if laser is still on the sensor, if not don't adjust and blink connector LED
				{
					ConnectorLed.State(ledState);
					ledState = !ledState;
				}
				
				
				else if (LevelFlag==false) 
				{	//If level flag is false create level position for reference to new level data and set flag to true
					LevelX = inputVoltageX;	//Set LevelX sensor position
					LevelY = inputVoltageY;	//Set LevelY sensor position
					LevelFlag = true; //set flag to true as to not rewrite the leveled voltages
				}

				else 
				{
					//Make X and Y adjustments if new x or y position is not within tolerance of the leveled values
					
					if ((Xpos > (LevelX + Xtol)) || (Xpos < (LevelX - Xtol))) 
					{ // If Xpos is greater than LevelX + Xtol or less than LevelX - Xtol
						MoveDistanceX(int32_t((LevelX - Xpos) / deltaX));
					} 
					
					if ((Ypos > (LevelY + Ytol))|| Ypos < (LevelY - Ytol)) 
					{ // If Ypos is greater than LevelY + Ytol or less than LevelY - Ytol
						MoveDistanceY(int32_t((Ypos - LevelY) / deltaY));
					} 
				
				} 
			}
	
			else 
			{
				LevelFlag = false; //If switch is off reset LevelFlag
				
				//Disable motors to allow for manual adjustment
				motorX.EnableRequest(false);
				motorY.EnableRequest(false);
			} 
		}
		count += 1;			//increase count to take 10 samples
		Delay_ms(delay);		// Wait a .075 second before the next reading.
	}
}
 



 
/*------------------------------------------------------------------------------
 * MoveDistanceX
 *
 *    Command "distance" number of step pulses away from the current position
 *    Returns when HLFB asserts (indicating the motorX has reached the commanded
 *    position)
 *
 * Parameters:
 *    int distance  - The distance, in step pulses, to move
 *
 * Returns: True/False depending on whether the move was successfully triggered.
 -------------------------------------------------------------------------------*/

void MoveDistanceX(int32_t distance) {
    // Check if a motorX alert is currently preventing motion
    // Clear alert if configured to do so 
     if (motorX.StatusReg().bit.AlertsPresent) 
	 {
         if(HANDLE_ALERTS)
		 {
             HandleAlertsX();
         } 
     }
 
    // Command the move of incremental distance
    motorX.Move(distance);
 
    // Waits for HLFB to assert (signaling the move has successfully completed)
    while ( (!motorX.StepsComplete() || motorX.HlfbState() != MotorDriver::HLFB_ASSERTED) &&
            !motorX.StatusReg().bit.AlertsPresent) {
        continue;
    }
   
}

void MoveDistanceY(int32_t distance) {
    // Check if a motorY alert is currently preventing motion
    // Clear alert if configured to do so 
     if (motorY.StatusReg().bit.AlertsPresent) 
	 {
		if(HANDLE_ALERTS)
		{
			HandleAlertsY();
        } 
    }
 
    // Command the move of incremental distance
    motorY.Move(distance);
 
    // Waits for HLFB to assert (signaling the move has successfully completed)
    while ( (!motorY.StepsComplete() || motorY.HlfbState() != MotorDriver::HLFB_ASSERTED) &&
            !motorY.StatusReg().bit.AlertsPresent) {
        continue;
    }
}

/*------------------------------------------------------------------------------
 * HandleAlerts
 *
 *    If a motor alert is present disable motor and wait for alert to clear
 *	  then re-enable the motor before making adjustment.
 *
 * Parameters:
 *    None
 *
 * Returns: Nothing
 -------------------------------------------------------------------------------*/
void HandleAlertsX(){
	if(motorX.AlertReg().bit.MotorFaulted){
		motorX.EnableRequest(false);
		Delay_ms(10);
		motorX.EnableRequest(true);
	}
	motorX.ClearAlerts();
}

void HandleAlertsY(){
	if(motorY.AlertReg().bit.MotorFaulted){
		motorY.EnableRequest(false);
		Delay_ms(10);
		motorY.EnableRequest(true);
	}
	motorY.ClearAlerts();
}