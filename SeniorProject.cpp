#include "ClearCore.h"

// Specify which motors to move.
// Options are: ConnectorM0, ConnectorM1, ConnectorM2, or ConnectorM3.
#define motorX ConnectorM0
#define motorY ConnectorM1

// Defines the bit-depth of the ADC readings (8-bit, 10-bit, or 12-bit)
// Supported adcResolution values are 8, 10, and 12
#define adcResolution 12

// Select the baud rate to match the target serial device
#define baudRate 9600
 
// Specify which serial connector to use: ConnectorUsb, ConnectorCOM0, or
// ConnectorCOM1
#define SerialPort ConnectorUsb
 
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
 *		 checks if laser position is within x and y tolerance
 *               calls move motor if out of tolerance, alternates starting with x and y
 *               loops every 1 second 
 *    
 *
 * Parameters:
 *    None
 *
 * Returns: 
 *    None
 */
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
	

    double inputVoltageSUM, inputVoltageY, inputVoltageX, deltaX, deltaY, Xtol, Ytol, SumX, SumY, SumSum, voltageX, voltageY, voltageSum = 0.0;
    bool LevelFlag, ledState = false;	//Used to set level position of first iteration of loop
    //bool Xflag = true;	//Used to determine which axis to move first
    double LevelX, LevelY, Xpos, Ypos = 0.0;
	int count = 0;
	//Define constants for PDP90A
	//const double Lx = 10E-3; // Length of x axis in m
	//const double Ly = 10E-3; // Length of y axis in m
	//const double en = 300E-6; // Output noise voltage in Vrms
	const double Xvoltol = 2E-2; // X axis tolerance
	const double Yvoltol = 2E-2; // Y axis tolerance
	const double deltavoltolY = 2E-4; // Steps for smallest delta voltage
	const double deltavoltolX = 2E-4; // Steps for smallest delta voltage
	
    int16_t adcSUM, adcY, adcX = 0;
 
    while (true) {
	
		
        //inputVoltageSUM = 0.0;
        //inputVoltageY = 0.0;
        //inputVoltageX = 0.0;
		leveling = inputPin.State();
        // Read the analog input (A-9 through A-12 may be configured as analog
        // inputs).
        adcSUM = ConnectorA12.State();
        // Convert the reading to a voltage.
        voltageSum = 10.0 * adcSUM / ((1 << adcResolution) - 1);
        
        adcY = ConnectorA10.State();
        // Convert the reading to a voltage.
        voltageY = 10.0 * adcY / ((1 << adcResolution) - 1);
        
        adcX = ConnectorA11.State();
        // Convert the reading to a voltage.
        voltageX = 10.0 * adcX / ((1 << adcResolution) - 1);
		
		SumX += voltageX;
		SumY += voltageY;
		SumSum += voltageSum;
		
		if(count == 10)
		{
			inputVoltageX = SumX/10;
			inputVoltageY = SumY/10;
			inputVoltageSUM =  SumSum/10;
			SumY = 0;
			SumX = 0;
			SumSum = 0;
			count = 0;

			if (leveling) 
			{	//Once switch has been set to the on position the bed is level, enter automated leveling state
				
				motorX.EnableRequest(true);
				motorY.EnableRequest(true);
				Xpos = inputVoltageX;	//New laser position for X
				Ypos = inputVoltageY;	//New laser position for Y
				
				if(inputVoltageSUM < 2.5)
				{
					ConnectorLed.State(ledState);
					ledState = !ledState;
				}
				
				
				else if (LevelFlag==false) 
				{	//If level flag is false calibrate level position to be compared to new level data and set flag to true
					LevelX = inputVoltageX;	//Set LevelX sensor position
					LevelY = inputVoltageY;	//Set LevelY sensor position
					Xtol = Xvoltol;
					Ytol = Yvoltol;
					deltaX = deltavoltolX;
					deltaY = deltavoltolY;
					LevelFlag = true;
				}

				else 
				{ // If Xflag is true move X axis first
					
                
					
					
					if ((Xpos > (LevelX + Xtol)) || (Xpos < (LevelX - Xtol))) 
					{ // If Xpos is greater than LevelX + Xtol
						MoveDistanceX(int32_t((LevelX - Xpos) / deltaX));
					} 
					
					if ((Ypos > (LevelY + Ytol))|| Ypos < (LevelY - Ytol)) 
					{ // If Ypos is greater than LevelY + Ytol
						MoveDistanceY(int32_t((Ypos - LevelY) / deltaY));
					} 
				
				} 
				
	/*			else 
				{ // If Xflag is false move Y axis first
					if ((Ypos > (LevelY + Ytol))|| Ypos < (LevelY - Ytol)) 
					{ // If Ypos is greater than LevelY + Ytol
					   MoveDistanceY( int32_t((LevelY - Ypos) / delta));
					}   
				
					if ((Xpos > (LevelX + Xtol)) || (Xpos < (LevelX - Xtol))) 
					{ // If Xpos is greater than LevelX + Xtol
						MoveDistanceX((LevelX - Xpos) / delta);
					} 
				}
				Xflag = !Xflag; // Switch flag for next iteration
	*/		}
	
			else 
			{
				LevelFlag = false; //If switch is off reset LevelFlag
				motorX.EnableRequest(false);
				motorY.EnableRequest(false);
			} 
		}
		count += 1;
		Delay_ms(75);		// Wait a .05 second before the next reading. 50
	}
}
 



 
/*------------------------------------------------------------------------------
 * MoveDistanceX
 *
 *    Command "distance" number of step pulses away from the current position
 *    Prints the move status to the USB serial port
 *    Returns when HLFB asserts (indicating the motorX has reached the commanded
 *    position)
 *
 * Parameters:
 *    int distance  - The distance, in step pulses, to move
 *
 * Returns: True/False depending on whether the move was successfully triggered.
 */
void MoveDistanceX(int32_t distance) {
    // Check if a motorX alert is currently preventing motion
    // Clear alert if configured to do so 
     if (motorX.StatusReg().bit.AlertsPresent) {
    //     SerialPort.SendLine("MotorX alert detected.");       
    //     PrintAlerts();
         if(HANDLE_ALERTS){
             HandleAlertsX();
         } //else {
    //         SerialPort.SendLine("Enable automatic alert handling by setting HANDLE_ALERTS to 1.");
    //     }
    //     SerialPort.SendLine("Move canceled.");      
    //     SerialPort.SendLine();
    //     return false;
     }
 
    // SerialPort.Send("Moving distance: ");
    // SerialPort.SendLine(distance);
 
    // Command the move of incremental distance
    motorX.Move(distance);
 
    // Waits for HLFB to assert (signaling the move has successfully completed)
    // SerialPort.SendLine("Moving.. Waiting for HLFB");
    while ( (!motorX.StepsComplete() || motorX.HlfbState() != MotorDriver::HLFB_ASSERTED) &&
            !motorX.StatusReg().bit.AlertsPresent) {
        continue;
    }
    // // Check if motorX alert occurred during move
    // // Clear alert if configured to do so 
    // if (motorX.StatusReg().bit.AlertsPresent) {
    //     SerialPort.SendLine("MotorX alert detected.");       
    //     PrintAlerts();
    //     if(HANDLE_ALERTS){
    //         HandleAlerts();
    //     } else {
    //         SerialPort.SendLine("Enable automatic fault handling by setting HANDLE_ALERTS to 1.");
    //     }
    //     SerialPort.SendLine("Motion may not have completed as expected. Proceed with caution.");
    //     SerialPort.SendLine();
    //     return false;
    // } else {
    //     SerialPort.SendLine("Move Done");
    //     return true;
    // }
}

void MoveDistanceY(int32_t distance) {
    // Check if a motorY alert is currently preventing motion
    // Clear alert if configured to do so 
     if (motorY.StatusReg().bit.AlertsPresent) {
    //     SerialPort.SendLine("MotorY alert detected.");       
    //     PrintAlerts();
     if(HANDLE_ALERTS){
		         HandleAlertsY();
         } 
    //         SerialPort.SendLine("Enable automatic alert handling by setting HANDLE_ALERTS to 1.");
    //     }
    //     SerialPort.SendLine("Move canceled.");      
    //     SerialPort.SendLine();
    //     return false;
     }
 
    // SerialPort.Send("Moving distance: ");
    // SerialPort.SendLine(distance);
 
    // Command the move of incremental distance
    motorY.Move(distance);
 
    // Waits for HLFB to assert (signaling the move has successfully completed)
    // SerialPort.SendLine("Moving.. Waiting for HLFB");
    while ( (!motorY.StepsComplete() || motorY.HlfbState() != MotorDriver::HLFB_ASSERTED) &&
            !motorY.StatusReg().bit.AlertsPresent) {
        continue;
    }
    /*
    // Check if motorY alert occurred during move
    // Clear alert if configured to do so */
    if (motorY.StatusReg().bit.AlertsPresent) {
      //  SerialPort.SendLine("MotorY alert detected.");       
    //    PrintAlerts();
        if(HANDLE_ALERTS){
            HandleAlertsY();
    //    } else {
    //        SerialPort.SendLine("Enable automatic fault handling by setting HANDLE_ALERTS to 1.");
        }
    //    SerialPort.SendLine("Motion may not have completed as expected. Proceed with caution.");
     //   SerialPort.SendLine();
     //   return false;
		}
    }

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