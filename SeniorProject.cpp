#include "ClearCore.h"
#include "SysTiming.h"

// Motor ports
#define motorX ConnectorM0
#define motorY ConnectorM1

// ADC resolution and input pin
#define adcResolution 12
#define inputPin ConnectorIO5

// Serial configuration
#define SerialPort ConnectorUsb
#define baudRate 9600

#define HANDLE_ALERTS (1)

// Motion parameters
const int32_t velocityLimit = 10000;
const int32_t accelerationLimit = 10000;

// Tolerances for leveling (voltage)
const double Xvoltol = 1.5E-2;
const double Yvoltol = 1.5E-2;
const double deltavoltolX = 2E-4;
const double deltavoltolY = 2E-4;

// State of input switch
int16_t leveling;

// Function prototypes
void MoveDistanceX(int32_t distance);
void MoveDistanceY(int32_t distance);
void HandleAlertsY();
void HandleAlertsX();


int main() {
    // Setup ADC and input mode
    AdcMgr.AdcResolution(adcResolution);
    inputPin.Mode(Connector::INPUT_DIGITAL);

    // Setup serial
    SerialPort.Mode(Connector::USB_CDC);
    SerialPort.Speed(baudRate);
    SerialPort.PortOpen();
    while (!SerialPort) { continue; }
    SerialPort.SendLine("Time_ms,LevelX,LevelY,inputVoltageX,inputVoltageY,inputVoltageSUM");

    // Motor configuration
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

    // Working variables
    double inputVoltageX = 0.0, inputVoltageY = 0.0, inputVoltageSUM = 0.0;
    double voltageX = 0.0, voltageY = 0.0, voltageSum = 0.0;
    double SumX = 0.0, SumY = 0.0, SumSum = 0.0;
    double LevelX = 0.0, LevelY = 0.0;
    double deltaX = 0.0, deltaY = 0.0, Xtol = 0.0, Ytol = 0.0;
    double Xpos = 0.0, Ypos = 0.0;
    bool LevelFlag = false;
    bool ledState = false;
    int count = 0;

    while (true) {
		
        leveling = inputPin.State();

        // Read ADC values
        int16_t adcSUM = ConnectorA12.State();
        int16_t adcY = ConnectorA10.State();
        int16_t adcX = ConnectorA11.State();

        // Convert to voltages
        voltageSum = 10.0 * adcSUM / ((1 << adcResolution) - 1);
        voltageY = 10.0 * adcY / ((1 << adcResolution) - 1);
        voltageX = 10.0 * adcX / ((1 << adcResolution) - 1);

        // Accumulate for averaging
        SumX += voltageX;
        SumY += voltageY;
        SumSum += voltageSum;

        if (++count == 10) {
            inputVoltageX = SumX / 10.0;
            inputVoltageY = SumY / 10.0;
            inputVoltageSUM = SumSum / 10.0;
            SumX = SumY = SumSum = 0.0;
            count = 0;

            // Log to serial with timestamp
            uint32_t now = Milliseconds();
            char buffer[150];
            snprintf(buffer, sizeof(buffer), "%lu,%.3f,%.3f,%.3f,%.3f,%.3f",
                     now, LevelX, LevelY, inputVoltageX, inputVoltageY, inputVoltageSUM);
            SerialPort.SendLine(buffer);

            if (leveling) {
                motorX.EnableRequest(true);
                motorY.EnableRequest(true);
                Xpos = inputVoltageX;
                Ypos = inputVoltageY;

                if (inputVoltageSUM < 2.5) {
                    ConnectorLed.State(ledState);
                    ledState = !ledState;
                }
                else if (!LevelFlag) {
					//SysTiming.ResetMilliseconds();
                    LevelX = inputVoltageX;
                    LevelY = inputVoltageY;
                    Xtol = Xvoltol;
                    Ytol = Yvoltol;
                    deltaX = deltavoltolX;
                    deltaY = deltavoltolY;
                    LevelFlag = true;
                }
                else {
                    if ((Ypos > LevelY + Ytol) || (Ypos < LevelY - Ytol)) {
                        MoveDistanceY(int32_t((Ypos - LevelY) / deltaY));
                    }
                    if ((Xpos > LevelX + Xtol) || (Xpos < LevelX - Xtol)) {
                        MoveDistanceX(int32_t((LevelX - Xpos) / deltaX));
                    }
                }
            }
            else {
                LevelFlag = false;
                motorX.EnableRequest(false);
                motorY.EnableRequest(false);
            }
        }

        Delay_ms(75);
    }
}

void MoveDistanceX(int32_t distance) {
    if (motorX.StatusReg().bit.AlertsPresent) {
        if(HANDLE_ALERTS) {
	    HandleAlertsX();
        }
    }
    motorX.Move(distance);
    while ((!motorX.StepsComplete() || motorX.HlfbState() != MotorDriver::HLFB_ASSERTED) &&
           !motorX.StatusReg().bit.AlertsPresent) {
        continue;
    }
}

void MoveDistanceY(int32_t distance) {
    if (motorY.StatusReg().bit.AlertsPresent) {
	if(HANDLE_ALERTS) {
	    HandleAlertsY();
	}
    }
    motorY.Move(distance);
    while ((!motorY.StepsComplete() || motorY.HlfbState() != MotorDriver::HLFB_ASSERTED) &&
           !motorY.StatusReg().bit.AlertsPresent) {
        continue;
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
