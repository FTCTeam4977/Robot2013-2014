#pragma config(Hubs,  S1, HTMotor,  HTServo,  none,     none)
#pragma config(Hubs,  S2, HTMotor,  HTMotor,  none,     none)
#pragma config(Sensor, S3,     GYRO,           sensorI2CHiTechnicGyro)
#pragma config(Motor,  mtr_S1_C1_1,     leftDrive,     tmotorTetrix, openLoop, encoder)
#pragma config(Motor,  mtr_S1_C1_2,     winchDrive,         tmotorTetrix, openLoop)
#pragma config(Motor,  mtr_S2_C1_1,     rightDrive,    tmotorTetrix, openLoop)
#pragma config(Motor,  mtr_S2_C1_2,     motorG,        tmotorTetrix, openLoop)
#pragma config(Motor,  mtr_S2_C2_1,     motorH,        tmotorTetrix, openLoop)
#pragma config(Motor,  mtr_S2_C2_2,     motorI,        tmotorTetrix, openLoop)
#pragma config(Servo,  srvo_S1_C2_1,    servo1,               tServoNone)
#pragma config(Servo,  srvo_S1_C2_2,    servo2,               tServoNone)
#pragma config(Servo,  srvo_S1_C2_3,    servo3,               tServoNone)
#pragma config(Servo,  srvo_S1_C2_4,    servo4,               tServoNone)
#pragma config(Servo,  srvo_S1_C2_5,    servo5,               tServoNone)
#pragma config(Servo,  srvo_S1_C2_6,    servo6,               tServoNone)
//*!!Code automatically generated by 'ROBOTC' configuration wizard               !!*//

#include "lib/FTC_PID.c"
#include "lib/FTC_Gyro.c"
#include "constants.c"
#include "subsystems/Drivetrain.c"

task main()
{
	initGyro();
	drivetrainInit();
	int counter = 0;
	int autonState = 0;
	while(true)
	{
		switch (autonState)
		{
			case 0:
				setDrivetrainSetpoint(1000);
				if (atDrivetrainSetpoint(counter))
				{
					counter = 0;
					autonState++;
				}
				break;

			case 1:
				setDrivetrainAngle(90);
				if (atDrivetrainSetpoint(counter))
				{
					resetDrivetrainDistance();
					counter = 0;
					autonState++;
					eraseDisplay();

				}
				break;

			case 2:
				setDrivetrainSetpoint(1000);
				if (atDrivetrainSetpoint(counter))
				{
					counter = 0;
					autonState++;
				}
				break;
		  case 3:
				setDrivetrainAngle(90*2);
				if (atDrivetrainSetpoint(counter))
				{
					resetDrivetrainDistance();
					counter = 0;
					autonState++;
				}
				break;

			case 4:
				setDrivetrainSetpoint(1000);
				if (atDrivetrainSetpoint(counter))
				{
					counter = 0;
					autonState++;
				}
				break;

			case 5:
				setDrivetrainAngle(90*3);
				if (atDrivetrainSetpoint(counter))
				{
					resetDrivetrainDistance();
					counter = 0;
					autonState++;
				}
				break;
			case 6:
				setDrivetrainSetpoint(1000);
				if (atDrivetrainSetpoint(counter))
				{
					counter = 0;
					autonState++;
				}
				break;
		}
		updateDrivetrain();
		nxtDisplayString(0, "%i", getDrivetrainDistance());
		nxtDisplayString(1, "%2.4f", getGyroAngle());
		nxtDisplayString(4, "State: %i", autonState);
		updateGyro();
	}
}
