#pragma config(Hubs,  S1, HTMotor,  HTMotor,  none,     none)
#pragma config(Hubs,  S2, HTMotor,  none,     none,     none)
#pragma config(Sensor, S3,     GYRO,           sensorI2CHiTechnicGyro)
#pragma config(Sensor, S4,     HTIRS2,         sensorI2CCustom)
#pragma config(Motor,  motorC,          autoArm,       tmotorNXT, PIDControl, encoder)
#pragma config(Motor,  mtr_S1_C1_1,     leftDrive,          tmotorTetrix, openLoop)
#pragma config(Motor,  mtr_S1_C1_2,     winch,         tmotorTetrix, openLoop)
#pragma config(Motor,  mtr_S1_C2_1,     flag,          tmotorTetrix, openLoop)
#pragma config(Motor,  mtr_S1_C2_2,     unused2,       tmotorTetrix, openLoop)
#pragma config(Motor,  mtr_S2_C1_1,     rightDrive,         tmotorTetrix, openLoop)
#pragma config(Motor,  mtr_S2_C1_2,     collector,     tmotorTetrix, openLoop)
//*!!Code automatically generated by 'ROBOTC' configuration wizard               !!*//

#include "drivers/hitechnic-irseeker-v2.h"
#include "lib/FTC_PID.c"
#include "lib/FTC_Gyro.c"
#include "constants.c"
#include "subsystems/Drivetrain.c"
#include "JoystickDriver.c"

void throwCube()
{
	while ( nMotorEncoder[autoArm] < 150 )
	{
		motor[autoArm] = 100;
	}
	motor[autoArm] = -100;
	wait1Msec(500);
	motor[autoArm] = 0;
	wait1Msec(200);
}

task main()
{
	initGyro();
	drivetrainInit();
	drivetrainGyroDrivestraight = false;
	bDisplayDiagnostics = false;
	HTIRS2setDSPMode(HTIRS2, DSP_1200);
	while (nPgmTime < 5000 )
	{
		int v1,v2,v3,v4,v5;
		HTIRS2readAllACStrength(HTIRS2, v1,v2,v3,v4,v5);
		nxtDisplayString(0, "%i       ", gyroData.offset);
		nxtDisplayString(1, "%2.4f       ", getGyroAngle());
		nxtDisplayString(2, "%2.4f       ", gyroData.degsec);
		nxtDisplayString(3, "%i       ", v3);
		updateGyro();
	}
	gyroData.deg = 0;
	bDisplayDiagnostics = true;
	waitForStart();
	resetDrivetrainDistance();
	nMotorEncoder[autoArm] = 0;
	int counter = 0;

	// drive out
	setDrivetrainSetpoint(4700);
	drivetrainMaxSpeed = 40;
	bool threwCube = false;
	while ( !atDrivetrainSetpoint(counter) )
	{
		int v1,v2,v3,v4,v5;
		if ( HTIRS2readAllACStrength(HTIRS2, v1,v2,v3,v4,v5) )
		{
			if ( v4 >= 170 && !threwCube ) // at backet
			{
				motor[leftDrive]  = 0;
				motor[rightDrive] = 0;
				throwCube();
				drivetrainMaxSpeed = 100;
				threwCube = true;
			}
			updateDrivetrain();
		}
	}

	if ( !threwCube )
	{
		throwCube();
		drivetrainMaxSpeed = 100;
	}

	resetDrivetrainDistance();

	synchronousDriveTo(2300);

	synchronousDriveTurn(30);
	resetDrivetrainDistance();

	synchronousDriveTo(1000);
	synchronousDriveTurn(60);

	resetDrivetrainDistance();

	synchronousDriveTo(3650);
	synchronousDriveTurn(0);
	resetDrivetrainDistance();

	synchronousDriveTo(-4800);

	motor[autoArm] = 50;
	wait1Msec(200);
}
