/*
 * This file controls:
 * - Drivetrain open-loop (teleop)
 * - Drivetrain closed-loop (autonomous)
 */

PID drivetrainPID;
PID drivetrainTurnPID;

bool drivetrainGyroDrivestraight = true;
bool drivetrainTurning = false;

// allows for "instant" zeroing... no waiting for an i2c packet
int drivetrainZeroDistance = 0;

void setDrivetrainSpeeds(int left, int right)
{
	motor[leftDrive]  = -left;
	motor[rightDrive] = right;
}

void setDrivetrainSetpoint(int setpoint)
{
	drivetrainPID.target = setpoint;
	drivetrainTurning = false;
}

void setDrivetrainSetpoint(int setpoint, int angle)
{
	drivetrainPID.target = setpoint;
	drivetrainTurnPID.target = angle;
	drivetrainTurning = false;
}

void setDrivetrainAngle(int angle)
{
	drivetrainTurnPID.target = angle;
	drivetrainTurning = true;
}

int getDrivetrainDistance()
{
	return (-nMotorEncoder[leftDrive]);
}

bool atDrivetrainSetpoint(int &counter)
{
	if ( drivetrainTurning )
	{
		if ( abs(drivetrainTurnPID.error) < DRIVETRAIN_TURN_STABLE_ERROR )
			counter++;
		else
			counter = 0;
		return counter > DRIVETRAIN_LOOPS_STABLE_REQUIRED;
	}
	else
	{
		if ( abs(drivetrainPID.error) < DRIVETRAIN_STABLE_ERROR )
			counter++;
		else
			counter = 0;
		return counter > DRIVETRAIN_LOOPS_STABLE_REQUIRED;
	}
}

void resetDrivetrainDistance()
{
	nMotorEncoder[leftDrive] = 0;
}

void updateDrivetrain()
{
	if ( drivetrainTurning )
	{
		int output = calcPID(drivetrainTurnPID, getGyroAngle());
		output = hlLimit(output, 45, -45);
		setDrivetrainSpeeds(output, -output);
	}
	else
	{
		int output = hlLimit(calcPID(drivetrainPID, getDrivetrainDistance()), 100, -100);
		int left = output;
		int right = output;
		if ( drivetrainGyroDrivestraight )
		{
			drivetrainTurnPID.error = drivetrainTurnPID.target-getGyroAngle();
			int turnAmt = drivetrainTurnPID.error*DRIVETRAIN_ANGLECORRECT_CONSTANT;
			nxtDisplayString(2, "%i %i", turnAmt, DRIVETRAIN_ANGLECORRECT_CONSTANT);
			left += turnAmt;
			right -= turnAmt;
		}
		setDrivetrainSpeeds(left, right);
	}
}

void drivetrainInit()
{
	initPID(drivetrainPID, DRIVETRAIN_P_CONSTANT, DRIVETRAIN_I_CONSTANT, DRIVETRAIN_D_CONSTANT);
	initPID(drivetrainTurnPID, DRIVETRAIN_TURN_P_CONSTANT, DRIVETRAIN_TURN_I_CONSTANT, DRIVETRAIN_TURN_D_CONSTANT);
	resetDrivetrainDistance();
}
