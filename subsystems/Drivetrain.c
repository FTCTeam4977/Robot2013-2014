/*
 * This file controls:
 * - Drivetrain open-loop (teleop)
 * - Drivetrain closed-loop (autonomous)
 */

PID drivetrainPID;
PID drivetrainTurnPID;

bool drivetrainGyroDrivestraight = true;
bool drivetrainTurning = false;

int drivetrainMaxSpeed = 100;

void setDrivetrainSpeeds(int left, int right)
{
	if ( left > drivetrainMaxSpeed )
		left = drivetrainMaxSpeed;
	else if ( left < -drivetrainMaxSpeed )
		left = -drivetrainMaxSpeed;
		
	if ( right > drivetrainMaxSpeed )
		right = drivetrainMaxSpeed;
	else if ( right < -drivetrainMaxSpeed )
		right = -drivetrainMaxSpeed;
	nxtDisplayString(1, "%i - %i", left, right);
	nxtDisplayString(5, "MS %i", drivetrainMaxSpeed);
	motor[leftDrive]  = -left;
	motor[rightDrive] = right;
}

void setDrivetrainSetpoint(long setpoint)
{
	drivetrainPID.target = setpoint;
	drivetrainTurning = false;
}

void setDrivetrainSetpoint(long setpoint, int angle)
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

long getDrivetrainDistance()
{
	return ((-nMotorEncoder[leftDrive])+nMotorEncoder[rightDriveEncoder])/2;
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
		nxtDisplayString(0, "%i", drivetrainPID.error);
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
	nMotorEncoder[rightDriveEncoder] = 0;
}

void updateDrivetrain()
{
	updateGyro();
	if ( drivetrainTurning )
	{
		int output = calcPID(drivetrainTurnPID, getGyroAngle());
		output = hlLimit(output, 45, -45);
		setDrivetrainSpeeds(output, -output);
	}
	else
	{
		int output = hlLimit(calcPID(drivetrainPID, getDrivetrainDistance()), 100, -100);
		int left = hlLimit(output, drivetrainMaxSpeed, -drivetrainMaxSpeed);
		int right = hlLimit(output, drivetrainMaxSpeed, -drivetrainMaxSpeed);
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


void synchronousDriveTo(int distance)
{
	setDrivetrainSetpoint(distance);
	int counter = 0;
	while ( !atDrivetrainSetpoint(counter) )
	{
		updateDrivetrain();
	}
}


void synchronousDriveTurn(int angle)
{
	setDrivetrainAngle(angle);
	int counter = 0;
	while ( !atDrivetrainSetpoint(counter) )
	{
		nxtDisplayString(0, "%i", drivetrainTurnPID.error);
		updateDrivetrain();
	}
}
