/*
 * This file controls:
 * - Winch open-loop (teleop)
 * - Winch closed-loop (autonomous/teleop)
 */

void setWinchMotor(int speed)
{
	motor[winch]=speed;
}
