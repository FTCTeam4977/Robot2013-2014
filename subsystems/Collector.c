/*
 * This file controls:
 * - Collector roller
 * - Collector angle
 */

void setCollectorPosition(int position)
{
	servo[collectorAngle1] = position;
	servo[collectorAngle2] = SERVO_MAXIMUM - position;
}

void setCollectorRoller(bool state)
{
	motor[collectorRoller] = (state ? COLLECTOR_ROLLER_MAXIMUM : 0);
}
