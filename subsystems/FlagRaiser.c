/*
 * This file controls:
 * - Flag raiser
 */

void setFlagRaiserMotor(bool state, bool slow = false)
{
	motor[flag] = (state?(slow?40:100):0);
}