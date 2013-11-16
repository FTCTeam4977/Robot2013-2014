
void setCollectorRoller(bool state, bool inverse = false)
{
	int speed = (state ? COLLECTOR_ROLLER_MAXIMUM : 0);
	if ( inverse )
		speed = -speed;
	motor[collector] = speed;
}
