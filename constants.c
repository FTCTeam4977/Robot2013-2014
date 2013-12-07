#define SERVO_MAXIMUM 255
#define SERVO_MINIMUM 0

#define COLLECTOR_ANGLE_DOWN 0
#define COLLECTOR_ANGLE_UP 255
#define COLLECTOR_ROLLER_MAXIMUM 100

#define DRIVETRAIN_P_CONSTANT 0.038
#define DRIVETRAIN_I_CONSTANT 0
#define DRIVETRAIN_D_CONSTANT 0

#define DRIVETRAIN_TURN_P_CONSTANT 2.5
#define DRIVETRAIN_TURN_I_CONSTANT 0
#define DRIVETRAIN_TURN_D_CONSTANT 0

#define DRIVETRAIN_STABLE_ERROR 500
#define DRIVETRAIN_TURN_STABLE_ERROR 5
#define DRIVETRAIN_LOOPS_STABLE_REQUIRED 20

#define DRIVETRAIN_ANGLECORRECT_CONSTANT 2

#define WINCH_P_CONSTANT 0
#define WINCH_I_CONSTANT 0
#define WINCH_D_CONSTANT 0

// Temporary fix for bad encoder port
#define rightDriveEncoder collector
