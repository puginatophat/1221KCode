using namespace vex;

extern brain Brain;

using signature = vision::signature;

// VEXcode devices
extern signature Vision18__RED_GOAL;
extern signature Vision18__BLUE_GOAL;
extern signature Vision18__SIG_3;
extern signature Vision18__SIG_4;
extern signature Vision18__SIG_5;
extern signature Vision18__SIG_6;
extern signature Vision18__SIG_7;
extern vision Vision18;
extern motor topLeft;
extern motor topRight;
extern motor bottomLeft;
extern motor bottomRight;
extern inertial inertialSensor;
extern controller Controller1;
extern motor testerMotor;
extern digital_out pneumaticsLeft;
extern digital_out pneumaticsRight;

/**
 * Used to initialize code/tasks/devices added using tools in VEXcode Pro.
 * 
 * This should be called at the start of your int main function.
 */
void  vexcodeInit( void );