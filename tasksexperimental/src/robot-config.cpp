#include "vex.h"

using namespace vex;
using signature = vision::signature;
using code = vision::code;

// A global instance of brain used for printing to the V5 Brain screen
brain  Brain;

// VEXcode device constructors
/*vex-vision-config:begin*/
signature Vision18__RED_GOAL = signature (1, 1821, 9955, 5888, -1175, 321, -427, 1, 0);
signature Vision18__BLUE_GOAL = signature (2, -3091, -2047, -2569, 5557, 11189, 8373, 2.7, 0);
signature Vision18__SIG_3 = signature (3, 0, 0, 0, 0, 0, 0, 3, 0);
signature Vision18__SIG_4 = signature (4, 0, 0, 0, 0, 0, 0, 3, 0);
signature Vision18__SIG_5 = signature (5, 0, 0, 0, 0, 0, 0, 3, 0);
signature Vision18__SIG_6 = signature (6, 0, 0, 0, 0, 0, 0, 3, 0);
signature Vision18__SIG_7 = signature (7, 0, 0, 0, 0, 0, 0, 3, 0);
vision Vision18 = vision (PORT18, 50, Vision18__RED_GOAL, Vision18__BLUE_GOAL, Vision18__SIG_3, Vision18__SIG_4, Vision18__SIG_5, Vision18__SIG_6, Vision18__SIG_7);
/*vex-vision-config:end*/
motor topLeft = motor(PORT1, ratio18_1, true);
motor topRight = motor(PORT10, ratio18_1, false);
motor bottomLeft = motor(PORT11, ratio18_1, false);
motor bottomRight = motor(PORT20, ratio18_1, true);
inertial inertialSensor = inertial(PORT4);
controller Controller1 = controller(primary);
motor triggerMotor = motor(PORT2, ratio18_1, false);
motor expansionMotor = motor(PORT19, ratio18_1, false);
motor intakeMotor = motor(PORT5, ratio18_1, false);
motor shootMotor = motor(PORT6, ratio18_1, false);

// VEXcode generated functions
// define variable for remote controller enable/disable
bool RemoteControlCodeEnabled = true;

/**
 * Used to initialize code/tasks/devices added using tools in VEXcode Pro.
 * 
 * This should be called at the start of your int main function.
 */
void vexcodeInit( void ) {
  // nothing to initialize
}