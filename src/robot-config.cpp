#include "vex.h"

using namespace vex;
using code = vision::code;

// A global instance of brain used for printing to the V5 Brain screen
brain  Brain;

// VEXcode device constructors
motor topLeft = motor(PORT13, ratio6_1, true);
motor topRight = motor(PORT14, ratio6_1, false);
motor bottomLeft = motor(PORT15, ratio6_1, true);
motor bottomRight = motor(PORT12, ratio6_1, false);
motor shootMotor = motor(PORT6, ratio6_1, false);
motor intakeMotor=motor(PORT5, ratio6_1, false);
controller Controller1 = controller(primary);
digital_out pneumaticsLeft = digital_out(Brain.ThreeWirePort.A);
digital_out pneumaticsRight = digital_out(Brain.ThreeWirePort.B);

// VEXcode generated functions
// define variable for remote controller enable/disable
bool RemoteControlCodeEnabled = true;

/**
 * Used to initialize code/tasks/devices added using tools in VEXcode Pro.
 * 
 * This should be called at the start of your int main function.
 */
void vexcodeInit( void ) 
{
  // nothing to initialize
 
}