#include "main.h"
#include "okapi/api.hpp"

using namespace okapi;


//controller stuff
Controller masterController;
ControllerDigital rampUp=ControllerDigital::L1;
ControllerDigital rampDown=ControllerDigital::L2;

ControllerDigital TakeIn=ControllerDigital::R1;
ControllerDigital TakeOut=ControllerDigital::R2;
//motor stuff
MotorGroup LeftDrive={-3,11};
MotorGroup RightDrive={-9,10};
auto drive = ChassisControllerFactory::create(
 LeftDrive,RightDrive,
 AbstractMotor::gearset::green
);
MotorGroup ramp({1,-5});

MotorGroup take({14,-15});

//auton stuff
auto auton = AsyncControllerFactory::motionProfile(
  10,
  2,
  1,
  drive
);


//other variables
int rampSpeed=100;
int takeSpeed=200;
bool constantIntake=false;
bool checking=false;

//functions for my sanity
bool Dinput(ControllerDigital ibutton){
 return masterController.getDigital(ibutton);
}



/**
 * A callback function for LLEMU's center button.
 *
 * When this callback is fired, it will toggle line 2 of the LCD text between
 * "I was pressed!" and nothing.
 */
void on_center_button() {
	static bool pressed = false;
	pressed = !pressed;
	if (pressed) {
		pros::lcd::set_text(2, "I was pressed!");
	} else {
		pros::lcd::clear_line(2);
	}

}



/**
 * Runs initialization code. This occurs as soon as the program is started.
 *
 * All other competition modes are blocked by initialize; it is recommended
 * to keep execution time for this mode under a few seconds.
 */
void initialize() {
	pros::lcd::initialize();
	pros::lcd::set_text(1, "Hello PROS User!");
	pros::lcd::register_btn1_cb(on_center_button);
	//odometer initialization
	LeftDrive.tarePosition();
  LeftDrive.setEncoderUnits(AbstractMotor::encoderUnits::rotations);

	RightDrive.tarePosition();
	RightDrive.setEncoderUnits(AbstractMotor::encoderUnits::rotations);

  take.tarePosition();
  take.setEncoderUnits(AbstractMotor::encoderUnits::rotations);

  ramp.setEncoderUnits(AbstractMotor::encoderUnits::rotations);
  //auton stuff

}

/**
 * Runs while the robot is in the disabled state of Field Management System or
 * the VEX Competition Switch, following either autonomous or opcontrol. When
 * the robot is enabled, this task will exit.
 */
void disabled() {}

/**
 * Runs after initialize(), and before autonomous when connected to the Field
 * Management System or the VEX Competition Switch. This is intended for
 * competition-specific initialization routines, such as an autonomous selector
 * on the LCD.
 *
 * This task will exit when the robot is enabled and autonomous or opcontrol
 * starts.
 */
void competition_initialize() {}

/**
 * Runs the user autonomous code. This function will be started in its own task
 * with the default priority and stack size whenever the robot is enabled via
 * the Field Management System or the VEX Competition Switch in the autonomous
 * mode. Alternatively, this function may be called in initialize or opcontrol
 * for non-competition testing purposes.
 *
 * If the robot is disabled or communications is lost, the autonomous task
 * will be stopped. Re-enabling the robot will restart the task, not re-start it
 * from where it left off.
 */
void autonomous() {
 	pros::lcd::print(0,"autonomous running");
  auton.generatePath({Point{10_in,0_in,0_deg}}, "red_a_1");
  auton.generatePath({Point{0_in,0_in,0_deg}}, "red_a_2");
  auton.setTarget("red_a_1",true);
  auton.waitUntilSettled();
  auton.setTarget("red_a_2",false);
  auton.waitUntilSettled();

}

/**
 * Runs the operator control code. This function will be started in its own task
 * with the default priority and stack size whenever the robot is enabled via
 * the Field Management System or the VEX Competition Switch in the operator
 * control mode.
 *
 * If no competition control is connected, this function will run immediately
 * following initialize().
 *
 * If the robot is disabled or communications is lost, the
 * operator control task will be stopped. Re-enabling the robot will restart the
 * task, not resume it from where it left off.
 */

//vars at the top
void opcontrol() {
//initialization stuff
 ramp.setGearing(AbstractMotor::gearset::red);
 take.setGearing(AbstractMotor::gearset::green);

	while (true) {

		//UPDATE VERSION EVERY TIME PROGRAM IS CHANGED SO UPLOAD ISSUES ARE KNOWN!!!
   	pros::lcd::print(0,"Drive 0.7.4 Dev");
		//driving
		drive.arcade(masterController.getAnalog(ControllerAnalog::leftY),
						 masterController.getAnalog(ControllerAnalog::rightX));
	  //moving the ramp
		if(Dinput(rampUp)){
			ramp.moveVelocity(rampSpeed);
		}
		else if(Dinput(rampDown)){
			ramp.moveVelocity(-rampSpeed);
		}
		else{
			ramp.moveVelocity(0);
		}
		pros::delay(20);
		//intake/outtake
		if(Dinput(TakeIn)||Dinput(TakeOut)){
			pros::delay(100);
			if(Dinput(TakeIn)&&Dinput(TakeOut)&&!checking){
					constantIntake=!constantIntake;
          checking=true;
			}
		else{
      checking=false;
    }
	}
  if((Dinput(TakeIn)||constantIntake)&&!Dinput(TakeOut)){
      take.moveVelocity(takeSpeed);
  }
  else if(Dinput(TakeOut)&&!Dinput(TakeIn)&&!constantIntake){
      take.moveVelocity(-takeSpeed);
  }
  else{
      take.moveVelocity(0);
  }

		pros::delay(20);
	}

}
