#include "main.h"
#include "okapi/api.hpp"
#include "lib7842/api.hpp"

using namespace okapi;
using namespace lib7842;

//motor constants(if odom is used)
const int FrontLeft=18;
const int FrontRight=-20;
const int rampPort=-19;
//controller stuff
Controller masterController;
ControllerDigital rampDown{ControllerDigital::L2};
ControllerDigital rampUp{ControllerDigital::L1};

ControllerDigital TakeIn{ControllerDigital::R1};
ControllerDigital TakeOut{ControllerDigital::R2};
//Scale for auton
ChassisScales Scales{{3.25_in,10.25_in},imev5GreenTPR};

//motor stuff
MotorGroup LeftDrive{FrontLeft,17};
MotorGroup RightDrive{FrontRight,-11};
std::shared_ptr<Motor> ramp=std::make_shared<Motor>(rampPort);
MotorGroup take{13,-14};

//pid & odom stuff for when it's time to test PID auton
//odom(Change the values when bot is built)
/*
IntegratedEncoder left(FrontLeft,false);
ADIEncoder mid(4,3);
IntegratedEncoder right(FrontRight,false);*/


//Pid(Only use a PD controller), will probably delete
//auto PID= IterativeControllerFactory::posPID(0.001, 0.0, 0.000);
//IterativePosPIDController::Gains pos{.002,.0000,.00003,.00};//<-position(KU:unknown,PU:unknown)
//IterativePosPIDController::Gains angle={.00/*30*/,.0000,.0000,.00};/*<-keeping it straight(don't use yet)*/
//IterativePosPIDController::Gains turn={.0035,.0000,.00015,.00};/*<-turning(don't use yet)*/

auto drive= ChassisControllerBuilder()
  .withMotors(LeftDrive,RightDrive)
  .withSensors(IntegratedEncoder(FrontLeft),IntegratedEncoder(FrontRight)) //<-encoders
  .withDimensions(AbstractMotor::gearset::green,Scales)
  .withClosedLoopControllerTimeUtil(25,5,250_ms)
  //.withGains(pos,turn,angle)
  .build();
//auton select
std::shared_ptr<GUI::Screen> screen;
GUI::Selector* selector;
//Tray Pid

//use acetousk pid test for ramp stuff
std::shared_ptr<AsyncPosPIDController> tray=std::make_shared<AsyncPosPIDController>(
  ramp->getEncoder(),
  ramp,
  TimeUtilFactory::withSettledUtilParams(),
  0.004,
  0.0,
  0,
  0.0
);


//other variables

const int rampTop=690;
const int rampBottom=-5;

const int takeSpeed(200);
const double driveSpeed(0.8);//<-percentage, 1=100%
bool constantIntake(false);
bool checking(false);

//functions for my sanity
bool Dinput(ControllerDigital ibutton){
 return masterController.getDigital(ibutton);
}


void auton(int mult=1){
  //Move foward & grab cubes
    drive->moveDistanceAsync(45_in);//<-move 2 sqaures
    take.moveVelocity(takeSpeed);
    drive->waitUntilSettled();
    take.moveVelocity(0);
    //return to start
    drive->moveDistance(-45_in);//<-move back 2 squares
    drive->waitUntilSettled();
    //turn
    drive->turnAngle(mult*90_deg);
    drive->waitUntilSettled();
    //move to scoring & stack
    drive->moveDistance(12_in);//<-move half a square
    drive->waitUntilSettled();
    tray->setTarget(rampTop);
    tray->waitUntilSettled();
    tray->setTarget(rampBottom);
    tray->waitUntilSettled();

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
  take.setGearing(AbstractMotor::gearset::green);

  ramp->tarePosition();
  ramp->setGearing(AbstractMotor::gearset::red);
  ramp->setEncoderUnits(AbstractMotor::encoderUnits::degrees);
  tray->startThread();
  tray->flipDisable(false);

  //auton stuff
  screen = std::make_shared<GUI::Screen>( lv_scr_act(), LV_COLOR_MAKE(153, 157, 161) );
	screen->startTask("screenTask");

	selector = dynamic_cast<GUI::Selector*>(
    	&screen->makePage<GUI::Selector>("Selector")
			.button("Default", [&]() {
         drive->moveDistance(-12_in);/*<-move half a square(push into small zone)*/
         drive->moveDistance(12_in);/*<-move back*/
       })
      .button("Red", [&]() { auton(-1); })
      .button("Blue", [&]() { auton(); })
      .button("Skills", [&]() {
         auton(-1);/*<-runs auton for red side*/
         drive->moveDistance(-60_in);//move half a square back
         drive->turnAngle(90_deg);
         drive->moveDistance(140_in);
         drive->turnAngle(-90_deg);
         drive->moveDistance(48_in);
         drive->turnAngle(-90_deg);
         auton();/*<runs auton for blue side*/
       })
      .build()
    );
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
    selector->run();
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

	while (true) {

		//UPDATE VERSION EVERY TIME PROGRAM IS CHANGED SO UPLOAD ISSUES ARE KNOWN!!!
   	pros::lcd::print(0,"Drive 1.0 Dev");
		//driving
    double left, right,
    turn(masterController.getAnalog(ControllerAnalog::leftX)),
    forward(masterController.getAnalog(ControllerAnalog::rightY));


    if(std::abs(forward)<=0.1){
        left=turn;
        right=-turn;
    }
    else{
      left=forward+(0.75*turn);
      right=forward-(0.75*turn);
}



    drive->getModel()->tank(left*driveSpeed,right*driveSpeed,.1);
	  //moving the ramp
		if(Dinput(rampUp)){
      tray->setTarget(rampTop);
      while(Dinput(rampUp)){
        pros::delay(20);
      }
		}
		else if(Dinput(rampDown)){

      tray->flipDisable(true);
      ramp->moveVelocity(-100);
      while(Dinput(rampDown)){
        pros::delay(20);
      }
      ramp->moveVelocity(0);
      tray->flipDisable(false);
      tray->setTarget(rampBottom
      );

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
  else if(Dinput(TakeOut)&&!constantIntake){
      take.moveVelocity(-takeSpeed);
  }
  else{
      take.moveVelocity(0);
  }

//auton button(COMMENT OUT FOR COMPS)
if(Dinput(ControllerDigital::A)){
  autonomous();
}
		pros::delay(20);
	}

}
