#include "main.h"
#include "lemlib/api.hpp" // IWYU pragma: keep
#include "lunarselector/selector.h"
#include "definitions.h"

// controller
pros::Controller controller(pros::E_CONTROLLER_MASTER);

// motor groups
pros::MotorGroup leftMotors({-8, -9, 10},
                            pros::MotorGearset::blue); // left motor group - ports 3 (reversed), 4, 5 (reversed)
pros::MotorGroup rightMotors({-2, 3, 4}, 
                            pros::MotorGearset::blue); // right motor group - ports 6, 7, 9 (reversed)

// Inertial Sensor on port 10
pros::Imu imu(6);

pros::Motor intake1(-5, pros::MotorGearset::green);
pros::Motor intake2(7, pros::MotorGearset::green);

pros::adi::DigitalOut pisstake('D');
pros::adi::DigitalOut lift('E');
pros::adi::DigitalOut mogo('F');
pros::adi::DigitalOut redirect('G');
pros::adi::DigitalOut stick('H');


// tracking wheels
// horizontal tracking wheel encoder. Rotation sensor, port 20, not reversed
pros::Rotation horizontalEnc(20);
// vertical tracking wheel encoder. Rotation sensor, port 11, reversed
pros::Rotation verticalEnc(-11);
// horizontal tracking wheel. 2.75" diameter, 5.75" offset, back of the robot (negative)
lemlib::TrackingWheel horizontal(&horizontalEnc, lemlib::Omniwheel::NEW_2, -0.5);
// vertical tracking wheel. 2.75" diameter, 2.5" offset, left of the robot (negative)
lemlib::TrackingWheel vertical(&verticalEnc, lemlib::Omniwheel::NEW_275, -2.5);

// drivetrain settings
lemlib::Drivetrain drivetrain(&leftMotors, // left motor group
                              &rightMotors, // right motor group
                              12.5, // 10 inch track width
                              lemlib::Omniwheel::NEW_275, // using new 4" omnis
                              480, // drivetrain rpm is 360
                              2 // horizontal drift is 2. If we had traction wheels, it would have been 8
);

// lateral motion controller
lemlib::ControllerSettings linearController(10, // proportional gain (kP)
                                            0, // integral gain (kI)
                                            0, // derivative gain (kD)
                                            0, // anti windup
                                            0, // small error range, in inches
                                            0, // small error range timeout, in milliseconds
                                            0, // large error range, in inches
                                            0, // large error range timeout, in milliseconds
                                            0 // maximum acceleration (slew)
);

// angular motion controller
lemlib::ControllerSettings angularController(0, // proportional gain (kP)
                                             0, // integral gain (kI)
                                             0, // derivative gain (kD)
                                             0, // anti windup
                                             0, // small error range, in degrees
                                             0, // small error range timeout, in milliseconds
                                             0, // large error range, in degrees
                                             0, // large error range timeout, in milliseconds
                                             0 // maximum acceleration (slew)
);

// sensors for odometry
lemlib::OdomSensors sensors(nullptr, // vertical tracking wheel
                            nullptr, // vertical tracking wheel 2, set to nullptr as we don't have a second one
                            &horizontal, // horizontal tracking wheel
                            nullptr, // horizontal tracking wheel 2, set to nullptr as we don't have a second one
                            &imu // inertial sensor
);

// input curve for throttle input during driver control
lemlib::ExpoDriveCurve throttleCurve(3, // joystick deadband out of 127
                                     10, // minimum output where drivetrain will move out of 127
                                     1.019 // expo curve gain
);

// input curve for steer input during driver control
lemlib::ExpoDriveCurve steerCurve(3, // joystick deadband out of 127
                                  10, // minimum output where drivetrain will move out of 127
                                  1.019 // expo curve gain
);

// create the chassis
lemlib::Chassis chassis(drivetrain, linearController, angularController, sensors, &throttleCurve, &steerCurve);

// Useful functions for auton
void mogoclamp(){
    leftMotors.move(-40);
    rightMotors.move(-40);
    pros::delay(250);
    mogo.set_value(true);
    leftMotors.move(0);
    rightMotors.move(0);
}
void mogoclamplong(){
    leftMotors.move(-50);
    rightMotors.move(-50);
    pros::delay(500);
    mogo.set_value(true);
    leftMotors.move(0);
    rightMotors.move(0);
}
void align(){
    leftMotors.move(30);
    rightMotors.move(30);
    pros::delay(300);
    leftMotors.move(0);
    rightMotors.move(0);
}
void backup(){
    leftMotors.move(-30);
    rightMotors.move(-30);
    pros::delay(300);
    leftMotors.move(0);
    rightMotors.move(0);
}
void backupsmall(){
    leftMotors.move(-30);
    rightMotors.move(-30);
    pros::delay(200);
    leftMotors.move(0);
    rightMotors.move(0);
}
void intakeone() { 
    intake1.move(127); 
    intake2.move(127);
    pros::delay(300);
    leftMotors.move(0);
    rightMotors.move(0);
}
void bottomtop(){
    intake1.move(127); 
    intake2.move(127);
    leftMotors.move(20);
    rightMotors.move(20);
    pros::delay(300);
    leftMotors.move(0);
    rightMotors.move(0);
    leftMotors.move(0);
    rightMotors.move(0);
    backupsmall();
    pisstake.set_value(true);
    intake1.move(127); 
    intake2.move(127);
    align();
    pisstake.set_value(false);
    pros::delay(400);
}

// Path names: 
// Test, Pos Red, Neg Red, Pos Blue, Neg Blue, Skills
void test(){
	// Code here
    chassis.setPose(0, 0, 0);

    // chassis.turnToHeading(90, 100000);

    chassis.moveToPoint(0, 48, 10000);
}
void posRed(){
	// Code here
    chassis.setPose(-52,-60,270);
    // chassis.moveToPoint(-22, -60, 2000, {.earlyExitRange = 2, .forwards = false, .maxSpeed = 127, .minSpeed = 60});
    chassis.moveToPoint(-22, -60, 2000, {.forwards = false, .maxSpeed = 127, .minSpeed = 60});
    chassis.swingToPoint(0, -48, DriveSide::LEFT, 2000, {.forwards = false});
    chassis.moveToPoint(0, -48, 2000, {.forwards = false, .maxSpeed = 127, .minSpeed = 20});
    mogo.set_value(true);
    pros::delay(100);
    chassis.turnToHeading(270, 1500, {.maxSpeed = 127, .minSpeed = 60});
    intake1.move(127);
    intake2.move(127);
    chassis.moveToPoint(-24, -48, 2000, {.forwards = true, .maxSpeed = 127, .minSpeed = 20});
    chassis.turnToPoint(-60, -63, 1500, {.maxSpeed = 127, .minSpeed = 60});
    chassis.moveToPoint(-60, -63, 2000, {.forwards = true, .maxSpeed = 127, .minSpeed = 40});
    bottomtop();
    chassis.moveToPose(-48, -36, 240, 2500,
                       {.forwards = false, .horizontalDrift = 6, .lead = 0.5, .maxSpeed = 127, .minSpeed = 60});
    mogoclamp();
    // chassis.moveToPose(-24, -48, , 1500,
    //                    {.forwards = false, .horizontalDrift = 6, .lead = 0.5, .maxSpeed = 127, .minSpeed = 60});
}
void negRed(){
	// Code here
}
void posBlue(){
	// Code here
}
void negBlue(){
	// Code here
}
void skills(){
    chassis.setPose(-58, 0, 90);
    intake1.move(127);
    intake2.move(127);
    chassis.moveToPose(-24, -24, 135, 2500,
                       {.forwards = true, .horizontalDrift = 6, .lead = 0.4, .maxSpeed = 127, .minSpeed = 60});
    pros::delay(100);
    chassis.turnToHeading(90, 1500, { .maxSpeed = 127, .minSpeed = 60 });
    mogoclamplong();
    chassis.turnToHeading(135, 1500, { .maxSpeed = 127, .minSpeed = 60 });
    chassis.moveToPoint(-24, -48, 2500, {.maxSpeed = 127, .minSpeed = 60});
    pros::delay(100);
    chassis.turnToHeading(270, 1500, { .maxSpeed = 127, .minSpeed = 60 });
    chassis.moveToPoint(-60, -48, 2500, { .maxSpeed = 127, .minSpeed = 60 });
    pros::delay(100);
    chassis.turnToHeading(135, 1500, { .maxSpeed = 127, .minSpeed = 60 });
    chassis.moveToPoint(-48, -60, 2000, { .maxSpeed = 127, .minSpeed = 60 });
    chassis.turnToHeading(90, 1500, { .maxSpeed = 127, .minSpeed = 60 });
    chassis.moveToPose(-60, -60, 45, 1500,
                       {.forwards = false, .horizontalDrift = 6, .lead = 0.5, .maxSpeed = 127, .minSpeed = 60});
    pros::delay(100);
    mogo.set_value(false);
    redirect.set_value(true);
    chassis.moveToPose(0, -60, 90, 3000,
                       {.forwards = true, .horizontalDrift = 6, .lead = 0.3, .maxSpeed = 127, .minSpeed = 60});
    chassis.turnToHeading(180, 1500, { .maxSpeed = 127, .minSpeed = 60 });
    pros::delay(200);
    lift.set_value(true);
    align();
    lift.set_value(false);
    backup();
    redirect.set_value(false);
    chassis.turnToHeading(90, 1500, {.maxSpeed = 127, .minSpeed = 60});
    chassis.moveToPose(24, -48, 90, 1500,
                       {.forwards = false, .horizontalDrift = 6, .lead = 0.4, .maxSpeed = 127, .minSpeed = 60});
    intakeone();
    chassis.turnToHeading(0, 1500, {.maxSpeed = 127, .minSpeed = 60});
    chassis.moveToPoint(24, -20, 2000, { .maxSpeed = 127, .minSpeed = 60 });
    intakeone();
    chassis.turnToPoint(53, -14, 1500, {.forwards = false, .maxSpeed = 127, .minSpeed = 60});
    chassis.moveToPoint(53, -14, 2000, {.forwards = false, .maxSpeed = 127, .minSpeed = 60 });
    chassis.turnToHeading(335, 1500, {.maxSpeed = 127, .minSpeed = 60});
    chassis.tank(-70,-70);
    pros::delay(1200);
}

/**
 * Runs initialization code. This occurs as soon as the program is started.
 *
 * All other competition modes are blocked by initialize; it is recommended
 * to keep execution time for this mode under a few seconds.
 */
void initialize() {
    intake1.set_brake_mode(MOTOR_BRAKE_COAST);
    intake2.set_brake_mode(MOTOR_BRAKE_COAST);
    leftMotors.set_brake_mode(MOTOR_BRAKE_COAST);
    rightMotors.set_brake_mode(MOTOR_BRAKE_COAST);
    // pros::Task select(brainScreen);
    pros::lcd::initialize(); // initialize brain screen
    // while(1){
    //     pros::lcd::print(2, "Intake1 : %f", intake1.get_torque()); 
    //     pros::lcd::print(4, "Intake2 : %f", intake2.get_torque()); 
    // }
    // chassis.calibrate(); // calibrate sensors

    // // the default rate is 50. however, if you need to change the rate, you
    // // can do the following.
    // // lemlib::bufferedStdout().setRate(...);
    // // If you use bluetooth or a wired connection, you will want to have a rate of 10ms

    // // for more information on how the formatting for the loggers
    // // works, refer to the fmtlib docs

    // // thread to for brain screen and position logging
    // pros::Task screenTask([&]() {
    //     while (true) {
    //         // print robot location to the brain screen
    //         pros::lcd::print(0, "X: %f", chassis.getPose().x); // x
    //         pros::lcd::print(1, "Y: %f", chassis.getPose().y); // y
    //         pros::lcd::print(2, "Theta: %f", chassis.getPose().theta); // heading
    //         // log position telemetry
    //         lemlib::telemetrySink()->info("Chassis pose: {}", chassis.getPose());
    //         // delay to save resources
    //         pros::delay(50);
    //     }
    // });
}

/**
 * Runs while the robot is disabled
 */
void disabled() {}

/**
 * runs after initialize if the robot is connected to field control
 */
void competition_initialize() {}

// get a path used for pure pursuit
// this needs to be put outside a function
ASSET(example_txt); // '.' replaced with "_" to make c++ happy

/**
 * Runs during auto
 *
 * This is an example autonomous routine which demonstrates a lot of the features LemLib has to offer
 */
void autonomous() {
    switch(autonState){
		// 6 different paths total
		case 0:
			test(); 
			break;
		case 1: 
			posRed(); 
			break;
		case 2: 
			negRed();  
			break;
		case 3: 
			posBlue(); 
			break;
		case 4: 
			negBlue(); 
			break;
		case 5:
			skills();
			break; 
		default: 
			break;
    }
    // // Move to x: 20 and y: 15, and face heading 90. Timeout set to 4000 ms
    // chassis.moveToPose(20, 15, 90, 4000);
    // // Move to x: 0 and y: 0 and face heading 270, going backwards. Timeout set to 4000ms
    // chassis.moveToPose(0, 0, 270, 4000, {.forwards = false});
    // // cancel the movement after it has traveled 10 inches
    // chassis.waitUntil(10);
    // chassis.cancelMotion();
    // // Turn to face the point x:45, y:-45. Timeout set to 1000
    // // dont turn faster than 60 (out of a maximum of 127)
    // chassis.turnToPoint(45, -45, 1000, {.maxSpeed = 60});
    // // Turn to face a direction of 90º. Timeout set to 1000
    // // will always be faster than 100 (out of a maximum of 127)
    // // also force it to turn clockwise, the long way around
    // chassis.turnToHeading(90, 1000, {.direction = AngularDirection::CW_CLOCKWISE, .minSpeed = 100});
    // // Follow the path in path.txt. Lookahead at 15, Timeout set to 4000
    // // following the path with the back of the robot (forwards = false)
    // // see line 116 to see how to define a path
    // chassis.follow(example_txt, 15, 4000, false);
    // // wait until the chassis has traveled 10 inches. Otherwise the code directly after
    // // the movement will run immediately
    // // Unless its another movement, in which case it will wait
    // chassis.waitUntil(10);
    // pros::lcd::print(4, "Traveled 10 inches during pure pursuit!");
    // // wait until the movement is done
    // chassis.waitUntilDone();
    // pros::lcd::print(4, "pure pursuit finished!");
}

void opcontrol() {
    // Variables
    bool mogoc = false;
    bool stickc = false;
    bool redirectc = false;
    bool liftc = false;
    bool pisstakec = false;
    int leftY, rightX;
    // std::vector<bool> cont = getAll(ALLBUTTONS);
    while (1) {
        leftY = controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);
        rightX = controller.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X);

        chassis.arcade(leftY, rightX);

        // Pistons
        if(controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_R2)) { stickc = !stickc; stick.set_value(stickc); }
        if(controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_L2)) { liftc = !liftc; lift.set_value(liftc); } 
        if(controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_DOWN)) { pisstakec = !pisstakec; pisstake.set_value(pisstakec); } 
        if(controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_Y)) { mogoc = !mogoc; mogo.set_value(mogoc); }
        if(controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_RIGHT)) { redirectc = !redirectc; redirect.set_value(redirectc); }

        // Motors
        if(redirectc == false){
            if(controller.get_digital(pros::E_CONTROLLER_DIGITAL_L1)) { intake1.move(127); intake2.move(127);}
            else if(controller.get_digital(pros::E_CONTROLLER_DIGITAL_R1)) { intake1.move(-127); intake2.move(-127);}
            else { intake1.brake(); intake2.brake(); }
        }
        else if(redirectc == true){
            if(controller.get_digital(pros::E_CONTROLLER_DIGITAL_L1)) { intake1.move(127); intake2.move(127); }
            else if(controller.get_digital(pros::E_CONTROLLER_DIGITAL_R1)) { intake1.move(-127); intake2.move(-127); }
            else { intake1.brake(); intake2.brake(); }
        }

        // Debug page
        pros::lcd::print(2, "Intake1 : %f W", intake1.get_torque()); 
        pros::lcd::print(4, "Intake2 : %f W", intake2.get_torque()); 

        pros::delay(10);
    }
}
