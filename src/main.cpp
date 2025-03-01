#include "main.h"
#include "lemlib/api.hpp" // IWYU pragma: keep
using namespace pros;
using namespace lemlib;

/*Auton Directory*/
//1. Red Solo AWP
//2. Blue Solo AWP
//3. Red 6 Ring (Ladder for 1/2?)
//4. Blue 6 Ring (Ladder for 1/2?)
//5. Red Goal Side (1/2 AWP)
//6. Blue Goal Side (1/2 AWP)
//7. Goal Rush Red
//8. Goal Rush Blue
//9. Skills
//10. Elims 1
//11. Elims 2
const int selectedAuton = 1;
bool onRedAlliance = true;

// controller
pros::Controller controller(pros::E_CONTROLLER_MASTER);

// motor groups
pros::MotorGroup leftMotors({-17, 10, -8}, pros::MotorGearset::blue);
pros::MotorGroup rightMotors({12, 21, -1}, pros::MotorGearset::blue);
pros::Motor intake(11, pros::MotorGearset::blue);
pros::Motor ladyBrown(-20, pros::MotorGearset::red);
//Sensors
pros::Rotation ladyBrownRotation(2);
pros::Optical left_sorter(7);
pros::Optical right_sorter(2);
pros::Rotation horizontalEnc(16);
pros::Rotation verticalEnc(3);
//Pneumatics
pros::ADIDigitalOut clamp('B');
pros::ADIDigitalOut doinkerRight('A'); //OG Doinker
pros::ADIDigitalOut doinkerLeft('C');

// Inertial Sensor on port 10
pros::Imu imu(9);

bool doinker1 = false;
bool doinker2 = false;

const int numStates = 3;
int states[numStates] = {1400, 6300, 20000};
int currentState = 0;
int target = 0;
void nextState(){
	currentState+= 1;
	if (currentState = 3){
		currentState = 0;
	}
	target = states[currentState];
}

double previousError = 0;

double lbControl(double error) {
	double kp = 1;
	double kd = 0.0;
	double output = kp * error + kd * (error - previousError);
	previousError = error;
	return output;
}

float colorofRing = (left_sorter.get_hue() + right_sorter.get_hue()) / 2;

void colorSort() {
	left_sorter.set_led_pwm(100);
    right_sorter.set_led_pwm(100);
        while (true) {
            if(onRedAlliance == true){
				printf("%s", "Color Sorting Blue");
                if(intake.get_power() >=1 && (left_sorter.get_hue() > 180 || right_sorter.get_hue() > 180)){
                    pros::Task::delay(50);
                    intake.move_velocity(0);
                    pros::Task::delay(200);
                    intake.move_velocity(-12000);
                }
            }
            else if(onRedAlliance == false){
                printf("%s", "Color Sorting Red");
                if(intake.get_target_velocity() == 600 && (left_sorter.get_hue() < 30 || right_sorter.get_hue() < 30)){
                    pros::Task::delay(50);
                    intake.move_velocity(0);
                    pros::Task::delay(150);
                    intake.move_velocity(-12000);
                }
            }
            pros::lcd::print(4, "Left Hue: %f", left_sorter.get_hue());
			pros::lcd::print(5, "Right Hue: %f", right_sorter.get_hue());
			pros::lcd::print(6, "Motor Voltage: %f", intake.get_power());
            pros::Task::delay(10);
        }
}

void autonSelector(){

}



// tracking wheels
// horizontal tracking wheel encoder. Rotation sensor, port 20, not reversed

// vertical tracking wheel encoder. Rotation sensor, port 11, reversed
// pros::Rotation verticalEnc(-11);
// horizontal tracking wheel. 2.75" diameter, 5.75" offset, back of the robot (negative)
lemlib::TrackingWheel horizontalPod(&horizontalEnc, lemlib::Omniwheel::NEW_275, 0);
lemlib::TrackingWheel verticalPod(&horizontalEnc, lemlib::Omniwheel::NEW_275, 0);
// vertical tracking wheel. 2.75" diameter, 2.5" offset, left of the robot (negative)
// lemlib::TrackingWheel vertical(&verticalEnc, lemlib::Omniwheel::NEW_275, -2.5);

// drivetrain settings
lemlib::Drivetrain drivetrain(&leftMotors, // left motor group
                              &rightMotors, // right motor group
                              10, // 10 inch track width
                              lemlib::Omniwheel::NEW_275, // using new 4" omnis
                              480, // drivetrain rpm is 360
                              8 // horizontal drift is 2. If we had traction wheels, it would have been 8
);

// lateral motion controller
lemlib::ControllerSettings linearController(8, // proportional gain (kP)
                                            0, // integral gain (kI)
                                            14, // derivative gain (kD)
                                            3, // anti windup
                                            1, // small error range, in inches
                                            100, // small error range timeout, in milliseconds
                                            3, // large error range, in inches
                                            500, // large error range timeout, in milliseconds
                                            20 // maximum acceleration (slew)
);

// angular motion controller
lemlib::ControllerSettings angularController(1.05, // proportional gain (kP)
                                             0, // integral gain (kI)
                                             2.2, // derivative gain (kD)
                                             3, // anti windup
                                             1, // small error range, in degrees
                                             100, // small error range timeout, in milliseconds
                                             3, // large error range, in degrees
                                             500, // large error range timeout, in milliseconds
                                             20 // maximum acceleration (slew)
);

// sensors for odometry
lemlib::OdomSensors sensors(&verticalPod, // vertical tracking wheel
                            nullptr, // vertical tracking wheel 2, set to nullptr as we don't have a second one
                            &horizontalPod, // horizontal tracking wheel
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

/**
 * Runs initialization code. This occurs as soon as the program is started.
 *
 * All other competition modes are blocked by initialize; it is recommended
 * to keep execution time for this mode under a few seconds.
 */
void initialize() {
    pros::lcd::initialize(); // initialize brain screen
    chassis.calibrate(); // calibrate sensors

	
    pros::Task colorSortTask(colorSort);
    // the default rate is 50. however, if you need to change the rate, you
    // can do the following.
    // lemlib::bufferedStdout().setRate(...);
    // If you use bluetooth or a wired connection, you will want to have a rate of 10ms

    // for more information on how the formatting for the loggers
    // works, refer to the fmtlib docs

    // thread to for brain screen and position logging
    pros::Task screenTask([&]() {
        while (true) {
            // print robot location to the brain screen
            pros::lcd::print(0, "X: %f", chassis.getPose().x); // x
            pros::lcd::print(1, "Y: %f", chassis.getPose().y); // y
            pros::lcd::print(2, "Theta: %f", chassis.getPose().theta); // heading
            // log position telemetry
            lemlib::telemetrySink()->info("Chassis pose: {}", chassis.getPose());
            // delay to save resources
            pros::delay(50);
        }
    });
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
    switch (selectedAuton){
        case 1: //Red Solo AWP 
        clamp.set_value(1);
        delay(250);
        intake.move_velocity(-12000);
        break;
        case 2: //Blue Solo AWP
        chassis.moveToPose(0, 0, 90, 1000, {.forwards = false}, false);
        break;
        case 3: //Red 6 Ring (Ladder for 1/2?)
        chassis.moveToPoint(0, 0, 1000, {.forwards = true}, false);
        break;
        case 4: //Blue 6 Ring (Ladder for 1/2?)
        chassis.turnToPoint(0, 0, 1000, {.direction = AngularDirection::CW_CLOCKWISE}, false);
        break;
        case 5: //Red Goal Side (1/2 AWP)
        intake.move_voltage(12000);
        break;
        case 6: //Blue Goal Side (1/2 AWP)
        ladyBrown.move_voltage(12000);
        break;
        case 7: //Goal Rush Red
        clamp.set_value(1);
        break;
        case 8: //Goal Rush Blue
        break;
        case 9: //Skills
        chassis.swingToHeading(90, DriveSide::LEFT, 1000, {.maxSpeed = 127}, true);
        break;
        case 10: //Elims 1
        chassis.swingToPoint(-25, 25, DriveSide::RIGHT, 1000, {.maxSpeed = 127}, true);
        break;
        case 11: //Elims 2
        chassis.setPose(0, 0, 0, false);
        break;
    }
}

/**
 * Runs in driver control
 */
void opcontrol() {
    // controller
    // loop to continuously update motors
    while (true) {
        // get joystick positions
        int leftY = controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);
        int rightX = controller.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X);
        // move the chassis with curvature drive
        chassis.arcade(leftY, rightX);

		if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_L1)) {
			intake.move_voltage(-12000);
		}
		else if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_L2)) {
			intake.move_voltage(12000);
		}
		else{
			intake.move_voltage(0);
		}
		if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_R1)){
			clamp.set_value(1);
		}
		if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_R2)){
			clamp.set_value(0);
		}
		if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_Y)){
			doinker1 = !doinker1;
			doinkerLeft.set_value(doinker1);
		}
        if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_B)){
            doinker2 = !doinker2;
            doinkerRight.set_value(doinker2);
        }
		if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_A)){
			nextState();
		}
        // delay to save resources
        pros::delay(20);
    }
}