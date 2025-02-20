#include "main.h"
#include "lemlib/api.hpp" // IWYU pragma: keep

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
pros::Optical intakeColor2(9);
//Pneumatics
pros::ADIDigitalOut clamp('B');
pros::ADIDigitalOut doinker('A');

// Inertial Sensor on port 10
pros::Imu imu(9);

bool doinker1 = false;

const int selectedAuton = 1;

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


bool sortingBlue = true;
bool sortingRed = false;
void colorSort() {
	left_sorter.set_led_pwm(100);
        while (true) {
            if(sortingBlue == true){
				printf("%s", "Color Sorting Blue");
                if(intake.get_power() >=1 && (left_sorter.get_hue() > 180)){
                    pros::Task::delay(48);
                    intake.move_velocity(0);
                    pros::Task::delay(200);
                    intake.move_velocity(-12000);
                }
            }
            else if(sortingRed == true){
                printf("%s", "Color Sorting Red");
                if(intake.get_power() >= 1 && ((left_sorter.get_hue() <= 25) || (left_sorter.get_hue() >= 340 && left_sorter.get_hue() < 360))){
                    pros::Task::delay(42);
                    intake.move_velocity(0);
                    pros::Task::delay(150);
                    intake.move_velocity(-12000);
                }
            }
            pros::lcd::print(4, "Ring Hue: %f", left_sorter.get_hue());
			pros::lcd::print(5, "Intake Motor voltage: %f", intake.get_power());
            pros::Task::delay(10);
        }
}



// tracking wheels
// horizontal tracking wheel encoder. Rotation sensor, port 20, not reversed
pros::Rotation horizontalEnc(16);
// vertical tracking wheel encoder. Rotation sensor, port 11, reversed
// pros::Rotation verticalEnc(-11);
// horizontal tracking wheel. 2.75" diameter, 5.75" offset, back of the robot (negative)
lemlib::TrackingWheel horizontal(&horizontalEnc, lemlib::Omniwheel::NEW_275, -5.75);
// vertical tracking wheel. 2.75" diameter, 2.5" offset, left of the robot (negative)
// lemlib::TrackingWheel vertical(&verticalEnc, lemlib::Omniwheel::NEW_275, -2.5);

// drivetrain settings
lemlib::Drivetrain drivetrain(&leftMotors, // left motor group
                              &rightMotors, // right motor group
                              10, // 10 inch track width
                              lemlib::Omniwheel::NEW_4, // using new 4" omnis
                              360, // drivetrain rpm is 360
                              2 // horizontal drift is 2. If we had traction wheels, it would have been 8
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
lemlib::ControllerSettings angularController(18, // proportional gain (kP)
                                             0, // integral gain (kI)
                                             0, // derivative gain (kD)
                                             3, // anti windup
                                             1, // small error range, in degrees
                                             100, // small error range timeout, in milliseconds
                                             3, // large error range, in degrees
                                             500, // large error range timeout, in milliseconds
                                             20 // maximum acceleration (slew)
);

// sensors for odometry
lemlib::OdomSensors sensors(nullptr, // vertical tracking wheel
                            nullptr, // vertical tracking wheel 2, set to nullptr as we don't have a second one
                            nullptr, // horizontal tracking wheel
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

	pros::Task lbControlTask([]{
		double error;
		double output;

		while (true) {
			error = target - ladyBrownRotation.get_position();
			output = lbControl(error);
			ladyBrown.move(output);
			pros::delay(20);
		}
	});
    pros::Task colorSortTask(colorSort);
    leftMotors.set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);
    rightMotors.set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);

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
    using namespace pros;
    chassis.setPose(0, 0, 0, false);
    // intake.move_voltage(-12000);
    // delay(800);
    // intake.move_voltage(0);
    // chassis.moveToPoint(0, 14, 1200, {.forwards = true}, false);
    chassis.turnToHeading(45, 1000, {.direction = AngularDirection::CW_CLOCKWISE, .minSpeed = 100});
    // chassis.turnToPoint(10, 0, 1000, {.direction = lemlib::AngularDirection::CW_CLOCKWISE, .minSpeed = 100});
    
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
		if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_B)){
			doinker1 = !doinker1;
			doinker.set_value(doinker1);
		}
		if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_A)){
			nextState();
		}

        // delay to save resources
        pros::delay(20);
    }
}