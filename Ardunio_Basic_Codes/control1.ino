
// Include the Arduino Stepper Library
#include <Stepper.h>

const int stepsPerRevolution = 100;
Stepper myStepper(stepsPerRevolution, 8, 9, 10, 11);


void setup()
{
	// set the speed at 60 rpm:
	myStepper.setSpeed(60);
	// initialize the serial port:
	Serial.begin(9600);
}

void loop() 
{
	// step one revolution in one direction:
	Serial.println("clockwise");
	myStepper.step(stepsPerRevolution);
	delay(500);

	// step one revolution in the other direction:
	Serial.println("counterclockwise");
	myStepper.step(-stepsPerRevolution);
	delay(500);
}
//Code Explanation:
//The sketch starts with including the Arduino Stepper Library. The stepper library comes packaged with the Arduino IDE and takes care of the sequencing of the pulses that are sent to the motor.


//Ezoic
// Include the Arduino Stepper Library
#include <Stepper.h>
//After including the library we define a variable called stepsPerRevolution. As the name suggests it is the number of steps per revolution that your motor is rated at. In our case it is 200.


//Next, we create an object of the Stepper library. The constructor of the Stepper class takes the steps per revolution of the motor and Arduino pin connections as arguments.
//In the setup section of the code, we set the speed of the stepper motor by calling the setSpeed() function and initialize the serial communication.

