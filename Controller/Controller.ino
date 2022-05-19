#include "src\bno055\Adafruit_BNO055.h"
#include "src\joystick\Joystick.h" // Joystick Library from here: https://github.com/MHeironimus/ArduinoJoystickLibrary
#include "src\Rotary.h"

//#define DEBUG



// I2C SDA: D2 SCL: D3
// normal switches
#define SW_RIGHT_0			4
#define SW_RIGHT_1			5
#define SW_RIGHT_14_L		6
#define SW_RIGHT_14_R		7
#define SW_AILERON_RUDDER	A0
#define SW_REVERSE_THRUST   A4

// rotary need pcint and shared bank
#define ROT0_BTN			9
#define ROT0_LEFT			0
#define ROT0_RIGHT			15

#define ROT1_BTN			16
#define ROT1_LEFT			1
#define ROT1_RIGHT			14

// analog 
#define AN_THROTTLE			A5

// output
#define LED_CALIBRATION		13


Joystick_ Joystick(JOYSTICK_DEFAULT_REPORT_ID,
	JOYSTICK_TYPE_JOYSTICK, 32, 1,
	true, true, false, true, true, false,
	true, true, false, false, false);

Adafruit_BNO055 bno(55, 0x29);

imu::Vector<3> vecOrient;

const uint32_t cycleDelta = 20;
const uint32_t blinkDelay = 100;
const uint32_t leverChangeDelay = 1000;

enum LeverPosition {
	Throttle,
	PropRPM,
	Mixture
};

uint8_t oldLeverPosition;
uint8_t newLeverPosition;
uint8_t selectedLeverPosition;
uint32_t nextLeverChange;
uint8_t leverChangeInProg;

uint32_t currentTime;

uint32_t nextUpdate = 0;
uint8_t calibBlinkState = HIGH;

float yawOffset = 0, rudder = 0;

uint32_t nextBlink;
uint8_t blinkEnable;

void setup() {

	pinMode(SW_AILERON_RUDDER, INPUT_PULLUP);
	pinMode(SW_RIGHT_0, INPUT_PULLUP);
	pinMode(SW_RIGHT_1, INPUT_PULLUP);

	pinMode(SW_RIGHT_14_L, INPUT_PULLUP);
	pinMode(SW_RIGHT_14_R, INPUT_PULLUP);

	pinMode(AN_THROTTLE, INPUT);
	pinMode(SW_REVERSE_THRUST, INPUT_PULLUP);

	pinMode(LED_CALIBRATION, OUTPUT);
	digitalWrite(LED_CALIBRATION, HIGH);

	pinMode(ROT0_BTN, INPUT_PULLUP);
	pinMode(ROT0_LEFT, INPUT_PULLUP);
	pinMode(ROT0_RIGHT, INPUT_PULLUP);

	pinMode(ROT1_BTN, INPUT_PULLUP);
	pinMode(ROT1_LEFT, INPUT_PULLUP);
	pinMode(ROT1_RIGHT, INPUT_PULLUP);

	attachInterrupt(digitalPinToInterrupt(ROT0_LEFT), rot0_int, FALLING);
	attachInterrupt(digitalPinToInterrupt(ROT1_LEFT), rot1_int, FALLING);

#ifdef DEBUG
	Serial.begin(115200);
#endif

	Joystick.setXAxisRange(-900, 900);
	Joystick.setYAxisRange(-450, 450);
	Joystick.setThrottleRange(0, 500);
	Joystick.setRxAxisRange(0, 500);
	Joystick.setRyAxisRange(0, 500);

	Joystick.setRudderRange(-450, 450);

	Joystick.begin(false);
	Joystick.setThrottle(0);
	Joystick.sendState();


	bool succ = bno.begin(Adafruit_BNO055::OPERATION_MODE_IMUPLUS);
#ifdef DEBUG
	if (succ == false) {
		Serial.println("BNO FAILED!");
		while (1);
	}
#endif

	delay(100);
	bno.setExtCrystalUse(true);
	delay(200);

}

void loop() {
	currentTime = millis();


	if (blinkEnable && currentTime >= nextBlink)
	{
		calibBlinkState = 1 - calibBlinkState;
		digitalWrite(LED_CALIBRATION, calibBlinkState);
		nextBlink = currentTime + blinkDelay;
	}

	if (currentTime >= nextUpdate)
	{
		rotary_left.Update();
		rotary_right.Update();

		uint8_t system, gyro, accel, mag;
		bno.getCalibration(&system, &gyro, &accel, &mag);

		if (gyro >= 3 && accel >= 3) {
			blinkEnable = false;
			digitalWrite(LED_CALIBRATION, LOW);
		}
		else if (gyro >= 1 && accel >= 1)
		{
			blinkEnable = true;
		}
		else
		{
			blinkEnable = false;
			digitalWrite(LED_CALIBRATION, HIGH);
		}


		imu::Quaternion quat = bno.getQuat();
		vecOrient = quat.toEuler() * (180.0 / PI);//bno.getVector(Adafruit_BNO055::VECTOR_EULER);
		float yaw = vecOrient.x(), roll = vecOrient.z(), pitch = vecOrient.y();
		
		if (digitalRead(SW_AILERON_RUDDER) == LOW) {
			yawOffset = yaw;
		}
		yaw = (yaw - yawOffset);
		
		roll = constrain(roll, -90, 90);
		pitch = constrain(pitch, -45, 45);
		if (yaw > -25 && yaw < 25)
			rudder = constrain(yaw, -22.5, 22.5);
			
		float throttle = constrain(analogRead(AN_THROTTLE) - 270, 0, 500);


		Joystick.setXAxis(roll * 10.0);
		Joystick.setYAxis(pitch * 10.0);
		Joystick.setRudder(rudder * 20.0);

		oldLeverPosition = newLeverPosition;
		if (digitalRead(SW_RIGHT_14_R) == LOW)
			newLeverPosition = Throttle;
		else if (digitalRead(SW_RIGHT_14_L) == LOW)
			newLeverPosition = Mixture;
		else
			newLeverPosition = PropRPM;

		if (newLeverPosition != oldLeverPosition) {
			nextLeverChange = currentTime + leverChangeDelay;
			leverChangeInProg = true;
		}
		else if (leverChangeInProg && (currentTime > nextLeverChange)) {
			selectedLeverPosition = newLeverPosition;
			leverChangeInProg = false;
		}

		if (!leverChangeInProg) {
			if (selectedLeverPosition == PropRPM) {
				if (!digitalRead(SW_REVERSE_THRUST)) // only allow reverse thrust in throttle setting
				{
					Joystick.pressButton(29);
					Joystick.setRxAxis(500);
				}
				else
				{
					Joystick.releaseButton(29);
					Joystick.setRxAxis(throttle);
				}
			}
			else if (selectedLeverPosition == Mixture) {
				if (!digitalRead(SW_REVERSE_THRUST)) // only allow reverse thrust in throttle setting
				{
					Joystick.pressButton(30);
					Joystick.setRyAxis(throttle);
				}
				else
				{
					Joystick.releaseButton(30);
					Joystick.setRyAxis(throttle);
				}
			}
			else if (selectedLeverPosition == Throttle)
			{
				if (!digitalRead(SW_REVERSE_THRUST)) // only allow reverse thrust in throttle setting
				{
					Joystick.pressButton(31);
					Joystick.setThrottle(500);
				}
				else
				{
					Joystick.releaseButton(31);
					Joystick.setThrottle(throttle);
				}
			}
		}

		if (!digitalRead(ROT0_BTN)) {
			Joystick.pressButton(15);
#ifdef DEBUG	
			Serial.println("LEFT BUTTON");
#endif
		}
		else
			Joystick.releaseButton(15);

		if (rotary_left.Left()) {
			Joystick.pressButton(16);
#ifdef DEBUG
			Serial.println("0 LEFT");
#endif
		}
		else
			Joystick.releaseButton(16);

		if (rotary_left.Right()) {
			Joystick.pressButton(17);
#ifdef DEBUG
			Serial.println("0 RIGHT");
#endif
		}
		else
			Joystick.releaseButton(17);

		int rightModifier = !digitalRead(SW_RIGHT_1) ? 3 : 0;

		if (!digitalRead(ROT1_BTN)) {
			Joystick.pressButton(18 + rightModifier);
#ifdef DEBUG
			Serial.println("RIGHT BUTTON");
#endif
		}
		else
			Joystick.releaseButton(18 + rightModifier);

		if (rotary_right.Left()) {
			Joystick.pressButton(19 + rightModifier);
#ifdef DEBUG
			Serial.println("1 LEFT");
#endif
		}
		else
			Joystick.releaseButton(19 + rightModifier);

		if (rotary_right.Right()) {
			Joystick.pressButton(20 + rightModifier);
#ifdef DEBUG
			Serial.println("1 RIGHT");
#endif
		}
		else
			Joystick.releaseButton(20 + rightModifier);

		if (!digitalRead(SW_RIGHT_0))
			Joystick.pressButton(0);
		else
			Joystick.releaseButton(0);



		Joystick.sendState();

#ifdef DEBUG
		Serial.print("Calib S");
		Serial.print(system);
		Serial.print("  G");
		Serial.print(gyro);
		Serial.print("  A");
		Serial.print(accel);
		Serial.print("  M");
		Serial.println(mag);

		Serial.print("Ang R");
		Serial.print(roll);
		Serial.print("  P");
		Serial.print(pitch);
		Serial.print("  T");
		Serial.println(throttle);
#endif
		nextUpdate = currentTime + cycleDelta;

	}
}

float mapf(float x, float in_min, float in_max, float out_min, float out_max) {
	return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

void rot0_int() {
	
	
	if (digitalRead(ROT0_RIGHT) == 1) {
		rotary_left._int_left = true;
	}
	else if (digitalRead(ROT0_RIGHT) == 0)
	{
		rotary_left._int_right = true;
	}

}

void rot1_int() {
	if (digitalRead(ROT1_RIGHT) == 1) {
		rotary_right._int_left = true;
	}
	else if (digitalRead(ROT1_RIGHT) == 0)
	{
		rotary_right._int_right = true;
	}
}