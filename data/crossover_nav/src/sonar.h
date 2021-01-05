//#define TRIGGER_PIN  7  // Arduino pin tied to trigger pin on the ultrasonic sensor.
//#define ECHO_PIN     8  // Arduino pin tied to echo pin on the ultrasonic sensor.
//#define MAX_DISTANCE 500 // Maximum distance we want to ping for (in centimeters). Maximum sensor distance is rated at 400-500cm.
//
//
//// ---------------------------------------------------------------------------
//// NewPing Library - v1.5 - 08/15/2012
////
//// AUTHOR/LICENSE:
//// Created by Tim Eckel - teckel@leethost.com
//// Copyright 2012 License: GNU GPL v3 http://www.gnu.org/licenses/gpl-3.0.html
////
//// LINKS:
//// Project home: http://code.google.com/p/arduino-new-ping/
//// Blog: http://arduino.cc/forum/index.php/topic,106043.0.html
////
//// DISCLAIMER:
//// This software is furnished "as is", without technical support, and with no 
//// warranty, express or implied, as to its usefulness for any purpose.
////
//// BACKGROUND:
//// When I first received an ultrasonic sensor I was not happy with how poorly
//// it worked. Quickly I realized the problem wasn't the sensor, it was the
//// available ping and ultrasonic libraries causing the problem. The NewPing
//// library totally fixes these problems, adds many new features, and breaths
//// new life into these very affordable distance sensors. 
////
//// FEATURES:
//// * Works with many different ultrasonic sensor models: SR04, SRF05, SRF06, DYP-ME007 & Parallax PING))).
//// * Interface with all but the SRF06 sensor using only one Arduino pin.
//// * Doesn't lag for a full second if no ping/echo is received.
//// * Ping sensors consistently and reliably at up to 30 times per second.
//// * Timer interrupt method for event-driven sketches.
//// * Built-in digital filter method ping_median() for easy error correction.
//// * Uses port registers for a faster pin interface and smaller code size.
//// * Allows you to set a maximum distance where pings beyond that distance are read as no ping "clear".
//// * Ease of using multiple sensors (example sketch with 15 sensors).
//// * More accurate distance calculation (cm, inches & uS).
//// * Doesn't use pulseIn, which is slow and gives incorrect results with some ultrasonic sensor models.
//// * Actively developed with features being added and bugs/issues addressed.
////
//// CONSTRUCTOR:
////   NewPing sonar(trigger_pin, echo_pin [, max_cm_distance])
////     trigger_pin & echo_pin - Arduino pins connected to sensor trigger and echo.
////       NOTE: To use the same Arduino pin for trigger and echo, specify the same pin for both values.
////     max_cm_distance - [Optional] Maximum distance you wish to sense. Default=500cm.
////
//// SYNTAX:
////   sonar.ping() - Send a ping and get the echo time (in microseconds) as a result. 
////   sonar.ping_in() - Send a ping and get the distance in whole inches.
////   sonar.ping_cm() - Send a ping and get the distance in whole centimeters.
////   sonar.ping_median(iterations) - Do multiple pings (default=5), discard out of range pings and return median in microseconds. 
////   sonar.convert_in(echoTime) - Convert echoTime from microseconds to inches (rounds to nearest inch).
////   sonar.convert_cm(echoTime) - Convert echoTime from microseconds to centimeters (rounds to nearest cm).
////   sonar.ping_timer(function) - Send a ping and call function to test if ping is complete.
////   sonar.check_timer() - Check if ping has returned within the set distance limit.
////   NewPing::timer_us(frequency, function) - Call function every frequency microseconds.
////   NewPing::timer_ms(frequency, function) - Call function every frequency milliseconds.
////   NewPing::timer_stop() - Stop the timer.
////
//// HISTORY:
//// 08/15/2012 v1.5 - Added ping_median() method which does a user specified
////   number of pings (default=5) and returns the median ping in microseconds
////   (out of range pings ignored). This is a very effective digital filter.
////   Optimized for smaller compiled size (even smaller than skteches that
////   don't use a library).
////
//// 07/14/2012 v1.4 - Added support for the Parallax PING))) sensor. Interface
////   with all but the SRF06 sensor using only one Arduino pin. You can also
////   interface with the SRF06 using one pin if you install a 0.1uf capacitor
////   on the trigger and echo pins of the sensor then tie the trigger pin to
////   the Arduino pin (doesn't work with Teensy). To use the same Arduino pin
////   for trigger and echo, specify the same pin for both values. Various bug
////   fixes.
////
//// 06/08/2012 v1.3 - Big feature addition, event-driven ping! Uses Timer2
////   interrupt, so be mindful of PWM or timing conflicts messing with Timer2
////   may cause (namely PWM on pins 3 & 11 on Arduino, PWM on pins 9 and 10 on
////   Mega, and Tone library). Simple to use timer interrupt functions you can
////   use in your sketches totaly unrelated to ultrasonic sensors (don't use if
////   you're also using NewPing's ping_timer because both use Timer2 interrupts).
////   Loop counting ping method deleted in favor of timing ping method after
////   inconsistant results kept surfacing with the loop timing ping method.
////   Conversion to cm and inches now rounds to the nearest cm or inch. Code
////   optimized to save program space and fixed a couple minor bugs here and
////   there. Many new comments added as well as line spacing to group code
////   sections for better source readability.
////
//// 05/25/2012 v1.2 - Lots of code clean-up thanks to Adruino Forum members.
////   Rebuilt the ping timing code from scratch, ditched the pulseIn code as it
////   doesn't give correct results (at least with ping sensors). The NewPing
////   library is now VERY accurate and the code was simplified as a bonus.
////   Smaller and faster code as well. Fixed some issues with very close ping
////   results when converting to inches. All functions now return 0 only when
////   there's no ping echo (out of range) and a positive value for a successful
////   ping. This can effectively be used to detect if something is out of range
////   or in-range and at what distance. Now compatible with Arduino 0023.
////
//// 05/16/2012 v1.1 - Changed all I/O functions to use low-level port registers
////   for ultra-fast and lean code (saves from 174 to 394 bytes). Tested on both
////   the Arduino Uno and Teensy 2.0 but should work on all Arduino-based
////   platforms because it calls standard functions to retrieve port registers
////   and bit masks. Also made a couple minor fixes to defines.
////
//// 05/15/2012 v1.0 - Initial release.
//// ---------------------------------------------------------------------------
//
//#ifndef NewPing_h
//#define NewPing_h
//
//#if defined(ARDUINO) && ARDUINO >= 100
//	#include <Arduino.h>
//#else
//	#include <WProgram.h>
//	#include <pins_arduino.h>
//#endif
//
//#include <avr/io.h>
//#include <avr/interrupt.h>
//
//// Shoudln't need to changed these values unless you have a specific need to do so.
//#define MAX_SENSOR_DISTANCE 500 // Maximum sensor distance can be as high as 500cm, no reason to wait for ping longer than sound takes to travel this distance and back.
//#define US_ROUNDTRIP_IN 146     // Microseconds (uS) it takes sound to travel round-trip 1 inch (2 inches total), uses integer to save compiled code space.
//#define US_ROUNDTRIP_CM 57      // Microseconds (uS) it takes sound to travel round-trip 1cm (2cm total), uses integer to save compiled code space.
//#define DISABLE_ONE_PIN false   // Set to "true" to save up to 26 bytes of compiled code space if you're not using one pin sensor connections.
//
//// Probably shoudln't change these values unless you really know what you're doing.
//#define NO_ECHO 0               // Value returned if there's no ping echo within the specified MAX_SENSOR_DISTANCE or max_cm_distance.
//#define MAX_SENSOR_DELAY 18000  // Maximum uS it takes for sensor to start the ping (SRF06 is the highest measured, just under 18ms).
//#define ECHO_TIMER_FREQ 24      // Frequency to check for a ping echo (every 24uS is about 0.4cm accuracy).
//#define PING_MEDIAN_DELAY 29    // Millisecond delay between pings in the ping_median method.
//
//// Conversion from uS to distance (round result to nearest cm or inch).
//#define NewPingConvert(echoTime, conversionFactor) (max((echoTime + conversionFactor / 2) / conversionFactor, (echoTime ? 1 : 0)))
//
//
//class NewPing {
//	public:
//		NewPing(uint8_t trigger_pin, uint8_t echo_pin, int max_cm_distance = MAX_SENSOR_DISTANCE);
//		unsigned int ping();
//		unsigned int ping_in();
//		unsigned int ping_cm();
//		unsigned int ping_median(uint8_t it = 5);
//		unsigned int convert_in(unsigned int echoTime);
//		unsigned int convert_cm(unsigned int echoTime);
//		void ping_timer(void (*userFunc)(void));
//		boolean check_timer();
//		unsigned long ping_result;
//		static void timer_us(unsigned int frequency, void (*userFunc)(void));
//		static void timer_ms(unsigned long frequency, void (*userFunc)(void));
//		static void timer_stop();
//	private:
//		boolean ping_trigger();
//		boolean ping_wait_timer();
//		uint8_t _triggerBit;
//		uint8_t _echoBit;
//		volatile uint8_t *_triggerOutput;
//		volatile uint8_t *_triggerMode;
//		volatile uint8_t *_echoInput;
//		unsigned int _maxEchoTime;
//		unsigned long _max_time;
//		static void timer_setup();
//		static void timer_ms_cntdwn();
//};
//
//
//#endif
//
//// ---------------------------------------------------------------------------
//// NewPing constructor
//// ---------------------------------------------------------------------------
//
//NewPing::NewPing(uint8_t trigger_pin, uint8_t echo_pin, int max_cm_distance) {
//	_triggerBit = digitalPinToBitMask(trigger_pin); // Get the port register bitmask for the trigger pin.
//	_echoBit = digitalPinToBitMask(echo_pin);       // Get the port register bitmask for the echo pin.
//
//	_triggerOutput = portOutputRegister(digitalPinToPort(trigger_pin)); // Get the output port register for the trigger pin.
//	_echoInput = portInputRegister(digitalPinToPort(echo_pin));         // Get the input port register for the echo pin.
//
//	_triggerMode = (uint8_t *) portModeRegister(digitalPinToPort(trigger_pin)); // Get the port mode register for the trigger pin.
//
//	_maxEchoTime = min(max_cm_distance, MAX_SENSOR_DISTANCE) * US_ROUNDTRIP_CM + (US_ROUNDTRIP_CM / 2); // Calculate the maximum distance in uS.
//
//#if DISABLE_ONE_PIN == true
//	*_triggerMode |= _triggerBit; // Set trigger pin to output.
//#endif
//}
//
//
//// ---------------------------------------------------------------------------
//// Standard ping methods
//// ---------------------------------------------------------------------------
//
//unsigned int NewPing::ping() {
//	if (!ping_trigger()) return NO_ECHO;                // Trigger a ping, if it returns false, return NO_ECHO to the calling function.
//	while (*_echoInput & _echoBit)                      // Wait for the ping echo.
//		if (micros() > _max_time) return NO_ECHO;       // Stop the loop and return NO_ECHO (false) if we're beyond the set maximum distance.
//	return (micros() - (_max_time - _maxEchoTime) - 5); // Calculate ping time, 5uS of overhead.
//}
//
//
//unsigned int NewPing::ping_in() {
//	unsigned int echoTime = NewPing::ping();          // Calls the ping method and returns with the ping echo distance in uS.
//	return NewPingConvert(echoTime, US_ROUNDTRIP_IN); // Convert uS to inches.
//}
//
//
//unsigned int NewPing::ping_cm() {
//	unsigned int echoTime = NewPing::ping();          // Calls the ping method and returns with the ping echo distance in uS.
//	return NewPingConvert(echoTime, US_ROUNDTRIP_CM); // Convert uS to centimeters.
//}
//
//
//unsigned int NewPing::ping_median(uint8_t it) {
//	unsigned int uS[it], last;
//	uint8_t j, i = 0;
//	uS[0] = NO_ECHO;
//	while (i < it) {
//		last = ping();           // Send ping.
//		if (last == NO_ECHO) {   // Ping out of range.
//			it--;                // Skip, don't include as part of median.
//			last = _maxEchoTime; // Adjust "last" variable so delay is correct length.
//		} else {                       // Ping in range, include as part of median.
//			if (i > 0) {               // Don't start sort till second ping.
//				for (j = i; j > 0 && uS[j - 1] < last; j--) // Insertion sort loop.
//					uS[j] = uS[j - 1]; // Shift ping array to correct position for sort insertion.
//			} else j = 0;              // First ping is starting point for sort.
//			uS[j] = last;              // Add last ping to array in sorted position.
//			i++;                       // Move to next ping.
//		}
//		if (i < it) delay(PING_MEDIAN_DELAY - (last >> 10)); // Millisecond delay between pings.
//	}
//	return (uS[it >> 1]); // Return the ping distance median.
//}
//
//
//// ---------------------------------------------------------------------------
//// Standard ping method support functions (not called directly)
//// ---------------------------------------------------------------------------
//
//boolean NewPing::ping_trigger() {
//#if DISABLE_ONE_PIN != true
//	*_triggerMode |= _triggerBit;    // Set trigger pin to output.
//#endif
//	*_triggerOutput &= ~_triggerBit; // Set the trigger pin low, should already be low, but this will make sure it is.
//	delayMicroseconds(4);            // Wait for pin to go low, testing shows it needs 4uS to work every time.
//	*_triggerOutput |= _triggerBit;  // Set trigger pin high, this tells the sensor to send out a ping.
//	delayMicroseconds(10);           // Wait long enough for the sensor to realize the trigger pin is high. Sensor specs say to wait 10uS.
//	*_triggerOutput &= ~_triggerBit; // Set trigger pin back to low.
//#if DISABLE_ONE_PIN != true
//	*_triggerMode &= ~_triggerBit;   // Set trigger pin to input (when using one Arduino pin this is technically setting the echo pin to input as both are tied to the same Arduino pin).
//#endif
//
//	_max_time =  micros() + MAX_SENSOR_DELAY;                  // Set a timeout for the ping to trigger.
//	while (*_echoInput & _echoBit && micros() <= _max_time) {} // Wait for echo pin to clear.
//	while (!(*_echoInput & _echoBit))                          // Wait for ping to start.
//		if (micros() > _max_time) return false;                // Something went wrong, abort.
//
//	_max_time = micros() + _maxEchoTime; // Ping started, set the timeout.
//	return true;                         // Ping started successfully.
//}
//
//
//// ---------------------------------------------------------------------------
//// Timer interrupt ping methods (won't work with ATmega8 and ATmega128)
//// ---------------------------------------------------------------------------
//
//void NewPing::ping_timer(void (*userFunc)(void)) {
//	if (!ping_trigger()) return;         // Trigger a ping, if it returns false, return without starting the echo timer.
//	timer_us(ECHO_TIMER_FREQ, userFunc); // Set ping echo timer check every ECHO_TIMER_FREQ uS.
//}
//
// 
//boolean NewPing::check_timer() {
//	if (micros() > _max_time) { // Outside the timeout limit.
//		timer_stop();           // Disable timer interrupt
//		return false;           // Cancel ping timer.
//	}
//
//	if (!(*_echoInput & _echoBit)) { // Ping echo received.
//		timer_stop();                // Disable timer interrupt
//		ping_result = (micros() - (_max_time - _maxEchoTime) - 13); // Calculate ping time, 13uS of overhead.
//		return true;                 // Return ping echo true.
//	}
//
//	return false; // Return false because there's no ping echo yet.
//}
//
//
//// ---------------------------------------------------------------------------
//// Timer2/Timer4 interrupt methods (can be used for non-ultrasonic needs)
//// ---------------------------------------------------------------------------
//
//// Variables used for timer functions
//void (*intFunc)();
//void (*intFunc2)();
//unsigned long _ms_cnt_reset;
//volatile unsigned long _ms_cnt;
//
//
//void NewPing::timer_us(unsigned int frequency, void (*userFunc)(void)) {
//	timer_setup();      // Configure the timer interrupt.
//	intFunc = userFunc; // User's function to call when there's a timer event.
//
//#if defined (__AVR_ATmega32U4__) // Use Timer4 for ATmega32U4 (Teensy/Leonardo).
//	OCR4C = min((frequency>>2) - 1, 255); // Every count is 4uS, so divide by 4 (bitwise shift right 2) subtract one, then make sure we don't go over 255 limit.
//	TIMSK4 = (1<<TOIE4);                  // Enable Timer4 interrupt.
//#else
//	OCR2A = min((frequency>>2) - 1, 255); // Every count is 4uS, so divide by 4 (bitwise shift right 2) subtract one, then make sure we don't go over 255 limit.
//	TIMSK2 |= (1<<OCIE2A);                // Enable Timer2 interrupt.
//#endif
//}
//
//
//void NewPing::timer_ms(unsigned long frequency, void (*userFunc)(void)) {
//	timer_setup();                       // Configure the timer interrupt.
//	intFunc = NewPing::timer_ms_cntdwn;  // Timer events are sent here once every ms till user's frequency is reached.
//	intFunc2 = userFunc;                 // User's function to call when user's frequency is reached.
//	_ms_cnt = _ms_cnt_reset = frequency; // Current ms counter and reset value.
//
//#if defined (__AVR_ATmega32U4__) // Use Timer4 for ATmega32U4 (Teensy/Leonardo).
//	OCR4C = 249;         // Every count is 4uS, so 1ms = 250 counts - 1.
//	TIMSK4 = (1<<TOIE4); // Enable Timer4 interrupt.
//#else
//	OCR2A = 249;           // Every count is 4uS, so 1ms = 250 counts - 1.
//	TIMSK2 |= (1<<OCIE2A); // Enable Timer2 interrupt.
//#endif
//}
//
//
//void NewPing::timer_stop() { // Disable timer interrupt.
//#if defined (__AVR_ATmega32U4__) // Use Timer4 for ATmega32U4 (Teensy/Leonardo).
//	TIMSK4 = 0;
//#else
//	TIMSK2 &= ~(1<<OCIE2A);
//#endif
//}
//
//
//// ---------------------------------------------------------------------------
//// Timer2/Timer4 interrupt method support functions (not called directly)
//// ---------------------------------------------------------------------------
//
//void NewPing::timer_setup() {
//#if defined (__AVR_ATmega32U4__) // Use Timer4 for ATmega32U4 (Teensy/Leonardo).
//	timer_stop(); // Disable Timer4 interrupt.
//	TCCR4A = TCCR4C = TCCR4D = TCCR4E = 0;
//	TCCR4B = (1<<CS42) | (1<<CS41) | (1<<CS40) | (1<<PSR4); // Set Timer4 prescaler to 64 (4uS/count, 4uS-1020uS range).
//	TIFR4 = (1<<TOV4);
//	TCNT4 = 0;    // Reset Timer4 counter.
//#else
//	timer_stop();           // Disable Timer2 interrupt.
//	ASSR &= ~(1<<AS2);      // Set clock, not pin.
//	TCCR2A = (1<<WGM21);    // Set Timer2 to CTC mode.
//	TCCR2B = (1<<CS22);     // Set Timer2 prescaler to 64 (4uS/count, 4uS-1020uS range).
//	TCNT2 = 0;              // Reset Timer2 counter.
//#endif
//}
//
//
//void NewPing::timer_ms_cntdwn() {
//	if (!_ms_cnt--) {            // Count down till we reach zero.
//		intFunc2();              // Scheduled time reached, run the main timer event function.
//		_ms_cnt = _ms_cnt_reset; // Reset the ms timer.
//	}
//}
//
//
//#if defined (__AVR_ATmega32U4__) // Use Timer4 for ATmega32U4 (Teensy/Leonardo).
//ISR(TIMER4_OVF_vect) {
//#else
//ISR(TIMER2_COMPA_vect) {
//#endif
//	if(intFunc) intFunc(); // If wrapped function is set, call it.
//}
//
//
//// ---------------------------------------------------------------------------
//// Conversion methods (rounds result to nearest inch or cm).
//// ---------------------------------------------------------------------------
//
//unsigned int NewPing::convert_in(unsigned int echoTime) {
//	return NewPingConvert(echoTime, US_ROUNDTRIP_IN); // Convert uS to inches.
//}
//
//
//unsigned int NewPing::convert_cm(unsigned int echoTime) {
//	return NewPingConvert(echoTime, US_ROUNDTRIP_CM); // Convert uS to centimeters.
//}
//
//
//
//
//
//
//NewPing sonar(TRIGGER_PIN, ECHO_PIN, MAX_DISTANCE); // NewPing setup of pins and maximum distance.
//
//int sonar_h = 0,sonar_old= 0,sonar_real=0,sonar_vel=0;
//
//
//void Get_sonar()
//{
//            sonar_h = sonar.ping()/* /US_ROUNDTRIP_CM*/; // Send ping, get ping time in microseconds (uS).
//            sonar_h*=max(cos(ahrs_r)*cos(ahrs_p),0.707);
////  say(sonar_h);saytab;
////  say(sonar_old);sayend;
////    if(sonar_h>30)
////    {
////      if(abs(sonar_h-sonar_old)<10){
////        if(sonar_h-sonar_old>0)                         { sonar_real++; }
////        else if(sonar_h-sonar_old<0)                    { sonar_real--; }
////      }
////    //  else if(sonar_h-sonar_old>10) { sonar_real--; }
////    //  else if(sonar_h-sonar_old<-10) {sonar_real++; }
////      if(sonar_real<=0) sonar_real=0;
////      sonar_old=sonar_h;
////    }
////    else
////    {
////      sonar_real=sonar_h;
////    }
//}
//
//
////float true_x,true_y,true_z,one_g;
////void calculate_true_angle()
////{
////  one_g = sqrt(pow(accel[XAXIS],2)+pow(accel[YAXIS],2)+pow(accel[ZAXIS],2));
//////
//////  true_x = accel[XAXIS] - one_g * sin(radians(ahrs_r));
//////  true_y = accel[YAXIS] - one_g * sin(radians(ahrs_p)) * cos(radians(ahrs_r));
//////  true_z = accel[ZAXIS] - one_g * cos(radians(ahrs_r)) * cos(radians(ahrs_p));  //checked
////}
////int smallRotate()
////{
////  if(abs(ahrs_r) <0.14 && abs(ahrs_p) <0.14) return 1;
////  else return 0;
////}
////
////float vz=0,sz=0,vz_sonar=0;
////int sonar_real_old=0;
////void correction_sonar(float accZ,float G_Dt)
////{
////         
////        sonar_vel = (sonar_real - sonar_real_old);
////        sonar_real_old=sonar_real;
//////altitude = true_x*sin(radians(ahrs_r)) - true_y*sin(radians(ahrs_p))*cos(radians(ahrs_r)) + true_z*cos(radians(ahrs_r))*cos(radians(ahrs_p));
////        accZ=one_g-10.91;
//////        accZ*=90;
////        altitude = altitude * 0.8 + accZ * 0.2;
////        applyDeadband(altitude,0.03);
////        if(smallRotate()) vz+=(altitude)*G_Dt;
////        vz=constrain(vz,-100,100);
////        sz=vz*G_Dt;
////        sz=sz*0.9+sonar_real*0.1;
////        
//////  sz+=vz*G_Dt;
////}

