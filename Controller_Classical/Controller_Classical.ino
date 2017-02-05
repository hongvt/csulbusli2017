/********************* BASIC AUTONOMOUS CONTROL SKETCH *********************/
/* _TYPE: CLASSICAL LEAD/LAG or PID or something
 * This sketch amounts to an autonomous controller for a simple flight path
 * for the csulbusli2017 paraglider stage.
 * _FLIGHTPATH CONTROL SUBSYSTEM:
 * --INPUT: 3d position relative to launch site provided by GPS / Altimeter
 * --OUTPUT: servo angle and fan speed
 * _DEPLOYMENT ROUTINE SUBSYSTEM:
 * --INPUT: 2 digital signals -- start stepper, stop stepper
 * --OUTPUT: Stepper motor drive
 * 
 * FEATURES:
 * -2nd order IIR filter function for smoothing data and applying compensation
 */

#include <Servo.h>
#include <Adafruit_GPS.h>
#include <Wire.h>
#include <SoftwareSerial.h>

/***************************************************************************/
/* GPS Things */
//Connect GPS Power and ground, 
//Connect GPS TX to pin3
//Connect GPS RX to pin2

SoftwareSerial mySerial(3,2);
// for hardware serial, comment out softwareserial thing,
// and uncomment the hardwareserial thing below:
//HardwareSerial mySerial = Serial1;

Adafruit_GPS GPS(&mySerial);

// GPSECHO -> 'false' turns off echoing. 'true' turns it on
#define GPSECHO  true

// set to use or not use interrupt -- you can change in the setup section
boolean usingInterrupt = false;
void useInterrupt(boolean); // Func prototype keeps Arduino 0023 happy

/*****************************************************************************/
/* Servo Things */
Servo fan;                                // create servo object for fan
Servo ser;                                // create servo object for servo
double fan_val = 0;                        // fan actuation variable
double ser_val = 0;                        // servo actuation variable

/* Measurements and State Variables */
double lat_deg_0;                         // latitude measured upon reset
double lat_deg_i;                         // current latitude
double long_deg_0;                        // longitude measured upon reset
double long_deg_i;                        // current longitude
double alt_0;                             // altitude measured upon reset
double alt_i;                             // current altitude

double x;                                 // x position (conditioned longitutde)
double y;                                 // y position (conditioned latitude)
double z;                                 // z position (conditioned altitude)

double dist;                              // distance squared from the oirigin
double desc;                              // descent rate
double error_dist;                        // distance error     
double error_desc;                        // descent rate error

const double RADIUS = 300;                // desired RADIUS from launch site
const double descent = 10;                // desired descent rate


/********************** IIR filter constants and arrays **********************/
/* Filter Coefficients for xyz position */
//these assume a sampling frequency of 5Hz... use Matlab script to calculate
double a[3] = {1, -1.077, 0.2899};        // 2d position data filter feedback
double b[3] = {0.05325, 0.1065, 0.05325}; // 2d position data filter feedforward
double ad[3] = {1, -1.077, 0.2899};       // descent derivative feedback
double bd[3] = {0.5325, 0 -0.5325};       // descent derivative feedforward
double acs[3] = {1,1,1};                  // steering controller feedback
double bcs[3] = {1,1,1};                  // steering controller feedforward
double aca[3] = {1,1,1};                  // altitude controller feedback
double bca[3] = {1,1,1};                  // altitude controller feedforward
/* Buffers */
static double xbuff[2] = {};              // Buffers for intermediate values
static double ybuff[2] = {};
static double zbuff[2] = {};
static double descbuff[2] = {};

static double csbuff[2] = {};
static double cabuff[2] = {};


//-----------------------------------------------------------------------------
//---------------------------------BEGIN SETUP---------------------------------
void setup() {
  
  /* initializations */
  Serial.begin(115200);
  GPS.begin(9600); 

  /* PWM Output Pin Setup */
  // PWM pins on arduino pro mini are 3,5,6,9,10,11
  fan.attach(10);  
  ser.attach(11);
  
  // remember to attach the stepper motor  

  /* GPS Setup */
  // uncomment to turn on recommended minimum fix data including altitude
  GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCGGA);
  // uncomment to turn on only recomended minimum data
  //GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCONLY);

  // Set the update rate to 10Hz
  GPS.sendCommand(PMTK_SET_NMEA_UPDATE_10HZ); 

  // Request updates on antenna status, comment out to keep quiet
  GPS.sendCommand(PGCMD_ANTENNA);

  // timer0 interrupt -- tries to read GPS every 1 ms
   useInterrupt(true);
   delay(1000);
  

  // in case you are not using the interrupt above, you'll
  // need to 'hand query' the GPS, not suggested :(
  if (! usingInterrupt) {
    // read data from the GPS in the 'main loop'
    char c = GPS.read();
    // if you want to debug, this is a good time to do it!
    if (GPSECHO)
      if (c) Serial.print(c);
      }
  
  // if a sentence is received, we can check the checksum, parse it...
  if (GPS.newNMEAreceived()) {
    // a tricky thing here is if we print the NMEA sentence, or data
    // we end up not listening and catching other sentences! 
    // so be very wary if using OUTPUT_ALLDATA and trytng to print out data
    //Serial.println(GPS.lastNMEA());   // this also sets the newNMEAreceived() flag to false
  
    if (!GPS.parse(GPS.lastNMEA()))   // this also sets the newNMEAreceived() flag to false
      return;  // we can fail to parse a sentence in which case we should just wait for another
      } 
        
    long_deg_0 = (double)GPS.longitudeDegrees;
    lat_deg_0 = (double)GPS.latitudeDegrees;   
    alt_0 =     (double)GPS.altitude;
  
  // illuminate status LEDs to verify stuff works
  
  // ESC and Servo initialization

  // Stepper motor initialization
}
//-----------------------------------END SETUP---------------------------------------
//-----------------------------------------------------------------------------------



// -------- Interrupt service routine -- looks for new GPS data and stores it -------
SIGNAL(TIMER0_COMPA_vect) {
  char c = GPS.read();
  // if you want to debug, this is a good time to do it!
  
  // I think we can comment the following thing out...
#ifdef UDR0
  if (GPSECHO)
    if (c) UDR0 = c;  
    // writing direct to UDR0 is much much faster than Serial.print 
    // but only one character can be written at a time. 
#endif
}


// --------------------------- Interrupt enable function ------------------------------
void useInterrupt(boolean v) {
  if (v) {
    // Timer0 is already used for millis() - we'll just interrupt somewhere
    // in the middle and call the "Compare A" function above
    OCR0A = 0xAF;
    TIMSK0 |= _BV(OCIE0A);
    usingInterrupt = true;
  } else {
    // do not call the interrupt function COMPA anymore
    TIMSK0 &= ~_BV(OCIE0A);
    usingInterrupt = false;
  }
}


uint32_t timer = millis();


//--------------------------------------------------------------------------------------
// ---------------------------------- BEGIN LOOP ---------------------------------------
void loop() {

    /*________________________________________________________________________________*/
    /*__________________________________MEASUREMENT___________________________________*/
    /* Read and Condition Data from GPS and Altimeter */

    // gps data has to be read and parsed. I think since we are using the ISR, we don't
    // need to worry about the thing immediately below
    if (! usingInterrupt) {
    // read data from the GPS in the 'main loop'
    char c = GPS.read();
    // if you want to debug, this is a good time to do it!
    if (GPSECHO)
      if (c) Serial.print(c);
      }

    // if a sentence is received, we can check the checksum, parse it...
    if (GPS.newNMEAreceived()) {
    // a tricky thing here is if we print the NMEA sentence, or data
    // we end up not listening and catching other sentences! 
    // so be very wary if using OUTPUT_ALLDATA and trytng to print out data
    //Serial.println(GPS.lastNMEA());   // this also sets the newNMEAreceived() flag to false
  
    if (!GPS.parse(GPS.lastNMEA()))   // this also sets the newNMEAreceived() flag to false
      return;  // we can fail to parse a sentence in which case we should just wait for another
      }

    // if millis() or timer wraps around, we'll just reset it
    if (timer > millis())  timer = millis();


    // proceed to data gathering and calculations every 100ms or so...
    // I bet there is a better way to do this...
    if (millis() - timer > 100) { 
    timer = millis(); // reset the timer
    
    // getting lat and long current data is something like this:        
    long_deg_i = (double)GPS.longitudeDegrees;
    lat_deg_i = (double)GPS.latitudeDegrees;   
    alt_i =     (double)GPS.altitude;
    // I don't know for sure if this type conversion is valid


    // I used long beach as a reference to map lat and long to xy grid in feet
    // Assuming lat and long data are in decimal degrees, and altitude is in meters.
    // I have no idea if the gps data is like this.
    
    x = (long_deg_i - long_deg_0)*301837;
    y = (lat_deg_i - lat_deg_0)*196850;
    z = (alt_i - alt_0)*3.28;

    /* Filter the state measurements by calling iir_2() */
    //leave this out for initial tests
    x = iir_2(x,xbuff,a,b);
    y = iir_2(y,ybuff,a,b);
    z = iir_2(z,zbuff,a,b);

    /* Calculate Distance Squared from the Origin */
    dist = x*x + y*y;
    
    /*________________________________________________________________________________*/
    /*____________________________________CONTROL_____________________________________*/
    /* 2 separate controllers 
     *
     * Controller 1: 
     */
       error_dist = RADIUS*RADIUS - dist;         // calculate distance error
       ser_val = iir_2(error_dist,csbuff,acs,bcs); // calculate control effort
     /*
     * Controller 2:
     */
       desc = iir_2(alt_i,descbuff,ad,bd);        // calculate descent rate
       error_desc = descent - desc;               // calculate descent rate error
       fan_val = iir_2(error_desc,cabuff,aca,bca); // calculate control effort   
           
    /*________________________________________________________________________________*/
    /*______________________________OUTPUT CONDITIONING_______________________________*/

    // may have to do some clipping of the control effort values... more later.
    
    //mapping below is for int valued inputs
    fan_val = map(fan_val, 0, 1024, 1000, 2000);    // map actuation value to pulse width    
    ser_val = map(ser_val, -50, 50, 1300, 1700);    // map actuation value to pulse width
    
    fan.writeMicroseconds(fan_val);                // write pwm 
    ser.writeMicroseconds(ser_val);                // write pwm    
  }
}
//-------------------------------------END LOOP----------------------------------------
//-------------------------------------------------------------------------------------



//-------------------------------------------------------------------------------------
//--------------------------DIRECT TRANSPOSE FORM II IIR FILTER------------------------
double iir_2(double input, double buff[2], double a[3], double b[3]){

double in_sum = 0;                  // input accumulator  
double out_sum = 0;                 // output accumulator
//static double buff[2] = {};       // delayed intermediate signals (w[n])

//the input accumulator recieves the input plus the scaled delayed intermediate sigs
in_sum = input;                    
in_sum = in_sum - (a[1]*buff[0]);
in_sum = in_sum - (a[2]*buff[1]);

//the output accumulator recieves scaled delayed intermediate signals
out_sum = b[0]*in_sum;
out_sum = out_sum + (b[1]*buff[0]);
out_sum = out_sum + (b[2]*buff[1]);

buff[1] = buff[0];                  // shift w[n-1] to w[n-2]
buff[0] = in_sum;                   // shift input accumulator value into w[n-1]

return out_sum;                     // return the filtered output
}
//------------------------------------------------------------------------------------
//------------------------------------------------------------------------------------
