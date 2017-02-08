/********************* BASIC AUTONOMOUS CONTROL SKETCH *********************/
/* _TYPE: NONLINEAR ADAPTIVE CONTROLLERg
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
 * -2nd order IIR filter function for smoothing data
 */

#include <Servo.h>
#include <Adafruit_GPS.h>
#include <Wire.h>
#include <SoftwareSerial.h>

/***************************************************************************/
/* GPS Things */
// If you're using a GPS module:
// Connect the GPS Power pin to 5V
// Connect the GPS Ground pin to ground
// If using software serial (sketch example default):
//   Connect the GPS TX (transmit) pin to Digital 3
//   Connect the GPS RX (receive) pin to Digital 2
// If using hardware serial (e.g. Arduino Mega):
//   Connect the GPS TX (transmit) pin to Arduino RX1, RX2 or RX3
//   Connect the GPS RX (receive) pin to matching TX1, TX2 or TX3

// If you're using the Adafruit GPS shield, change 
// SoftwareSerial mySerial(3, 2); -> SoftwareSerial mySerial(8, 7);
// and make sure the switch is set to SoftSerial

// If using software serial, keep this line enabled
// (you can change the pin numbers to match your wiring):
SoftwareSerial mySerial(3, 2);

// If using hardware serial (e.g. Arduino Mega), comment out the
// above SoftwareSerial line, and enable this line instead
// (you can change the Serial number to match your wiring):

//HardwareSerial mySerial = Serial1;


Adafruit_GPS GPS(&mySerial);


// GPSECHO -> 'false' turns off echoing. 'true' turns it on
#define GPSECHO  false

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
double lat_deg_0;                         // latitude measured upon reset (deg)
double lat_deg_i;                         // current latitude             (deg)
double long_deg_0;                        // longitude measured upon reset(deg)
double long_deg_i;                        // current longitude            (deg)
double alt_0;                             // altitude measured upon reset  (m)
double alt_i;                             // current altitude              (m)

double x;                                 // x position (conditioned longitutde)(ft)
double y;                                 // y position (conditioned latitude)  (ft)
double z;                                 // z position (conditioned altitude)  (ft)
int angle;                                // Course over ground angle          (deg)

double dist;                              // distance squared from the oirigin (ft^2)
double desc;                              // DESCENT rate                      (ft/s)
double error_dist;                        // distance error                    (ft^2)     
double error_desc;                        // DESCENT rate error                (ft/s)

const double RADIUS_MAX = 300;            // desired RADIUS from launch site (ft) 
const double RADIUS_MIN = 200;            // minimum desired RADIUS from launch site (ft)
const double DESCENT = 5;                 // desired DESCENT rate (ft/s)


/********************** IIR filter constants and arrays **********************/
/* Filter Coefficients for xyz position */
//these assume a sampling frequency of 5Hz... use Matlab script to calculate
double a[3] = {1, -1.077, 0.2899};        // 2d position data filter feedback
double b[3] = {0.05325, 0.1065, 0.05325}; // 2d position data filter feedforward
double ad[3] = {1, -1.077, 0.2899};       // DESCENT derivative feedback
double bd[3] = {0.5325, 0 -0.5325};       // DESCENT derivative feedforward
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


const int T_SAMP = 200;                   // Sample period (ms)

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

  // Set the update rate to 5Hz
  GPS.sendCommand(PMTK_SET_NMEA_UPDATE_5HZ); 

  // Request updates on antenna status, comment out to keep quiet
  GPS.sendCommand(PGCMD_ANTENNA);

  // timer0 interrupt -- tries to read GPS every 1 ms
   useInterrupt(true);
   delay(10000);
  
  readSTATE();

  long_deg_0 = long_deg_i;            //Store position of the launch site
  lat_deg_0 = lat_deg_i;
  alt_0 = alt_i;
  
  // Also, the cold start response time on the GPS is like 35 seconds. We have to make sure
  // we spend enough time looking for data so we store good data for the launch site location

  
  // illuminate status LEDs to verify stuff works
  
  // ESC and Servo initialization

  // Stepper motor initialization
}
//-----------------------------------END SETUP---------------------------------------
//-----------------------------------------------------------------------------------


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
//-------------------------------------------------------------------------------------

uint32_t timer = millis();


//--------------------------------------------------------------------------------------
// ---------------------------------- BEGIN LOOP ---------------------------------------
void loop() {
    /*__________________________________MEASUREMENT___________________________________*/
    /* Read and Condition Data from GPS and Altimeter */
    readSTATE();                          //function reads lat, long, alt and bearing

    // if millis() or timer wraps around, reset it
    if (timer > millis())  timer = millis();

    // proceed to calculations every T_SAMP (ms)
    // I bet there is a better way to do this...
    if (millis() - timer > T_SAMP) { 
    timer = millis(); // reset the timer

    transform_coordinates();            //convert lat, long and altitude to xyz in feet    

    /* Filter the state measurements by calling iir_2() */
    //leave this out for initial tests
    //x = iir_2(x,xbuff,a,b);
    //y = iir_2(y,ybuff,a,b);
    //z = iir_2(z,zbuff,a,b);

    /* Calculate Distance Squared from the Origin */
    dist = x*x + y*y;
    
    /*________________________________________________________________________________*/
    /*____________________________________CONTROL_____________________________________*/
     error_dist = RADIUS_MAX*RADIUS_MAX - dist;         // calculate distance error
     
      /* Controller 1: Turning Radius */
     if (dist < RADIUS_MIN*RADIUS_MIN){ser_val = 0;}
     else if (dist > RADIUS_MAX*RADIUS_MAX)
        {ser_val = error_dist*10;}            
     else {ser_val = ser_val;}
     
     /* Controller 2: Descent Rate */
       desc = iir_2(alt_i,descbuff,ad,bd);          // calculate DESCENT rate
       error_desc = DESCENT - desc;                 // calculate DESCENT rate error
       fan_val = iir_2(error_desc,cabuff,aca,bca);  // calculate control effort   
           
    /*____________________________________OUTPUT_______________________________________*/

    output_conditioning();        
    fan.writeMicroseconds(fan_val);                // write pwm 
    ser.writeMicroseconds(ser_val);                // write pwm    
    }
}
//-------------------------------------END LOOP----------------------------------------
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



//------------------------------GPS READING FUNCTION----------------------------------
void readSTATE(){
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
      //Serial.println(GPS.lastNMEA());   // sets the newNMEAreceived() flag to false
  
      if (!GPS.parse(GPS.lastNMEA()))   // sets the newNMEAreceived() flag to false
        return;  // wait for another sentence if there is failure to parse
        }    
    
    // What is the minimum of data that we need?
    // --x,y position relative to launch site
    // --altitude relative to launch site
    // --bearing angle     
    long_deg_i =  (double)GPS.longitudeDegrees;
    lat_deg_i =   (double)GPS.latitudeDegrees; 
    angle =       (int)GPS.angle;               //assuming NED for now
    alt_i =       (double)GPS.altitude; 
    
    // if angle is in ENU, uncomment the following:
    // angle = angle - 90;
    // angle = -angle;
    // angle = angle%360;
}


//------------------------------OUTPUT CONDITIONING------------------------------------
void output_conditioning(){
  // Clipping for actuation signals:
    if(ser_val < -50)       {ser_val = -50; }  
    else if(ser_val > 50)   {ser_val = 50; }
    else                 

    if(fan_val < 0)         {fan_val = 0; }
    else if (fan_val > 1000){fan_val = 1000; }
    else
    
    //mapping below is for int valued inputs
    fan_val = map(fan_val, 0, 1024, 1000, 2000);    // map actuation value to pulse width    
    ser_val = map(ser_val, -50, 50, 1300, 1700);    // map actuation value to pulse width
}

void transform_coordinates(){
  // I used long beach as a reference to map lat and long to xy grid in feet
    // Assuming lat and long data are in decimal degrees, and altitude is in meters.
    // I have no idea if the gps data is like this.    
    x = (long_deg_i - long_deg_0)*301837;
    y = (lat_deg_i - lat_deg_0)*196850;
    
    z = (alt_i - alt_0)*3.28;           //may be better to read from barometer
}


//-----------------------------WHAT REGION ARE WE IN?-------------------------------
int region(double x, double y){
  int product = 0;
  int sum = 0;
  int difference = 0;
  x = int(x);
  y = int(y);

// Eventually, I will figure out a way to get this thing to tell us what to do based
// on the region that the glider is in around the origin

  return region;
}



