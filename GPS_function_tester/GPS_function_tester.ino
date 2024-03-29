/************ BASIC AUTONOMOUS CONTROL SKETCH FOR DEBUGGING **************/
/* _TYPE: NONLINEAR ADAPTIVE CONTROLLERg
 * This sketch amounts to a debugger for an autonomous controller for controller 1
 * as per the controller_nonlinear sketch in this repo
 * _FLIGHTPATH CONTROL SUBSYSTEM:
 * --INPUT: 3d position relative to launch site provided by GPS
 * --OUTPUT: servo angle and fan speed
 * _DEPLOYMENT ROUTINE SUBSYSTEM:
 * --INPUT: 2 digital signals -- start stepper, stop stepper
 * --OUTPUT: Stepper motor drive
 * 
 * FEATURES:
 * -2nd order IIR filter function for smoothing data
 * -a bunch of serial prints for debugging.
 */

#include <Servo.h>
#include <Adafruit_GPS.h>
#include <SoftwareSerial.h>

/***************************************************************************/
/* GPS Things */
// If using software serial (sketch example default):
//   Connect the GPS TX (transmit) pin to Digital 3
//   Connect the GPS RX (receive) pin to Digital 2
// If using hardware serial (e.g. Arduino Mega):
//   Connect the GPS TX (transmit) pin to Arduino RX1, RX2 or RX3
//   Connect the GPS RX (receive) pin to matching TX1, TX2 or TX3

SoftwareSerial mySerial(3, 2);
//HardwareSerial mySerial = Serial1;

Adafruit_GPS GPS(&mySerial);

// GPSECHO -> 'false' turns off echoing. 'true' turns it on
#define GPSECHO  false

// set to use or not use interrupt -- you can change in the setup section
boolean usingInterrupt = true;
void useInterrupt(boolean); // Func prototype keeps Arduino 0023 happy
void readSTATE();
void storeSTATE();
void transform_coordinates();
double iir_2(double, double[2], double[3], double[3]);
int region(double, double);
int bearing_command(int);
int ang_err_condition(int);
void output_conditioning();

/*****************************************************************************/
/* Servo Things */
Servo fan;                                // create servo object for fan
Servo ser;                                // create servo object for servo
int fan_val = 1000;                       // fan actuation variable
int ser_val = 1500;                       // servo actuation variable

/* Measurements and State Variables */
double lat_deg_0;                         // latitude measured upon reset (deg)
double lat_deg_i;                         // current latitude             (deg)
double long_deg_0;                        // longitude measured upon reset(deg)
double long_deg_i;                        // current longitude            (deg)
double alt_0;                             // altitude measured upon reset  (m)
double alt_i;                             // current altitude              (m)
unsigned int angle_i;                     // Course over ground angle     (deg)

double x;                                 // x position (conditioned longitutde)(ft)
double y;                                 // y position (conditioned latitude)  (ft)
double z;                                 // z position (conditioned altitude)  (ft)
double dist;                              // distance squared from the oirigin (ft^2)
double desc;                              // DESCENT rate                      (ft/s)
uint8_t region_i;                         // region based on x and y coordinates ()

/*--ERROR SIGNALS--*/
double error_dist;                        // distance error                    (ft^2) 
double error_angle;                       // angle error                        (deg)    
double error_desc;                        // DESCENT rate error                (ft/s)
/*--CONTROL INPUTS--*/
unsigned int angle_c;                     // desired bearing angle
const double RADIUS_MAX = 20;            // desired RADIUS from launch site (ft) 
const double RADIUS_MIN = 10;            // minimum desired RADIUS from launch site (ft)
const double DESCENT_1 = 0;               // desired DESCENT rate (ft/s)
const double DESCENT_2 = 1;
const double SER_BIAS = 10;               // servo position for desired turning radius
/*--CONTROLLER GAINS--*/
double K_ANGLE = 1;
double K_DIST = 1;
double K_DESC = 1; 
/*--CONTROLLER SELECTOR--*/
uint8_t controller;
/********************** IIR filter constants and arrays **********************/
/*--Filter Coefficients for xyz position, descent rate and compensation--*/
//these assume a sampling frequency of 5Hz... use Matlab script to calculate
double a[3] = {1, -1.077, 0.2899};        // 2d position data filter feedback
double b[3] = {0.05325, 0.1065, 0.05325}; // 2d position data filter feedforward
double ad[3] = {1, .4, .04};              // DESCENT derivative feedback
double bd[3] = {.72, 0 -.72};             // DESCENT derivative feedforward
//double acs[3] = {1,1,1};                  // steering controller feedback
//double bcs[3] = {1,1,1};                  // steering controller feedforward
//double aca[3] = {1,1,1};                  // altitude controller feedback
//double bca[3] = {1,1,1};                  // altitude controller feedforward
/*--Buffers--*/
//static double xbuff[2] = {};              // Buffer for intermediate xs
//static double ybuff[2] = {};              // Buffer for intermediate ys
//static double zbuff[2] = {};              // Buffer for intermediate zs
static double descbuff[2] = {};           // Buffer for intermidiate dz/dts
//static double csbuff[2] = {};             // Buffer for steering compensator
//static double cabuff[2] = {};             // Buffer for altitude compensator
/*--SAMPLE TIME--*/
const int T_SAMP = 1000;                  // Sample period (ms)
uint32_t timer = millis();                // Timer for sampler
//-----------------------------------------------------------------------------
//---------------------------------BEGIN SETUP---------------------------------
void setup() {
  
  /* initializations */
  Serial.begin(115200);
  Serial.println("testing the thingy");
  GPS.begin(9600); 

  /* PWM Output Pin Setup */
  // PWM pins on arduino pro mini are 3,5,6,9,10,11
  fan.attach(10);  
  ser.attach(11);
  //pinMode(10,OUTPUT);
  //pinMode(9,OUTPUT);
  fan.writeMicroseconds(1000);                 // write pwm 
  ser.writeMicroseconds(1500);                 // write pwm  

  /* GPS Setup */
  // uncomment to turn on recommended minimum fix data including altitude
  GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCGGA);
  // uncomment to turn on only recomended minimum data
  //GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCONLY);

  // Set the update rate to 5Hz
  GPS.sendCommand(PMTK_SET_NMEA_UPDATE_1HZ); 

  // Request updates on antenna status, comment out to keep quiet
  GPS.sendCommand(PGCMD_ANTENNA);

  // timer0 interrupt -- tries to read GPS every 1 ms
   useInterrupt(true);
   delay(1000);
     // while(GPS.fix!=1){
   //Serial.println(GPS.fix);
      //  delay(100);
    //}
  while(alt_0 == 0){
  readSTATE();
  storeSTATE();
  long_deg_0 = long_deg_i;            //Store position of the launch site
  lat_deg_0 = lat_deg_i;
  alt_0 = alt_i;
  }

  Serial.println("\nTEST OF CONTROLLER 1: DRIVE AROUND WITH GPS OUT THE WINDOW!");
  Serial.print("location of origin: ");
  Serial.print(lat_deg_0); Serial.print(", "); Serial.println(long_deg_0);
  Serial.print("altitude: ");
  Serial.println(GPS.altitude);
  Serial.println("data to come will be in the following format:");
  Serial.println("x,   y,   z;");

  
  // LED must illuminate only if the location in degrees measured is nonzero
  
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

//--------------------------------------------------------------------------------------
// ---------------------------------- BEGIN LOOP ---------------------------------------
void loop() {
    readSTATE();
    /*________________________________SAMPLING TIMER___________________________________*/
    if (timer > millis())  timer = millis();  // wrap timer reset
    if (millis() - timer >= T_SAMP) {         // once we've counted up to T_SAMP, do stuff
    timer = millis();                       
    /*_________________________MEASUREMENT AND DATA FILTERING__________________________*/
    storeSTATE();                              // function reads lat, long, alt, bearing
    transform_coordinates();                  // convert lat, long and alt to xyz in (ft)
    /* Filter the state measurements by calling iir_2() */
    //x = iir_2(x,xbuff,a,b);
    //y = iir_2(y,ybuff,a,b);
    //z = iir_2(z,zbuff,a,b);
    desc = iir_2(z,descbuff,ad,bd);                   // estimate current descent rate
    dist = x*x + y*y;                                 // calcualte distance^2 from the origin     
     /*____________________________________CONTROL_____________________________________*/
    controller = (dist > 1.5*RADIUS_MAX*RADIUS_MAX)? 1:2; // controller phase determined by dist
    //controller = 1; 
//    Serial.print(controller);   Serial.print(",");
//    Serial.print(x,1);          Serial.print(","); 
//    Serial.print(y,1);          Serial.print(","); 
//    Serial.print(z,1);          Serial.println(";");
    switch(controller){
      case 1: /* Controller 1: go straight toward the launch site */
        region_i = region(x,y);                       // determine what region we arein
        angle_c = bearing_command(region_i);          // determine angle over ground we should have
        error_angle = angle_c - angle_i;              // calculate angle error
        error_angle = ang_err_condition(error_angle); // wrap angle error
        error_desc = DESCENT_1 - desc;                // calculate descent rate error for phase 1
        ser_val = int(error_angle*K_ANGLE);
//      Serial.print("R: ");
//      Serial.println(region_i);     //Serial.print(",");
//      Serial.print("AC: ");
//      Serial.println(angle_c);      //Serial.print(",");
//      Serial.print("AI: ");
//      Serial.println(angle_i);      //Serial.print(",");
      //Serial.print("EA: ");
      //Serial.println(error_angle,1);//Serial.print(",");
//      Serial.print("DC: ");
//      Serial.println(DESCENT_1);    //Serial.print(",");
//      Serial.print("D: ");
//      Serial.println(desc,1);       //Serial.print(",");
      //Serial.print(error_desc,1); //Serial.print(",");         
        break;
      case 2: /* Controller 2: spiral downard around launch site */
        error_desc = DESCENT_2 - desc;                // calcualte descent rate error for phase 2
        if(dist < RADIUS_MIN*RADIUS_MIN){ser_val = 0;}// go straight if within RADIUS_MIN
        else if(dist > RADIUS_MAX*RADIUS_MAX){
          error_dist = RADIUS_MAX*RADIUS_MAX - dist;
          ser_val = int(error_dist*K_DIST);}              // apply compensation if beyond RADIUS_MAX
        else  {ser_val = SER_BIAS;}                   // sert servo to the bias angle
//      Serial.print("RC: ");
//      Serial.println(RADIUS_MAX*RADIUS_MAX); // Serial.print(",");
//      Serial.print("RI: ");
//      Serial.println(dist);                   //Serial.print(",");
//      Serial.print("DR: ");
//      Serial.println(desc);
      //Serial.print(error_dist);             Serial.print(",");
        break;
      default:
        break;
    }
    
    fan_val = int(error_desc*K_DESC);                   // calculate control effort for fan                    
    /*____________________________________OUTPUT_______________________________________*/
    output_conditioning();        
    //fan.writeMicroseconds(fan_val);                 // write pwm 
    //ser.writeMicroseconds(ser_val);                 // write pwm  
//   Serial.print("ser: ");
//   Serial.println(ser_val);     //Serial.print(",");
//   Serial.print("fan: ");
//   Serial.println(fan_val);     //Serial.println(";");
//   Serial.println("\n--------------------");
   //print stuff   
     
    }
    fan.writeMicroseconds(fan_val);                 // write pwm 
    ser.writeMicroseconds(ser_val);                 // write pwm  
 
}
//-------------------------------------END LOOP----------------------------------------
//-------------------------------------------------------------------------------------





//------------------------------GPS READING FUNCTION----------------------------------
void readSTATE(){
// gps data has to be read and parsed. I think since we are using the ISR, we don't
    // need to worry about the thing immediately below
    if (! usingInterrupt) {
    // read data from the GPS in the 'main loop'
      char c = GPS.read();
    // if you want to debug, this is a good time to do it!
     // if (GPSECHO)
        //if (c) Serial.print(c);
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
}
void storeSTATE(){
    // What is the minimum data that we need?
    // --x,y position relative to launch site
    // --altitude relative to launch site
    // --bearing angle
    //while(!GPS.fix){
    //  delay(1);
    //}
    
    if(GPS.fix){
    long_deg_i =  (double)GPS.longitudeDegrees;
    lat_deg_i =   (double)GPS.latitudeDegrees; 
    angle_i =     (int)GPS.angle;               //assuming NED for now
    alt_i =       (double)GPS.altitude; 
    }
    // if angle is in ENU, uncomment the following:
    // angle_i = (360-(angle_i-90))%360;
}

//--------------------TRANSFROM LONG, LAT, ALTITUDE TO X,Y,Z--------------------------
void transform_coordinates(){
  // I used long beach as a reference to map lat and long to xy grid in feet
    // Assuming lat and long data are in decimal degrees, and altitude is in meters.
    // I have no idea if the gps data is like this.    
    x = (long_deg_i - long_deg_0)*302777;
    y = (lat_deg_i - lat_deg_0)*364574;    
    z = (alt_i - alt_0)*3.28;           //may be better to read from barometer
}

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

//-----------------------------WHAT REGION ARE WE IN?----------------------------------
int region(double x1, double x2){
  //divides x and y coordinate system into 8 pie slices, 
  //------------------//
  //        y         //
  //  \     |     /   //
  //    \ 3 | 2 /     //
  //____4_\_|_/_1___x //
  //    5 / | \ 8     //
  //    / 6 | 7 \     //
  //  /     |     \   //
  //------------------//
  //tells you which one coordinate (x1,x2) is in.
  uint8_t reg =     0;
  uint8_t quad =    0;
  int product = 0;
  int sum =     0;
  int diff =    0;
  unsigned int ratio =   0;
  
  x1 = int(x1);                      //type conversion to speed up math and division by 32
  x2 = int(x2);                      //to ensure no overflow while within 1 mile of origin

  if(x1==0)           {x1 = 1;}         //easy way to deal with being on axis
  if(x2==0)           {x2 = 1;}         //don't consider it a possiblity :)
  
  product = x1*x2;                      //elementary operations
  sum =     x1+x2;
  diff =    x1-x2;
  ratio = abs(x2/x1);

  if(product>0){                        //determine quadrant from elementary ops
    if      (sum>0)   {quad = 1;}
    else if (sum<0)   {quad = 3;}
  }
  else if(product<0){
    if      (diff<0)  {quad = 2;}
    else if (diff>0)  {quad = 4;}
  }
  else                {quad = 0;}
  
  reg = 2*quad;                         // determine region from quadrant and quotient 
  if((quad%2)==1){
    if(ratio<1)         {reg = reg - 1;}   
  }
  if((quad%2)==0){
    if(ratio>0)         {reg = reg - 1;}
  }

  return reg;                           // return integer value of the region
}

//---------------------WHAT ANGLE OVER GROUND SHOULD WE BE GOING?-----------------------
int bearing_command(int locus){
  int ENU = 0;
  int NED = 0;
  //The region we find ourselves in determines what angle we ought to be going
  //If the GPS outputs an angle over ground in ENU...
  ENU = (45*((locus+3)%8)+22)%360;
  NED = (360-(ENU-90))%360;

  return NED;                           // remember to check if NED is the correct thing
}

//-------------------------------CONDITION ANGLE ERROR----------------------------------
int ang_err_condition(int err){
  //wraps angle error around so |error| is never more than 180. 
  if(err >= 180)            {err = err - 360;}
  else if(err <= -180)      {err = err + 360;}
  else                      {err = err;}
  
  return err;
}

//------------------------------OUTPUT CONDITIONING--------------------------------------
void output_conditioning(){
  // we have to figure out more about the physical system to properly do this
  // making it work is just a matter of knowing the thrust characteristics of the fan
  // and the relationship between turning radius and servo angle
  // Clipping for actuation signals:
//   Serial.print("ser: ");
//   Serial.println(ser_val);     //Serial.print(",");
//   Serial.print("fan: ");
//   Serial.println(fan_val);     //Serial.println(";");
//   Serial.println("\n--------------------");
    if(ser_val < -50)       {ser_val = -50; }  
    else if(ser_val > 50)   {ser_val = 50; }
    else {}                

    if(fan_val < -10)         {fan_val = -10; }
    else if (fan_val > 10){fan_val = 10; }
    else{}
    
    //mapping below is for int valued inputs
    fan_val = map(fan_val, 0, 10, 1000, 2000);    // map actuation value to pulse width    
    ser_val = map(ser_val, -50, 50, 1450, 1550);    // map actuation value to pulse width
}


