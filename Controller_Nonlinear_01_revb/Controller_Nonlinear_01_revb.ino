
/********************* BASIC AUTONOMOUS CONTROL SKETCH *********************/
/* _TYPE: NONLINEAR ADAPTIVE CONTROLLER
 * This sketch amounts to an autonomous controller for a spiral flight path
 * for the csulbusli2017 paraglider thing.
 * _FLIGHTPATH CONTROL SUBSYSTEM:
 * --INPUT: 3d position relative to launch site provided by GPS
 * --OUTPUT: servo angle and fan speed
 * _DEPLOYMENT ROUTINE SUBSYSTEM:
 * --INPUT: serial data from gps, enable signal from altimeter,
 * --OUTPUT: servo PWM, fan PWM, enable signal for Pi, various indicator LEDs
 * 
 * FEATURES:
 * -adaptive control with 2 basic controllers:
 *    -if: distance from target is large
 *          *bearing angle is controlled directly based on x,y position
 *    -if: distance from target is small
 *          *distance from target is controlled indirectly with servo angle
 * -2nd order IIR filter function
 */

#include <Servo.h>
#include <Adafruit_GPS.h>
//#include <SoftwareSerial.h>

/***************************************************************************/
/* GPS Things */
//SoftwareSerial mySerial(3, 2);
//Adafruit_GPS GPS(&mySerial);
HardwareSerial mySerial = Serial1;
Adafruit_GPS GPS(&Serial1);

#define GPSECHO  false
boolean usingInterrupt = false;

/*****************************************************************************/
/* Servo Things */
Servo fan;                                // create servo object for fan
Servo ser;                                // create servo object for servo
double fan_val = 0;                       // fan actuation variable
double ser_val = 0;                       // servo actuation variable

/****************************************************************************/
/* Measurements and State Variables */
//states measured from GPS
double lat_deg_0;                         // latitude measured upon reset      (deg)
double lat_deg_i;                         // current latitude                  (deg)
double long_deg_0;                        // longitude measured upon reset     (deg)
double long_deg_i;                        // current longitude                 (deg)
double alt_0;                             // altitude measured upon reset       (m)
double alt_i;                             // current altitude                   (m)
unsigned int angle_i;                     // Course over ground angle          (deg)
//states mapped to control coordinate system
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
unsigned int angle_c;                     // desired bearing angle             (deg)
const double RADIUS_MAX = 250;            // desired RADIUS from launch site    (ft) 
const double RADIUS_MIN = 200;            // minimum RADIUS from launch site    (ft)
const double DESCENT_1  = 0;              // desired DESCENT rate              (ft/s)
const double DESCENT_2  = 1;
const double SER_BIAS   = -5;             // servo position for desired turning radius
const int    SER_MIN    = 1200;           // servo min pulse length             (us)
const int    SER_MAX    = 1800;           // servo max pulse length             (us)
const int    FAN_MIN    = 1000;           // fan min pulse length               (us)
const int    FAN_MAX    = 2000;           // fan max pulse length               (us)

/*--CONTROLLER GAINS--*/
double K_ANGLE  = 5;
double K_DIST   = 5;
double K_DESC   = 5; 

/*--CONTROLLER FLOW--*/
uint8_t controller;
bool    start;

/********************** IIR filter constants and arrays **********************/
/*--Filter Coefficients for xyz position, descent rate and compensation--*/
//this assumes a sampling frequency of 1Hz... use Matlab script to calculate
double ad[3] = {1, -1.333, 0.4444};       // DESCENT derivative feedback
double bd[3] = {0.05556, 0 -0.5556};      // DESCENT derivative feedforward
/*--Buffers--*/
static double descbuff[2] = {};           // Buffer for intermidiate dz/dts

/*--SAMPLE TIMING--*/
const int T_SAMP  = 1000;                 // Sample period (ms)
uint32_t timer    = millis();             // Timer for sampler

/*************************** Function Prototypes *****************************/
void useInterrupt(boolean); 
void readSTATE();
void storeSTATE();
void transform_coordinates();
double iir_2(double, double[2], double[3], double[3]);
int region(double, double);
int bearing_command(int);
int ang_err_condition(int);
void output_conditioning();

//-----------------------------------------------------------------------------
//---------------------------------BEGIN SETUP---------------------------------
void setup() {
  
  /* initializations */
  Serial.begin(115200);
  GPS.begin(9600); 
  Serial.println("Setup is starting to start starting...");
 
  /* GPS Setup */
  GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCGGA); // recommended minimum fix data plus altitude
  GPS.sendCommand(PMTK_SET_NMEA_UPDATE_1HZ);    // update rate = 1Hz 
  GPS.sendCommand(PGCMD_ANTENNA);               // updates on antenna status

  /* PWM Output Pin Setup */
  fan.attach(10, FAN_MIN, FAN_MAX);  
  ser.attach(11, SER_MIN, SER_MAX);
  pinMode(10,OUTPUT);                           // fan output pin
  pinMode(11,OUTPUT);                           // servo output pin
  pinMode(A0,OUTPUT);                           // data indicator LED
  pinMode(A1,INPUT);                            // Turn on control system
  pinMode(A2,OUTPUT);                           // Pi control
  fan.writeMicroseconds(1000);                  // write pwm 
  ser.writeMicroseconds(1500);
  digitalWrite(A4,LOW);                         // keep Pi turned off for now.
    
  start = LOW;                                  // don't start yet!            
   
   useInterrupt(false);                         // timer0 interrupt -- read GPS every 1ms
   delay(2000);                                 // wait a couple seconds

  while(alt_i == 0){                            // don't proceed unless you are getting
    readSTATE();                                // good data
    storeSTATE();  
  }

  for(int ii = 0; ii < 100; ii++){              // read GPS a bunch of times
    digitalWrite(A0,HIGH);
    delay(5);
    readSTATE();  
    storeSTATE();
    long_deg_0 = long_deg_i;                    // Store position of the launch site
    lat_deg_0 = lat_deg_i;
    alt_0 = alt_i;
    digitalWrite(A0,LOW);
    delay(50);
  }

  digitalWrite(A0,HIGH);                        // Status LED: all is well

  digitalWrite(A2,HIGH);                        // turn the pi on to see if it works
  delay(2000);
  digitalWrite(A2,LOW);                         // turn the pi back off for now
  
  Serial.println("\nAUTONOMOUS LANDING CONTROL FOR A PARAGLIDER!");
  Serial.print("location of origin: ");
  Serial.print(lat_deg_0); Serial.print(", "); Serial.println(long_deg_0);
  Serial.print("altitude: ");
  Serial.println(GPS.altitude); 
}
//------------------------------------END SETUP---------------------------------------
//------------------------------------------------------------------------------------


// ---------------------- look for new GPS data and stores it ------------------------
SIGNAL(TIMER0_COMPA_vect) {
  char c = GPS.read();  
#ifdef UDR0
  if (GPSECHO)
    if (c) UDR0 = c;  
#endif

}
// --------------------------- Interrupt enable function -----------------------------
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

    if(digitalRead(A1)==LOW){                 // test output of altimiter
      start = HIGH;                           // flip start to high if low pulse comes
    }
    
    readSTATE();                              // parse some gps data
    /*________________________________SAMPLING TIMER___________________________________*/
    if (timer > millis())  timer = millis();  // wrap timer reset
    if (millis() - timer >= T_SAMP) {         // once we've counted up to T_SAMP, do stuff
    timer = millis();                       
    /*_________________________MEASUREMENT AND DATA FILTERING__________________________*/
    storeSTATE();                             // store data in lat, long, alt, bearing
    transform_coordinates();                  // convert lat, long and alt to xyz in (ft)
    
    if(alt_i > 50){                           // turn the respberrypi on when you launch
      digitalWrite(A2,HIGH);
    }
 
    if(start == HIGH){
    Serial.println("control mode: ON");
    desc = iir_2(z,descbuff,ad,bd);           // estimate current descent rate
    dist = x*x + y*y;                         // calcualte distance^2 from the origin 
     /*____________________________________CONTROL_____________________________________*/
    controller = (dist > 1.5*RADIUS_MAX*RADIUS_MAX)? 1:2; // dist determines controller
    
    Serial.println(controller); Serial.print(",");
    Serial.print("(x,y,z) = (");
    Serial.print(x,1);          Serial.print(","); 
    Serial.print(y,1);          Serial.print(","); 
    Serial.print(z,1);          Serial.println(");");
    
    switch(controller){
      case 1: /* Controller 1: go straight toward the launch site */
        region_i = region(x,y);                       // determine what region we arein
        angle_c = bearing_command(region_i);          // determine angle over ground we should have
        error_angle = angle_c - angle_i;              // calculate angle error
        error_angle = ang_err_condition(error_angle); // wrap angle error
        error_desc = DESCENT_1 - desc;                // calculate descent rate error for phase 1
        ser_val = error_angle*K_ANGLE;

      Serial.print("R: ");
      Serial.println(region_i);
      Serial.print("AC: ");
      Serial.println(angle_c);
      Serial.print("AI: ");
      Serial.println(angle_i);
      Serial.print("DC: ");
      Serial.println(DESCENT_1);   
      Serial.print("D: ");
      Serial.println(desc,1);      
      
        break;        
      case 2: /* Controller 2: spiral downard around launch site */
        
        error_desc = DESCENT_2 - desc;                // calcualte descent rate error for phase 2
        if(dist < RADIUS_MIN*RADIUS_MIN){ser_val = 0;}// go straight if within RADIUS_MIN
        else if(dist > RADIUS_MAX*RADIUS_MAX){
          error_dist = RADIUS_MAX*RADIUS_MAX - dist;
          ser_val = error_dist*K_DIST;}               // apply compensation if beyond RADIUS_MAX
        else  {ser_val = SER_BIAS;}                   // sert servo to the bias angle
     
      Serial.print("RC: ");
      Serial.println(RADIUS_MAX*RADIUS_MAX); 
      Serial.print("RI: ");
      Serial.println(dist);                   
      Serial.print("DR: ");
      Serial.println(desc);
        
        break;        
      default:
        break;
    }
    fan_val = error_desc*K_DESC;                   // calculate control effort for fan                    
    }
    else{
      Serial.println("control mode: OFF");
      fan_val = 0;
      ser_val = 0;
    }
    /*____________________________________OUTPUT_______________________________________*/
    
    output_conditioning();     

    Serial.print("ser: ");
    Serial.println((int)ser_val);
    Serial.print("fan: ");
    Serial.println((int)fan_val);
    Serial.println("\n--------------------");
    }
    fan.writeMicroseconds((int)fan_val);              // write pwm 
    ser.writeMicroseconds((int)ser_val);              // write pwm 
}
//-------------------------------------END LOOP----------------------------------------
//-------------------------------------------------------------------------------------




//------------------------------GPS READING FUNCTIONS----------------------------------
void readSTATE(){
    // gps data has to be read and parsed. 
    if (! usingInterrupt) {
    // read data from the GPS
      char c = GPS.read();
      if (GPSECHO)
        if (c) Serial.print(c);
        } 

    // if a sentence is received, we can check the checksum, parse it...
    if (GPS.newNMEAreceived()) {
      if (!GPS.parse(GPS.lastNMEA()))   // sets the newNMEAreceived() flag to false
        return;  // wait for another sentence if there is failure to parse
        }     
}
void storeSTATE(){
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
    x = (long_deg_i - long_deg_0)*302777;
    y = (lat_deg_i - lat_deg_0)*364574;    
    z = (alt_i - alt_0)*3.28;           
}


//--------------------------DIRECT TRANSPOSE FORM II IIR FILTER------------------------
double iir_2(double input, double buff[2], double a[3], double b[3]){

double in_sum = 0;                  // input accumulator  
double out_sum = 0;                 // output accumulator

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
int region(double x_1, double x_2){
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
  uint8_t reg =     0;
  uint8_t quad =    0;
  int x1;
  int x2;
  int product = 0;
  int sum =     0;
  int diff =    0;
  unsigned int ratio =   0;
  
  x1 = (int)x_1/32;                     //type conversion to speed up math, and division by 32
  x2 = (int)x_2/32;                     //to ensure no overflow will occur if within 1 mile of origin

  if(x1==0)           {x1 = 1;}         //easy way to deal with being on axis
  if(x2==0)           {x2 = 1;}         //don't consider it a possiblity :)
  
  product = x1*x2;                      //elementary operations
  sum =     x1+x2;
  diff =    x1-x2;

  ratio = abs(x2/x1);

  if(product>0){                        //determine quadrant
    if      (sum>0)   {quad = 1;}
    else if (sum<0)   {quad = 3;}
  }
  else if(product<0){
    if      (diff<0)  {quad = 2;}
    else if (diff>0)  {quad = 4;}
  }
  else                {quad = 0;}
  
  reg = 2*quad;                         //ascribe position to a numbered region 
  if((quad%2)==1){
    if(ratio<1)         {reg = reg - 1;}   
  }
  if((quad%2)==0){
    if(ratio>0)         {reg = reg - 1;}
  }

  return reg;                           //return integer value of the region
}


//---------------------WHAT ANGLE OVER GROUND SHOULD WE BE GOING?-----------------------
int bearing_command(int locus){
  int ENU = 0;
  int NED = 0;
  //The region we find ourselves in determines what angle we ought to be going
  //If the GPS outputs an angle over ground in ENU...
  ENU = (45*((locus+3)%8)-1)%360;
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
    
      if      (ser_val < -50)   {ser_val = -50;}  
      else if (ser_val > 50)    {ser_val = 50;}
      else                      {ser_val = ser_val;}

      if      (fan_val < 0)     {fan_val = 0;}
      else if (fan_val > 10)    {fan_val = 10;}
      else                      {fan_val = fan_val;}

    fan_val = map(fan_val, 0, 10, (double)FAN_MIN, (double)FAN_MAX);     
    ser_val = map(ser_val, -50, 50, (double)SER_MIN, (double)SER_MAX);    
}
