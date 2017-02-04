/*CANNOT READ GPS YET!!! CANNOT CONTROL ANYTHING!!! IT DOES NOTHING YET!!!*/
/*THIS IS JUST ME GETTING MY THOUGHTS IN ORDER*/

/********************* BASIC AUTONOMOUS CONTROL SKETCH *********************/
/* 
 * This sketch amounts to an autonomous controller for a simple flight path.
 * It relies on a 3d position input, which will ultimately be provided by 
 * the gps and the altimeter. The data provided by these instruments needs
 * to be conditioned.
 * 
 * FEATURES:
 * -2nd order IIR filter function for removing noise from data
 * -adaptive controller for several input scenarios
 */


#include <Servo.h>

/*****************************************************************************/
/* Servo Things */
Servo fan;                                // create servo object for fan
Servo ser;                                // create servo object for servo
int fanVAL = 0;                           // fan actuation variable
int serVAL = 0;                           // servo actuation variable

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

/********************** IIR filter constants and arrays **********************/
/* Filter Coefficients for xyz position */
const double a[3] = {};                   // to be determined
const double b[3] = {};                   // to be determined
/* Buffers */
static double xbuff[2] = {};              // Buffer for intermediate values
static double ybuff[2] = {};
static double zbuff[2] = {};

/************************* Controller Related Things *************************/
/* Classical Controller Gains */
double kp = 1;                            // Proportional Gain
double ki = 0;                            // Integral Gain
double kd = 0;                            // derivative Gain


void setup() {
  
  /* initializations */
  Serial.begin(9600);                     // set up serial comm for debug
  pinMode(A0, INPUT);                     // pin where theta (x[0]) will be read from

  /* PWM Output Pin Setup */
  fan.attach(7);  
  ser.attach(9);

  // read from sensors
  // figure out when to do GPS.read -- I think this can happen in an interrupt
  
  // store current GPS location, and altitude
  // lat_deg_0 = GPS.latitude;
  // long_deg_0 = GPS.longitude;
  // alt_0 = ...

  // illuminate status LEDs to verify stuff works
  
  // ESC and Servo initialization
  
}

// ---------------------------------- BEGIN LOOP ---------------------------------------
void loop() {

    /*________________________________________________________________________________*/
    /*__________________________________MEASUREMENT___________________________________*/
    /* Read and Condition Data from GPS and Altimeter */

    // gps data has to be read and parsed. I don't have the GPS, so I can't test
    // this, so I will leave it to be figured out later. 
    
    // getting lat and long current data is something like this:
        
    //long_deg_i = GPS.longitude;
    //lat_deg_i = GPS.latitude;    
    // there might be a type conversion -- I don't know.
    // there is also altitude -- on gps -- I don't know if we can use this.

    // I used long beach as a reference to map lat and long to xy grid in feet
    // Assuming lat and long data are in decimal degrees, and altitude is in meters.
    // I have no idea if the gps data is like this.
    
    //x = (long_deg_i - long_deg_0)*301837;
    //y = (lat_deg_i - lat_deg_0)*196850;
    //z = (alt_i - alt_0)*3.28;

    /* Filter the state measurements by calling iir_2() */
    //leave this out for initial tests
    //I will eventually calculate filter coefficients for like a 15 Hz cutoff
    //don't uncomment without the correct filter coefficients
    //x = iir_2(x,xbuf,a,b);
    //y = iir_2(y,ybuf,a,b);
    //z = iir_2(z,zbuf,a,b);

    /* Calculate Distance Squared from the Origin */
    //dist = x*x + y*y;
    
    /*________________________________________________________________________________*/
    /*____________________________________CONTROL_____________________________________*/
    /* 2 interacting controllers 
     *
     * Controller 1: maintain a distance of 300 ft from the target
     * three scenarios: 
     *  (1). current position is much greater than 300 ft from launch site
     *        --fly directly toward the launch site until within 300ft, then go to (2)
     *  (2). current position is very close to 300 ft
     *        --fly in a predefined pattern for a predetermined amount of time
     *        --once spiral downard maintaining 300ft distance from launch site
     *  (3). current position is much closer than 300 ft from target
     *        --fly straight until near 300ft from target, then go to (2)
     *  
     * Controller 2: maintain a reasonable descent rate
     * two scenarios:
     *  (1). current altitude is above a predefined threshold
     *        --maintain a predefined descent rate
     *  (2), current altitude is below a predefined threshold
     *        --flatten out a bit and await manual takeover
     *        
     * Controller will outptu fanVAL and serVAL
     */
    

    /*________________________________________________________________________________*/
    /*______________________________OUTPUT CONDITIONING_______________________________*/
    //mapping below is for int valued inputs
    fanVAL = map(fanVAL, 0, 1024, 1000, 2000);    // map actuation value to pulse width    
    serVAL = map(serVAL, 0, 1024, 1000, 2000);    // map actuation value to pulse width
    
    fan.writeMicroseconds(fanVAL);                // write pwm 
    ser.writeMicroseconds(serVAL);                // write pwm    
}
//-------------------------------------END LOOP----------------------------------------



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
//----------------------------------------------------------------------------------
