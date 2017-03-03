//test rig for region finding function. Ascribes a number between 1 and 8
//based on the x and y coordinate location of the gps relative to launch site.
//------------------//
//        y         //
//        |         //
//      3 | 2       //
//____4___|___1___x //
//    5   |   8     //
//      6 | 7       //
//        |         //
//------------------//
double x = 0;
double y = 0;
float omega = .01;
float t = 0;

uint8_t region_i = 0;
unsigned int ENU = 0;
unsigned int NED = 0;


void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
}

void loop() {

  if(t > 10000) {t = 0;}
  t=t+.9;
  delay(20);
  
  x = 1000*cos(omega*t);
  y = 1000*sin(omega*t);

  region_i = region(x,y);
  ENU = (45*((region_i+3)%8)+22)%360;         //Map region to desired bearing in ENU
  NED = (360-(ENU-90))%360;                   //convert to NED

 //Open up serial plotter or serial monitor to verify this works
 //division by factors of 30 is so it is easy to see everything
  Serial.print(x/30);
  Serial.print(",");
  Serial.print(y/30);
  Serial.print(",");
  Serial.print(region(x,y));
  Serial.print(",");
  Serial.print(ENU/30);
  Serial.print(",");
  Serial.println(NED/30);
  }

//-----------------------------WHAT REGION ARE WE IN?-------------------------------
int region(double x1, double x2){
  uint8_t reg =     0;
  uint8_t quad =    0;
  int product = 0;
  int sum =     0;
  int diff =    0;
  unsigned int ratio =   0;
  
  x1 = int(x1/32);                      //type conversion to speed up math, and division by 32
  x2 = int(x2/32);                      //to ensure no overflow will occur if within 1 mile of origin

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



