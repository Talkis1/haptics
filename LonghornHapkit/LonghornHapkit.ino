
//--------------------------------------------------------------------------
// Ann Majewicz Fey, University of Texas at Austin
// Last Modified: 08.27.21
// Code to test basic functionaility of the Longhorn Hapkit (w/ encoder)
//--------------------------------------------------------------------------

// INCLUDES



#define ENCODER_OPTIMIZE_INTERRUPTS
#include <TimerOne.h>  // This library manages the timing of the haptic loop 
#include <Encoder.h>   // This library manages the encoder read.
// #define Spring(k, x) (-k*x)
// #define Damping(v,b) (b*v)
double stall_Torque = 0.0167; //Nm, found from motor datasheet.
char input;


// Pin Declarations
const int PWMoutp = 4;
const int PWMoutn = 5;
const int PWMspeed = 11;

const int encoder0PinA = 2;
const int encoder0PinB = 3;

Encoder encoder(encoder0PinA,encoder0PinB);

double encoderResolution = 48;
double pos = 0; 
double lastPos = 0; 
double lastVel = 0; 

// Kinematics variables
double xh = 0;           // position of the handle [m]
double theta_s = 0;      // Angle of the sector pulley in deg
double xh_prev;          // Distance of the handle at previous time step
double xh_prev2;
double dxh;              // Velocity of the handle
double dxh_prev;
double dxh_prev2;
double dxh_filt;         // Filtered velocity of the handle
double dxh_filt_prev;
double dxh_filt_prev2;

// *******************************************
// UNCOMMENT THESE AND INCLUDE CORRECT NUMBERS
// *******************************************
double rh = 0.09;   //[m] 
double rp = 0.005;  //[m] 
double rs = 0.074;  //[m] 
// *******************************************

// Force output variables
double force = 0;           // force at the handle
double Tp = 0;              // torque of the motor pulley
double duty = 0;            // duty cylce (between 0 and 255)
unsigned int output = 0;    // output command to the motor


// Timing Variables: Initalize Timer and Set Haptic Loop
boolean hapticLoopFlagOut = false; 
boolean timeoutOccured = false; 
#define spring(k_val, x_val) (k_val*x_val)
#define damp(b_val, v_val) (b_val*v_val)
//--------------------------------------------------------------------------
// Initialize
//--------------------------------------------------------------------------
void setup()
{
  // Set Up Serial
  Serial.begin(115200);

  // Output Pins
  pinMode(PWMoutp, OUTPUT);
  pinMode(PWMoutn, OUTPUT);
  pinMode(PWMspeed, OUTPUT);

 // Haptic Loop Timer Initalization
  Timer1.initialize(); 
  long period = 1000; // [us]  10000 [us] - 100 Hz 
  long frequency = 1/period;
  Timer1.attachInterrupt(hapticLoop,period); 

  // Init Position and Velocity
  lastPos = encoder.read();
  lastVel = 0;

  // Initalize motor direction and set to 0 (no spin)
  digitalWrite(PWMoutp, HIGH);
  digitalWrite(PWMoutn, LOW);
  analogWrite(PWMspeed, 0);
  
}

//--------------------------------------------------------------------------
// Main Loop
//--------------------------------------------------------------------------

void loop(){
  if(timeoutOccured){
    Serial.println("timeout occured");
  }
}

// --------------------------
// Haptic Loop
// --------------------------
void hapticLoop(){

  // See if flag is out (couldn't finish before another call) 
  if(hapticLoopFlagOut)
  {
    timeoutOccured = true;
  }
  //*************************************************************
  //*** Section 1. Compute position and velocity using encoder (DO NOT CHANGE!!) ***  
  //*************************************************************
  pos = encoder.read();
  double vel = (.80)*lastVel + (.20)*(pos - lastPos)/(.01);


  //*************************************************************
  //*** Section 2. Compute handle position in meters ************
  //*************************************************************

  // ADD YOUR CODE HERE
  double theta_pul = ((2*3.14)/48)*pos;
  double xh = ((rh*rp)/rs)*theta_pul;  // 5.96*theta_pul;
    
  // xh = xh; // convert xh to meters from cm

  // SOLUTION:
  // Define kinematic parameters you may need
    
  // Step 2.1: print updatedPos via serial monitor
  //*************************************************************

  //Serial.println(updatedPos);
      
  // Step 2.2: Compute the angle of the sector pulley (ts) in degrees based on updatedPos
  //*************************************************************

  //  double ts = ; // NOTE: You can get this from the encoder. 

  // Step 2.3: Compute the position of the handle based on ts
  //*************************************************************

  //  xh = rh*(ts*3.14159/180);       // Again, these numbers may not be correct. You need to determine these relationships. 

  // Step 2.4: print xh via serial monitor
  //*************************************************************

  //Serial.println(xh,5);
      
  // Step 2.5: compute handle velocity
  //*************************************************************
  //  vh = -(.95*.95)*lastLastVh + 2*.95*lastVh + (1-.95)*(1-.95)*(xh-lastXh)/.0001;  // filtered velocity (2nd-order filter)
  //  lastXh = xh;
  //  lastLastVh = lastVh;
  // lastVh = vh;

  //*************************************************************
  //*** Section 3. Assign a motor output force in Newtons *******  
  //*************************************************************

  // Init force 
    
  // int var = 1;
  // double x_wall = -.005;
  // int b = 30;
  // int k = 300;
  // int force = 0;
  // Serial.println(k);
  // Serial.println(Spring(k,xh));
  // if (xh < x_wall){
  //       double force = Spring(k, xh);}
  // else{
  //   double force = 0;}
  // switch (var){
  //   case 1:
  //     if (xh < x_wall){
  //       double force = Spring(k, xh);}
  //     else{
  //       double force = 0;
  //     }
  //     break;
  //   case 2:
  //     double force = Damping(vel, b);
  //     break;
  // }


  // This is just a simple example of a haptic wall that only uses encoder position.
  // You will need to add the rest of the following cases. You will want to enable some way to select each case. 
  // Options for this are #DEFINE statements, swtich case statements (i.e., like a key press in serial monitor), or 
  // some other method. 
  
  // Virtual Wall 
  //*************************************************************
  
  
  // Linear Damping 
  //*************************************************************
  // double B = 1; // damping constant
  // double damping = velocity * B;

  // Nonlinear Friction
  //*************************************************************
  

  // A Hard Surface 
  //*************************************************************
  

  // Bump and Valley  
  //*************************************************************


  // Texture 
  //*************************************************************

  // CHALLENGE POINTS: Try simulating a paddle ball! Hint you need to keep track of the virtual balls dynamics and 
  // compute interaction forces relative to the changing ball position.  
  //*************************************************************
    


  //*************************************************************
  //*** Section 4. Force output (do not change) *****************
  //*************************************************************

  // Determine correct direction 
  //*************************************************************
  // double force = spring(xh, int 300, double -.005);
  double force = .5;
  double b = .01;
  int k = 100;
  double x_wall = -.005;
  double hapticWall = .5;
  if (xh<x_wall){
    hapticWall = spring(k,xh);
  }
  else{
    hapticWall = 0;
  }
  double damping = damp(b,vel);

  bool forceType = false;

  if (forceType){
    force = hapticWall;
  }
  else{
    force = damping;
  }

  if (abs(force)>0){
    digitalWrite(PWMoutp, HIGH);
    digitalWrite(PWMoutn, LOW);
  }
  else {
    digitalWrite(PWMoutp, LOW);
    digitalWrite(PWMoutn, HIGH);
  } 
    
  // Convert force to torque, limit torque to motor, and write out
  //*************************************************************
  Tp = force *((rh*rp)/rs); // ans: force*(rh*rp)/rs; //torque = ? See slides for relationship
  double force_max=stall_Torque/((rh*rp)/rs);
  // Add some code here to limit Tp based on the stall torque. 
  if(abs(Tp) > stall_Torque){
    Tp = stall_Torque;
  }

  // Write out the motor speed.
  //*************************************************************    
  analogWrite(PWMspeed, (abs(Tp)/stall_Torque)*255); // This ensures we aren't writing more than the motor can provide

  Serial.println((abs(Tp)/stall_Torque)*255);
        // Serial.println(force);
  
  // Update variables 
  lastVel = vel;
  lastPos = pos; 

  while(Serial.available()){
    input = Serial.read();
    if(input == 'E'){
      Serial.println("Program Terminated.");
      digitalWrite(PWMoutp, LOW);
      digitalWrite(PWMoutn, LOW);
      analogWrite(PWMspeed, abs(0));
      exit(0);
    }
  }

}
