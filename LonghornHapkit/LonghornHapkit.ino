
//--------------------------------------------------------------------------
// Ann Majewicz Fey, University of Texas at Austin
// Last Modified: 08.27.21
// Code to test basic functionaility of the Longhorn Hapkit (w/ encoder)
//--------------------------------------------------------------------------

// INCLUDES



#define ENCODER_OPTIMIZE_INTERRUPTS
#include <TimerOne.h>  // This library manages the timing of the haptic loop 
#include <Encoder.h>   // This library manages the encoder read.
#define EFFECT_NONE 0
#define EFFECT_HAPTIC_WALL 1
#define EFFECT_DAMPING 2
#define EFFECT_NONLINEAR_FRICTION 3
#define EFFECT_HARD_SURFACE 4
#define EFFECT_BUMP_VALLEY 5
#define EFFECT_TEXTURE 6
int ACTIVE_EFFECT = EFFECT_NONE;
// #define Spring(k, x) (-k*x)
// #define Damping(v,b) (b*v)
double pi = 3.14159;
double stall_Torque = 0.0167; //Nm, found from motor datasheet.
char input;
double timeStart = 0;
double timeEnd = 0;
double Fsin = 0;


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
double xh_;          // Distance of the handle at previous time step
double xh_prev2;
double vh = 0.0;
double lastXh = 0.0;
double lastLastVh = 0.0;
double lastVh = 0.0;
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
double time = 0;

// Timing Variables: Initalize Timer and Set Haptic Loop
boolean hapticLoopFlagOut = false; 
boolean timeoutOccured = false; 
// #define spring(k_val, x_val) (k_val*x_val)
// #define damp(b_val, v_val) (b_val*v_val)
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
// Functions
//--------------------------------------------------------------------------

double spring(double xh,double x_wall,double k){
  double force = 0.0;
  double hapticWall = 0;
  if (xh<x_wall){
    hapticWall = k*xh;
  }
  else{
    hapticWall = 0;
  }
  return hapticWall;
}

double _damping(double B, double vh){
  double damping = vh * B;
  return damping;
}

double friction(double B, double vh, double Cn, double lastLastVh){
  double changeVh = vh-lastLastVh;
  double damping = _damping(B,vh);
  if (vh < -changeVh){
    // Serial.println("first if");
  force = Cn*constrain(vh,-1,1)+damping;
  }else if (-changeVh < vh && vh < 0){
    // Serial.println("second if");
    force = -3;
  }else if (0 < vh && vh < changeVh){
    // Serial.println("third if");
    force = 3;
  }else if (vh > changeVh){
    // Serial.println("fourth if");
    force = Cn*constrain(vh,-1,1)+damping;
  }
  return force;
}

double hardWall(double x_wall, double maxTime, double xh, double k,
                double amplitude){
  double hapticWall = .5;
  if (xh<=x_wall && xh_ <=x_wall){
    timeStart = millis();
  }else if (xh>x_wall){
    timeEnd = millis();
  }
  double elapsedTime = (timeStart-timeEnd)/1000;
  if (elapsedTime<0){
    elapsedTime = 0;
  }else if (elapsedTime>maxTime){
    elapsedTime=maxTime;
  }
  // Serial.println(x_wall);
  xh_ = xh;
  double freq = (1/(2*pi))*sqrt(k/.5);
  Fsin = amplitude*abs(vh)*exp(log(.01)*elapsedTime/maxTime)*sin(2*pi*elapsedTime*freq);

  if (xh<x_wall){
    hapticWall = k*xh+Fsin;
  }
  else{
    hapticWall = 0;
  }
  return hapticWall;
}

double bump_valley(double xh, double k){
  double x0 = 0;
  double force = 0;
  double maxXh = .065/2;

  force = k*(abs(xh)-maxXh);
  return force;
}

double texture(double xh, double velo, double b){
  double force = 0;
  if (xh>0 && xh<.01 || xh>.02 && xh<.03 || xh>.04 && xh<.05){
    force = b*velo;
  }
  return force;
}

//--------------------------------------------------------------------------
// Main Loop
//--------------------------------------------------------------------------

void loop(){
  if(timeoutOccured){
    Serial.println("timeout occured");
  }
  if (Serial.available() > 0) {
    int userInput = Serial.parseInt(); // Read an integer from the serial monitor
    if (userInput >= EFFECT_NONE && userInput <= EFFECT_TEXTURE) {
      ACTIVE_EFFECT = userInput; // Update the active effect based on user input
      Serial.print("Active Effect is now: ");
      Serial.println(ACTIVE_EFFECT);
    } else {
      Serial.println("Invalid input. Please enter integer 0-6");
    }
    // Clear the serial buffer
    while (Serial.available() > 0) {
      Serial.read();
    }
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
  double theta_pul = ((2*3.14)/48)*pos; // pulley position
  double xh = ((rh*rp)/rs)*theta_pul;  // 5.96*theta_pul; // handle position

  // SOLUTION:
  // Define kinematic parameters you may need
  // parameters were defined above
    
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
  vh = -(.95*.95)*lastLastVh + 2*.95*lastVh + (1-.95)*(1-.95)*(xh-lastXh)/.0001;  // filtered velocity (2nd-order filter)
  lastXh = xh;
  lastLastVh = lastVh;
  lastVh = vh;

  //*************************************************************
  //*** Section 3. Assign a motor output force in Newtons *******  
  //*************************************************************

  // This is just a simple example of a haptic wall that only uses encoder position.
  // You will need to add the rest of the following cases. You will want to enable some way to select each case. 
  // Options for this are #DEFINE statements, swtich case statements (i.e., like a key press in serial monitor), or 
  // some other method. 
  
  // Virtual Wall 
  //*************************************************************

  // Linear Damping 
  //*************************************************************

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


  double B = .3; // damping constant
  double Cn=.01;
  double x_wall = 0;
  double maxTime = 20;
  int k = 200;
  double amplitude = .5;
  // force = spring(xh, 0, k);
  // force = _damping(B, vh);
  // force = friction(B, vh, Cn, lastLastVh);
  // force = hardWall(x_wall, maxTime, xh, k, amplitude);
  // force = bump_valley(xh, 30);
  // force = texture(xh, vh, B);
    switch (ACTIVE_EFFECT) {
    case EFFECT_NONE:
      force = 0;
      break;
    case EFFECT_HAPTIC_WALL:
      force = spring(xh, 0, k);
      break;
    case EFFECT_DAMPING:
      force = _damping(B, vh);
      break;
    case EFFECT_NONLINEAR_FRICTION:
      force = friction(B, vh, Cn, lastLastVh);
      break;
    case EFFECT_HARD_SURFACE:
      force = hardWall(x_wall, maxTime, xh, k, amplitude);
      break;
    case EFFECT_BUMP_VALLEY:
      force = bump_valley(xh, 30);
      break;
    case EFFECT_TEXTURE:
      force = texture(xh, vh, B);
      break;
    default:
      force = 0;
      break;
  }
    
  Serial.println(xh);
  if (force>0){
    digitalWrite(PWMoutp, HIGH);
    digitalWrite(PWMoutn, LOW);
  }
  else {
    digitalWrite(PWMoutp, LOW);
    digitalWrite(PWMoutn, HIGH);

  } 
  // force = 2.75;
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
  // Serial.println(vel);
  // Serial.println((abs(Tp)/stall_Torque)*255);
        // Serial.println(force);
  // float changeVh = vh-lastLastVh;
  // Serial.println(vh);
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
