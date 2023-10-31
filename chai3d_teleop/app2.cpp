//==============================================================================
/*
    \author    Group 6 - Michael Kasman, Tristan Alkis, Carl Stott
*/
//==============================================================================

//------------------------------------------------------------------------------
#include "chai3d.h"
#include "string.h"
#include "Eigen/Dense"
#include "math.h"
#include "time.h"
#include <chrono>
#include <thread>


//------------------------------------------------------------------------------
using namespace chai3d;
using namespace std;
using namespace std::this_thread;     // sleep_for, sleep_until
using namespace std::chrono_literals; // ns, us, ms, s, h, etc.
using std::chrono::system_clock;
//------------------------------------------------------------------------------
#ifndef MACOSX
#include "GL/glut.h"
#else
#include "GLUT/glut.h"
#endif
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
// GENERAL SETTINGS
//------------------------------------------------------------------------------

// stereo Mode
/*
    C_STEREO_DISABLED:            Stereo is disabled
    C_STEREO_ACTIVE:              Active stereo for OpenGL NVDIA QUADRO cards
    C_STEREO_PASSIVE_LEFT_RIGHT:  Passive stereo where L/R images are rendered next to each other
    C_STEREO_PASSIVE_TOP_BOTTOM:  Passive stereo where L/R images are rendered above each other
*/
cStereoMode stereoMode = C_STEREO_DISABLED;

// Fullscreen display check
bool fullscreen = false;

// Mirrored display check
bool mirroredDisplay = false;


//------------------------------------------------------------------------------
// DECLARED VARIABLES
//------------------------------------------------------------------------------

// The world entity that encompasses all objects within the virtual environment
cWorld* world;

// The camera used to visualize and render the virtual world within the window display
cCamera* camera;

// A light source responsible for illuminating the objects within the virtual world
cDirectionalLight* light;

// The handler responsible for managing haptic devices in the virtual environment
cHapticDeviceHandler* handler;

// A pointer referencing the currently active haptic device in the virtual environment
cGenericHapticDevicePtr hapticDevice;

// A pointer referencing the haptic device designated as the follower in the virtual environment
cGenericHapticDevicePtr hapticDeviceFollower;

// A label used to display the simulation's running frequency in Hertz (Hz)
cLabel* labelHapticRate;

// A small spherical object (cursor) representing the haptic device within the virtual environment
cShapeSphere* cursor;

// A small spherical object (cursor) representing the follower haptic device within the virtual environment
cShapeSphere* cursorFollower;

// Declaration of a pointer to a mesh object used within the virtual environment
cMesh* mesh;

// A flag indicating whether the haptic simulation is currently running or not
bool simulationRunning = false;

// A flag indicating whether the haptic simulation has terminated or concluded its execution
bool simulationFinished = false;

// A frequency counter used to measure the haptic rate of the simulation, indicating how often haptic updates are processed
cFrequencyCounter frequencyCounter;

// Pointer to the haptic thread responsible for running the main haptics simulation loop
cThread* hapticsThread;

// Variables for the dimensions and properties of the computer screen and the GLUT display window within the virtual environment
int screenW;
int screenH;
int windowW;
int windowH;
int windowPosX;
int windowPosY;
cVector3d prevVelo(0.0, 0.0, 0.0);
// A string variable representing the current operating mode of the virtual environment
string mode;

/*
- Geomagic properties specified in centimeters
- DOF represents the degrees of freedom, and L1, L2, and L3 are specific lengths in the Geomagic device
- jointngles array stores the current joint angles of the Geomagic device
*/
const int DOF = 3;
const double L1 = 13.21;
const double L2 = 13.21;
const double L3 = 13.21;
double jointAngles[DOF] = {};

//------------------------------------------------------------------------------
// DECLARED FUNCTIONS
//------------------------------------------------------------------------------

// Callback function triggered when the window display is resized
void resizeWindow(int w, int h);

// Callback function triggered when a key is pressed
void keySelect(unsigned char key, int x, int y);

// Callback function to render the graphic scene
void updateGraphics(void);

// Callback function of GLUT timer
void graphicsTimer(int data);

// Function to close the application
void close(void);

// Main haptics simulation loop function
void updateHaptics(void);

//==============================================================================
/*
    TEMPLATE:    application.cpp

    Description of your application.
*/
//==============================================================================

int main(int argc, char* argv[])
{
    //--------------------------------------------------------------------------
    // INITIALIZATION
    //--------------------------------------------------------------------------

    cout << endl;
    cout << "-----------------------------------" << endl;
    cout << "CHAI3D" << endl;
    cout << "-----------------------------------\n\n" << endl;
    cout << "Keyboard Options:\n" << endl;
    cout << "[f] Toggle full screen mode" << endl;
    cout << "[m] Toggle vertical mirroring" << endl;
    cout << "[k] Enable Spring Force Feedback" << endl;
    cout << "[d] Enable Viscous Damping Field" << endl;
    cout << "[w] Enable Virtual Wall" << endl;
    cout << "[s] Enable Decaying Sinusoid" << endl;
    cout << "[t] Teleoperation: Unilateral" << endl;
    cout << "[b] Teleoperation: Bilateral 1:1" << endl;
    cout << "[l] Teleoperation: Bilateral 1:1" << endl;
    cout << "[p] Teleoperation: Bilateral Position Scaling" << endl;
    cout << "[x] Teleoperation: Bilateral Force Scaling" << endl;
    cout << "[n] Stop feedback" << endl;
    cout << "[q] Exit application" << endl;

    cout << endl << endl;


    //--------------------------------------------------------------------------
    // OPENGL - WINDOW DISPLAY
    //--------------------------------------------------------------------------

    // Initialize GLUT
    glutInit(&argc, argv);

   // Function to retrieve the resolution of the computer display and position the window accordingly
    screenW = glutGet(GLUT_SCREEN_WIDTH);
    screenH = glutGet(GLUT_SCREEN_HEIGHT);
    windowW = (int)(0.8 * screenH);
    windowH = (int)(0.5 * screenH);
    windowPosY = (screenH - windowH) / 2;
    windowPosX = windowPosY;

    // Function to initialize the OpenGL GLUT window
    glutInitWindowPosition(windowPosX, windowPosY);
    glutInitWindowSize(windowW, windowH);

    if (stereoMode == C_STEREO_ACTIVE)
        glutInitDisplayMode(GLUT_RGB | GLUT_DEPTH | GLUT_DOUBLE | GLUT_STEREO);
    else
        glutInitDisplayMode(GLUT_RGB | GLUT_DEPTH | GLUT_DOUBLE);

    // Function to create display context and initialize GLEW library
    glutCreateWindow(argv[0]);

#ifdef GLEW_VERSION
    // Initialize GLEW
    glewInit();
#endif

    // Functions to setup GLUT options
    glutDisplayFunc(updateGraphics);
    glutKeyboardFunc(keySelect);
    glutReshapeFunc(resizeWindow);
    glutSetWindowTitle("CHAI3D");

    // Set fullscreen mode
    if (fullscreen)
    {
        glutFullScreen();
    }

    //--------------------------------------------------------------------------
    // WORLD - CAMERA - LIGHTING
    //--------------------------------------------------------------------------

    // Create a new world object
    world = new cWorld();

    // Set the background color of the environment to black
    world->m_backgroundColor.setBlack();

    // Create a camera and insert it into the virtual world
    camera = new cCamera(world);
    world->addChild(camera);

    /* Position and Orient the Camera
    - Set the camera position (eye) in the virtual world
    - Set the position where the camera is looking at (target)
    - Set the direction of the up vector for the camera
    */
    camera->set(cVector3d(0.5, 0.0, 0.0),    // Camera position (eye)
                cVector3d(0.0, 0.0, 0.0),    // Look at position (target)
                cVector3d(0.0, 0.0, 1.0));   // Direction of the (up) vector

    // Set the near and far clipping planes of the camera
    camera->setClippingPlanes(0.01, 10.0);

    // Set stereo mode
    camera->setStereoMode(stereoMode);

    // Configure stereo eye separation and focal length (applies only if stereo is enabled)
    camera->setStereoEyeSeparation(0.01);
    camera->setStereoFocalLength(0.5);

    // Set vertical mirrored display mode
    camera->setMirrorVertical(mirroredDisplay);

    // Create a directional light source and attach it to the virtual world
    light = new cDirectionalLight(world);

    // Add the light source to the virtual world
    world->addChild(light);

    // Enable light source
    light->setEnabled(true);

    // Set the direction of light beam
    light->setDir(-1.0, 0.0, 0.0);

    // Create a sphere (cursor) to represent the haptic device
    cursor = new cShapeSphere(0.01);

    // Insert cursor inside world
    world->addChild(cursor);

    mesh = new cMesh();


    //--------------------------------------------------------------------------
    // HAPTIC DEVICE
    //--------------------------------------------------------------------------

    // Create a handler for haptic devices
    handler = new cHapticDeviceHandler();

    // Get the first available haptic device
    handler->getDevice(hapticDevice, 0);

    // Open a connection to haptic device
    hapticDevice->open();

    // Calibrate the device
    hapticDevice->calibrate();

    // Retrieve information about the current haptic device
    cHapticDeviceInfo info = hapticDevice->getSpecifications();

    // Show the reference frame if the haptic device supports orientation sensing
    if (info.m_sensedRotation == true)
    {
        // Enable the display of the reference frame
        cursor->setShowFrame(true);

        // Set the size of the displayed reference frame
        cursor->setFrameSize(0.05);
    }

    // Enable the gripper if the haptic device has a gripper, allowing it to simulate a user switch
    hapticDevice->setEnableGripperUserSwitch(true);

    //--------------------------------------------------------------------------
    // WIDGETS
    //--------------------------------------------------------------------------

    // Create a font object for rendering text in the virtual environment
    cFontPtr font = NEW_CFONTCALIBRI20();

    // Create a label to display the haptic rate (in Hz) of the simulation
    labelHapticRate = new cLabel(font);
    labelHapticRate->m_fontColor.setWhite();
    camera->m_frontLayer->addChild(labelHapticRate);


    //--------------------------------------------------------------------------
    // START SIMULATION
    //--------------------------------------------------------------------------

    // Create a thread responsible for running the main haptics rendering loop
    hapticsThread = new cThread();
    hapticsThread->start(updateHaptics, CTHREAD_PRIORITY_HAPTICS);

    // Set up a callback to close the application when it exits.
    atexit(close);

    // Set up a timer function for the main graphics rendering loop and enter the GLUT main loop
    glutTimerFunc(50, graphicsTimer, 0);
    glutMainLoop();

    // Exit
    return (0);
}

//------------------------------------------------------------------------------

void resizeWindow(int w, int h)
{
    windowW = w;
    windowH = h;
}

//------------------------------------------------------------------------------

void keySelect(unsigned char key, int x, int y)
{
    // Handle different keyboard options
    switch (key) 
    {
        case 27: // ESC key
        case 'q':
            close();
            exit(0);
            break;
        case 'f':
            // Toggle fullscreen mode
            if (fullscreen) 
            {
                windowPosX = glutGet(GLUT_INIT_WINDOW_X);
                windowPosY = glutGet(GLUT_INIT_WINDOW_Y);
                windowW = glutGet(GLUT_INIT_WINDOW_WIDTH);
                windowH = glutGet(GLUT_INIT_WINDOW_HEIGHT);
                glutPositionWindow(windowPosX, windowPosY);
                glutReshapeWindow(windowW, windowH);
                fullscreen = false;
            } 
            else 
            {
                glutFullScreen();
                fullscreen = true;
            }
            break;
        case 'm':
            // Toggle vertical mirroring
            mirroredDisplay = !mirroredDisplay;
            camera->setMirrorVertical(mirroredDisplay);
            break;
        case 'k':
            // Enable Spring Force Feedback mode
            mode = "Spring";
            cout << "Enabled Spring Force" << endl;
            break;
        case 'd':
            // Enable Viscous Damping Field mode
            mode = "Damping";
            cout << "Enabled Viscous Damping Field" << endl;
            break;
        case 'w':
            // Enable Virtual Wall mode
            mode = "Wall";
            cout << "Enabled Virtual Wall" << endl;
            break;
        case 's':
            // Enable Decaying Sinusoid Feedback mode
            mode = "Sinusoid";
            cout << "Enabled Decaying Sinusoid" << endl;
            break;
        case 't':
            // Enable Unilateral Teleoperation mode
            mode = "Teleop_Uni";
            cout << "Enabled Unilateral Teleop" << endl;
            break;
        case 'b':
            // Enable Bilateral Teleoperation mode (1:1)
            mode = "Teleop_Bi_1:1";
            cout << "Enabled Bilateral Teleop 1:1" << endl;
            break;
        case 'l':
            // Enable Bilateral Teleoperation mode (1:1)
            mode = "Teleop_Bi_1:1FF";
            cout << "Enabled Bilateral Teleop 1:1FF" << endl;
            break;
        case 'p':
            // Enable Bilateral Position Scaled Teleoperation mode
            mode = "Teleop_Bi_Pos";
            cout << "Enabled Bilateral Position Scaled Teleop" << endl;
            break;
        case 'x':
            // Enable Bilateral Force Scaled Teleoperation mode
            mode = "Teleop_Bi_Force";
            cout << "Enabled Bilateral Force Scaled Teleop" << endl;
            break;
        case 'n':
            // Stop all feedback
            mode = "None";
            cout << "Stopped Feedback" << endl;
            break;
    }
}

//------------------------------------------------------------------------------

void close(void)
{
    // Stop the simulation
    simulationRunning = false;

    // Wait for graphics and haptics loops to terminate
    while (!simulationFinished) { cSleepMs(100); }

    // Close the haptic device
    hapticDevice->close();

    // Delete resources
    delete hapticsThread;
    delete world;
    delete handler;
}

//------------------------------------------------------------------------------

void graphicsTimer(int data)
{
    if (simulationRunning)
    {
        glutPostRedisplay();
    }

    glutTimerFunc(50, graphicsTimer, 0);
}

//------------------------------------------------------------------------------

void updateGraphics(void)
{
    /////////////////////////////////////////////////////////////////////
    // UPDATE WIDGETS
    /////////////////////////////////////////////////////////////////////

    // Update the label to display haptic rate data
    labelHapticRate->setText(cStr(frequencyCounter.getFrequency(), 0) + " Hz");

    // Update position of label
    labelHapticRate->setLocalPos((int)(0.5 * (windowW - labelHapticRate->getWidth())), 15);

    /////////////////////////////////////////////////////////////////////
    // RENDER SCENE
    /////////////////////////////////////////////////////////////////////

    // Update shadow maps (if any)
    world->updateShadowMaps(false, mirroredDisplay);

    // Render the world using the camera view and window dimensions
    camera->renderView(windowW, windowH);

    // Swap the front and back buffers to display the rendered image
    glutSwapBuffers();

    // Wait until all OpenGL commands are completed
    glFinish();

    // Check for any OpenGL errors
    GLenum err;
    err = glGetError();
    if (err != GL_NO_ERROR) cout << "Error:  %s\n" << gluErrorString(err);
}

//------------------------------------------------------------------------------

// Spring Force Feedback Global Variables
bool isSpringInitialized = true; // Flag indicating whether the spring system is initialized
cVector3d initialSpringPosition; // Initial position used for spring feedback calculation

// Virtual Wall Global Variables 
const double lengthX = 10;
const double lengthY = 10;
const cVector3d pos(1, 1, 0);
const cMatrix3d orient(1, 0, 0,
    0, 1, 0,
    0, 0, 1);
const double kWall = 70;
bool firstWall = true;

// Decaying Sinusoid Global Variables
time_t timer;
bool isFirstImpact = true;
cVector3d impactVelocity;
chrono::high_resolution_clock::time_point impactTime; // Timestamp indicating the time of impact (microseconds)
double dt;

// Teleoperation Global Variables
const double kpf = 25;
const double kdf = 3;
const double kpm = 20;
const double kdm = 1;
const double kif = 30;
double newXCoordinate;
double newYCoordinate;
double newZCoordinate;
bool isFirstTimeSetup = true;
bool isFirstForceReading = true;
bool hasFollowerChild = false;
bool isFirstError = true;
cVector3d feedbackForce(0, 0, 0);
double alpha; // Alpha value used for smoothing force values
cVector3d initialError(0, 0, 0);
chrono::high_resolution_clock::time_point currentTime;
chrono::high_resolution_clock::time_point previousTime;

void updateHaptics(void)
{
    // Initialize frequency counter
    frequencyCounter.reset();

    // Set simulation flags
    simulationRunning = true;
    simulationFinished = false;

    // Main haptic simulation loop
    while (simulationRunning)
    {
        /////////////////////////////////////////////////////////////////////
        // READ HAPTIC DEVICE
        /////////////////////////////////////////////////////////////////////

        // Read position from haptic device
        cVector3d position;
        hapticDevice->getPosition(position);

        // Read orientation from haptic device
        cMatrix3d rotation;
        hapticDevice->getRotation(rotation);

        // Read user-switch status from button 0 of the haptic device
        bool button = false;
        hapticDevice->getUserSwitch(0, button);

        // Read linear velocity from the haptic device
        cVector3d linearVelocity;
        hapticDevice->getLinearVelocity(linearVelocity);

        /////////////////////////////////////////////////////////////////////
        // UPDATE 3D CURSOR MODEL
        /////////////////////////////////////////////////////////////////////

        // Update position and orientation of the cursor based on haptic device data
        cursor->setLocalPos(position);
        cursor->setLocalRot(rotation);

        /////////////////////////////////////////////////////////////////////
        // COMPUTE FORCES
        /////////////////////////////////////////////////////////////////////
        
        // CHAI 3D Rendering
        cVector3d force(0, 0, 0);
        cVector3d torque(0, 0, 0);
        double gripperForce = 0.0;

        // Spring Force Feedback
        if (mode.compare("Spring") == 0) 
        {
            // Spring Constant (N/m)
            double Ks = 20;

            // Initialize initial spring position defined by location of stylus at start
            if (isSpringInitialized) 
            {
                initialSpringPosition = position;
                isSpringInitialized = false;
            }

            force = Ks * (initialSpringPosition - position);

        }

        // Viscous Damping Field
        if (mode.compare("Damping") == 0) 
        {
            cHapticDeviceInfo info = hapticDevice->getSpecifications();

            // Damping Constant
            double B = 1.0 * info.m_maxLinearDamping;

            // Linear Damping
            force = -B * linearVelocity;
        }

        // Virtual Wall 
        if (mode.compare("Wall") == 0) 
        {
            // Wall Visualization
            if (firstWall) 
            {
                cMesh* mesh = new cMesh();
                cColorf* color = new cColorf();
                color->setYellowGold();
                mesh->m_material->setYellowGold();
                world->addChild(mesh);
                cCreatePlane(mesh, lengthX, lengthY, pos, orient, *color);
                firstWall = false;
            }

            // Compute half edges
            double halfLengthX = 0.5 * lengthX;
            double halfLengthY = 0.5 * lengthY;

            // Compute positions of the 3 vertices for wall visualization
            cVector3d v0 = cAdd(pos, cMul(orient, cVector3d(-halfLengthX, -halfLengthY, 0.0)));
            cVector3d v1 = cAdd(pos, cMul(orient, cVector3d(halfLengthX, -halfLengthY, 0.0)));
            cVector3d v2 = cAdd(pos, cMul(orient, cVector3d(halfLengthX, halfLengthY, 0.0)));

            // Compute the unit normal vector to the surface of the plane
            cVector3d planeNormal = cComputeSurfaceNormal(v0, v1, v2);

            // Compute the vector from the haptic device point to a point on the plane
            cVector3d deviceToWallVector = v0 - position;

            // Calculate the dot product between plane normal and vector from haptic device to the plane (rounded to hundredths place)
            double normalDeviceDot = cDot(planeNormal, deviceToWallVector);
            float roundNormalDeviceDot = ceil(normalDeviceDot * 100.0) / 100.0;

            // Calculate the magnitude of force applied by the wall based on the distance between the haptic device and the wall surface
            double d = cAbs(cDot(deviceToWallVector, planeNormal));

            // If the dot product is less than or equal to zero, the haptic device is on or behind the wall.
            // Compute the force applied by the wall using Hooke's Law (F = -k * d) and the plane's normal vector.
            if (roundNormalDeviceDot <= 0.0) 
            {
                force = -kWall * d * planeNormal;
                // Visualize scaled force applied by the wall
                // cDrawArrow(position, force, 1);
            }

        }

        // Decaying Sinusoid
        if (mode.compare("Sinusoid") == 0) 
        {
            // Sinusoid variables
            double A = 7.0; // Amplitude of the sinusoidal force
            double lambda = 1.0; // Decay constant for the sinusoidal force
            double frequency = 25; // Frequency of the sinusoidal force in Hz
            double omega = 2 * M_PI * frequency; // Angular frequency of the sinusoidal force

            // Wall Visualization
            if (firstWall) 
            {
                cMesh* mesh = new cMesh();
                cColorf* color = new cColorf();
                color->setYellowGold();
                mesh->m_material->setYellowGold();
                world->addChild(mesh);
                cCreatePlane(mesh, lengthX, lengthY, pos, orient, *color);
                firstWall = false;
            }
            

            // Compute half edges
            double halfLengthX = 0.5 * lengthX;
            double halfLengthY = 0.5 * lengthY;

            // Compute positions of the 3 vertices for wall visualization
            cVector3d v0 = cAdd(pos, cMul(orient, cVector3d(-halfLengthX, -halfLengthY, 0.0)));
            cVector3d v1 = cAdd(pos, cMul(orient, cVector3d(halfLengthX, -halfLengthY, 0.0)));
            cVector3d v2 = cAdd(pos, cMul(orient, cVector3d(halfLengthX, halfLengthY, 0.0)));

            // Compute the unit normal vector to the surface of the plane
            cVector3d planeNormal = cComputeSurfaceNormal(v0, v1, v2);

            // Compute the vector from the haptic device point to a point on the plane
            cVector3d deviceToWallVector = v0 - position;

            // Calculate the dot product between plane normal and vector from haptic device to the plane (rounded to hundredths place)
            double normalDeviceDot = cDot(planeNormal, deviceToWallVector);
            float roundNormalDeviceDot = ceil(normalDeviceDot * 100.0) / 100.0;

            // Calculate the magnitude of force applied by the wall based on the distance between the haptic device and the wall surface
            double d = cAbs(cDot(deviceToWallVector, planeNormal));

            // If the dot product is zero or negative, the point is on or below the plane
            if (roundNormalDeviceDot <= 0.0) 
            {

                // If this is the first impact, store time and impact velocity
                if (isFirstImpact) 
                {
                    impactTime = chrono::high_resolution_clock::now();
                    hapticDevice->getLinearVelocity(impactVelocity);
                    isFirstImpact = false;
                }
                // Calculate time difference since impact
                dt = (chrono::high_resolution_clock::now() - impactTime).count();
                // Convert from ns to s
                dt = dt / 1000000000;
                // Compute sinusoidal force with decay and wall force
                force = -A * impactVelocity * exp(-lambda * dt) * cCosDeg(omega * dt) + (-kWall * d * planeNormal);

                // Visualize the scaled force applied by wall
                cDrawArrow(position, force, 1);
            }
            else 
            {
                isFirstImpact = true;
            }
        }

        cVector3d force_test(0, 0, 0);

        // Teleoperation Implementation
        if (mode.compare("Teleop_Uni") == 0 || mode.compare("Teleop_Bi_Pos") == 0 || mode.compare("Teleop_Bi_Force") == 0 || mode.compare("Teleop_Bi_1:1") == 0 || mode.compare("Teleop_Bi_1:1FF") == 0) {
            if (isFirstTimeSetup) 
            {

                // Get a handle to the follower haptic device
                handler->getDevice(hapticDeviceFollower, 1);

                // Open a connection to the follower haptic device
                hapticDeviceFollower->open();

                // Calibrate the follower haptic device (if necessary)
                hapticDeviceFollower->calibrate();

                // Retrieve information about the follower haptic device
                cHapticDeviceInfo infoFollower = hapticDeviceFollower->getSpecifications();

                // Create a sphere (cursor) to represent the follower haptic device
                cursorFollower = new cShapeSphere(0.01);

                // Insert the follower cursor into the world
                world->addChild(cursorFollower);

                // Display a reference frame if the follower haptic device supports orientations
                if (infoFollower.m_sensedRotation == true)
                {
                    // Display reference frame
                    cursorFollower->setShowFrame(true);

                    // Set the size of the reference frame
                    cursorFollower->setFrameSize(0.05);
                }

                // If the device has a gripper, enable the gripper to simulate a user switch
                hapticDeviceFollower->setEnableGripperUserSwitch(true);

                // Setup completed, no longer the first time
                isFirstTimeSetup = false;

                // Follower haptic device is set up
                hasFollowerChild = true;
            }

            // Read position and orientation of the follower haptic device
            cVector3d positionFollower;
            cMatrix3d rotationFollower;
            hapticDeviceFollower->getPosition(positionFollower);
            hapticDeviceFollower->getRotation(rotationFollower);

            // Read linear velocity of the follower haptic device and the master haptic device
            cVector3d linearVelocityFollower;
            cVector3d linearVelocityMaster;
            hapticDeviceFollower->getLinearVelocity(linearVelocityFollower);
            hapticDevice->getLinearVelocity(linearVelocityMaster);

            // Update position and orientation of the follower cursor
            cursorFollower->setLocalPos(positionFollower);
            cursorFollower->setLocalRot(rotationFollower);

            // Calculate force applied by the follower haptic device
            cVector3d forceFollower = kpf * (position - positionFollower) + kdf * (linearVelocityMaster - linearVelocityFollower);

            if (mode.compare("Teleop_Bi_1:1") == 0) 
            {
                // feedforward term calculation
                
                double Kp = 100;
                double Kd = 1;
                // Calculate force applied by master haptic device in 1:1 mode with follower
                force = Kp * (positionFollower - position) + Kd * (linearVelocityFollower - linearVelocityMaster);
                //sleep_for(10ns);
                //sleep_until(system_clock::now() + .01s);

            }

            if (mode.compare("Teleop_Bi_1:1FF") == 0)
            {
                // feedforward term calculation
                
                double Jm = .0054;
                double Bm = 1;
                cVector3d veloChange = linearVelocityFollower - prevVelo;
                prevVelo = linearVelocityFollower;
                cVector3d FF = veloChange * Jm;
                cVector3d ffTerm(FF(0)* positionFollower(0), FF(1)* positionFollower(1), FF(2)* positionFollower(2));
                double FFgain = .001;
                double Kp = 100;
                double Kd = 1;
                cout << FF(0) << endl;
                cout << positionFollower(0) << endl;
                cout << "multiplication of terms"<<ffTerm << endl;
                
                /*cVector3d FFterm = (FF(0) * positionFollower(0),
                    FF(1) * positionFollower(1),
                    FF(2) * positionFollower(2));*/
                // Calculate force applied by master haptic device in 1:1 mode with follower
                force = Kp * (positionFollower - position) + Kd * (linearVelocityFollower - linearVelocityMaster) + FFgain*ffTerm;
                //sleep_for(10ns);
                //sleep_until(system_clock::now() + .01s);

            }

            if (mode.compare("Teleop_Bi_Pos") == 0) 
            {

                double k = .1; // Scaling factor

                // Calculate force applied by master haptic device, scaled down position of follower
                force = kpm * (positionFollower - position * k) + kdm * (linearVelocityMaster - linearVelocityFollower);
                // determining force of follwer, scaled up pos of master
                forceFollower = kpf * (position * k - positionFollower ) +
                    kdf * (linearVelocityMaster - linearVelocityFollower);
            }

            if (mode.compare("Teleop_Bi_Force") == 0) 
            {
                // Initialization: Store initial force, timestamp, and reset flag
                if (isFirstForceReading) 
                {
                    feedbackForce = force;
                    previousTime = chrono::high_resolution_clock::now();
                    isFirstForceReading = false; 
                }

                // Read current force from the follower device
                hapticDeviceFollower->getForce(force);

                // Apply low-pass filter to smooth the force readings
                currentTime = chrono::high_resolution_clock::now();
                dt = (currentTime - previousTime).count();
                dt = dt / 1000000000; // converting from ns to s
                alpha = dt / (1.0 / 1500.0);
                feedbackForce += alpha * (force - feedbackForce);
                double k = 1.5;
                // Scale the master force and update the force applied by the master device
                force = -feedbackForce * k; // Scaling factor applied to the feedback force

                // Update the timestamp for the next iteration
                previousTime = currentTime;
            }
 
            // Send computed force, torque, and gripper force to the follower haptic device
            hapticDeviceFollower->setForceAndTorqueAndGripperForce(forceFollower, torque, gripperForce);
        }

        // Remove force feedback effects if mode is set to "None"
        if (mode.compare("None") == 0) 
        {
            world->removeChild(mesh);
            // Reset teleoperation haptic device follower initialization flag
            isFirstTimeSetup = true;
        }

        /////////////////////////////////////////////////////////////////////
        // APPLY FORCES
        /////////////////////////////////////////////////////////////////////

        // Send computed force, torque, and gripper force to the main haptic device
        hapticDevice->setForceAndTorqueAndGripperForce(force, torque, gripperForce);

        // Update frequency counter to track simulation haptic rate
        frequencyCounter.signal(1);
    }

    // Signal that haptics simulation has terminated
    simulationFinished = true;
}
