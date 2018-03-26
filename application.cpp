//==============================================================================
/*
    \author    Your Name
*/
//==============================================================================

//------------------------------------------------------------------------------
#include "chai3d.h"
//#include "MyCToolCursor.h"
//#include "MyCGenericHapticDevice.h"
//#include "MyCGenericTool.h"
//#include "MyCGenericHapticDevice.h"
//------------------------------------------------------------------------------
#include <GLFW/glfw3.h>
//------------------------------------------------------------------------------
using namespace chai3d;
using namespace std;
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

// fullscreen mode
bool fullscreen = false;

// mirrored display
bool mirroredDisplay = false;


//------------------------------------------------------------------------------
// DECLARED VARIABLES
//------------------------------------------------------------------------------

// a world that contains all objects of the virtual environment
cWorld* world;

// a camera to render the world in the window display
cCamera* camera;
double movementRate = 0.0005;
cVector3d camPos = cVector3d(0.20, 0.0, 0.05);
cVector3d camLookAt = cVector3d(0.0, 0.0, 0.0);
bool movingCamera = false;
double rotSpeed = 0.5 * M_PI / 180.0;		// 0.5 degrees per haptic frame

// a light source to illuminate the objects in the world
cDirectionalLight *light;

// an object loaded from a 3d mesh file
cMultiMesh* course[1] = { NULL };

// a haptic device handler
cHapticDeviceHandler* handler;

// a pointer to the current haptic device
cGenericHapticDevicePtr hapticDevice;

// a fake haptic device handler
cHapticDeviceHandler* left_handler;

// a pointer to the fake haptic device
cGenericHapticDevicePtr left_hapticDevice;

// a label to display the rates [Hz] at which the simulation is running
cLabel* labelRates;

// a small sphere (cursor) representing the haptic device 
//cShapeSphere* cursor;

cToolCursor* tool;
cVector3d lastUserPos = cVector3d(0.0, 0.0, 0.0);
cToolCursor* ball;
cVector3d ballAvatar(-0.02, 0.0, 0.0);
cVector3d ballVel = cVector3d(0.0, 0.0, 0.0);
cVector3d lastBallImagePos = cVector3d(0.0, 0.0, 0.0);
const double contactK_BtoT = 2000.0;	// N/m contact force
const double contactB_BtoT = 1.0;   // Nm/s	dampening 
const double ballR = 0.005;	// radius of the ball and tool
const double ballM = 0.60; //kg
const double k_ballSpring = 4000.0; // N/m spring constant
const double b_ballSpring = 10.0;  // Ns/m spring dampening 
double deltaT = 0.001;
boolean clubActive = false;
const int buttonTimeout = 500;  // cycles of the haptic loop

// flag to indicate if the haptic simulation currently running
bool simulationRunning = false;

// flag to indicate if the haptic simulation has terminated
bool simulationFinished = false;

// a frequency counter to measure the simulation graphic rate
cFrequencyCounter freqCounterGraphics;

// a frequency counter to measure the simulation haptic rate
cFrequencyCounter freqCounterHaptics;

// haptic thread
cThread* hapticsThread;

// a handle to window display context
GLFWwindow* window = NULL;

// current width of window
int width  = 0;

// current height of window
int height = 0;

// swap interval for the display context (vertical synchronization)
int swapInterval = 1;


//------------------------------------------------------------------------------
// DECLARED FUNCTIONS
//------------------------------------------------------------------------------

// callback when the window display is resized
void windowSizeCallback(GLFWwindow* a_window, int a_width, int a_height);

// callback when an error GLFW occurs
void errorCallback(int error, const char* a_description);

// callback when a key is pressed
void keyCallback(GLFWwindow* a_window, int a_key, int a_scancode, int a_action, int a_mods);

// this function renders the scene
void updateGraphics(void);

// this function contains the main haptics simulation loop
void updateHaptics(void);

// this function closes the application
void close(void);


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
    cout << "-----------------------------------" << endl << endl << endl;
    cout << "Keyboard Options:" << endl << endl;
    cout << "[f] - Enable/Disable full screen mode" << endl;
    cout << "[m] - Enable/Disable vertical mirroring" << endl;
    cout << "[q] - Exit application" << endl;
    cout << endl << endl;


    //--------------------------------------------------------------------------
    // OPENGL - WINDOW DISPLAY
    //--------------------------------------------------------------------------

    // initialize GLFW library
    if (!glfwInit())
    {
        cout << "failed initialization" << endl;
        cSleepMs(1000);
        return 1;
    }

    // set error callback
    glfwSetErrorCallback(errorCallback);

    // compute desired size of window
    const GLFWvidmode* mode = glfwGetVideoMode(glfwGetPrimaryMonitor());
    int w = 0.8 * mode->height;
    int h = 0.5 * mode->height;
    int x = 0.5 * (mode->width - w);
    int y = 0.5 * (mode->height - h);

    // set OpenGL version
    glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 2);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 1);

    // set active stereo mode
    if (stereoMode == C_STEREO_ACTIVE)
    {
        glfwWindowHint(GLFW_STEREO, GL_TRUE);
    }
    else
    {
        glfwWindowHint(GLFW_STEREO, GL_FALSE);
    }

    // create display context
    window = glfwCreateWindow(w, h, "CHAI3D", NULL, NULL);
    if (!window)
    {
        cout << "failed to create window" << endl;
        cSleepMs(1000);
        glfwTerminate();
        return 1;
    }

    // get width and height of window
    glfwGetWindowSize(window, &width, &height);

    // set position of window
    glfwSetWindowPos(window, x, y);

    // set key callback
    glfwSetKeyCallback(window, keyCallback);

    // set resize callback
    glfwSetWindowSizeCallback(window, windowSizeCallback);

    // set current display context
    glfwMakeContextCurrent(window);

    // sets the swap interval for the current display context
    glfwSwapInterval(swapInterval);

#ifdef GLEW_VERSION
    // initialize GLEW library
    if (glewInit() != GLEW_OK)
    {
        cout << "failed to initialize GLEW library" << endl;
        glfwTerminate();
        return 1;
    }
#endif


    //--------------------------------------------------------------------------
    // WORLD - CAMERA - LIGHTING
    //--------------------------------------------------------------------------

    // create a new world.
    world = new cWorld();

    // set the background color of the environment
    world->m_backgroundColor.setBlack();

    // create a camera and insert it into the virtual world
    camera = new cCamera(world);
    world->addChild(camera);

    // position and orient the camera
    camera->set( camPos,    // camera position (eye)
                 camLookAt,    // look at position (target)
                 cVector3d (0.0, 0.0, 1.0));   // direction of the (up) vector

    // set the near and far clipping planes of the camera
    camera->setClippingPlanes(0.01, 10.0);

    // set stereo mode
    camera->setStereoMode(stereoMode);

    // set stereo eye separation and focal length (applies only if stereo is enabled)
    camera->setStereoEyeSeparation(0.01);
    camera->setStereoFocalLength(0.5);

    // set vertical mirrored display mode
    camera->setMirrorVertical(mirroredDisplay);

    // create a directional light source
    light = new cDirectionalLight(world);

    // insert light source inside world
    world->addChild(light);

    // enable light source
    light->setEnabled(true);

    // define direction of light beam
    light->setDir(-1.0, 0.0, 0.0); 

    
	const std::string courseNames[1] = { "courseBv5.3ds" };

    // create the courses
	for (int i = 0; i < 1; i++) {
		course[i] = new cMultiMesh();
		course[i]->loadFromFile(courseNames[i]); //tray.obj"); //courseBv3.obj");
		course[i]->setLocalPos(course[i]->getLocalPos() + cVector3d(0.0, 0.0, -0.02));
		course[i]->scale(1.0);
		course[i]->createAABBCollisionDetector(ballR); //RADIUS OF TOOL
		course[i]->m_material->setUseHapticShading(true);
		course[i]->computeBTN();
		course[i]->setWireMode(false);
		course[i]->setStiffness(2000.0, true);
		cMesh* mesh = course[i]->getMesh(0);
		mesh->m_material->setGreenLawn();
		cTexture2dPtr albedoMap = cTexture2d::create();
		albedoMap->loadFromFile("Cork_001_colour.jpg");
		albedoMap->setWrapModeS(GL_REPEAT);
		albedoMap->setWrapModeT(GL_REPEAT);
		albedoMap->setUseMipmaps(true);
		mesh->m_texture = albedoMap;
		mesh->setUseTexture(true);
		//mesh->setHapticEnabled(true, true);
		//mesh->m_material->setTransparencyLevel(0);
		//course[i]->m_material->setStaticFriction(0.5);
		//course[i]->m_material->setDynamicFriction(0.3);
	}
	world->addChild(course[0]); 

    //--------------------------------------------------------------------------
    // HAPTIC DEVICE
    //--------------------------------------------------------------------------

    // create a haptic device handler
    handler = new cHapticDeviceHandler();

    // get a handle to the first haptic device
    handler->getDevice(hapticDevice, 0);

    // open a connection to haptic device
    hapticDevice->open();

    // calibrate device (if necessary)
    hapticDevice->calibrate();

    // retrieve information about the current haptic device
    cHapticDeviceInfo info = hapticDevice->getSpecifications();

    // if the device has a gripper, enable the gripper to simulate a user switch
    hapticDevice->setEnableGripperUserSwitch(true);

	tool = new cToolCursor(world); 
	tool->setHapticDevice(hapticDevice);
	tool->setRadius(ballR);
	tool->enableDynamicObjects(true); // Special variant of god object algorithm for moving objects
	tool->start();
	world->addChild(tool);

	// create the ball that will be knocked around as a psudo-tool
	ball = new cToolCursor(world);
	ball->setHapticDevice(hapticDevice);
	ball->setRadius(ballR);
	ball->enableDynamicObjects(true);
	ball->start();
	world->addChild(ball);


	//get the 2nd haptic device for rate control  
	// create a haptic device handler
	left_handler = new cHapticDeviceHandler();

	// get a handle to the first haptic device
	left_handler->getDevice(left_hapticDevice, 1);

	// open a connection to haptic device
	left_hapticDevice->open();

	// calibrate device (if necessary)
	left_hapticDevice->calibrate();

	// retrieve information about the current haptic device
	cHapticDeviceInfo left_info = left_hapticDevice->getSpecifications();

	// if the device has a gripper, enable the gripper to simulate a user switch
	left_hapticDevice->setEnableGripperUserSwitch(true);

	//--------------------------------------------------------------------------
    // WIDGETS
    //--------------------------------------------------------------------------

    // create a font
    cFontPtr font = NEW_CFONTCALIBRI20();
    
    // create a label to display the haptic and graphic rates of the simulation
    labelRates = new cLabel(font);
    labelRates->m_fontColor.setWhite();
    camera->m_frontLayer->addChild(labelRates);


    //--------------------------------------------------------------------------
    // START SIMULATION
    //--------------------------------------------------------------------------

    // create a thread which starts the main haptics rendering loop
    hapticsThread = new cThread();
    hapticsThread->start(updateHaptics, CTHREAD_PRIORITY_HAPTICS);

    // setup callback when application exits
    atexit(close);


    //--------------------------------------------------------------------------
    // MAIN GRAPHIC LOOP
    //--------------------------------------------------------------------------

    // call window size callback at initialization
    windowSizeCallback(window, width, height);

    // main graphic loop
    while (!glfwWindowShouldClose(window))
    {
        // get width and height of window
        glfwGetWindowSize(window, &width, &height);

        // render graphics
        updateGraphics();

        // swap buffers
        glfwSwapBuffers(window);

        // process events
        glfwPollEvents();

        // signal frequency counter
        freqCounterGraphics.signal(1);
    }

    // close window
    glfwDestroyWindow(window);

    // terminate GLFW library
    glfwTerminate();

    // exit
    return 0;
}

//------------------------------------------------------------------------------

void windowSizeCallback(GLFWwindow* a_window, int a_width, int a_height)
{
    // update window size
    width  = a_width;
    height = a_height;
}

//------------------------------------------------------------------------------

void errorCallback(int a_error, const char* a_description)
{
    cout << "Error: " << a_description << endl;
}

//------------------------------------------------------------------------------

void keyCallback(GLFWwindow* a_window, int a_key, int a_scancode, int a_action, int a_mods)
{
    // filter calls that only include a key press
    if (a_action != GLFW_PRESS)
    {
        return;
    }

    // option - exit
    else if ((a_key == GLFW_KEY_ESCAPE) || (a_key == GLFW_KEY_Q))
    {
        glfwSetWindowShouldClose(a_window, GLFW_TRUE);
    }

    // option - toggle fullscreen
    else if (a_key == GLFW_KEY_F)
    {
        // toggle state variable
        fullscreen = !fullscreen;

        // get handle to monitor
        GLFWmonitor* monitor = glfwGetPrimaryMonitor();

        // get information about monitor
        const GLFWvidmode* mode = glfwGetVideoMode(monitor);

        // set fullscreen or window mode
        if (fullscreen)
        {
            glfwSetWindowMonitor(window, monitor, 0, 0, mode->width, mode->height, mode->refreshRate);
            glfwSwapInterval(swapInterval);
        }
        else
        {
            int w = 0.8 * mode->height;
            int h = 0.5 * mode->height;
            int x = 0.5 * (mode->width - w);
            int y = 0.5 * (mode->height - h);
            glfwSetWindowMonitor(window, NULL, x, y, w, h, mode->refreshRate);
            glfwSwapInterval(swapInterval);
        }
    }

    // option - toggle vertical mirroring
    else if (a_key == GLFW_KEY_M)
    {
        mirroredDisplay = !mirroredDisplay;
        camera->setMirrorVertical(mirroredDisplay);
    }

	else if (a_key == GLFW_KEY_C) {
		if (movingCamera) {
			movingCamera = false;
		}
		else {
			movingCamera = true;
		}
	}
}

//------------------------------------------------------------------------------

void close(void)
{
    // stop the simulation
    simulationRunning = false;

    // wait for graphics and haptics loops to terminate
    while (!simulationFinished) { cSleepMs(100); }

    // close haptic device
    hapticDevice->close();
	//fake_hapticDevice->close();

    // delete resources
    delete hapticsThread;
    delete world;
    delete handler;
}

//------------------------------------------------------------------------------

void updateGraphics(void)
{
    /////////////////////////////////////////////////////////////////////
    // UPDATE WIDGETS
    /////////////////////////////////////////////////////////////////////

    // update haptic and graphic rate data
    labelRates->setText(cStr(freqCounterGraphics.getFrequency(), 0) + " Hz / " +
        cStr(freqCounterHaptics.getFrequency(), 0) + " Hz");

    // update position of label
    labelRates->setLocalPos((int)(0.5 * (width - labelRates->getWidth())), 15);


    /////////////////////////////////////////////////////////////////////
    // RENDER SCENE
    /////////////////////////////////////////////////////////////////////

    // update shadow maps (if any)
    world->updateShadowMaps(false, mirroredDisplay);

    // render world
    camera->renderView(width, height);

    // wait until all GL commands are completed
    glFinish();

    // check for any OpenGL errors
    GLenum err;
    err = glGetError();
    if (err != GL_NO_ERROR) cout << "Error:  %s\n" << gluErrorString(err);
}

//------------------------------------------------------------------------------

void updateHaptics(void)
{
    // simulation in now running
    simulationRunning  = true;
    simulationFinished = false;

	cMatrix3d rot;
	hapticDevice->getRotation(rot);
	ball->setDeviceGlobalRot(rot);
	ball->setDeviceGlobalForce(cVector3d(0.0, 0.0, 0.0));

	bool first = 1;
	
	tool->setDeviceGlobalForce(cVector3d(0.0, 0.0, 0.0));

	int counter = 0;
	int dampeningCounter = 0;
	int button0_counter = 0;
	int button1_counter = 0;
	int button2_counter = 0;
	int button3_counter = 0;

	bool button0, button1, button2, button3;

	

    // main haptic simulation loop
    while(simulationRunning)
    {

		// determine if a button has been pressed
		button0 = false;
		button1 = false;
		button2 = false;
		button3 = false;

		if (button0_counter >= buttonTimeout) {		// get button data
			hapticDevice->getUserSwitch(0, button0);
			//left_hapticDevice->getUserSwitch(0, button0);
		}
		if (button1_counter >= buttonTimeout) {
			hapticDevice->getUserSwitch(1, button1);
			//left_hapticDevice->getUserSwitch(1, button1);
		}
		if (button2_counter >= buttonTimeout) {
			hapticDevice->getUserSwitch(1, button1);
			//left_hapticDevice->getUserSwitch(1, button1);
		}
		if (button3_counter >= buttonTimeout) {
			hapticDevice->getUserSwitch(3, button3);
			//left_hapticDevice->getUserSwitch(3, button3);
		}
		
		if (button0 == true) {										// if button was pressed, reset counter and toggle associated variable 
			button0_counter = 0;
			if (clubActive) {
				clubActive = false;
				tool->m_image->m_material->setOrange();				// TODO: Coloring the tool sphere doesnt work
			}
			else {
				clubActive = true;
				tool->m_image->m_material->setGray();
			}
			printf("club has been toggled to: %d \n", clubActive);
		}
		else if (button0_counter < buttonTimeout) {
			button0_counter++;
		}
		/*
		if (button1 == true) {										// button 1 (currently) allows the tool to move the camera
			button1_counter = 0;
		}
		else if (button1_counter < buttonTimeout) {
			button1_counter++;
		}

		if (button2 == true) {
			button2_counter = 0;
		}
		else if (button2_counter < buttonTimeout) {
			button2_counter++;
		}

		if (button3 == true) {
			button3_counter = 0;
		}
		else if (button3_counter < buttonTimeout) {
			button3_counter++;
		}
		*/
		world->computeGlobalPositions();

        /////////////////////////////////////////////////////////////////////
        // READ HAPTIC DEVICE
        /////////////////////////////////////////////////////////////////////

        /////////////////////////////////////////////////////////////////////
        // UPDATE 3D CURSOR MODEL
        /////////////////////////////////////////////////////////////////////
		if (movingCamera) {
			tool->updateFromDevice();
			cVector3d offset;
			hapticDevice->getPosition(offset);
			camPos = camPos + movementRate * offset;
			camLookAt = camLookAt + movementRate * offset;
			camera->set(camPos,    // camera position (eye)
				camLookAt,    // look at position (target)
				cVector3d(0.0, 0.0, 1.0));   // direction of the (up) vector

			tool->setLocalPos(tool->getLocalPos() + movementRate * offset);

			cVector3d force(0, 0, 0);
			cVector3d torque(0, 0, 0);
			double gripperForce = 0.0;

			force = -offset * 250.0;

			hapticDevice->setForceAndTorqueAndGripperForce(force, torque, gripperForce);
			
			/*
			if (button1) {
				cMatrix3d rot;
				hapticDevice->getRotation(rot);
				rot.setAxisAngleRotationRad(cVector3d(0.0, 0.0, 1.0), rotSpeed);
				//hapticDevice->setLocalRot(rot);
			}
			if (button3) {
				cMatrix3d rot = tool->getLocalRot();
				rot.setAxisAngleRotationRad(cVector3d(0.0, 0.0, 1.0), -rotSpeed);
				tool->setLocalRot(rot);
			}*/
		}
		else {
			// update position and orienation of cursor
			tool->updateFromDevice();

			if (first) {
				ballAvatar = cVector3d(-0.03, 0.0, 0.0);
				ball->m_image->setLocalPos(ballAvatar);
				ballVel = cVector3d(0.0, 0.0, 0.0);
				first = false;
			}

			ball->updateFromUser(ballAvatar, ballVel);

			/////////////////////////////////////////////////////////////////////
			// COMPUTE FORCES
			/////////////////////////////////////////////////////////////////////
			//ball->updateToolImagePosition();
			tool->computeInteractionForces();
			ball->computeInteractionForces();

			cVector3d force(0, 0, 0);
			cVector3d torque(0, 0, 0);
			double gripperForce = 0.0;
			cVector3d forceOnBall = cVector3d(0.0, 0.0, -9.81) * ballM;		// apply gravity to ball
			forceOnBall += k_ballSpring * (ball->m_image->getGlobalPos() - ballAvatar);	// force of the spring connecting the ball and its proxy
			forceOnBall += -b_ballSpring * cProjectPointOnLine(ballVel, cVector3d(0.0, 0.0, 0.0), (ball->m_image->getGlobalPos() - ballAvatar));  // dampen the spring force

			///*
			cVector3d force_contact(0.0, 0.0, 0.0);
			cVector3d toolToBall = (tool->m_image->getGlobalPos()) - (ball->m_image->getGlobalPos());
			// calculate contact force with a sphere if club is not active, a box if it is.
			if ((((tool->m_image->getGlobalPos()) - (ball->m_image->getGlobalPos())).length() < (2 * ballR)) && (counter > 1000) && (!clubActive)) {
				double depth = (2 * ballR) - ((tool->m_image->getGlobalPos()) - (ball->m_image->getGlobalPos())).length();
				cVector3d userVel = (tool->m_image->getGlobalPos() - lastUserPos) / deltaT;
				cVector3d ballImageVel = (ball->m_image->getGlobalPos() - lastBallImagePos) / deltaT;
				double vel_contact = (userVel - ballImageVel).length();
				force_contact = ((contactK_BtoT * depth) - (contactB_BtoT * vel_contact)) * cNormalize((ball->m_image->getGlobalPos()) - (tool->m_image->getGlobalPos()));
				/*if (force_contact.length() > 5.0) {
					dampeningCounter = 0;
				}
				if (dampeningCounter > 300) {
					cVector3d userVel = (tool->m_image->getLocalPos() - lastUserPos) / deltaT;
					cVector3d ballImageVel = (ball->m_image->getLocalPos() - lastBallImagePos) / deltaT;
					double vel_contact = (userVel - ballImageVel).length();
					force_contact -= (contactB_BtoT * vel_contact) * cNormalize((ball->m_image->getLocalPos()) - (tool->m_image->getLocalPos()));
				}*/
				tool->addDeviceLocalForce(-force_contact);		// apply the contact force to the tool
				forceOnBall.add(force_contact);					// apply contact force to the ball
			}
			else if (((cProjectPointOnLine((ball->m_image->getGlobalPos()), (tool->m_image->getGlobalPos()), cVector3d(-1.0, 0.0, 0.0))).length() < (2 * ballR)) && (counter > 1000) && (clubActive)) {
				double depth = (2 * ballR) - (cProjectPointOnLine((ball->m_image->getGlobalPos()), (tool->m_image->getGlobalPos()), cVector3d(-1.0, 0.0, 0.0))).length();
				cVector3d userVel = (tool->m_image->getGlobalPos() - lastUserPos) / deltaT;
				cVector3d ballImageVel = (ball->m_image->getGlobalPos() - lastBallImagePos) / deltaT;
				double vel_contact = (userVel - ballImageVel).length();
				force_contact = ((contactK_BtoT * depth) - (contactB_BtoT * vel_contact)) * cNormalize((ball->m_image->getGlobalPos()) - (tool->m_image->getGlobalPos()));
				/*if (force_contact.length() > 5.0) {
				dampeningCounter = 0;
				}
				if (dampeningCounter > 300) {
				cVector3d userVel = (tool->m_image->getLocalPos() - lastUserPos) / deltaT;
				cVector3d ballImageVel = (ball->m_image->getLocalPos() - lastBallImagePos) / deltaT;
				double vel_contact = (userVel - ballImageVel).length();
				force_contact -= (contactB_BtoT * vel_contact) * cNormalize((ball->m_image->getLocalPos()) - (tool->m_image->getLocalPos()));
				}*/
				tool->addDeviceLocalForce(-force_contact);		// apply the contact force to the tool
				forceOnBall.add(force_contact);					// apply contact force to the ball
			}
			else if (counter <= 1000) {							// if it is before the first second of the simulation, dont apply contact force.
				counter++;
			}

			if (dampeningCounter <= 300) {					// if less than half a second has passed, increment dampening counter
				dampeningCounter++;
			}

			cVector3d accel_ball = forceOnBall / ballM;
			ballVel.add(deltaT * accel_ball);
			ballAvatar = ballAvatar + ballVel * deltaT;

			lastBallImagePos = ball->m_image->getGlobalPos();
			lastUserPos = tool->m_image->getGlobalPos();

			//*/
			/////////////////////////////////////////////////////////////////////
			// APPLY FORCES
			/////////////////////////////////////////////////////////////////////
			tool->applyToDevice();
		}

        // signal frequency counter
        freqCounterHaptics.signal(1);
    }
    
    // exit haptics thread
    simulationFinished = true;
}

//------------------------------------------------------------------------------
