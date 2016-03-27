SYSTEM_MODE(SEMI_AUTOMATIC);



// I2Cdev and MPU6050 must be installed as libraries, or else the .cpp/.h files
// for both classes must be in the include path of your project
#include "I2Cdev.h"

#include "MPU6050_6Axis_MotionApps20.h"
//#include "MPU6050.h" // not necessary if using MotionApps include file

// Arduino Wire library is required if I2Cdev I2CDEV_ARDUINO_WIRE implementation
// is used in I2Cdev.h
#if (I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE) && !defined (SPARK)
   // #include "Wire.h"
#endif

// class default I2C address is 0x68
// specific I2C addresses may be passed as a parameter here
// AD0 low = 0x68 (default for SparkFun breakout and InvenSense evaluation board)
// AD0 high = 0x69
MPU6050 mpu;
//MPU6050 mpu(0x69); // <-- use for AD0 high

// uncomment "OUTPUT_READABLE_QUATERNION" if you want to see the actual
// quaternion components in a [w, x, y, z] format (not best for parsing
// on a remote host such as Processing or something though)
//#define OUTPUT_READABLE_QUATERNION

// uncomment "OUTPUT_READABLE_EULER" if you want to see Euler angles
// (in degrees) calculated from the quaternions coming from the FIFO.
// Note that Euler angles suffer from gimbal lock (for more info, see
// http://en.wikipedia.org/wiki/Gimbal_lock)
//#define OUTPUT_READABLE_EULER

// uncomment "OUTPUT_READABLE_YAWPITCHROLL" if you want to see the yaw/
// pitch/roll angles (in degrees) calculated from the quaternions coming
// from the FIFO. Note this also requires gravity vector calculations.
// Also note that yaw/pitch/roll angles suffer from gimbal lock (for
// more info, see: http://en.wikipedia.org/wiki/Gimbal_lock)
#define OUTPUT_READABLE_YAWPITCHROLL

// uncomment "OUTPUT_READABLE_REALACCEL" if you want to see acceleration
// components with gravity removed. This acceleration reference frame is
// not compensated for orientation, so +X is always +X according to the
// sensor, just without the effects of gravity. If you want acceleration
// compensated for orientation, us OUTPUT_READABLE_WORLDACCEL instead.
//#define OUTPUT_READABLE_REALACCEL

// uncomment "OUTPUT_READABLE_WORLDACCEL" if you want to see acceleration
// components with gravity removed and adjusted for the world frame of
// reference (yaw is relative to initial orientation, since no magnetometer
// is present in this case). Could be quite handy in some cases.
//#define OUTPUT_READABLE_WORLDACCEL

// uncomment "OUTPUT_TEAPOT" if you want output that matches the
// format used for the InvenSense teapot demo
//#define OUTPUT_TEAPOT



#if defined (SPARK)
#define LED_PIN D7 // (Spark Core is D7)
#else
#define LED_PIN 13 // (Arduino is 13, Teensy is 11, Teensy++ is 6)
#endif

bool blinkState = false;

// MPU control/status vars
bool dmpReady = false;  // set true if DMP init was successful
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer

// orientation/motion vars
Quaternion q;           // [w, x, y, z]         quaternion container
VectorInt16 aa;         // [x, y, z]            accel sensor measurements
VectorInt16 aaReal;     // [x, y, z]            gravity-free accel sensor measurements
VectorInt16 aaWorld;    // [x, y, z]            world-frame accel sensor measurements
VectorFloat gravity;    // [x, y, z]            gravity vector
float euler[3];         // [psi, theta, phi]    Euler angle container
float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector

/*
// ================================================================
// 							INTERNAL TIMER						===
// ================================================================

#include "SparkIntervalTimer.h"
IntervalTimer myTimer;
volatile int timer=0;

void incrementTimer(){
	Serial.println("Testing whether volatile works:");
	Serial.print(timer); Serial.print("->");
	timer++;
	Serial.print(timer);
}
*/
// ================================================================
// ===              MPU INTERRUPT DETECTION ROUTINE             ===
// ================================================================

volatile bool mpuInterrupt = false;     // indicates whether MPU interrupt pin has gone high
void dmpDataReady() {
    mpuInterrupt = true;
}
/*
// ================================================================
//				SWORD TIP PRESSED INTERRUPT ROUTINE				===
// ================================================================
volatile bool swordtipInterrupt = false;     // indicates whether MPU interrupt pin has gone high
void tipPressed() {
    swordtipInterrupt = !swordtipInterrupt;
	Serial.println("I CHANGED HERE MADAFUKKAAAAAAAA");
}

// ==================================================================
// ===						PRESSURE SENSOR						  ===
// ==================================================================
int PS1AnalogPin = A0; // PS1 is connected to analog 0
int PS2AnalogPin = A1; //  PS2 is connected to analog 0 
int LED1pin = 4;     
int LED2pin = 5;
int PS1Reading;      // the analog reading from the FSR resistor divider
int PS2Reading;      // the analog reading from the FSR resistor

int PS1Min = 0;
int PS1Max = 1023;
int PS2Min = 0;
int PS2Max = 1023;

int readPressure(){
  PS1Reading = analogRead(PS1AnalogPin);
  PS1Reading = map(PS1Reading,PS1Min,PS1Max, 0.0,10.0);
  PS1Reading = constrain(PS1Reading, 0.0, 10.0);

  PS2Reading = analogRead(PS2AnalogPin);
  PS2Reading = map(PS2Reading,PS2Min,PS2Max, 0.0,10.0);
  PS2Reading = constrain(PS2Reading, 0.0, 10.0);
  
  if(PS1Reading!=0 ||PS2Reading!=0 )
	  Serial.println("we should be interrupting timer here");
  
  Serial.print("Pressure1 = ");
  Serial.println(PS1Reading);
  Serial.println(" ");

  // For  Pressure sensor 2
  
  

  Serial.print("Pressure2 = ");
  Serial.println(PS2Reading);
  Serial.println(" ");
  
  // For the LEDS
 
  if (PS1Reading > PS2Reading){
    while (PS1Reading != 0){
   digitalWrite(11, HIGH);
   digitalWrite(12, LOW);
    }
  }
  else if (PS1Reading < PS2Reading) {
    while (PS2Reading != 0){
   digitalWrite(12, HIGH);
   digitalWrite(11, LOW);
    }
  }
  else {
    digitalWrite(12, LOW);
    digitalWrite(11, LOW);
    
  }
  Serial.println("about to return from readPressure()");
  return (PS1Reading||PS2Reading) ;
}
 */

// ================================================================
// ===						TCP CONNECTION						===
// ================================================================
#define DATA_QUAT 0.00
#define DATA_EUL 1.00
#define DATA_YPR 2.00
#define DATA_AAR 3.00
#define DATA_AAW 4.00
#define DATA_PRESS 5
#define DATA_TIME 6
#define TRANS_HIT 1
#define TRANS_FAULT 0
#define TRANSMISSION_ATTEMPTS 5
#define FAULT_HIT -1
TCPClient client;
byte server[] = { 192, 168, 1, 28 }; // to the queen(one as a server photon)  wlan IP
float imuPacket[5];
long int timePacket[2];
int pressurePacket[3];
bool start=true;
/*
bool receivedStart(){

	int readVal = client.read();
	Serial.print("I have read"); Serial.println(readVal);
	if(readVal==49){
		Serial.println("WE HAVE RECEIVED THE RIGHT VALUE");
		Serial.println();
		Serial.println();
		Serial.println();
		Serial.println();
		delay(5000);
		client.flush();  //make sure that there is nothing to be read: client.connected() returns true if connection closed and data needs to be read.
		return true;
	}
	else
		Serial.println("ERROR: we received value: "); Serial.print(readVal);
		return false;
}

bool sendIMUPacket(float cmd, float d0 ,float d1, float d2, float d3){
		
	imuPacket[0]=cmd;
	imuPacket[1]=d0;
	imuPacket[2]=d1;
	imuPacket[3]=d2;
	imuPacket[4]=d3;
	
	
	return (client.write((const uint8_t*)&imuPacket, sizeof(imuPacket))>0)? true:false;
}

bool sendPressurePacket(int cmd){
	if(cmd){
		pressurePacket[0] = DATA_PRESS;
		pressurePacket[1] = PS1Reading;
		pressurePacket[2]=PS2Reading;
	}
	else{
		pressurePacket[0] = DATA_PRESS;
		pressurePacket[1] = FAULT_HIT;
		pressurePacket[2]= FAULT_HIT;
	}
	return (client.write((uint8_t*)&pressurePacket,sizeof(pressurePacket))>0)? true:false;
}

bool sendTimePacket(){
	timePacket[0]=DATA_TIME;
	timePacket[1]=timer;
	client.write((uint8_t*)timePacket,sizeof(timePacket));
	Serial.println("There has been a hit! Assign point.");
	return (client.write((uint8_t*)&timePacket,sizeof(timePacket))>0)? true:false;
}*/


// ================================================================
// ===                      INITIAL SETUP                       ===
// ================================================================

void setup() {
	
    // join I2C bus (I2Cdev library doesn't do this automatically)
    #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
		#if defined (SPARK)
			Wire.setSpeed(CLOCK_SPEED_400KHZ);
			Wire.begin();
		#else
			Wire.begin();
			TWBR = 24; // 400kHz I2C clock (200kHz if CPU is 8MHz)
		#endif
    #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
        Fastwire::setup(400, true);
    #endif
	
	WiFi.on();
	WiFi.connect();

	while(!WiFi.ready())
	{
		Spark.process();
		delay(100);                 
	}
  Spark.process();

  WiFi.ping(WiFi.gatewayIP()); 

    // initialize serial communication
    // (115200 chosen because it is required for Teapot Demo output, but it's
    // really up to you depending on your project)
    Serial.begin(115200);

    while (!Serial){
		Spark.process(); // wait for Leonardo enumeration, others continue immediately
		delay(100);
	}
	
	Serial.println("connecting to wifi");
    
    //while(!WiFi.ready());
    Serial.println(WiFi.localIP());
    Serial.println(WiFi.subnetMask());
    Serial.println(WiFi.gatewayIP());
    Serial.println(WiFi.SSID());

  Serial.println("connecting to server");

  if (client.connect(server, 2222))
  {
    Serial.println("server connected");
	
	delay(5000);
  }
  else
  {
    Serial.println("connection failed");
	delay(5000);
  }

/*
  
  pinMode(LED1pin,OUTPUT);
  pinMode(LED2pin,OUTPUT);
  
  attachInterrupt(D3, tipPressed, FALLING);	//at the moment it's set at RISING, might have to vhange depending on sword sword configuration

*/
    // NOTE: 8MHz or slower host processors, like the Teensy @ 3.3v or Ardunio
    // Pro Mini running at 3.3v, cannot handle this baud rate reliably due to
    // the baud timing being too misaligned with processor ticks. You must use
    // 38400 or slower in these cases, or use some kind of external separate
    // crystal solution for the UART timer.

    // initialize device
    Serial.println(F("Initializing I2C devices..."));
    mpu.initialize();

    // verify connection
    Serial.println(F("Testing device connections..."));
    Serial.println(mpu.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));

    // wait for ready
    Serial.println(F("\nSend any character to begin DMP programming and demo: "));
    while (Serial.available() && Serial.read()); // empty buffer
    while (!Serial.available());                 // wait for data
    while (Serial.available() && Serial.read()); // empty buffer again

    // load and configure the DMP
    Serial.println(F("Initializing DMP..."));
    devStatus = mpu.dmpInitialize();

    // supply your own gyro offsets here, scaled for min sensitivity
    mpu.setXGyroOffset(220);
    mpu.setYGyroOffset(76);
    mpu.setZGyroOffset(-85);
    mpu.setZAccelOffset(1788); // 1688 factory default for my test chip

    // make sure it worked (returns 0 if so)
    if (devStatus == 0) {
		// turn on the DMP, now that it's ready
		Serial.println(F("Enabling DMP..."));
		mpu.setDMPEnabled(true);

		// enable Arduino interrupt detection
		Serial.println(F("Enabling interrupt detection (Arduino external interrupt 0)..."));
	#if defined (SPARK)
		attachInterrupt(D2, dmpDataReady, RISING);
	#else
		attachInterrupt(0, dmpDataReady, RISING);
	#endif
		mpuIntStatus = mpu.getIntStatus();

		// set our DMP Ready flag so the main loop() function knows it's okay to use it
		Serial.println(F("DMP ready! Waiting for first interrupt..."));
		dmpReady = true;

		// get expected DMP packet size for later comparison
		packetSize = mpu.dmpGetFIFOPacketSize();
		//mpu.setDMPEnabled(false);
	} else {
		// ERROR!
		// 1 = initial memory load failed
		// 2 = DMP configuration updates failed
		// (if it's going to break, usually the code will be 1)
		Serial.print(F("DMP Initialization failed (code "));
		Serial.print(devStatus);
		Serial.println(F(")"));
	}

    // configure LED for output
    pinMode(LED_PIN, OUTPUT);
}



// ================================================================
// ===                    MAIN PROGRAM LOOP                     ===
// ================================================================
// ================================================================
// === read incoming start ========================================
//	============== start timer ====================================
// ======================read IMU data ============================
// ============================when tip pressed====================
// =================================read pressure==================
// ====================================Y=======N===================
// =================================send======send=================
// ===========================time&pressure======fault=============

void loop() {
	/*while(!start){
		start = receivedStart();
		Serial.println("I am waiting for start signal");
	}*/
	if(start){/*
		Serial.println("Received start signal");
		Serial.println("Starting Internal timer");
		myTimer.interrupt_SIT(INT_ENABLE);
		myTimer.begin(incrementTimer,2,hmSec);
		//Serial.println(F("Enabling DMP..."));
        //mpu.setDMPEnabled(true);
		
        
		swordtipInterrupt ? Serial.println("TRUE!!!!") : Serial.println("FALSE");
		delay(3000);
        // get expected DMP packet size for later comparison
        //packetSize = mpu.dmpGetFIFOPacketSize();*/;
		while(start){
	

			// wait for MPU interrupt or extra packet(s) available
			while (!mpuInterrupt && fifoCount < packetSize) {
				/*Serial.println("WHILE PASSED"); delay(500);
				if(swordtipInterrupt){
					start = false;
					mpu.setDMPEnabled(false);
					mpu.resetFIFO();
					swordtipInterrupt=false;
					myTimer.interrupt_SIT(INT_DISABLE);	
					int i=0;
					if(readPressure()){
						
						for(; i<TRANSMISSION_ATTEMPTS && !sendPressurePacket(TRANS_HIT); i++){
							Serial.println("ERROR: could not write pressure packet to server");
							Serial.println("Attempt "); Serial.print(i); Serial.print("failed");
							Serial.println("Attempting "); Serial.print(i+1); Serial.print("/5");								
						}
						if(i==TRANSMISSION_ATTEMPTS){
							Serial.println("ERROR: exceeded timeout attempting to send pressure packet");
							// ********END PROGRAM?*******************
							// ******OR****TRY****CONNECT***AGAIN*****
						}
							
					}else{
						Serial.println("Hit not on target area");
						for(; i<TRANSMISSION_ATTEMPTS && !sendPressurePacket(TRANS_FAULT); i++){
							Serial.println("ERROR: could not write pressure packet to server");
							Serial.println("Attempt "); Serial.print(i); Serial.print("failed");
							Serial.println("Attempting "); Serial.print(i+1); Serial.print("/5");								
						}
						if(i==TRANSMISSION_ATTEMPTS){
							Serial.println("ERROR: exceeded timeout attempting to send pressure packet");
							// ********END PROGRAM?*******************
							// ******OR****TRY****CONNECT***AGAIN*****
						}
					}
					for(i=0; i<TRANSMISSION_ATTEMPTS && !sendTimePacket(); i++){
							Serial.println("ERROR: could not write time packet to server");
							Serial.println("Attempt "); Serial.print(i); Serial.print("failed");
							Serial.println("Attempting "); Serial.print(i+1); Serial.print("/5");								
					}
					if(i==TRANSMISSION_ATTEMPTS){
						Serial.println("ERROR: exceeded timeout attempting to send time packet");
						// ********END PROGRAM?*******************
						// ******OR****TRY****CONNECT***AGAIN*****
					}
					break;
				}else break;
				*/
			}
			Serial.println("Not in while"); delay (500);
			// reset interrupt flag and get INT_STATUS byte
			mpuInterrupt = false;
			
		//	mpu.setFIFOEnabled(true);
			mpuIntStatus = mpu.getIntStatus();
			
			// get current FIFO count
			fifoCount = mpu.getFIFOCount();

			// check for overflow (this should never happen unless our code is too inefficient)
			if ((mpuIntStatus & 0x10) || fifoCount == 1024) {
				// reset so we can continue cleanly
				mpu.resetFIFO();
				Serial.println(F("FIFO overflow!"));
			// otherwise, check for DMP data ready interrupt (this should happen frequently)
			} else if ((mpuIntStatus & 0x02) > 0) {
				// wait for correct available data length, should be a VERY short wait
				while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();

				// read a packet from FIFO
				mpu.getFIFOBytes(fifoBuffer, packetSize);
				
				// track FIFO count here in case there is > 1 packet available
				// (this lets us immediately read more without waiting for an interrupt)
				fifoCount -= packetSize;

				#ifdef OUTPUT_READABLE_QUATERNION
					// display quaternion values in easy matrix form: w x y z
					mpu.dmpGetQuaternion(&q, fifoBuffer);
					Serial.print("quat\t");
					
					Serial.print(q.w);
					Serial.print("\t");
					Serial.print(q.x);
					Serial.print("\t");
					Serial.print(q.y);
					Serial.print("\t");
					Serial.println(q.z);
				#endif

				#ifdef OUTPUT_READABLE_EULER
					// display Euler angles in degrees
					mpu.dmpGetQuaternion(&q, fifoBuffer);
					mpu.dmpGetEuler(euler, &q);
					Serial.print("euler\t");
					
					Serial.print(euler[0] * 180/M_PI);
					Serial.print("\t");
					Serial.print(euler[1] * 180/M_PI);
					Serial.print("\t");
					Serial.println(euler[2] * 180/M_PI);
				#endif

				#ifdef OUTPUT_READABLE_YAWPITCHROLL
					// display Euler angles in degrees
					mpu.dmpGetQuaternion(&q, fifoBuffer);
					mpu.dmpGetGravity(&gravity, &q);
					mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
					Serial.print("ypr\t");
					
					Serial.print(ypr[0] * 180/M_PI);
					Serial.print("\t");
					Serial.print(ypr[1] * 180/M_PI);
					Serial.print("\t");
					Serial.println(ypr[2] * 180/M_PI);
				#endif

				#ifdef OUTPUT_READABLE_REALACCEL
					// display real acceleration, adjusted to remove gravity
					mpu.dmpGetQuaternion(&q, fifoBuffer);
					mpu.dmpGetAccel(&aa, fifoBuffer);
					mpu.dmpGetGravity(&gravity, &q);
					mpu.dmpGetLinearAccel(&aaReal, &aa, &gravity);
					Serial.print("areal\t");
					
					Serial.print(aaReal.x);
					Serial.print("\t");
					Serial.print(aaReal.y);
					Serial.print("\t");
					Serial.println(aaReal.z);
				#endif

				#ifdef OUTPUT_READABLE_WORLDACCEL
					// display initial world-frame acceleration, adjusted to remove gravity
					// and rotated based on known orientation from quaternion
					mpu.dmpGetQuaternion(&q, fifoBuffer);
					mpu.dmpGetAccel(&aa, fifoBuffer);
					mpu.dmpGetGravity(&gravity, &q);
					mpu.dmpGetLinearAccel(&aaReal, &aa, &gravity);
					mpu.dmpGetLinearAccelInWorld(&aaWorld, &aaReal, &q);
					Serial.print("aworld\t");
					
					Serial.print(aaWorld.x);
					Serial.print("\t");
					Serial.print(aaWorld.y);
					Serial.print("\t");
					Serial.println(aaWorld.z);
				#endif

				// blink LED to indicate activity
				blinkState = !blinkState;
				digitalWrite(LED_PIN, blinkState);
			}else{
				Serial.println("WTF IS HAPPENING?");
			}
		}
	/*
		swordtipInterrupt=false;
		Serial.println("Sword Tip Pressed interrupt detected");
		delay(5000);*/
	}
	
}
