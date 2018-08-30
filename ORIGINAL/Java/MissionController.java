import java.util.Scanner;
import java.net.*;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;
import java.io.*;
 
public class MissionController {	
	public static String sentence;
	public static Socket clientSocket;
    public static BufferedReader inFromServer;	
    public static float yAxis, xAxis;
    public static int btn0, btn1, btn2, btn3, btn4, btn5, btn6, btn7, btn8, btn9, btn10, btn11;
	
	public static int maneuverDir;
	
	public static double prevRadius, radius, theta, slope;
	public static String taskId = "X";
	
	/* Complete Mission Logger */
	private static String logFileName;
	private static Writer logWriter;
	
	/* Interrupt from Arduino to start mission */
	public static boolean executeMission = false;
	public static long missionStartTime, missionEndTime, missionStartDelay = 10;
	
	/* IMU Drift Correction Settings */
	public static boolean correctIMUDrift = false;
	public static double imuDriftRate = 0.000116;
	
	/* TCP Server for Image Processing */
	protected static TCPServer IPServer = new TCPServer();
	
	/* Serial Communication */
	private static SSC arduinoComm = new SSC();
	private static SSC IMUComm = new SSC();
	public static String IMUString = null;
	
	/* Inertial Measurement Unit */
	private static IMU VN100 = new IMU();
	
	/* State Variables */
	protected static MissionIO heading = new MissionIO();
	protected static MissionIO depth = new MissionIO();
	
	/* Control Variables */
	private static MissionIO leftSpeed = new MissionIO();
	private static MissionIO rightSpeed = new MissionIO();
	private static MissionIO frontSpeed = new MissionIO();
	private static MissionIO backSpeed = new MissionIO();
	private static MissionIO frontSwaySpeed = new MissionIO();
	private static MissionIO backSwaySpeed = new MissionIO();
	private static MissionIO statusLED = new MissionIO();
	
	/* Sensor Feedback */
	private static MissionIO voltage12VBatt = new MissionIO();
	private static MissionIO voltage18VBatt = new MissionIO();
	private static MissionIO sysTemperature = new MissionIO();
	private static MissionIO currentLeftSurgeT = new MissionIO();
	private static MissionIO currentRightSurgeT = new MissionIO();
	private static MissionIO currentForeSwayT = new MissionIO();
	private static MissionIO currentAftSwayT = new MissionIO();
	private static MissionIO currentForeHeaveT = new MissionIO();
	private static MissionIO currentAftHeaveT = new MissionIO();
	
	
	/* Task Controllers */
		//Feedback Controllers
		protected static Controller surgeController = new Controller();						
		protected static Controller heaveController = new Controller();  					
		protected static Controller swayController = new Controller();
		//No Feedback Controllers						
		protected static Controller simpleSurgeController = new Controller();			
		protected static Controller simpleHeaveController = new Controller();			
		protected static Controller simpleSwayController = new Controller();
		//Stop Motion Controller			
		protected static Controller stopMotionController = new Controller();
		
	/* Tasks */
		// Feedback Controlled Tasks
		protected static Task Surge = new Task();																	
		protected static Task Heave = new Task();																	
		protected static Task Sway = new Task();	
		// No Feedback Tasks
		protected static Task simpleSurge = new Task();
		protected static Task simpleHeave = new Task();
		protected static Task simpleSway = new Task();
		// Rotation Task																				
		protected static Task Rotate = new Task();
		// Stop Motion Task
		protected static Task StopMotion = new Task();

	/* File Logger */
	private static FileLogger missionLogger = new FileLogger();
	
	/* TCP Logger */
	private static Logger logger = new Logger();
	
	/* Problem Statement Tasks */
	private static BuoyTask yellowBuoy = new BuoyTask();
	private static BuoyTask redBuoy = new BuoyTask();
	private static GateTask gate = new GateTask();
	private static LaneTask orangeLane = new LaneTask();
	private static BuoyTask maneuveringTask = new BuoyTask();
	
	private static boolean oneBuoy = false;
	private static boolean alternate = false;
	
	//Global vars to keep track on AUV state
	private static int globalHeading = 0, globalDepth = 0, globalSwaySpeed = 0, globalSurgeSpeed = 0;
	
	private static int missionStartHeading = 0, missionStartDepth = 0;
	private static int missionStartSurgeSpeed = 100, missionStartSwaySpeed = 0;
	private static int laneFollowDepth = 0, buoyDetectionStartDepth = 0, crossGateDepth = 0, crossGateHeading = 0, buoyLastHeading = 0, countBuoyFrames = 0, countNonZeroBuoyFrames = 0, countGateFrames = 0;
	private static int depthUpdateCount = 0;
	
	//Flags
	private static boolean flagB = false, hardHit = false, buoysDetectedOnce = false, buoysOutOfScope = false, flagG = false, gateDetectedOnce = false, gateCrossed = false, arenaParamsSet = false;
	
	//Hardcoded heading value w.r.t. arena
	private static int buoysHeading = 0, gateHeading = 0, markerHeading = 0, octagonHeading = 0;
	
	//Threads
	private static Thread IPThread, logThread;
		
	public static void logData(String logText){
		logToFile(false, "\u001B[34m" + logText + "\u001B[0m");
	}

	private static Process proc = null;
	
	public static void logToFile(boolean appendNewLine, String str){
		try{
			if( appendNewLine ){
				System.out.println(str);
				logWriter.write(str + "\r\n");
				logWriter.flush();
			}
			else{
				System.out.print(str);
				logWriter.write(str);
				logWriter.flush();
			}
		}
		catch(Exception e){
			e.printStackTrace();
		}
	}
	
	public static void InitializeMission() throws InterruptedException{
		/* File Logger */
		missionLogger.initialize();
		missionLogger.createNewLog();
		
		/* 12V Bank Voltage Sensor */
		voltage12VBatt.typeInput = true;
		voltage12VBatt.serialComm = arduinoComm;
		voltage12VBatt.identifier = 'h';
		voltage12VBatt.multiplier = 4;
		
		/* 18V Bank Voltage Sensor */
		voltage18VBatt.typeInput = true;
		voltage18VBatt.serialComm = arduinoComm;
		voltage18VBatt.identifier = 'H';
		voltage18VBatt.multiplier = 4;
		
		/* System Temperature Sensor */
		sysTemperature.typeInput = true;
		sysTemperature.serialComm = arduinoComm;
		sysTemperature.identifier = 'T';
		sysTemperature.multiplier = 4;
		
		/* Current Sensor Left Surge */
		currentLeftSurgeT.typeInput = true;
		currentLeftSurgeT.serialComm = arduinoComm;
		currentLeftSurgeT.identifier = 'Z'; //Z
		currentLeftSurgeT.multiplier = 4;
		
		/* Current Sensor Right Surge */
		currentRightSurgeT.typeInput = true;
		currentRightSurgeT.serialComm = arduinoComm;
		currentRightSurgeT.identifier = 'Y';  //Y
		currentRightSurgeT.multiplier = 4;
		
		/* Current Sensor Fore Sway */
		currentForeSwayT.typeInput = true;
		currentForeSwayT.serialComm = arduinoComm;
		currentForeSwayT.identifier = 'V';
		currentForeSwayT.multiplier = 4;
		
		/* Current Sensor Aft Sway */
		currentAftSwayT.typeInput = true;
		currentAftSwayT.serialComm = arduinoComm;
		currentAftSwayT.identifier = 'U';
		currentAftSwayT.multiplier = 4;
		
		/* Current Sensor Fore Heave */
		currentForeHeaveT.typeInput = true;
		currentForeHeaveT.serialComm = arduinoComm;
		currentForeHeaveT.identifier = 'X'; //x
		currentForeHeaveT.multiplier = 4;
		
		/* Current Sensor Aft Heave */
		currentAftHeaveT.typeInput = true;
		currentAftHeaveT.serialComm = arduinoComm;
		currentAftHeaveT.identifier = 'W'; //W
		currentAftHeaveT.multiplier = 4;
		
		/* Heading */
		heading.typeInput = true;
		heading.serialComm = IMUComm;
		heading.identifier = 'A';

		/* Depth */
		depth.typeInput = true;
		depth.serialComm = arduinoComm;
		depth.identifier = 'B';
		depth.multiplier = 4;

		/* Status LED */		
		statusLED.typeOutput = false;
		statusLED.serialComm = arduinoComm;
		statusLED.identifier = 'D';
		
		/* Left Speed */
		leftSpeed.typeOutput = true;
		leftSpeed.serialComm = arduinoComm;
		leftSpeed.identifier = 'F';

		/* Right Speed */
		rightSpeed.typeOutput = true;
		rightSpeed.serialComm = arduinoComm;
		rightSpeed.identifier = 'E';

		/* Front Speed */
		frontSpeed.typeOutput = true;
		frontSpeed.serialComm = arduinoComm;
		frontSpeed.identifier = 'G';

		/* Back Speed */
		backSpeed.typeOutput = true;
		backSpeed.serialComm = arduinoComm;
		backSpeed.identifier = 'C';

		/* Front Sway Speed */
		frontSwaySpeed.typeOutput = true;
		frontSwaySpeed.serialComm = arduinoComm;
		frontSwaySpeed.identifier = 'Q';
			
		/* Back Sway Speed */
		backSwaySpeed.typeOutput = true;
		backSwaySpeed.serialComm = arduinoComm;
		backSwaySpeed.identifier = 'P';

		/* Surge PID Controller */
		surgeController.setType(1);
		surgeController.setBounds(0, 254);
		surgeController.setSecondaryBounds(-30,30);
		surgeController.setCoeffecients(3.5F, 0.0F, 0.01F);
		surgeController.setMultipliers(-1, 1);
		surgeController.setBaseValues(180, 180);
		surgeController.setOutputCount(2);
		surgeController.setHeadingController(true);
		surgeController.setErrorLimit(6);
		
		/* Heave PID Controller */
		heaveController.setType(1);
		heaveController.setBounds(-254, 254);
		heaveController.setCoeffecients(10.0F, 0.8F, 0.5F); 
		heaveController.setMultipliers(-1,-1);
		heaveController.setBaseValues(0, 0); //Find the base values; 10,10
		heaveController.setOutputCount(2);

		/* Sway PID Controller */
		swayController.setType(1);
		swayController.setBounds(-254, 254);
		swayController.setCoeffecients(6.0F, 0.05F, 2.7F);
		swayController.setMultipliers(1, -1);
		swayController.setBaseValues(200, 200);
		swayController.setOutputCount(2);
		swayController.setHeadingController(true);

		/* Simple Surge Controller */
		simpleSurgeController.setType(0);
		simpleSurgeController.setBounds(-254, 254);
		simpleSurgeController.setMultipliers(1, 1);
		simpleSurgeController.setOutputCount(2);

		/* Simple Heave Controller */
		simpleHeaveController.setType(0);
		simpleHeaveController.setBounds(0, 254);
		simpleHeaveController.setMultipliers(-1, -1);
		simpleHeaveController.setBaseValues(0, 0);
		simpleHeaveController.setOutputCount(2);

		/* Simple Sway Controller */
		simpleSwayController.setType(0);
		simpleSwayController.setBounds(-254, 254);
		simpleSwayController.setMultipliers(1, -1);
		simpleSwayController.setOutputCount(2);
		
		/* Stop Motion Controller */
		stopMotionController.setType(0);
		stopMotionController.setBaseValue(0);
		stopMotionController.setMultiplier(1);
		stopMotionController.setOutputCount(1);
		
		/* Serial Communication With Arduino */
		arduinoComm.initialize("/dev/ttyACM0", 230400, false);
		//arduinoComm.initialize("/dev/ttyACM0", 115200);
        //arduinoComm.initialize("/dev/ttyACM1", 115200, false);
		/* Serial Communication With IMU */
		IMUComm.imuObject = VN100;
		IMUComm.initialize("/dev/ttyUSB0", 230400, true);
		//IMUComm.initialize("/dev/ttyUSB0", 230400);
		
		/* Feedback Controlled Surge Motion */
		Surge.initialize(heading, leftSpeed, rightSpeed, surgeController);

		/* Feedback Controlled Heave Motion */
		Heave.initialize(depth, frontSpeed, backSpeed, heaveController);

		/* Feedback Controlled Sway Motion */
		Sway.initialize(heading, frontSwaySpeed, backSwaySpeed, swayController);

		/* Without Feedback Surge Motion */
		simpleSurge.initialize(new MissionIO(), leftSpeed, rightSpeed, simpleSurgeController);
		
		/* Without Feedback Heave Motion */
		simpleHeave.initialize(new MissionIO(), frontSpeed, backSpeed, simpleHeaveController);
		
		/* Without Feedback Sway Motion */
		simpleSway.initialize(new MissionIO(), frontSwaySpeed, backSwaySpeed, simpleSwayController);
		
		/* Rotation */
		Rotate.initialize(heading, leftSpeed, rightSpeed, surgeController);

		/* Stop Motion */
		StopMotion.initialize(new MissionIO(), new MissionIO('J', arduinoComm, false), stopMotionController);
		
		/* Yellow Buoy Hit Task */
		yellowBuoy.initialize("Green", 206, 80, 2, 7);	// (280, 90, 2, 7);
		yellowBuoy.setDepthLimits(30, 300);
		yellowBuoy.setSwaySpeedLimits(-200, 200);
		
		/* Red Buoy Hit Task */
		redBuoy.initialize("Red", 195, 80, 2, 10);		//190
		redBuoy.setDepthLimits(30, 300);
		redBuoy.setSwaySpeedLimits(-200, 200);
		
		/* Pass Through Gate Task */
		gate.initialize("Gate", 70, 200, 2, 7);
		gate.setDepthLimits(90, 150);
		gate.setSwaySpeedLimits(-180, 160);
		
		/* Orane Lane Align Task */
		orangeLane.initialize(250);
		orangeLane.setSwaySpeedLimits(-160, 140);
		orangeLane.setSurgeSpeedLimits(-140, 100);
		
		/* Maneuvering Task */
		maneuveringTask.initialize("Maneuvering Gate", 100, 70, 4, 10);
		maneuveringTask.setDepthLimits(60, 160);
		maneuveringTask.setSwaySpeedLimits(-160, 140);
	}
	
	/* Halt Mission on Arduino Interrupt During Mission */
	
	public static void halt() throws InterruptedException, IOException {
		IPThread.stop();
		StopMotion.Do();
		logToFile(true,"HALTMISSION");
		System.exit(1);
	}
	
	
	/* Update system variables */
	public static void updateSysVars() throws InterruptedException{
		// 12V Bank Voltage
		voltage12VBatt.update();
		voltage12VBatt.getUpdatedValue();
		// 18V Bank Voltage
		voltage18VBatt.update();
		voltage18VBatt.getUpdatedValue();
	}
	
	/* Logging values in a file */
	public static void recordState(float time){
		missionLogger.parameterValues.add(time);		
		missionLogger.parameterValues.add((float) leftSpeed.getValue());		
		missionLogger.parameterValues.add((float) rightSpeed.getValue());		
		missionLogger.parameterValues.add((float) frontSpeed.getValue());		
		missionLogger.parameterValues.add((float) backSpeed.getValue());		
		missionLogger.parameterValues.add((float) frontSwaySpeed.getValue());		
		missionLogger.parameterValues.add((float) backSwaySpeed.getValue());		
		missionLogger.parameterValues.add((float) depth.getValue());		
		missionLogger.parameterValues.add((float) heading.getValue());		
		missionLogger.parameterValues.add((float) voltage12VBatt.getValue());		
		missionLogger.parameterValues.add((float) voltage18VBatt.getValue());		
	}
	 
	/* Adjust surge controller parameters *
	 * @params : speed [0 - 255], speed lower bound [ (-255) - 255], speed upper bound [ (-255) - 255)] *
	 * e.g. : For a static controller about zero speed, call setSurgeParameters(0, -240, 240) */
	public static void setSurgeParameters(int speed, int lBound, int uBound) throws InterruptedException{
		surgeController.setMultipliers(-1, 1);
		surgeController.setBaseValues(speed, speed);
		surgeController.setBounds(lBound, uBound);
	}
	
	/* Adjust surge controller multipliers *
	 * @params : left multiplier [ > 0 ], right multiplier [ > 0 ] */
	public static void setSurgeMultipliers(int lMultiplier, int rMultiplier){
	 	surgeController.setMultipliers(-lMultiplier, rMultiplier);	
	}
	
	/* Adjust sway controller multipliers *
	 * @params : front multiplier [ > 0 ], back multiplier [ > 0 ] */
	public static void setSwayMultipliers(int fMultiplier, int bMultiplier){
	 	swayController.setMultipliers(fMultiplier, -bMultiplier);	
	}
	
	/* Adjust sway controller parameters *
	 * @params : speed [0 - 255], speed lower bound [ (-255) - 255], speed upper bound [ (-255) - 255)] */
	public static void setSwayParameters(int speed, int lBound, int uBound) throws InterruptedException{
		swayController.setMultipliers(1, -1);
		swayController.setBaseValues(speed, speed);
		swayController.setBounds(-254, 254);
	}
	
	/* Adjust surge controller to move in forward direction *
	 * @params : speed [0 - 255] */
	public static void setForward(int speed) throws InterruptedException{
		setSurgeParameters(speed, 0, 254);
	}
		
	/* Adjust surge controller to move in backward direction *
	 * @params : speed [0 - 255] */
	public static void setBackward(int speed) throws InterruptedException{
		setSurgeParameters(-speed, -254, 0);
	}
		
	/* Adjust sway controller to move in right direction *
	 * @params : speed [0 - 255] */
	public static void setRight(int speed) throws InterruptedException{
		setSwayParameters(speed, 0, 254);
	}
	
	/* Adjust sway controller to move in left direction *
	 * @params : speed [0 - 255] */
	public static void setLeft(int speed) throws InterruptedException{
		setSwayParameters(-speed, -254, 0);
	}
	
	/* Adjust sway controller to move in appropriate direction *
	 * @params : speed [-255 - 255] */
	public static void setSway(int speed) throws InterruptedException{
		// Left
		if( speed < 0 )
			setSwayParameters(speed, -254, 0);
		// Right
		else if( speed >= 0 )
			setSwayParameters(speed, 0, 254);
	}
	
	/* Motion control along x-direction *
	 * @params : time [seconds], target heading[0 - 359], target depth[ > 0 ] */
	public static void xSurge(int time, int targetHeading, int targetDepth) throws InterruptedException, FileNotFoundException{
		/* Update System Variables */
		updateSysVars();
		
		/* Flush Logger */
		missionLogger.flush();
		
		/* Set Log Title */
		missionLogger.logTitle = "Surge @Heading: "+targetHeading+" @Depth: "+targetDepth +" @SurgeCoeffecients: "+surgeController.getCoeffecientsAsString()+" @HeaveCoeffecients: "+heaveController.getCoeffecientsAsString();
		
		/* Start TCP Logger */
		logger.sendData(missionLogger.logTitle + "\r\n\r\n");
		logger.start();
		
		/* Timed Task */
		long startTime = System.currentTimeMillis();
		long currentTime = startTime; 
		
		while( currentTime <= startTime + time * 1000 ){
			Surge.At(targetHeading);	
			Heave.At(targetDepth);	
			recordState( (float)(currentTime - startTime) );
			currentTime = System.currentTimeMillis();
			logToFile(false, "\rLeft Speed: " + leftSpeed.getValue() + ", Right Speed: " + rightSpeed.getValue() + ", targetHeading: "+ targetHeading + ", Current Heading : " + heading.getUpdatedValue() );
		}
	
		/* Stop TCP Logger */
		logger.stop();
		
		/* Stop Motion */
		StopMotion.Do();
		
		/* Dump Logged Values */
		missionLogger.dump();
	}
	
	/* Motion control along y-direction *
	 * @params : time [seconds], target heading[0 - 359], target depth[ > 0 ] */
	public static void ySway(int time, int targetHeading, int targetDepth) throws InterruptedException, FileNotFoundException{
		/* Update System Variables */
		updateSysVars();
		
		/* Flush Logger */
		missionLogger.flush();
		
		/* Log Title */
		missionLogger.logTitle = "Sway @Heading: "+targetHeading+" @SwayCoeffecients: "+swayController.getCoeffecientsAsString()+" @Depth: "+targetDepth+" @HeaveCoeffecients: "+heaveController.getCoeffecientsAsString();
		
		/* Timed Task */
		long startTime = System.currentTimeMillis();
		long currentTime = startTime; 
		
		logToFile(true, "Format : {frontSwaySpeed, backSwaySpeed, time, depth, heading, $VNYMR}" );
			  	
		while( currentTime <= startTime + time * 1000 ){
			Sway.At(targetHeading);
			Heave.At(targetDepth);
			recordState( (float)(currentTime - startTime) );
			currentTime = System.currentTimeMillis();
			
			/* Log Data */
			depth.update();
			heading.update();
			frontSwaySpeed.update();
			backSwaySpeed.update();
			int dt = (int)(currentTime - startTime);
			logToFile(true, frontSwaySpeed.getValue() + "," + backSwaySpeed.getValue() + "," + dt + "," + depth.getUpdatedValue() + "," + heading.getUpdatedValue() + ","+IMUString ); 
			
			//Thread.sleep(20);
		}
		
		/* Stop Motion */
		StopMotion.Do();	
		
		/* Dump Logged Values */
		missionLogger.dump();
	}
	
	/* Motion control along z-direction *
	 * @params : time [seconds], target heading[0 - 359], target depth[ > 0 ] */
	public static void zHeave(int time, int targetDepth) throws InterruptedException, FileNotFoundException{
		/* Update System Variables */
		updateSysVars();
		
		heading.update();//
		int currentHeading = heading.getUpdatedValue();//
		//surgeController.setBaseValues(0,0);//
		swayController.setBaseValues(0,0);//
		swayController.setBounds(-254, 254);
		
		/* Flush Logger */
		missionLogger.flush();
		
		/* Log Title */
		missionLogger.logTitle = "Dive @Depth: "+targetDepth+" @HeaveCoeffecients: "+heaveController.getCoeffecientsAsString();
		
		/* Timed Task */
		long startTime = System.currentTimeMillis();
		long currentTime = startTime; 
		
		while( currentTime <= startTime + time * 1000 ){
			Heave.At(targetDepth);
			Sway.At(currentHeading);//
			recordState( (float)(currentTime - startTime) );
			currentTime = System.currentTimeMillis();
				
			/* Log Data */
			depth.update();
			heading.update();
			currentForeHeaveT.update();
			currentAftHeaveT.update();
			int dt = (int)(currentTime - startTime);
			logToFile(true, currentForeHeaveT.getUpdatedValue() + "," + currentAftHeaveT.getUpdatedValue() + "," + dt + "," + depth.getUpdatedValue() + "," + heading.getUpdatedValue() + ","+IMUString ); 
					
		}
		
		/* Stop Motion */
		StopMotion.Do();	
		
		/* Dump Logged Values */
		missionLogger.dump();
	}
	
	/* Combined motion control along all axes *
	 * @params : time of flight [seconds], target heading [0 - 359], target depth [ > 0 ] */
	public static void xyzSpace(int time, int targetHeading, int targetDepth) throws InterruptedException, FileNotFoundException{
		/* Update System Variables */
		updateSysVars();
		
		/* Flush Logger */
		missionLogger.flush();
		
		/* Log Title */
		missionLogger.logTitle = "Sway @Heading: "+targetHeading+" @SwayCoeffecients: "+swayController.getCoeffecientsAsString()+" @Depth: "+targetDepth+" @HeaveCoeffecients: "+heaveController.getCoeffecientsAsString();
		
		/* Log Title */
		missionLogger.logTitle = "xyz-Combined @Heading: "+targetHeading+" @SurgeCoeffecients: "+surgeController.getCoeffecientsAsString()+" @Depth: "+targetDepth+" @HeaveCoeffecients: "+heaveController.getCoeffecientsAsString()+" @SwayCoeffecients: "+swayController.getCoeffecientsAsString();

		/* Timed Task */
		long startTime = System.currentTimeMillis();
		long currentTime = startTime; 
		
		while( currentTime <= startTime + time * 1000 ){
			Surge.At(targetHeading);
			Heave.At(targetDepth);
			Sway.At(targetHeading);
			recordState( (float)(currentTime - startTime) );
			currentTime = System.currentTimeMillis();
		}
		
		/* Stop Motion */
		StopMotion.Do();

		/* Dump Logged Values */
		missionLogger.dump();
	}
	
	/* Brake forward motion *
	 * @params : target heading [0 - 359] */
	public static void brakeForward(int targetHeading) throws InterruptedException, FileNotFoundException{
		/* Timed Task */
		moveBackward(120, 1, targetHeading);				
	}
	
	/* Move forward at target heading and at target depth *
	 * @params : speed [0 - 255], time of flight [seconds], target heading [0 - 359], target depth [ > 0 ] */
	public static void moveForward(int speed, int time, int targetHeading, int targetDepth) throws InterruptedException, FileNotFoundException{
		/* Adjust the Controller */
		setForward(speed);
		
		xSurge(time, targetHeading, targetDepth);
	}
	
	/* Move forward at target heading *
	 * @params : speed [0 - 255], time of flight [seconds], target heading [0 - 359] */
	public static void moveForward(int speed, int time, int targetHeading) throws InterruptedException, FileNotFoundException{
		/* Update Depth */
		depth.update();
		int currentDepth = depth.getUpdatedValue();
		
		moveForward(speed, time, targetHeading, currentDepth);
	}
	
	/* Move forward at current heading *
	 * @params : speed [0 - 255], time of flight [seconds] */
	public static void moveForward(int speed, int time) throws InterruptedException, FileNotFoundException{
		/* Update Heading */
		heading.update();
		int currentHeading = heading.getUpdatedValue();

		moveForward(speed, time, currentHeading);
	}

	/* Move backward at target heading and at target depth *
	 * @params : speed [0 - 255], time of flight [seconds], target heading [0 - 359], target depth [ > 0 ] */
	public static void moveBackward(int speed, int time, int targetHeading, int targetDepth) throws InterruptedException, FileNotFoundException{
		/* Adjust Controller */
		setBackward(speed);

		xSurge(time, targetHeading, targetDepth);
	}
		
	/* Move backward at target heading *
	 * @params : speed [0 - 255], time of flight [seconds], target heading [0 - 359] */	
	public static void moveBackward(int speed, int time, int targetHeading) throws InterruptedException, FileNotFoundException{
		/* Update Depth */
		depth.update();
		int currentDepth = depth.getUpdatedValue();
		
		moveBackward(speed, time, targetHeading, currentDepth);
	}
	
	/* Move backward at current heading *
	 * @params : speed [0 - 255], time of flight [seconds] */
	public static void moveBackward(int speed, int time) throws InterruptedException, FileNotFoundException{
		/* Update Heading */
		heading.update();
		int currentHeading = heading.getUpdatedValue();
		
		moveBackward(speed, time, currentHeading);
	}

	/* Dive to a given depth *
	 * @params : target depth [ > 0 ], time of flight [seconds] */
	public static void dive(int diveDepth, int time) throws InterruptedException, FileNotFoundException{
		zHeave(time, diveDepth);
	}
	
	/* Sway Leftwards at given depth *
	 * @params : speed [0 - 255], time of flight [seconds], depth [ > 0 ] */
	public static void swayLeft(int speed, int time, int targetDepth) throws InterruptedException, FileNotFoundException{
		/* Update heading */
		heading.update();
		int currentHeading = heading.getUpdatedValue();
		
		/* Adjust Controller */
		setLeft(speed);
		
		ySway(time, currentHeading, targetDepth);	
	}
	
	/* Sway Leftwards *
	 * @params : speed [0 - 255], time of flight [seconds] */
	public static void swayLeft(int speed, int time) throws InterruptedException, FileNotFoundException{
		/* Update heading */
		depth.update();
		int currentDepth = depth.getUpdatedValue();
		
		swayLeft(speed, time, currentDepth);		
	}
	
	/* Sway Rightwards at given depth *
	 * @params : speed [0 - 255], time of flight [seconds], depth [ > 0 ] */
	public static void swayRight(int speed, int time, int targetDepth) throws InterruptedException, FileNotFoundException{
		/* Update heading */
		heading.update();
		int currentHeading = heading.getUpdatedValue();
		
		/* Adjust Controller */
		setRight(speed);
		
		ySway(time, currentHeading, targetDepth);		
	}
	
	/* Sway Rightwards *
	 * @params : speed [0 - 255], time of flight [seconds] */
	public static void swayRight(int speed, int time) throws InterruptedException, FileNotFoundException{
		/* Update heading */
		depth.update();
		int currentDepth = depth.getUpdatedValue();
		
		swayRight(speed, time, currentDepth);		
	}
	
	/* Combined surge, heave and sway at target heading *
	 * @params : surge speed [0 - 255], target heading [0 - 359], target depth [ > 0 ], sway speed [0 - 359], time [seconds]*/
	public static void combined(int surgeSpeed, int targetHeading, int targetDepth, int swaySpeed, int time) throws InterruptedException, FileNotFoundException{
		/* Set Surge Parameters */
		if( surgeSpeed > 0 ) setForward(surgeSpeed);
		else setBackward(surgeSpeed);
		
		/* Set Sway Parameters */
		if( swaySpeed > 0) setRight(swaySpeed);
		else setLeft(swaySpeed);
				
		xyzSpace(time, targetHeading, targetDepth);
	}
	
	/* Combined surge, heave and sway at current heading *
	 * @params : surge speed [0 - 255], target heading [0 - 359], target depth [ > 0 ], sway speed [0 - 359], time [seconds]*/
	public static void combined(int surgeSpeed, int targetDepth, int swaySpeed, int time) throws InterruptedException, FileNotFoundException{
		/* Update heading */
		heading.update();
		int currentHeading = heading.getUpdatedValue();
		
		combined(surgeSpeed, currentHeading, targetDepth, swaySpeed, time);
	}
	
	/* Simple Surge Controller *
	 * @params: */
  public static void simpleSurge(int speed, int time, char dir) throws InterruptedException{
  	logToFile(true, "\r\nSimple Surge @Speed : " + speed + ", @Time : " + time + ", @Dir : " + dir );
  	logToFile(true, "Format : {time,depth,heading,$VNYMR}" );
  
  	//Set Multipliers & Base Values
  	simpleSurgeController.setMultipliers(1, 1);
  	simpleSurgeController.setBounds(-254, 254);
  	
  	//Set Timing
		long startTime = System.currentTimeMillis();
		long currentTime = startTime; 
				
  	switch( dir ){
  		// Forward
  		case 'F':
  			simpleSurgeController.setBaseValues(speed, speed);
  			
				//Execute task
				simpleSurge.Do();	
  			
  			/* Timed Task */						
				while( currentTime <= startTime + time * 1000 ){
				
					/* Log Data */
					depth.update();
					heading.update();
					currentLeftSurgeT.update();
					currentRightSurgeT.update();
					int dt = (int)(currentTime - startTime);
					logToFile(true, currentLeftSurgeT.getUpdatedValue() + "," + currentRightSurgeT.getUpdatedValue() + "," + dt + "," + depth.getUpdatedValue() + "," + heading.getUpdatedValue() + ","+IMUString ); 
					
					currentTime = System.currentTimeMillis();
					//Thread.sleep(20);
				}
  		break;
			// Backwards
  		case 'B':
  			simpleSurgeController.setBaseValues(-speed, -speed);
  			
				//Execute task
				simpleSurge.Do();	
						
  			/* Timed Task */
				while( currentTime <= startTime + time * 1000 ){
				
					/* Log Data */
					depth.update();
					heading.update();
					int dt = (int)(currentTime - startTime);
					logToFile(true, dt + "," + depth.getUpdatedValue() + "," + heading.getUpdatedValue() + ","+IMUString ); 
					
					currentTime = System.currentTimeMillis();
					//Thread.sleep(20);
				}
  		break;
  		default:
  		break;
  	}
  	
  	// Stopmotion
  	StopMotion.Do();
  }
		
	/* Simple Sway Controller *
	 * @params: */
  public static void simpleSway(int speed, int time, char dir) throws InterruptedException{
  	logToFile(true, "\r\nSimple Sway @Speed : " + speed + ", @Time : " + time + ", @Dir : " + dir );
  	logToFile(true, "Format : {time,depth,heading,$VNYMR}" );
  	
  	//Set Multipliers
  	simpleSwayController.setMultipliers(1, 1);
  	simpleSwayController.setBounds(-254, 254);
  	
  	//Set Timing
		long startTime = System.currentTimeMillis();
		long currentTime = startTime; 
						
  	switch( dir ){
  		// Right
  		case 'R':
  			simpleSwayController.setBaseValues(speed, speed);
  			
				//Execute Task	
				simpleSway.Do();
  			
  			/* Timed Task */
				while( currentTime <= startTime + time * 1000 ){
				
					/* Log Data */
					depth.update();
					heading.update();
					currentForeSwayT.update();
					currentAftSwayT.update();
					int dt = (int)(currentTime - startTime);
					logToFile(true, currentForeSwayT.getUpdatedValue() + "," + currentAftSwayT.getUpdatedValue() + "," + dt + "," + depth.getUpdatedValue() + "," + heading.getUpdatedValue() + ","+IMUString ); 
					
					currentTime = System.currentTimeMillis();
					//Thread.sleep(20);
				}
  		break;
			// Left
  		case 'L':
  			simpleSwayController.setBaseValues(-speed, -speed);
  			
				//Execute Task	
				simpleSway.Do();
  			  			
  			/* Timed Task */
				while( currentTime <= startTime + time * 1000 ){
				
					/* Log Data */
					depth.update();
					heading.update();
					currentForeSwayT.update();
					currentAftSwayT.update();
					int dt = (int)(currentTime - startTime);
					logToFile(true, currentForeSwayT.getUpdatedValue() + "," + currentAftSwayT.getUpdatedValue() + "," + dt + "," + depth.getUpdatedValue() + "," + heading.getUpdatedValue() + ","+IMUString ); 
					
					currentTime = System.currentTimeMillis();
					//Thread.sleep(20);
				}
  		break;
  		default:
  		break;
  	}
  	
  	// Stopmotion
  	StopMotion.Do();
  }		
	
		
	/* Simple Heave Controller *
	 * @params: */
  public static void simpleHeave(int speed, int time, char dir) throws InterruptedException{
  	logToFile(true, "\r\nSimple Heave @Speed : " + speed + ", @Time : " + time + ", @Dir : " + dir );
  	logToFile(true, "Format : {time,depth,heading,$VNYMR}" );
  	
  	//Set Multipliers
  	simpleHeaveController.setMultipliers(-1, -1);
  	simpleHeaveController.setBounds(-254, 254);
  	
  	//Set Timing
		long startTime = System.currentTimeMillis();
		long currentTime = startTime; 
						
  	switch( dir ){
  		// Upward
  		case 'U':
  			simpleHeaveController.setBaseValues(speed, speed);
			
				//Execute Task
				simpleHeave.Do();
  			
  			/* Timed Task */
				while( currentTime <= startTime + time * 1000 ){
				
					/* Log Data */
					depth.update();
					heading.update();
					currentForeHeaveT.update();
					currentAftHeaveT.update();
					int dt = (int)(currentTime - startTime);
					logToFile(true, currentForeHeaveT.getUpdatedValue() + "," + currentAftHeaveT.getUpdatedValue() + "," + dt + "," + depth.getUpdatedValue() + "," + heading.getUpdatedValue() + ","+IMUString ); 
					
					currentTime = System.currentTimeMillis();
					//Thread.sleep(10);
				}
  		break;
			// Downward
  		case 'D':
  			simpleHeaveController.setBaseValues(-speed, -speed);
			
				//Execute Task
				simpleHeave.Do();
  			  			
  			/* Timed Task */
				while( currentTime <= startTime + time * 1000 ){
				
					/* Log Data */
					depth.update();
					heading.update();
					currentForeHeaveT.update();
					currentAftHeaveT.update();
					int dt = (int)(currentTime - startTime);
					logToFile(true, currentForeHeaveT.getUpdatedValue() + "," + currentAftHeaveT.getUpdatedValue() + "," + dt + "," + depth.getUpdatedValue() + "," + heading.getUpdatedValue() + ","+IMUString ); 
					
					currentTime = System.currentTimeMillis();
					//Thread.sleep(10);
				}
  		break;
  		default:
  		break;
  	}
  	
  	// Stopmotion
  	StopMotion.Do();
  }		
		
		
	/* Simple Yaw Controller *
	 * @params: */
  public static void simpleYaw(int speed, int time, char dir) throws InterruptedException{
  	logToFile(true, "\r\nSimple Yaw @Speed : " + speed + ", @Time : " + time + ", @Dir : " + dir );
  	logToFile(true, "Format : {time,depth,heading,$VNYMR}" );
  	
  	//Set Multipliers
  	simpleSwayController.setMultipliers(-1, -1);
  	simpleSwayController.setBounds(-254, 254);
  	
  	//Set Timing
		long startTime = System.currentTimeMillis();
		long currentTime = startTime; 
				
  	switch( dir ){
  		// Anti-Clockwise
  		case 'C':
  			simpleSwayController.setBaseValues(speed, -speed);
  			
  			/* Timed Task */
				while( currentTime <= startTime + time * 1000 ){
				
					/* Log Data */
					depth.update();
					heading.update();
					currentForeSwayT.update();
					currentAftSwayT.update();
					int dt = (int)(currentTime - startTime);
					logToFile(true, currentForeSwayT.getUpdatedValue() + "," + currentAftSwayT.getUpdatedValue() + "," + dt + "," + depth.getUpdatedValue() + "," + heading.getUpdatedValue() + ","+IMUString ); 
					
					simpleSway.Do();
					currentTime = System.currentTimeMillis();
					//Thread.sleep(10);
				}
  		break;
			// Clockwise
  		case 'A':
  			simpleSwayController.setBaseValues(-speed, speed);
  			  			
  			/* Timed Task */
				while( currentTime <= startTime + time * 1000 ){
				
					/* Log Data */
					depth.update();
					heading.update();
					currentForeSwayT.update();
					currentAftSwayT.update();
					int dt = (int)(currentTime - startTime);
					logToFile(true, currentForeSwayT.getUpdatedValue() + "," + currentAftSwayT.getUpdatedValue() + "," + dt + "," + depth.getUpdatedValue() + "," + heading.getUpdatedValue() + ","+IMUString ); 
					
					simpleSway.Do();
					currentTime = System.currentTimeMillis();
					//Thread.sleep(10);
				}
  		break;
  		default:
  		break;
  	}
  	
  	// Stopmotion
  	StopMotion.Do();
  }	
  
	public static char updateRemoteControl(){
	  try{ 
			sentence = inFromServer.readLine(); 
			// logToFile(true, sentence); 
	  } 
	  catch(Exception e){ 
	  	e.printStackTrace(); 
	  }
	  logToFile(true, "\r\n");
	  
	  if( Integer.parseInt(sentence) ==  38 ){ // Up - Forward
	  	logToFile(true, "Key Pressed: Up");
	  	return 'U';
	  }
	  else if( Integer.parseInt(sentence) ==  40 ){ // Down - Backward
	  	logToFile(true, "Key Pressed: Down");
	  	return 'B';
	  }
	  else if( Integer.parseInt(sentence) ==  37 ){ // Left - Sway
	  	logToFile(true, "Key Pressed: Left");
	  	return 'L';
	  }
	  else if( Integer.parseInt(sentence) ==  39 ){ // Right - Sway
	  	logToFile(true, "Key Pressed: Right");
	  	return 'R';
	  }
	  else if( Integer.parseInt(sentence) ==  87 ){ // W - Up
	  	logToFile(true, "Key Pressed: W");
	  	return 'W';
	  }
	  else if( Integer.parseInt(sentence) ==  83 ){ // S - Down
	  	logToFile(true, "Key Pressed: S");
	  	return 'S';
	  }
	  else if( Integer.parseInt(sentence) ==  65 ){ // A - Rotate Anti-Clockwise
	  	logToFile(true, "Key Pressed: A");
	  	return 'A';
	  }
	  else if( Integer.parseInt(sentence) ==  68 ){ // D - Rotate Clockwise
	  	logToFile(true, "Key Pressed: D");
	  	return 'D';
	  }
	  else if( Integer.parseInt(sentence) ==  88 ){ // X - Stop Motion
	  	logToFile(true, "Key Pressed: X");
	  	return 'X';
	  }
		return 'X';  
	}
	
	public static void main(String[] args) throws InterruptedException, FileNotFoundException{
	
		/* Parse Arguments */	
		alternate = Boolean.parseBoolean(args[0]);
		missionStartDelay = Integer.parseInt(args[1]);
		oneBuoy = Boolean.parseBoolean(args[2]);
		
		/* Configure FileLogger */
		ArrayList<String> names = new ArrayList<String>();
		names.add("time"); 
		names.add("leftSurgeSpeed"); 
		names.add("rightSurgeSpeed"); 
		names.add("frontSpeed"); 
		names.add("backSpeed"); 
		names.add("frontSwaySpeed"); 
		names.add("backSwaySpeed"); 
		names.add("depth"); 
		names.add("heading");
		missionLogger.parameterCount = 9; 
		
		/* Complete Mission Logger */
		int logCount = new File("/home/auv/Desktop/RoboSub2014/auvs/MissionController/logs/complete/").list().length;
		String logFileName = "/home/auv/Desktop/RoboSub2014/auvs/MissionController/logs/complete/" + (logCount + 1)+ ".txt";
		try{ logWriter = new BufferedWriter(new OutputStreamWriter(new FileOutputStream(logFileName), "utf-8"));}
		catch(Exception e) { e.printStackTrace(); }
		
		logThread = new Thread("Log Thread"){
			public void run(){
				long startTime = System.currentTimeMillis();
				int port = 8080;
				float secondsPassed = 0;
				String str = null;
				try{
					logger.initialize(port);
				}
				catch(Exception e){
					e.printStackTrace();
				}
				while(true){
					if( logger.log ){
						try{
							str = VN100.getDataString(); 
							
							if( logger.sendData(str) != -1 ){
							  Thread.sleep(20);
							}
							else{
								Thread.sleep(20);
								logger.closeServer();
								logger.initialize(port);
							}
							
							Thread.sleep(10);
						}
						catch(Exception e){
							e.printStackTrace();
						}
					}
					try{ Thread.sleep(20); } catch(Exception e) {}
				}
			}
		};

		Thread remoteControlThread = new Thread("Remote Control Thread"){
		  
			public void run(){		
				System.out.println("Starting Remote Control Thread");
				
				/* Connect to Remote Control Input Server */
			  try { clientSocket = new Socket("192.168.1.4", 9898); }
			  catch(Exception e){ e.printStackTrace(); }
			  
			  /* Bind IO Streams to the server */
			  //DataOutputStream outToServer = new DataOutputStream(clientSocket.getOutputStream());
			  try { inFromServer = new BufferedReader(new InputStreamReader(clientSocket.getInputStream())); }
			  catch(Exception e){ e.printStackTrace(); }
			  
			  /* Control Parameters */
			  int controllerSurgeSpeed = 160, controllerSwaySpeed = 160, controllerDepth = 0;
			  int deltaSurge = 5, deltaSway = 5, deltaDepth = 1;
			  int manualDepth = 10;
			  
			  /* Make sure vehicle is in a stopped state */
			  try { StopMotion.Do(); }  catch(Exception e){}
			  
			  /* Refresh heading and depth */
		  	try{ heading.update(); }  catch(Exception e){}
		  	try{ depth.update(); }  catch(Exception e){}
		  	
		  	/* Keep listening to control inputs */
			  while( true ){
				  char inp = updateRemoteControl();
				  
				  if( inp == 'U' ){
						try{ setForward( controllerSurgeSpeed ); } catch(Exception e){}
						int tHeading = heading.getUpdatedValue();
						logToFile(true, "Moving Forward :: Depth : " + manualDepth + " : Heading : " + heading.getUpdatedValue() );
						while( inp == 'U' ){
							inp = updateRemoteControl();
							logToFile(true, "Moving Forward :: Depth : " + manualDepth + " : Heading : " + heading.getUpdatedValue() );
							try{ Surge.At( tHeading ); Heave.At(manualDepth); } catch(Exception e){}
							try{ Thread.sleep(50); } catch(Exception e){}
						}
						try { StopMotion.Do(); }  catch(Exception e){}
				  }
				  else if( inp == 'B' ){
						try{ setBackward( controllerSurgeSpeed ); } catch(Exception e){}
						int tHeading = heading.getUpdatedValue();
						logToFile(true, "Moving Backward :: Depth : " + manualDepth + " : Heading : " + heading.getUpdatedValue() );
						while( inp == 'B' ){
							inp = updateRemoteControl();
							logToFile(true, "Moving Backward :: Depth : " + manualDepth + " : Heading : " + heading.getUpdatedValue() );
							try{ Surge.At( tHeading ); Heave.At(manualDepth); } catch(Exception e){}
							try{ Thread.sleep(50); } catch(Exception e){}
						}	
						try { StopMotion.Do(); }  catch(Exception e){}
				  }
				  
				  if( inp == 'L' ){
						try{ setLeft( controllerSwaySpeed ); }	catch(Exception e){}
					
						surgeController.setMultipliers(-2, 2);
						surgeController.setBaseValues(0, 0);
						surgeController.setBounds(-254, 254);
		
						int tHeading = heading.getUpdatedValue();
						logToFile(true, "Sway Left :: Depth : " + manualDepth + " : Heading : " + heading.getUpdatedValue() );
						while( inp == 'L' ){
							inp = updateRemoteControl();
							logToFile(true, "Sway Left :: Depth : " + manualDepth + " : Heading : " + heading.getUpdatedValue() );
							try{ Sway.At( tHeading ); Heave.At(manualDepth); } catch(Exception e){}
							try{ Surge.At( tHeading ); } catch(Exception e){}
						}
						try { StopMotion.Do(); }  catch(Exception e){}
				  }
				  else if( inp == 'R' ){
						try{ setRight( controllerSwaySpeed ); }  catch(Exception e){}
					
						surgeController.setMultipliers(-2, 2);
						surgeController.setBaseValues(0, 0);
						surgeController.setBounds(-254, 254);
		
						int tHeading = heading.getUpdatedValue();
						logToFile(true, "Sway Right :: Depth : " + manualDepth + " : Heading : " + heading.getUpdatedValue() );
						while( inp == 'R' ){
							inp = updateRemoteControl();
							logToFile(true, "Sway Right :: Depth : " + manualDepth + " : Heading : " + heading.getUpdatedValue() );
							try{ Sway.At( tHeading ); Heave.At(manualDepth); } catch(Exception e){}
							try{ Surge.At( tHeading ); } catch(Exception e){}
						}
						try { StopMotion.Do(); }  catch(Exception e){}
				  }				  
				  else if( inp == 'A' ){
					logToFile(true, "Rotate Anti-Clockwise :: Depth : " + manualDepth + " : Heading : " + heading.getUpdatedValue() );
					 while( inp == 'A' ){
						 inp = updateRemoteControl();
						 logToFile(true, "Rotate Anti-Clockwise :: Depth : " + manualDepth + " : Heading : " + heading.getUpdatedValue() );
						 leftSpeed.setValue(157);
						 rightSpeed.setValue(-157);
						 try{ Heave.At(manualDepth); } catch(Exception e){}
					 }
					try { StopMotion.Do(); }  catch(Exception e){}
				  }
				  else if( inp == 'D' ){
					logToFile(true, "Rotate Clockwise");
					 while( inp == 'D' ){
						 inp = updateRemoteControl();
						 logToFile(true, "Rotate Clockwise :: Depth : " + manualDepth + " : Heading : " + heading.getUpdatedValue() );
						 leftSpeed.setValue(-157);
						 rightSpeed.setValue(157);
						 try{ Heave.At(manualDepth); } catch(Exception e){}
					 } 
					try { StopMotion.Do(); }  catch(Exception e){}
				  }
				  
				  if( inp == 'W' ){
					logToFile(true, "Upwards");
					 while( inp == 'W' ){
						 inp = updateRemoteControl();
						 logToFile(true, "Upwards :: Depth : " + manualDepth + " : Heading : " + heading.getUpdatedValue() );
						 frontSpeed.setValue(150);
						 backSpeed.setValue(150);
						 
						 try{ depth.update(); } catch(Exception e){}
						 manualDepth = depth.getUpdatedValue();
						 
					 }
					try { StopMotion.Do(); }  catch(Exception e){}
				  }
				  else if( inp == 'S' ){
					logToFile(true, "Downwards");
					 while( inp == 'S' ){
						 inp = updateRemoteControl();
						 logToFile(true, "Downwards :: Depth : " + manualDepth + " : Heading : " + heading.getUpdatedValue() );
						 frontSpeed.setValue(-150);
						 backSpeed.setValue(-150);
						 
						 try{ depth.update(); } catch(Exception e){}
						 manualDepth = depth.getUpdatedValue();						 
						 
					 }
					try { StopMotion.Do(); }  catch(Exception e){}
				  }
				  else if( inp == 'X' ){
					logToFile(true, "Stop Motion");
					 while( inp == 'S' ){
					 	inp = updateRemoteControl();
					 	logToFile(true, "Stop Motion");
						try { StopMotion.Do(); }  catch(Exception e){}
					 }
				  }			  
			  }
			}
		};

		IPThread = new Thread("Image Processing Thread"){
			public void run(){
				/* Initialize IP Server */
				try{ IPServer.initialize();	} catch(Exception e){ e.printStackTrace(); }	
				/* Wait till the server is up */				
				while( !IPServer.isRunning ){
					try { Thread.sleep(1); } catch(Exception e){}							
				}
	
				// Set AUV in forward gear with base speed
				missionStartSurgeSpeed = 85;
				try { setForward(85); } catch (Exception e) {}	
				globalSurgeSpeed = missionStartSurgeSpeed;
				
				// Starting the mission, Execute the mission in proper heading only
				// Update parameters
				try { heading.update(); } catch (Exception e) {}				
				missionStartHeading = heading.getUpdatedValue();
				globalHeading = missionStartHeading;
				missionStartDepth = 0;
						
				//First Task - Lane Follow, Set Depth
				laneFollowDepth = 46;  //Global Setting //110
				globalDepth = laneFollowDepth;
				
				/* Hardcoded Gate Cross *
				logToFile(true, "Gate Cross (HC) : Target heading : " + missionStartHeading );
				try { moveForward(110,24,missionStartHeading,150); } catch (Exception e) {}	
				try { heading.update(); } catch (Exception e) {}	
				int iniHd = heading.getUpdatedValue();
				logToFile(true, "Gate Crossed (HC) : Current heading : " + iniHd);*/
				
				/* Gate */
				logToFile(true, "Gate Hardcoded Cross");		
				try{ moveForward(200,23,missionStartHeading,100); } catch (Exception e) {}	
				System.out.println("Gate Cross Over" + " " + taskId);
				
				/* Update Heading */	
				try { heading.update(); } catch (Exception e) {}				
				missionStartHeading = heading.getUpdatedValue();
				System.out.println("Gate Cross Over" + " " + taskId);
				
				while( !taskId.equals("Q") ){
					switch( taskId ){ 
					
						case "P":	
							/* Align to lane */						
							try { orangeLane.align(missionStartHeading, 70); } catch (Exception e) {}					
							break;
						case "B":							
							/* Red buoy first */
							if( !yellowBuoy.taskComplete ){
							
								/* Start with hitting red buoy */
								try { heading.update(); } catch (Exception e) {}	
								logToFile(true, "Current Heading: " + heading.getUpdatedValue() + ", Target Heading: " + missionStartHeading);
								if ( !redBuoy.taskComplete )
									  try { redBuoy.hit(missionStartHeading); } catch (Exception e) {}	
							
								/* Transition from red buoy to yellow buoy */
								if ( redBuoy.taskComplete && !yellowBuoy.taskComplete && !oneBuoy ){
									  logToFile(true, "Transition from Red buoy to Yellow buoy");
									  try{ depth.update(); } catch (Exception e) {}
									  int redToYellowTransitionDepth = depth.getUpdatedValue() + 30;
									  
									  try{ moveBackward(160, 5, missionStartHeading, redToYellowTransitionDepth ); } catch (Exception e) {}
									  try{ swayRight(160, 9, redToYellowTransitionDepth ); } catch (Exception e) {}
									  try{ StopMotion.Do(); } catch (Exception e) {}
									  logToFile(true, " completed");									  
									  
										/* Notify IP server to detect next Buoy */
										logToFile(true, "Notifying IP Server");
										IPServer.sendMessage("HYB\0");
										
										/* Set target depth for yellow buoy : Assuming yellow buoy is above red buoy */ 
										int	yellowBuoyStartDepth = redBuoy.getTargetDepth() - 0;  
										yellowBuoy.setStartDepth( yellowBuoyStartDepth );
										try { depth.update(); } catch (Exception e) {}	
										logToFile(true, "Setting yellow buoy start depth to : " + yellowBuoyStartDepth + ", Current depth : " + depth.getUpdatedValue() );
								}
								
								/* Hit yellow buoy */
								/* === */ int tempTargetHeading = missionStartHeading - 2;
								try { heading.update(); } catch (Exception e) {}	
								logToFile(true, "Current Heading: " + heading.getUpdatedValue() + ", Target Heading: " + missionStartHeading );
								
								if ( redBuoy.taskComplete && !oneBuoy )
									  try { yellowBuoy.hit(tempTargetHeading); } catch (Exception e) {}	
									  
								/* Transition from yellow buoy to path */
							  if( redBuoy.taskComplete && yellowBuoy.taskComplete && !oneBuoy ){
							  
							  	/* === */ tempTargetHeading = missionStartHeading - 0; 
							  	try { heading.update(); } catch (Exception e) {}	
							  	logToFile(true, "Transition from yellow buoy to path => Current Heading: " + heading.getUpdatedValue() + ", Target Heading': " + tempTargetHeading);							  	
							  										
									/* Transition */
							  	try{ moveBackward(110, 1, tempTargetHeading); } catch (Exception e) {}	
							  	
							  	/* LED Buoy *
							  	logToFile(true, "LED Buoy");
							  	try{ moveBackward(110, 2, tempTargetHeading); } catch (Exception e) {}	
									try{ swayRight(160, 8, redToYellowTransitionDepth ); } catch (Exception e) {} 
									try{ depth.update(); } catch (Exception e) {} 
							  	try{ moveForward(130, 9, tempTargetHeading, depth.getUpdatedValue() - 60 ); } catch (Exception e) {}	*/
							  	 
									try { heading.update(); depth.update(); } catch (Exception e) {}	
							  	int buoyToPathTransitionDepth = depth.getUpdatedValue() - 80;
									logToFile(true, "Moving Forward: Current Heading: " + heading.getUpdatedValue() + ", Target Heading': " + tempTargetHeading + ", Transition Depth : " + buoyToPathTransitionDepth );
									
							  	try{ dive(buoyToPathTransitionDepth, 4); } catch (Exception e) {}
							  	try{ moveForward(130, 9, tempTargetHeading, buoyToPathTransitionDepth); } catch (Exception e) {}	
							  	try{ StopMotion.Do(); } catch (Exception e) {}								  	
							  	
									/* Notify IP server to detect path */
									logToFile(true, "Notifying IP Server");
									IPServer.sendMessage("ATP\0");
							  }
							  
							  /* Transition from Red buoy to Path (Skipping Yellow Buoy) */
							  if( redBuoy.taskComplete && oneBuoy && !alternate){
							  	logToFile(true,"Hard Move to Green ");
							 	 	tempTargetHeading = missionStartHeading + 10;
							  
							  	/* Move Back */
							  	try{ moveBackward(160, 3, tempTargetHeading); } catch (Exception e) {}
							  	
							  	/* Sway Right */
							  	try{ swayRight(160, 7); } catch (Exception e) {}
							  	
							  	/* Go Forward Hit Yellow Buoy and Back */
							  	try{ moveForward(90, 9); } catch (Exception e) {}
							  	try{ moveBackward(160, 4, tempTargetHeading); } catch (Exception e) {}
							  	try{ swayRight(160, 8); } catch (Exception e) {}
							  	try{ 
							  		Rotate.By(100,'A'); 
							  		depth.update();
							  		int LEDDepth = depth.getUpdatedValue() - 40;
							  		dive(LEDDepth, 2);
							  		swayRight(160, 10); 
							  		swayLeft(160, 5); 
							  		Rotate.By(50,'C'); 
							  		
							  		/* Maneuvering */
							  		moveForward(160, 16, tempTargetHeading - 30);
							  		StopMotion.Do();							  		
							  	} catch (Exception e) {}
							  }
							  	/* Transition from Red to Green to LED Buoy (alternate configuration)*/
							  	if( redBuoy.taskComplete && oneBuoy && alternate ){
							 	 		tempTargetHeading = missionStartHeading + 10;
							 	 		logToFile(true, "Hard Move to Green, LED");
							  		
							  		/* Option 1 *
							  		try{
							  			moveBackward(160, 3, tempTargetHeading);
							  			swayRight(160, 7); 
							  			moveForward(90, 9);
							  			moveBackward(120, 4, tempTargetHeading);
							  			
							  			depth.update();
							  			int LEDDepth = depth.getUpdatedValue() - 60;
							  			dive(LEDDepth, 2);
							  			moveForward(110, 2);							  			
							  			swayLeft(160, 12); 
							  			
											// Maneuvering 
											moveForward(160, 16, tempTargetHeading - 30);
											StopMotion.Do();							  		
							  			
							  		}
							  		catch(Exception e){
							  		}*/
							  		
							  		/* Option 2 */
							  		try{
							  			depth.update();
							  			heading.update();
							  			int dDepth = depth.getUpdatedValue();
							  			logToFile(true,"Moving back after Red hard hit");
							  			moveBackward(110, 2,  heading.getUpdatedValue(), dDepth+10);
							  			logToFile(true,"Rotate anticlockwise");
							  			Rotate.By(90,'A');
							  			
							  			heading.update();
							  			logToFile(true,"Moving backward towards green");
							  			moveBackward(130, 5, heading.getUpdatedValue() , dDepth+10);
							  			logToFile(true,"Swaying towards green");
							  			swayRight(140, 13, dDepth+10); 		
							  			logToFile(true,"Swaying away from green");					  			
							  			swayLeft(120, 10, dDepth+10); 
							  			
							  			// Move towards LED Buoy
							  			int LEDDepth = dDepth - 115;
							  			logToFile(true,"Rising to LED Buoy level");
							  			dive(LEDDepth, 6);
							  			
							  			logToFile(true,"Moving forward towards LED Buoy");
							  			heading.update();
							  			moveBackward(125, 5, heading.getUpdatedValue()  - 4); // +6
							  			
							  			logToFile(true,"Move towards LED Buoy");
							  			swayRight(130, 16, LEDDepth); 
							  			moveForward(130, 2, heading.getUpdatedValue());
							  			logToFile(true,"Move away from LED Buoy");
							  			swayLeft(130, 12, LEDDepth);
							  			
							  			// Towards Maneuvering Gate 
							  			heading.update(); 
							  			int currH = heading.getUpdatedValue();
							  			logToFile(true,"Moving towards maneuvering gate: Current heading: " + currH + ", missionStartHeading: "+ missionStartHeading +", At depth: " + 280);
							  			Rotate.By(80,'C'); 
							  			moveForward(130, 6,missionStartHeading+10, 150); 
							  			StopMotion.Do();
							  			try{ Thread.sleep(3000); } catch(Exception e){}
							  			Rotate.By(40,'A'); 
							  			moveForward(130, 40,missionStartHeading+15, 280); //280
							  				
							  		}
							  		catch(Exception e){
							  		}
							  	
							  	/*  */
							  	while(true){
							  		System.out.println("Mission End");
							  		System.exit(0);
							  		try{ StopMotion.Do(); } catch(Exception e){}
							  		try{ Thread.sleep(50); } catch(Exception e){}
							  	} 
								  		
							  }
							  
							}						
							break;
						case "M":
							/* Maneuvering Task */
							if( !maneuveringTask.taskComplete )
								try{ maneuveringTask.hit(missionStartHeading); } catch(Exception e) {}
								
							break;
						case "G":
							/* Cross gate */
							try { gate.hit(missionStartHeading); } catch (Exception e) {}	
							break;
						case "S":
							try { StopMotion.Do(); }  catch(Exception e){}
							break;
						default:
							break;
					}
					try {
						Thread.sleep(50);
					} catch (InterruptedException e) {
						// TODO Auto-generated catch block
						e.printStackTrace();
					}	
				}	
			}
		};
		
		/* Initialize hardware and mission control variables */		
		InitializeMission();
				
		/* Wait for execute mision interrupt from arduino */
	  executeMission = true;
		while( !executeMission ){
			logToFile(false, "Waiting for arduino execute mission interrupt ...\r");
			Thread.sleep(100);
		}
		logToFile(true, "");
		
		missionStartTime = System.currentTimeMillis();		
		while( System.currentTimeMillis() - missionStartTime < missionStartDelay * 1000){
			int tp = (int)(System.currentTimeMillis() - missionStartTime)/1000;
			logToFile(false, "\rInterrupt received : Starting Mission in " + (missionStartDelay - 1 - tp) + " seconds ...");
			try{ statusLED.update(); } catch(Exception e){ e.printStackTrace(); }
			Thread.sleep(20);
		}
		logToFile(true, "");
		
		/* Start IP Thread */
		//IPThread.start();
		
		/* Start Logging */
		logThread.start();
		//logger.start();

		/* Start IP System Process *
	
		try{
			proc = Runtime.getRuntime().exec("/home/auv/Desktop/RoboSub2014/final/start_ip.sh");
			// Process proc = Runtime.getRuntime().exec("/home/auv/Desktop/Roboub2014/final_wo_green/start_ip.sh");
			proc.waitFor();
			
			BufferedReader stdInput = new BufferedReader(new InputStreamReader(proc.getInputStream()));

      BufferedReader stdError = new BufferedReader(new InputStreamReader(proc.getErrorStream()));

      logToFile(true, "IP PROCESS STANDARD OUTPUT\r\n");
      String s = null;
      while ((s = stdInput.readLine()) != null) {
          logToFile(true, s);
      }

      logToFile(true, "IP PROCESS ERROR OUTPUT\r\n");
      while ((s = stdError.readLine()) != null) {
          logToFile(true, s);
      }
        
		}
		catch(Exception e){
			e.printStackTrace();
		}*/
					
		final Scanner userInput = new Scanner(System.in);
		
		/* CLI for AUV Debugging */
		int opt = -1, idx = 1;
      
      	
		while( opt != 19 ){
			idx = 1;
			logData("\u001B[33m Choose an option:\r\n\u001B[0m");
			logData("\u001B[37m" + idx + ".\u001B[34m Surge Forward\r\n");
			logData("\u001B[37m" + (++idx) + ".\u001B[34m Surge Backward\r\n");
			logData("\u001B[37m" + (++idx) + ".\u001B[34m Dive\r\n");
			logData("\u001B[37m" + (++idx) + ".\u001B[34m Sway Left\r\n");
			logData("\u001B[37m" + (++idx) + ".\u001B[34m Sway Right\r\n");
			logData("\u001B[37m" + (++idx) + ".\u001B[34m Rotate\r\n");
			logData("\u001B[37m" + (++idx) + ".\u001B[34m Combined\r\n");
			logData("\u001B[37m" + (++idx) + ".\u001B[34m Heading Value\r\n");
			logData("\u001B[37m" + (++idx) + ".\u001B[34m Depth Value\r\n");
			logData("\u001B[37m" + (++idx) + ".\u001B[34m Start using IP thread\r\n");
			logData("\u001B[37m" + (++idx) + ".\u001B[34m Set Arena Parameters\r\n");
			logData("\u001B[37m" + (++idx) + ".\u001B[34m Set Marker Parameters\r\n");
			logData("\u001B[37m" + (++idx) + ".\u001B[34m Start Remote Control Thread\r\n");
			logData("\u001B[37m" + (++idx) + ".\u001B[34m Battery Voltages\r\n");
			logData("\u001B[37m" + (++idx) + ".\u001B[34m Surge at Speed\r\n");
			logData("\u001B[37m" + (++idx) + ".\u001B[34m Sway at Speed\r\n");
			logData("\u001B[37m" + (++idx) + ".\u001B[34m Dive at Speed\r\n");
			logData("\u001B[37m" + (++idx) + ".\u001B[34m Yaw at Speed\r\n");
			logData("\u001B[37m" + (++idx) + ".\u001B[34m Quit\r\n");
			opt = Integer.parseInt(userInput.next());
			
			/* Temporary Variables */
      int iniHd = 0, time = 0, speed = 0, direction = 0, tempDepth = 0;
      char dir;
      
      /* User Options */
			switch( opt ){
				// Move Forward
				case 1:
					heading.update();
					iniHd = heading.getUpdatedValue();
					logToFile(false, "Moving forward at heading: " + iniHd + "Enter time....");
					time = Integer.parseInt(userInput.next());
					moveForward(200, time, iniHd);
					break;
				// Move Backward
				case 2:
					heading.update();
					iniHd = heading.getUpdatedValue();
					logToFile(false, "Moving backward at heading: " + iniHd + "Enter time....");
					time = Integer.parseInt(userInput.next());
					moveBackward(180, time);
					break;
				// Dive
				case 3:
					logToFile(false, "Enter depth....");
					tempDepth = Integer.parseInt(userInput.next());
					logToFile(false, "Enter time....");
					time = Integer.parseInt(userInput.next());
					dive(tempDepth, time);
					break;
				// Sway Left
				case 4:
					logToFile(false, "Enter speed....");
					speed = Integer.parseInt(userInput.next());
					logToFile(false, "Enter sway time....");
					time = Integer.parseInt(userInput.next());
					swayLeft(speed, time);
					break;
				// Sway Right
				case 5:
					logToFile(false, "Enter speed....");
					speed = Integer.parseInt(userInput.next());
					logToFile(false, "Enter sway time....");
					time = Integer.parseInt(userInput.next());
					swayRight(speed, time);
					break;
				// Rotate
				case 6:
					char axis; int angle;
					logToFile(false, "Enter rotation direction (Clockwise (C)/ Anti-Clockwise(A)....");
					axis = userInput.next().charAt(0);
					logToFile(false, "Enter the angle....");
					angle = Integer.parseInt(userInput.next());
					
					// Calculate target heading & depth
					depth.update();
					heading.update();
					int tgtHeading = 0;
					iniHd = heading.getUpdatedValue();
					
					if(axis == 'C'){
						tgtHeading = heading.getUpdatedValue() + angle + 6;
					}
					else if(axis == 'A'){						
						tgtHeading = heading.getUpdatedValue() - angle + 6;
					}					
					logToFile(true, "Current Heading: " + iniHd + ", Target Heading: " + tgtHeading);
					
					Rotate.By(angle,axis);
					setLeft(0);
					ySway(40, tgtHeading, depth.getUpdatedValue());
					
					break;
				// Combined
				case 7:
					iniHd = heading.getUpdatedValue();
					logToFile(false, "Enter time....");
					time = Integer.parseInt(userInput.next());
					/**combined(100, iniHd, time);**/
					break;
				// Measure current heading
				case 8:
					heading.update();
					logToFile(true,  "Current Heading: " + heading.getUpdatedValue());
					
					/*double hC = (double)((System.currentTimeMillis() - missionStartTime)*0.000116);
					int correctedHeading = (int) ( (double) heading.getUpdatedValue() - 
																				 (double)((System.currentTimeMillis() - missionStartTime)*0.000116) );					
					logToFile(true,  "Current Heading: " + heading.getUpdatedValue() + ", Corrected : " + correctedHeading + ", Correction : -" + hC );*/
					break;
				// Measure current depth
				case 9:
					depth.update();
					logToFile(true,  "Current Depth: " + depth.getUpdatedValue() );
					break;
				// IP Configuration
				case 10:
					IPThread.start();
					//IP Section.
					/*if( arenaParamsSet ){
						IPThread.start();
					}
					else {
						logToFile(true, "************************");
						logToFile(true, "Enter Arena Params First");
						logToFile(true, "************************");
					}*/
					break;
				// Arena Params
				case 11:				
					//Update Heading
					heading.update();
					iniHd = heading.getUpdatedValue();
					
					//Set arena parameters
					logData("\r\n");
					logData("Setting Arena Parameters\r\n");
					logData("Buoys Heading..."); logData(iniHd+"\r\n");
					buoysHeading = iniHd;
					//buoysHeading = Integer.parseInt(userInput.next());
					logData("Gate Heading....");
					gateHeading = iniHd; logData(iniHd+"\r\n");
					//gateHeading = Integer.parseInt(userInput.next());
					logData("\r\n");
					arenaParamsSet = true;
					break;
				// Marker Params
				case 12:
					logData("Setting Marker Heading\r\n");
					heading.update();
					markerHeading = heading.getUpdatedValue();
					logData("Marker Heading....");
					logData(markerHeading+"\r\n");
					break;
				// Vehicle Joystick Control
				case 13:
					remoteControlThread.start();
					break;
				// Read Voltages
				case 14:
					voltage12VBatt.update();
					float voltage =  (float)(voltage12VBatt.getUpdatedValue()*5*3)/1024 + 0.45f;
					logToFile(true, "\r\n"+"12V Battery: "+voltage); 
					voltage18VBatt.update();
					voltage = (float)(voltage18VBatt.getUpdatedValue()*5*4)/1024 + 0.31f;
					logToFile(true, "18V Battery: "+voltage);
					
					sysTemperature.update();
					float sysTemp = (float)(sysTemperature.getUpdatedValue()*5)/(float)(1024*0.01);
					logToFile(true, "Temperature: "+sysTemp+"\r\n");
					break;
				// Quit
				case 15:
					logToFile(false, "Enter Surge Speed....");
					speed = Integer.parseInt(userInput.next());
					logToFile(false, "Enter Time....");
					time = Integer.parseInt(userInput.next());
					logToFile(false, "Enter Direction (F/B)....");
					dir = userInput.next().charAt(0);
					simpleSurge(speed, time, dir);
					break;
				case 16:			
					logToFile(false, "Enter Sway Speed....");
					speed = Integer.parseInt(userInput.next());
					logToFile(false, "Enter Time....");
					time = Integer.parseInt(userInput.next());
					logToFile(false, "Enter Direction (R/L)....");
					dir = userInput.next().charAt(0);
					simpleSway(speed, time, dir);		
					break;
				case 17:					
					logToFile(false, "Enter Dive Speed....");
					speed = Integer.parseInt(userInput.next());
					logToFile(false, "Enter Time....");
					time = Integer.parseInt(userInput.next());
					logToFile(false, "Enter Direction (U/D)....");
					dir = userInput.next().charAt(0);
					simpleHeave(speed, time, dir);	
					break;
				case 18:					
					logToFile(false, "Enter Yaw Speed....");
					speed = Integer.parseInt(userInput.next());
					logToFile(false, "Enter Time....");
					time = Integer.parseInt(userInput.next());
					logToFile(false, "Enter Direction (C/A)....");
					dir = userInput.next().charAt(0);
					simpleYaw(speed, time, dir);	
					break;
				case 19:	
					System.exit(0);				
					break;
				default:
					break;
			}

			//logData("Trial Exited\r\n");
		}

	}
}
