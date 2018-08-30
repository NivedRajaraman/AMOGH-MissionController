import java.io.*;

public class GateTask extends MissionController {
	private String color;
	private int startDepth, targetDepth;
	private int startSurgeSpeed, targetSurgeSpeed, targetSwaySpeed;
	private int startHeading;
	private int depthIncrement = 2, depthIncrementAtCycleCount = 7;
	private int minDepthLimit, maxDepthLimit;
	private int minSwaySpeedLimit, maxSwaySpeedLimit;
	private boolean buoyDetectedOnce = false, enabledDepthLmits = false, enabledSwaySpeedLimits = false;
	public boolean taskComplete = false, hardHit = false;
	public float buoyConfidence = 0.0f;
	private long missionStartTime =0;
	
	private int cycleCount = 0, countDetectedFrames = 0;
	
	public void initialize(String _color, int _sDepth, int _sSurgeSpeed, int dI, int dIACC) throws InterruptedException{
		/* Buoy Color */
		color = _color;
		/* Depth to Start With */
		startDepth = _sDepth;
		/* Start Surge Speed */
		startSurgeSpeed = _sSurgeSpeed;
		/* Depth Increment Value */
		depthIncrement = dI;
		/* Depth Increment Frequency */
		depthIncrementAtCycleCount = dIACC;
		
		/* Set Surge Speed */
		setForward( startSurgeSpeed );
		/* Set Initial Depth */
		targetDepth = startDepth;
	}
	
	public void setDepthLimits(int min, int max){
		minDepthLimit = min;
		maxDepthLimit = max;
		enabledDepthLmits = true;
	}
	
	public void setSwaySpeedLimits(int min, int max){
		minSwaySpeedLimit = min;
		maxSwaySpeedLimit = max;
		enabledSwaySpeedLimits = true;
	}
	
	private void correctScope(int targetHeading) throws InterruptedException{
		/* Stop vehicle */
		StopMotion.Do();
		/* Pull backwards */
		setBackward(80);
		/* Set zero sway */
		setLeft(0);
		
		while( radius == 0 && theta == 0 ){
			Surge.At(targetHeading);
			Sway.At(targetHeading);
			Heave.At(targetDepth);
			
			MissionController.logToFile(true, "Correcting Scope: Target heading " + targetHeading + 
			                   ", Current Heading: " + heading.getUpdatedValue() +
			                   ", Current Depth: " + targetDepth);
			
			Thread.sleep(20);
			
			//remove later
			targetHeading = heading.getUpdatedValue();
		}
				
	}
	
	private void hardHitBuoy() throws InterruptedException, FileNotFoundException{
		heading.update();
		//MissionController.logToFile(true, "Cross gate at heading : " + heading.getUpdatedValue() );
		int hhGateHeading = heading.getUpdatedValue();
		hhGateHeading = startHeading + 2	;
		MissionController.logToFile(true, "Cross gate at heading : " + hhGateHeading );
		
		MissionController.logToFile(true, "");
		
		//If left gate
		MissionController.logToFile(false, "\rHardhit : Right Sway, Depth : " + targetDepth);
		swayRight(120, 3, targetDepth);
		MissionController.logToFile(true, "");
		
		setRight(0);		
		setForward(110);		
		
		long startTime = System.currentTimeMillis();
		long currentTime = startTime; 
		
		while( currentTime <= startTime + 11 * 1000 ){
			Sway.At(hhGateHeading);
			Surge.At(hhGateHeading);			
			Heave.At(targetDepth);
			currentTime = System.currentTimeMillis();
			MissionController.logToFile(false, "\rHardhit : Moving Forward, Depth : " + targetDepth );
			Thread.sleep(20);
		}
		
		MissionController.logToFile(true, "");
		
		StopMotion.Do();
		
		hardHit = true;
		
	}
	
	private void calculate(int targetHeading) throws InterruptedException{
		/* Increment in depth value */
		int deltaDepth = 0;
		
		/* Multlipication/Reduction factor for Sway Thrust */
		float reverseSwayThrustMultiplier = 1.40f; //0.80
		float forwardThrustMultiplier = -1.00f;    //0.65

		/* Increment counter */
		cycleCount++;
		
		/* Once in every N times update the depth */
		if(cycleCount > depthIncrementAtCycleCount){  
			deltaDepth = depthIncrement;   
			cycleCount = 0;
		}
		
		/* Calulate Depth and Sway Values */
		// Buoy in the horizontal plane to the Right
		if(theta == 0){
			targetSwaySpeed = (int) ( reverseSwayThrustMultiplier * radius * 255 / 320);
		}
		// First Quadrant, Buoy to the Right
		else if(theta > 0 && theta < 90){
			targetDepth -= deltaDepth;
			targetSwaySpeed = (int) ( reverseSwayThrustMultiplier * radius * Math.cos( (theta * 3.1416) / 180 ) * 255 / 320);
		}
		// Second Quadrant, Buoy to the Left
		else if(theta >= 90 && theta < 180){
			targetDepth -= deltaDepth;
			targetSwaySpeed = (int) ( forwardThrustMultiplier * radius * Math.cos( ((180 - theta) * 3.1416) / 180 ) * 255 / 320);
		}
		// Buoy in the horizontal plane to the Left
		else if(theta == 180){
			targetSwaySpeed = (int) ( forwardThrustMultiplier * radius * 255 / 320);
		}
		// Third Quadrant, Buoy to the Left
		else if(theta > 180 && theta < 270){
			targetDepth += deltaDepth;
			targetSwaySpeed = (int) ( forwardThrustMultiplier * radius * Math.cos(((theta - 180) * 3.1416) / 180) * 255 / 320);
		}
		// Fourth Quadrant, Buoy to the Right
		else if(theta >= 270 && theta < 360){
			targetDepth += deltaDepth;
			targetSwaySpeed = (int) ( reverseSwayThrustMultiplier * radius * Math.cos( ((360 - theta) * 3.1416) / 180 ) * 255 / 320);
		}
		
		//Sway Speed Limits
		if( enabledSwaySpeedLimits ){
			targetSwaySpeed = (targetSwaySpeed <= minSwaySpeedLimit ? minSwaySpeedLimit : targetSwaySpeed);
			targetSwaySpeed = (targetSwaySpeed >= maxSwaySpeedLimit ? maxSwaySpeedLimit : targetSwaySpeed);
		}
		
		// Depth Limits
		if( enabledDepthLmits ){
			targetDepth = (targetDepth <= minDepthLimit ? minDepthLimit : targetDepth); 
			targetDepth = (targetDepth >= maxDepthLimit ? maxDepthLimit : targetDepth); 
		}
	}
	
	private void detect(int targetHeading) throws InterruptedException{
		
		/* Calculate Values */
		calculate(targetHeading);
		
		/* Surge Speed Control */
		targetSurgeSpeed = startSurgeSpeed;
		
		/* Deploy Values */
		setForward(targetSurgeSpeed);
		swayController.setBaseValues(targetSwaySpeed, targetSwaySpeed);
										
		/* Execute Tasks */
		Sway.At(targetHeading);
		Heave.At(targetDepth);
		Surge.At(targetHeading);
		//simpleSurge(targetSurgeSpeed, 0, 'F');

		/* Log data */
		MissionController.logToFile(true, "Gate => Radius: "+radius+" Theta: "+theta);
		MissionController.logToFile(true, "TargetHeading: "+targetHeading+", TargetDepth: "+targetDepth+", TargetSwaySpeed: " + targetSwaySpeed);
		MissionController.logToFile(true, "");
	}
	
	public void hit(int _sHeading) throws InterruptedException, FileNotFoundException {
		/* Start Heading */
		startHeading = _sHeading;
		
		missionStartTime = System.currentTimeMillis();
		
		while( !taskComplete ){
			/*long dT = System.currentTimeMillis() - missionStartTime;
			if( dT > 15*1000){
			   System.out.println("Stop Gate IP Processing");
				 StopMotion.Do();
				 taskComplete = true;
				 break;
			}*/
			
			MissionController.logToFile(true, "Gate cross : Task Has Been Started ");
					
			// Normal Detection
			if( radius != 999 && theta != 999 && hardHit == false ){
				// Something detected
				if( radius != 0 || theta != 0){
					countDetectedFrames++;
				}
				
				// Check if buoy has been detected confidently
				if( radius == 777 && theta == 777 ){
					MissionController.logToFile(true, "Setting buoy confidence to 1.0");
					buoyConfidence = 1.0f;
				}
			
				// Buoys out of scope
				//if( radius == 0 && theta == 0 && countDetectedFrames > 15 ){			
				if( radius == 0 && theta == 0 && buoyConfidence == 1.0 ){			
						MissionController.logToFile(true, "Buoy out of scope");
						correctScope(startHeading);
				}
				
				//Stabilization of the vehicle
				else if( radius == 888 && theta == 888){
					MissionController.logToFile(false, "Buoy under scope, Stabilization started, Braking, ");
					
					/* Brake to Nullify Forward Inertia */
					brakeForward(startHeading);
					
					/* Set Parameters */
				  int fixedDepth = targetDepth;
					boolean swayStabilized = false;
					MissionController.logToFile(true, ", Depth fixed at : " + fixedDepth);
					
					/* Stop Surge Motion */
					setForward(0);
		
					/* Start Stabilization */
					while(!hardHit){
					
						/* Check if Buoy goes out of scope and correct it */
						if(radius == 0 && theta == 0){
							correctScope(startHeading);
							
							/* Need to stabilize again */
							swayStabilized = false;
						}
						
						/* Calculate values and set motion */
						setForward(0);													// Zero Surge
						calculate(startHeading);								// Calculate Sway Speed and Target Depth
						setSway((int)(0.75*targetSwaySpeed)); 	// Stabilization needs further lower speeds
						
						/* Print Values */						
						MissionController.logToFile(true, "Buoy => Radius: "+radius+" Theta: "+theta);
						MissionController.logToFile(true, "TargetHeading: "+startHeading+", FixedDepth: "+fixedDepth+", TargetSwaySpeed: " + targetSwaySpeed);
						MissionController.logToFile(true, "");						
						
						/* Deploy Values */
						Surge.At(startHeading);
						Sway.At(startHeading);
						Heave.At(fixedDepth);
						
						/* Check for stabilization and HardHit */
						if( Math.abs(targetSwaySpeed) < 20 ) {		//
							/* Set Flag */
							swayStabilized = true;
							
							/* Stop Sway */
							setLeft(0);
							targetSwaySpeed = 0;
							
							/* Go for HardHit */
							targetDepth = fixedDepth;
							hardHitBuoy();
						}
												
						Thread.sleep(20);
					}
					
					MissionController.logToFile(true, "Stabilization & HardHit Completed!");
					taskComplete = true;
					
					StopMotion.Do();
				}
				/* Fetch buoy coordinates */
				else{
					detect(startHeading);
				}
			}
			
			/* Normal HardHit */
			else if( radius == 999 && theta == 999 && hardHit == false ){
				hardHitBuoy();
				/*
				 *
				 Transition Tasks
				*
				*/
				taskComplete = true;
			}
		}
		
		MissionController.logToFile(true, "Hit "+color+" Buoy : Task Has Been Completed");
	}
}
