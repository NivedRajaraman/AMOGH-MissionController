import java.io.*;

public class LaneTask extends MissionController {
	/* Heading Control Variables */
	private int startHeading, targetHeading, finishHeading;
	
	/* Parmeters revceived from IP Server */
	private double prevRadius, xIntercept, yIntercept, headingCorrection = 0;
	
	/* Depth Control Variables */
	private int startDepth, targetDepth;
	private int minDepthLimit, maxDepthLimit;
	private boolean enabledDepthLmits = false;
	
	/* Sway Control Variables */
	private int targetSwaySpeed;
	private int minSwaySpeedLimit, maxSwaySpeedLimit; 
	private boolean enabledSwaySpeedLimits = false;
	
	/* Surge Control Variables */
	private int targetSurgeSpeed, baseSurgeSpeed = 0;
	private int minSurgeSpeedLimit, maxSurgeSpeedLimit;
	private boolean enabledSurgeSpeedLimits = false;
	
	/* Camera Parameters */
	int camWidth = 640, camHeight = 480;
	
	/* Task Status Flags */
	private boolean pathFound = false;
	private boolean inNearScope = false, alignTimeout = false, aligned = false, follow = false;	// 3 Step detection :: Detect while !inNearScope => Align => Follow
	private boolean taskComplete = false;
	private int inNearScopeConfidence = 0, alignIterations = 0;
	private char rotationDir;
	
	/* Initializaion Routine */
	public void initialize(int hDepth){		
		/* Set hover depth */
		startDepth = hDepth;
	}

	/* Set sway speed limits */
	public void setSwaySpeedLimits(int min, int max){
		minSwaySpeedLimit = min;
		maxSwaySpeedLimit = max;
		enabledSwaySpeedLimits = true;
	}
	
	/* Set surge speed limits */
	public void setSurgeSpeedLimits(int min, int max){
		minSurgeSpeedLimit = min;
		maxSurgeSpeedLimit = max;
		enabledSurgeSpeedLimits = true;
	}
	
	/* Set depth limits */
	public void setDepthLimits(int min, int max){
		minDepthLimit = min;
		maxDepthLimit = max;
		enabledDepthLmits = true;
	}
	
	void calculateTargetHeading() throws InterruptedException{
		/* Calculate Heading Correction */
		if(slope >= 0 && slope < 90){
			headingCorrection = 90 - slope;
		}
		else if(slope >= 90 && slope < 180){
			headingCorrection = 90 - slope;
		}
		
		/* Update heading and set target heading */
		heading.update();
		targetHeading = heading.getUpdatedValue() + (int) headingCorrection;
	}
	
	int calculate3(){
		int swayOrder = 2, surgeOrder = 2;
		
		/* Check quadrant */
		int quadrant = (int)(Math.abs(theta)/90) + 1; 
	
		/* Multlipication/Reduction factor for Sway Thrust */
		float reverseSwayThrustMultiplier = 0.80f; 
		float forwardThrustMultiplier = -0.65f;    

		/* Quadrant 1 : Path to the top-right, move top-right */
		if( quadrant == 1 ){
			targetSwaySpeed = (int) ( reverseSwayThrustMultiplier * Math.pow(radius * Math.cos( (theta * 3.1416) / 180 ), swayOrder) * 255 / Math.pow(camWidth/2, swayOrder) );
			targetSurgeSpeed = (int) ( Math.pow(radius * Math.sin( (theta * 3.1416 ) / 180 ), surgeOrder) * 255 / Math.pow( camHeight/2 , surgeOrder) );
		}
		/* Quadrant 2 : Path to the top-left, move top-left */
		else if( quadrant == 2 ){
			targetSwaySpeed = (int) ( forwardThrustMultiplier * Math.pow(radius * Math.cos( ((180 - theta) * 3.1416) / 180 ), swayOrder) * 255 / Math.pow(camWidth/2, swayOrder) );
			targetSurgeSpeed = (int) ( Math.pow(radius * Math.sin( ((180 - theta) * 3.1416 ) / 180 ), surgeOrder) * 255 / Math.pow( camHeight/2 , surgeOrder) );
		}
		/* Quadrant 3 : Path to the bottom-left, move bottom-left */
		else if( quadrant == 3 ){
			targetSwaySpeed = (int) ( forwardThrustMultiplier * Math.pow(radius * Math.cos(((theta - 180) * 3.1416) / 180), swayOrder) * 255 / Math.pow(camWidth/2, swayOrder) );
			targetSurgeSpeed = (int) ( -Math.pow(radius * Math.sin( ((theta - 90) * 3.1416 ) / 180 ), surgeOrder) * 255 / Math.pow( camHeight/2 , surgeOrder) );
		}
		/* Quadrant 4 : Path to the bottom-right, move bottom-right */
		else if( quadrant == 4 ){
			targetSwaySpeed = (int) ( reverseSwayThrustMultiplier * Math.pow(radius * Math.cos( ((360 - theta) * 3.1416) / 180 ), swayOrder) * 255 / Math.pow(camWidth/2, swayOrder) );
			targetSurgeSpeed = (int) ( -Math.pow(radius * Math.sin( ((360 - theta) * 3.1416 ) / 180 ), surgeOrder) * 255 / Math.pow( camHeight/2 , surgeOrder) );
		}			
	
		/* Surge speed limits */
		targetSurgeSpeed = (targetSurgeSpeed <= minSurgeSpeedLimit ? minSurgeSpeedLimit : targetSurgeSpeed);
		targetSurgeSpeed = (targetSurgeSpeed >= maxSurgeSpeedLimit ? maxSurgeSpeedLimit : targetSurgeSpeed);
		
		/* Sway speed limits */
		targetSwaySpeed = (targetSwaySpeed <= minSwaySpeedLimit ? minSwaySpeedLimit : targetSwaySpeed);
		targetSwaySpeed = (targetSwaySpeed >= maxSwaySpeedLimit ? maxSwaySpeedLimit : targetSwaySpeed);
		
		return quadrant;
	}
	
	public void align(int sHeading, int bSSpeed) throws InterruptedException, FileNotFoundException {
		/* Set start heading */
		startHeading = sHeading; 
		
		/* Set reference value */
		targetHeading = startHeading;
		
		/* Set base surge speed */
		baseSurgeSpeed = bSSpeed;
		
		/* Set initial depth */
		targetDepth = startDepth;
		
		inNearScopeConfidence = 0;
		/* Brake to Nullify Forward Inertia */
		// brakeForward(startHeading);
			
	  MissionController.logToFile(true, "Task : Path Align Begin");
			
		while( !taskComplete ){
			
			/* Stage 0 : Keep moving straight untill path is found */
			if( !pathFound ){
				
				/* Set forward motion and zero sway */
				setForward(baseSurgeSpeed);
				setLeft(0);
							
				do {		
					if( radius != 0 || theta != 0 ){
						MissionController.logToFile(true, "Found Path");
						pathFound = true;
						break;
					}
									
					Surge.At(targetHeading);
					Sway.At(targetHeading);
					Heave.At(targetDepth);				
					
					MissionController.logToFile(false, "\rFind Path [Moving Forward] => targetHeading : " + targetHeading + ", " + ", targetDepth : " + targetDepth + ", forward speed : " + baseSurgeSpeed + ", Radius : " + radius + ", Theta : " + theta + ", Slope : " + slope );
					Thread.sleep(20);
					
				} while( radius == 0 && theta == 0 );
				
			}
			
			/* Stage I : Detect and bring the path much into the field of view */			
			/* Initial detection based on radius and theta */
				
			if( pathFound ){
			
				while( !inNearScope ){
					MissionController.logToFile(true, "");
				
					int quadrant = 1;
					/* Calculate target sway speed & target surge speed */
					if( radius != 0 && slope != 90 )
						quadrant = calculate3();
					
					/* Setup controllers */
					setSurgeParameters(targetSurgeSpeed, -254, 254);
					setSway(targetSwaySpeed);
		
					/* Deploy values */
					Surge.At(targetHeading);
					Sway.At(targetHeading);
					Heave.At(targetDepth);
					
					/* Check confidence */
					if( radius != 0 && prevRadius != 0 ){
						if( radius != prevRadius ){
							if( radius < 150 ){
								inNearScopeConfidence++;
							}
						}
					}
					else
						inNearScopeConfidence = 0;				
					
					/* Check if buoy is in scope */
					if( inNearScopeConfidence > 5 ){
						MissionController.logToFile(true, "Path in Scope");
						inNearScope = true;
						break;
					}
					
					/* Update params */
					prevRadius = radius;
					
					MissionController.logToFile(true, "Bring path in scope");
					MissionController.logToFile(true, "Quadrant : " + quadrant + ", targetSurgeSpeed : " + targetSurgeSpeed + ", targetSwaySpeed : " + targetSwaySpeed);
					MissionController.logToFile(true, "Confidence : " + inNearScopeConfidence + ", targetHeading : " + targetHeading + ", targetDepth : " + targetDepth);
					MissionController.logToFile(true, "Radius : " + radius + ", Theta : " + theta + ", Slope : " + slope );
					Thread.sleep(20);
				}
				
			}
			
			/* Align to path */
			if( inNearScope ){
				while( !aligned ){
					MissionController.logToFile(true, "");
									
					/* Stop forward motion during align */
					setForward(0);
										
					/***
						* Algorithm 1 
					***/
					
					//MissionController.logToFile(true, "Align to path : Algorithm #1");
					
					/* Calculate target heading */
					//int prevHeading = targetHeading;
					//calculateTargetHeading();
					
					/* Calculate target sway speed & target surge speed */
					//calculate3();
					
					/* Setup controller */
					//setSway(targetSwaySpeed);
					
					/* Deploy Values */
					/*Sway.At(targetHeading);
					Heave.At(targetDepth);
					
					MissionController.logToFile(true, "Change heading from " + prevHeading + " to " + targetHeading + ": headingCorrection : " + headingCorrection);
					MissionController.logToFile(true, "targetHeding : " + targetHeading + ", targetDepth : " + targetDepth + ", targetSwaySpeed" + targetSwaySpeed);
					
					// Works untill the controller has stabilized
					if( headingCorrection >= 0 ){
						Rotate.By((float) headingCorrection, 'C');
					}
					else if( headingCorrection < 0 ){
						Rotate.By((float) Math.abs(headingCorrection), 'A');
					}
					StopMotion.Do();*/
										
					
					/***
						* Algorithm 2 
					***/				
					// MissionController.logToFile(true, "Align to path : Algorithm #2");
					
					/* Calculate target heading */
					// int prevHeading = targetHeading;
					// calculateTargetHeading();
					// MissionController.logToFile(true, "Change heading from " + prevHeading + " to " + targetHeading + ": headingCorrection : " + headingCorrection);
					
					/* Calculate target sway speed */
					// calculate3();
						
					/* Setup controllers */
					// setSurgeParameters(0, -254, 254);
					// setSway(targetSwaySpeed);
					
					/* Deploy Values : Rotation using surge controller and sway using sway controller */
					//heading.update();
					//int currentHeading = heading.getUpdatedValue();
					//int dH = targetHeading - currentHeading;
					/*while( Math.abs(dH) > 5 ){
						Sway.At(targetHeading);
						Surge.At(targetHeading);
						Heave.At(targetDepth);
						
						currentHeading = heading.getUpdatedValue();
						dH = targetHeading - currentHeading;
						
						MissionController.logToFile(true, "Align Path [Rotation + Sway]");
						MissionController.logToFile(true, "\rtargetHeding : " + targetHeading + ", currentHeading : " + currentHeading + ", deltaHeading : " + dH);
						MissionController.logToFile(false, "\rtargetDepth : " + targetDepth + ", targetSwaySpeed : " + targetSwaySpeed);
						Thread.sleep(20);
					}
					MissionController.logToFile(true, "");*/
					
					
					/***
						* Algorithm 3 : Rotation and Sway loops different 
					***/			
					MissionController.logToFile(true, "Align to path : Algorithm #3");
					
					/* Calculate target heading */
					int prevHeading = targetHeading;
					calculateTargetHeading();
					
					/* Setup controllers */
					setSurgeParameters(0, -254, 254);
					setSway(0);
					
					heading.update();
					int currentHeading = heading.getUpdatedValue();
					int dH = targetHeading - currentHeading;
					/* Rotation Loop */
					
					//StopMotion.Do();
					// if( dH > 0 ) setSwayParameters(0, -254, 0)
					// else setSwayParameters(0, 0, 254)
					
					if( dH > 0 ) rotationDir = 'C';
					else if( dH <= 0 ) rotationDir = 'A';
					
					// setSurgeMultipliers(2,2);
										
					while( Math.abs(dH) > 5 ){
						Sway.At(targetHeading);
						Surge.At(targetHeading);
						Heave.At(targetDepth);
						
						currentHeading = heading.getUpdatedValue();
						dH = targetHeading - currentHeading;
						
						MissionController.logToFile(true, "\r\nAlign Path [Rotation]");
						MissionController.logToFile(true, "targetHeding : " + targetHeading + ", currentHeading : " + currentHeading + ", deltaHeading : " + dH);
						MissionController.logToFile(true, "targetDepth : " + targetDepth + ", targetSwaySpeed : 0 ");
						MissionController.logToFile(true, "Radius : " + radius + ", Theta : " + theta + ", Slope : " + slope );
						Thread.sleep(20);
					}
					
					// setSurgeMultipliers(1,1);
					
					heading.update();
					targetHeading = heading.getUpdatedValue();
					
					/* Check on iterations */
					if( alignIterations == 0 ){
						MissionController.logToFile(true, "Aligned to path");
						aligned = true;	
						
						/* Sway Loop */
						// setSurgeParameters(0, -254, 254);
						// setSway(targetSwaySpeed);
					
						/* Calculate target heading */
						// calculateTargetHeading();
					
						/*while( Math.abs(targetSwaySpeed) > 20 ){
							Sway.At(targetHeading);
							Surge.At(targetHeading);
							Heave.At(targetDepth);
						
							if( radius != 0 && slope != 90 )
								calculate3();
								
							setSway(targetSwaySpeed);
						
							MissionController.logToFile(true, "\r\nAlign Path [Sway (Maintain Heading)]");
							MissionController.logToFile(true, "targetHeding : " + targetHeading + ", currentHeading : " + currentHeading + ", deltaHeading : " + dH);
							MissionController.logToFile(true, "targetDepth : " + targetDepth + ", targetSwaySpeed : " + targetSwaySpeed);
							MissionController.logToFile(true, "Radius : " + radius + ", Theta : " + theta + ", Slope : " + slope );
						
							Thread.sleep(20);
						}*/
								
						/*targetSwaySpeed = 70;
						while( Math.abs(targetSwaySpeed) > 20 ){
						
							if( rotationDir == 'C' ){
								setLeft(targetSwaySpeed);								
							}
							else if( rotationDir == 'A' ){
								setRight((int)(1.2*targetSwaySpeed));	
							}
							
							if( radius != 0 && slope != 90 )
								calculate3();
							/*
								If path is completely out of scope or such that loop condition 
								is never met then it may keep swaying/doing whatever it was
							*
							
							targetSwaySpeed = Math.abs(targetSwaySpeed);
							currentHeading = heading.getUpdatedValue();
							
							Sway.At(targetHeading);
							Surge.At(targetHeading);
							Heave.At(targetDepth);
							
							MissionController.logToFile(true, "\r\nAlign Path [Sway (Maintain Heading)]");
							MissionController.logToFile(true, "targetHeding : " + targetHeading + ", currentHeading : " + currentHeading );
							MissionController.logToFile(true, "targetDepth : " + targetDepth + ", targetSwaySpeed : " + targetSwaySpeed);
							MissionController.logToFile(true, "Radius : " + radius + ", Theta : " + theta + ", Slope : " + slope );
							
						}*/
						 
						break;
					}		
						
					alignIterations++;
					MissionController.logToFile(true, "Align Iteration Count : " + alignIterations);
					Thread.sleep(20);
				}				
			}
				
			/* Follow path and go straight */
			if( aligned ){
				setSway(0);
				Sway.At(targetHeading);
				
				MissionController.logToFile(true, "Follow path : Go staright at current heading");
				moveForward(110, 10, targetHeading - 4, 200);
				StopMotion.Do();
				taskComplete = true;
			}
			
			Thread.sleep(20);
		}
		MissionController.logToFile(true, "Task : Path Align Complete");
	}
}
