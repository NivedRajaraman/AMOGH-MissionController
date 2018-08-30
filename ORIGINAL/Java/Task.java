
public class Task {

	private Controller TaskController = new Controller();

	public void initialize(MissionIO TIn, MissionIO TOut, Controller TC){
		TC.controllerInput = TIn;
		TC.controllerOutput1 = TOut;
		TaskController = TC;
	}
	
	public void initialize(MissionIO TIn, MissionIO TOut1, MissionIO TOut2, Controller TC){
		TC.controllerInput = TIn;
		TC.controllerOutput1 = TOut1;
		TC.controllerOutput2 = TOut2;
		TaskController = TC;
	}
	
	public void At(float target) throws InterruptedException{
		TaskController.setTarget(target);
		TaskController.execute();
	}	
	
	/* Assuming That Heading Increases In Clockwise Direction */
	public void By(float target, char mode) throws InterruptedException{	//mode => Clockwise/Anti-Clockwise
		
		/* Make A Copy Of The Present State Of The Controller */
		Controller tempController = new Controller();
		tempController = TaskController;
		
		/* Update The Controller Input */
		TaskController.controllerInput.update();
		//System.out.println("Updated Input Value: " + TaskController.controllerInput.getUpdatedValue() );
		
		/* Find The Target */
		target = (mode == 'C') ? TaskController.controllerInput.getUpdatedValue() + target : TaskController.controllerInput.getUpdatedValue() - target;
		target = (target > 360) ? target - 360 : ( target < 0 ? target + 360 : target ) ;
		System.out.println("Target: " + target);
		
		switch(mode){
			/* Clockwise Rotation */
			case 'C':									
				//TaskController.setMultipliers(-2, 2);	// (Left, Right)
				TaskController.setBaseValues(0, 0);
				TaskController.setBounds(-254, 254);
				TaskController.setTarget(target);
				TaskController.setErrorLimit(20);		// Error Limit +-3 Degrees
				while( !TaskController.hasStabilized ){
					//System.out.println( TaskController.controllerOutput1.getValue() + ", " + TaskController.controllerOutput2.getValue() + ", " + TaskController.controllerInput.getValue() );
					TaskController.execute();
				}
				TaskController.setMultipliers(-1, 1);	// (Left, Right)
				break;
			/* Anti-Clockwise Rotation */
			case 'A':									
				//TaskController.setMultipliers(-2, 2);	// (Left, Right)
				TaskController.setBaseValues(0, 0);
				TaskController.setBounds(-254, 254);
				TaskController.setTarget(target);
				TaskController.setErrorLimit(20);		// Error Limit +-3 Degrees
				while( !TaskController.hasStabilized ){
					//System.out.println( TaskController.controllerOutput1.getValue() + ", " + TaskController.controllerOutput2.getValue() + ", " + TaskController.controllerInput.getValue() );
					TaskController.execute();
				}
				TaskController.setMultipliers(-1, 1);	// (Left, Right)
				//break;
		}
		
		/* Restore The Contoller To Its Original Stage */
		TaskController = tempController;
	}
	
	public void Do() throws InterruptedException{
			TaskController.execute();	
	}
}
