import java.util.ArrayList;

public class Controller {

	private int controllerType = 1;	//0=> Simple, 1 => PID, 2=> LQC
	private int lBound = 0, uBound = 0, negativeLowerBound = 0, positiveLowerBound = 0, errorLimit = 0, totalCycleCount = 0, stableCycleCount = 0;
	private int numOutputs = 2;
	private float target = 0;
	private PID pidC = new PID();
	private LQC lqC = new LQC();
	private int Output1BaseValue = 0, Output2BaseValue = 0;
	private boolean baseValuesSet = false;
	public boolean hasStabilized = false;
	
	public MissionIO controllerInput = new MissionIO();
	public MissionIO controllerOutput1 = new MissionIO();
	public MissionIO controllerOutput2 = new MissionIO();
	public int Output1CorrectionMultiplier = 1;
	public int Output2CorrectionMultiplier = 1; 
	
	public int getType(){
		return controllerType;
	}
	
	public float getTarget(){
		return target;
	}
	
	private int checkBounds(int val){
		/*if( val > 0 && val < positiveLowerBound )	System.out.println("Controller::checkBounds() Force positive lower bound to " + positiveLowerBound + "from " + val);
		else if( val < 0 && val > negativeLowerBound ) System.out.println("Controller::checkBounds() Force negative lower bound to " + negativeLowerBound + " from " + val);
		
		val = val > 0 ? ( val > positiveLowerBound ? val : positiveLowerBound) :  (val < negativeLowerBound ? negativeLowerBound : val ) ;*/
		return val > uBound ? uBound : ( val < lBound ? lBound : val);
	}
	
	public void setErrorLimit(int lim){
		hasStabilized = false;
		errorLimit = lim;
	}
	
	public void setHeadingController(boolean value){
		pidC.isHeadingController = value;
	}
	
	public void setBaseValue(int b1){
		baseValuesSet = true;
		Output1BaseValue = b1;
	}
	
	public void setBaseValues(int b1, int b2){
		baseValuesSet = true;
		Output1BaseValue = b1;
		Output2BaseValue = b2;
	}
	
	public void setOutputCount(int c){
		numOutputs = c;
	}
	
	public void setType(int type){
		controllerType = type; 
	}	
	
	public void setTarget(float tgt){
		totalCycleCount = 0;
		switch(controllerType){
			case 0:
				target = tgt;
				break;
			case 1:
				target = tgt;
				pidC.setTarget(tgt);
				break;
			case 2:
				break;
		}
	}
	
	public void setBounds(int lB, int uB){
		lBound = lB;
		uBound = uB;
		
		/*switch(controllerType){
			case 1:
				pidC.setBounds(lBound, uBound);
				break;
			case 2:
				break;
		}*/
	}
	
	public void setSecondaryBounds(int nLB, int pLB){
		negativeLowerBound = nLB;
		positiveLowerBound = pLB;
	}

	public void setMultiplier(int m1){
		Output1CorrectionMultiplier = m1;
	}
	
	public void setMultipliers(int m1, int m2){
		Output1CorrectionMultiplier = m1;
		Output2CorrectionMultiplier = m2;
	}
	
	public int setCoeffecients(float coef1, float coef2){
		if( controllerType == 2){
			//lqC.setCoeffecients(K_P, K_I, K_D);
			return 0;
		}
		else{
			return -1;
		}
	}
	
	public ArrayList getCoeffecients(){
		if( controllerType == 1){
			return pidC.getCoeffecients();
		}
		else{
			return new ArrayList();
		}
	}
	
	public String getCoeffecientsAsString(){
		if( controllerType == 1){
			ArrayList<Float> al = pidC.getCoeffecients();
			String str = "[K_P: "+al.get(0)+", K_I: "+al.get(1)+", K_D: "+al.get(2)+"]";
			return str;
		}
		else{
			return null;
		}
	}
	
	public int setCoeffecients(float K_P, float K_I, float K_D){
		if( controllerType == 1){
			pidC.setCoeffecients(K_P, K_I, K_D);
			return 0;
		}
		else{
			return -1;
		}
	}	
	
	public void execute() throws InterruptedException
	{			
		/* Simple Controller With One or Two Outputs */
		if( controllerType == 0 ){
		 	if( numOutputs == 1 ){
				controllerOutput1.setValue( checkBounds( (int)(Output1BaseValue*Output1CorrectionMultiplier) ) );			
			}
			else if( numOutputs == 2 ){
				controllerOutput1.setValue( checkBounds( (int)(Output1BaseValue*Output1CorrectionMultiplier) ) );		
				controllerOutput2.setValue( checkBounds( (int)(Output2BaseValue*Output2CorrectionMultiplier) ) );		
			}
		}
		/* Feedback Controller With One or Two Inputs */
		else if(controllerType == 1)
		{
			/* Update The Controller Input */
			controllerInput.update();
			
			/* Fetch The Latest Input Value And Calculate The Correction */
			int correction = (int) pidC.getValue( controllerInput.getUpdatedValue() );
			
			/* Set The Controller Output Value*/
			if( numOutputs == 1){
				if( baseValuesSet ){
					controllerOutput1.setValue( checkBounds(Output1BaseValue + Output1CorrectionMultiplier*correction ) );
				}
				else {
					controllerOutput1.setValue( checkBounds(controllerOutput1.getUpdatedValue() + Output1CorrectionMultiplier*correction ) );
				}
			}
			else if( numOutputs == 2 ){
				if( baseValuesSet ){
					// System.out.println("1: " + Output1BaseValue + " + " + Output1CorrectionMultiplier + " * " + correction);
					// System.out.println("2: " + Output2BaseValue + " + " + Output2CorrectionMultiplier + " * " + correction);
					
					controllerOutput1.setValue( checkBounds( Output1BaseValue + Output1CorrectionMultiplier*correction ) );
					controllerOutput2.setValue( checkBounds( Output2BaseValue + Output2CorrectionMultiplier*correction ) );
				}
				else {
					controllerOutput1.setValue( checkBounds(controllerOutput1.getUpdatedValue() + Output1CorrectionMultiplier*correction ) );
					controllerOutput2.setValue( checkBounds( controllerOutput2.getUpdatedValue() + Output2CorrectionMultiplier*correction ) );
				}
			}
			
			/* Check If The Controller Has Stabilized*/
			if( Math.abs( pidC.getError() ) < errorLimit ){
				stableCycleCount++;		
				
				if( stableCycleCount > totalCycleCount/5 )
					hasStabilized = true;
				else
					hasStabilized = false;		
			}			
		}
		else {
		}
		/* Increment The Cycle Count */
		totalCycleCount++;
	}
	
}
