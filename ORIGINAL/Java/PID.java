import java.util.ArrayList;

public class PID {
	
	private float K_P, K_I, K_D;
	private float prevValue = 0, targetValue = 0, error = 0, previousError = 0, oldError = 0, correction = 0, retVal = 0;
	private float prevdError = 0, dError = 0, ddError = 0;
	public float currValue = 0;
	private float upperBound = 0, lowerBound = 0;
	private float tempError = 0, pTerm = 0, iTerm = 0, dTerm = 0;
	public float diff = 0, intg = 0;
	public boolean isHeadingController = false;
	
	PID(){}
	
	private float calculateError(float currentValue){
		previousError = error;
		error = currentValue - targetValue;
		currValue = currentValue;
		
		if(isHeadingController){
			diff = (float)(currentValue - prevValue) / 0.02f;
			intg += (float) diff*0.02f;
			
			if(error >= 180) error = error - 360;
			else if(error < -180 ) error = error + 360;
		}

		prevValue = currentValue;
		return error;
	}
	
	public void setCoeffecients(float KP, float KI, float KD){
		K_P = KP;
		K_I = KI;
		K_D = KD;
	}
	
	public ArrayList getCoeffecients(){
		ArrayList<Float> coeffs = new ArrayList();
		coeffs.add(K_P);
		coeffs.add(K_I);
		coeffs.add(K_D);
		
		return coeffs;
	}
	
	public void setBounds(float lBound, float uBound){
		lowerBound = lBound;
		upperBound = uBound;
	}
	
	public void setTarget(float tValue){
		targetValue = tValue;
	}
	
	public float getValue(float cValue){					//currentValue
		tempError = calculateError(cValue);
		diff = tempError - previousError; 	
		intg += tempError;
		
		correction = (K3*intg - (cValue*K1 + diff*K2))*(255/18);
		
		return correction;
		
		/*tempError = calculateError(cValue);
		pTerm = K_P*tempError;		
		iTerm += K_I*tempError;
		if(iTerm > upperBound) iTerm = upperBound;
		if(iTerm < lowerBound) iTerm = lowerBound;		
		dTerm = K_D*(tempError - previousError);
		correction = pTerm + iTerm - dTerm;
		
		//retVal = correction > upperBound ? upperBound : ( correction < lowerBound ? lowerBound : correction );
		return correction;*/
		
		/*tempError = calculateError(cValue);
		dError = tempError - previousError;
		ddError = dError - prevdError;
		
		prevdError = dError;
		pTerm = K_P*dError;
		iTerm += K_I*tempError;
		if(iTerm > upperBound) iTerm = upperBound;
		if(iTerm < lowerBound) iTerm = lowerBound;
		dTerm = K_D*ddError;
		
		correction += pTerm + iTerm - dTerm;
		return correction;*/
	}
	
	public void clear(){
		error = 0;
		previousError = 0;
	}
	
	public float getError(){
		return tempError;
	}
	
}
