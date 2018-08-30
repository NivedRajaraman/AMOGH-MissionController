public class MissionIO {
	
	private int value;
	public float multiplier = 1;
	public char identifier; //E.g.: R => Right +ve Speed, r => Right -ve Speed
	public boolean typeInput = false, typeOutput = false;
		
	public SSC serialComm = new SSC();
	
	MissionIO(){
	}
	
	MissionIO(char iCh, SSC ser, boolean typeIn){
		identifier = iCh;
		serialComm = ser;
		typeInput = typeIn;
		typeOutput = !typeIn;
	}
	
	public int getValue(){
		return value;
	}
	
	public int getUpdatedValue() {
		if(typeInput || !typeOutput){
			value = (int)((float) multiplier * (float) (serialComm.getValue(identifier)));
		}
		return value;
	}
	
	public void setValue(int val){	
		//value = val;
		value = Math.abs(val);
		if( typeOutput || !typeInput ){
			if(val >= 0){
				serialComm.writeData( (byte)identifier, (byte)((char)value) );
			}
			else if(val < 0){				
				serialComm.writeData( (byte)(identifier+32), (byte)((char)value) );
			}
		}
	}
	
	public void update() throws InterruptedException{
		if( typeInput || !typeOutput){
			if( !serialComm.isIMU ){
				serialComm.writeData((byte)identifier,(byte)identifier);
			}
			Thread.sleep(10);
		}
	}
	
	public void setSerialCom(SSC sCom){
		serialComm = sCom;
	}
	
}
