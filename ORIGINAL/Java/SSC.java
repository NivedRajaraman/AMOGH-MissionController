import javax.print.DocFlavor.INPUT_STREAM;

import jssc.SerialPort;
import jssc.SerialPortEvent;
import jssc.SerialPortEventListener;
import jssc.SerialPortException;
import java.util.List;
import java.util.ArrayList;

public class SSC {

		/* Temporary Data buffer */
	  public List<Byte> ls = new ArrayList<Byte>();
	  
	  /* Serialport Object */
    private SerialPort serialPort;
    
    /* Port parameters */
		private String portName;
		private int portBaudRate;
		private long bytesSent = 0;
		
		/* Flags */
		public boolean portReopened = false;
    public volatile boolean isIMU = false;
		public boolean fillPacket = false;
		public boolean clearBuffer = false;
		
		/* IMU object and values */
    public IMU imuObject = new IMU();  
    
    /* Get count of bytes sent */
		public long getSentBytesCount(){
			return bytesSent;
		}
		
		/* Write string to serial port 
		 * @params: [str] string to be sent */
    public void writeData(String str){
    	try {
			serialPort.writeBytes( str.getBytes() );
			bytesSent++;
			} catch (SerialPortException e) {
				e.printStackTrace();
			}
    }
    
    /* Write data to serial port 
     * @params: [b1] byte 1 - identifier, [b2] byte 2 - value :  as per the two byte protocol */
    public void writeData(byte b1, byte b2){    
    
    	/* Serial port error temporary bugfix */
  		if( getSentBytesCount() > 32000 && !portReopened ){
  			MissionController.logToFile(true, "WD2: Byte count > 32000, Closing and Reopening Serial Port");
  			try{ serialPort.closePort(); } catch( Exception e) { e.printStackTrace(); }
  			initialize(portName, portBaudRate, isIMU);
  			portReopened = true;
  		}
       
      /* Send data to Arduino */             		
    	if( !isIMU ){
	    	try {
			  	byte data[] = new byte[] { b1, b2 };
			  	serialPort.writeBytes( data );
					bytesSent += 2;
				} catch (SerialPortException e) {
						e.printStackTrace();
				}
    	}
    	
    }
    
    /* Get value received on serial port
     * @params: [ch] char identifier of parameter whose value is desired as per the two byte protocol */
    public int getValue(char ch){    
    	/* Return yaw in the case of IMU */	
    	if( isIMU )	return (int) imuObject.getYaw();
    	
    	/* Process the lis and extract value */
    	byte retVal = 0;
    	int sz = ls.size(), uint8_val;
    	for( int i=0; i < (sz - 1); i++ ){    		   		
    		if( ls.get(i) == ch ){
    			retVal = ls.get(i+1);
    			ls.remove(i);
    			ls.remove(i);
    			break;
    		}
    	}
    	uint8_val = retVal & 0xFF;
    	return uint8_val;
    }  
    
    /* Convert a Byte List to String
     * @param: [_ls] Byte List */
    public String ByteToString(List<Byte> _ls){
			char[] _carray = new char[_ls.size()];
			for(int i=0; i<_ls.size(); i++){
				_carray[i] = (char)(_ls.get(i) & 0xFF);
			}
			return String.valueOf(_carray);
		}
	
		/* Initialize serial communication at specified port *
		 * @params: [port] portname, [baudRate] desired baud rate */
    public void initialize(String port, int baudRate, boolean _isIMU) {
    		isIMU = _isIMU;
    
    		portBaudRate = baudRate;
    		portName = port;
    		
        serialPort = new SerialPort(port); 
        try {
            serialPort.openPort();
            serialPort.setParams(baudRate, 8, 1, SerialPort.PARITY_NONE );
            int mask = SerialPort.MASK_RXCHAR + SerialPort.MASK_CTS + SerialPort.MASK_DSR;
            serialPort.setEventsMask(mask);
            serialPort.addEventListener(new SerialPortReader());
        }
        catch (SerialPortException ex) {
            ex.printStackTrace();
        }
        
        try {
						Thread.sleep(2000);
						serialPort.readBytes(); // Clear Buffer
				} catch (Exception e) {
						e.printStackTrace();
				}
    }
    
    /* Serial Events Listener Class */
    class SerialPortReader implements SerialPortEventListener {
        public void serialEvent(SerialPortEvent event) {
				boolean isNegativeValue = false;
            if(event.isRXCHAR()){						
            	if(event.getEventValue() >= 1){		  				            
                    try {                    		
                        byte buffer[] = serialPort.readBytes();
                        
                        if( isIMU ){
													for(int i = 0; i < buffer.length; i++){	
			                    	 //MissionController.logToFile(false,  (char) buffer[i]);
			                    	
			                    	if( (char) buffer[i] == '$'){
			                    		fillPacket = true;                         		
			                    	}
			                    	
			                    	if( fillPacket ){ 	
			                    		ls.add( buffer[i] );
			                    	}
			                    	
			                    	if( (char) buffer[i] == '*'){	
			                    		fillPacket = false;                        		
			                    	}
			                    	
			                    	
			                    	                	
			                    	if( ls.size() == 118 && !fillPacket){
			                    	  //MissionController.logToFile(true, );
			                    	  ls.remove(ls.size()-1);
			                    		//MissionController.logToFile(true,  " == " + ByteToString(ls) );
			                    		MissionController.IMUString = ByteToString(ls);
			                    		imuObject.setValues(ls);	 
			                    		ls.clear();
			                    	} else if(ls.size() == 118) {
			                    		MissionController.logToFile(true, "*******************" + ByteToString(ls) );
			                    		ls.clear();
			                    		fillPacket = false;
			                    	}
			                    	
			                    }
                        } // End if IMU
                        
                        else{
                        	/*for(int i = 0; i < buffer.length; i++)
				                    MissionController.logToFile(false, (char) buffer[i]);*/
				                   	
                        	/* Check for execute mission interrupt string : '@@' */
                        	if( buffer.length >= 2 && !MissionController.executeMission ){                        		
                        		for(int i = 0; i < buffer.length - 1; i++){
                        			if( (char) buffer[i] == '@' && (char) buffer[i+1] == '@'){
		                      			MissionController.executeMission = true;
		                      		}
		                      	}
                        	}
                        	/*else if( buffer.length >= 2 && MissionController.executeMission ){               		
                        		for(int i = 0; i < buffer.length - 1; i++){
                        			if( (char) buffer[i] == '@' && (char) buffer[i+1] == '@'){
		                      			try{ MissionController.halt(); } catch(Exception e) { e.printStackTrace(); }
		                      		}
		                      	}
                        	}*/
                        	
                        	/* Add data to buffer for processing, ignore '@' */
                        	for(int i = 0; i < buffer.length; i++){
                        		if(buffer[i] != '@') ls.add( buffer[i] );
                        	}
                        }
                    }
                    catch (SerialPortException ex) {
                        ex.printStackTrace();
                    }
                }
            }
        }
    }
    
}
