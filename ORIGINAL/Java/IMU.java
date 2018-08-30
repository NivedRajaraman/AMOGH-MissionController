import java.util.List;
import java.text.DecimalFormat;

public class IMU{
	/* Attitude */
	public float yaw, pitch, roll;

	/* Magnetometer */
	public float magnetometer_x, magnetometer_y, magnetometer_z;

	/* Accleration */
	public float accl_x, accl_y, accl_z;

	/* Angular Rates */
	public float ang_rate_x, ang_rate_y, ang_rate_z;

	/* List & String containing IMU Params */
	public List<Byte> dataList;
	private String dataStr;


	private float round(float val){
				return val;
		  	// return Math.round(val*10000)/10000.0f;
		  	// DecimalFormat numberFormat = new DecimalFormat("#.00");
		    // return Float.valueOf(numberFormat.format(val));
	}

	public void printData(){
		MissionController.logToFile(true, "Yaw  : " + yaw + ", Pitch : " + pitch + ", Roll : " + roll);
		MissionController.logToFile(true, "MagX : " + magnetometer_x + ", MagY  : " + magnetometer_y + ", MagZ : " + magnetometer_z);
		MissionController.logToFile(true, "AclX : " + accl_x + ", AclY  : " + accl_y + ", AclZ : " + accl_z);
		MissionController.logToFile(true, "AngR : " + ang_rate_x + ", AngY  : " + ang_rate_y + ", AngZ : " + ang_rate_z);
		MissionController.logToFile(true, "");
	}

	private String ByteToString(List<Byte> ls){
		char[] _carray = new char[ls.size()];
		for(int i=0; i<ls.size(); i++){
			_carray[i] = (char)(ls.get(i) & 0xFF);
		}
		return String.valueOf(_carray);
	}

	public void setValues(List<Byte> ls_data){

		dataStr = ByteToString(ls_data);
		//dataStr = ByteToString(dataList);
		String[] tokens = dataStr.split(",");

		// MissionController.logToFile(true, "Data : " + dataStr + " End" );
		try{
			/* Attitude */
			yaw = round( Float.valueOf(tokens[1]) + 180);
			pitch = round( Float.valueOf(tokens[2]) );
			roll = round( Float.valueOf(tokens[3]) );

			/* Magnetometer */
			magnetometer_x = round( Float.valueOf(tokens[4]) );
			magnetometer_y = round( Float.valueOf(tokens[5]) );
			magnetometer_z = round( Float.valueOf(tokens[6]) );

			/* Accleration */
			accl_x = round( Float	.valueOf(tokens[7]) );
			accl_y = round( Float.valueOf(tokens[8]) );
			accl_z = round( Float.valueOf(tokens[9]) );

			/* Angular Rates */
			ang_rate_x = round( Float.valueOf(tokens[10]) );
			ang_rate_y = round( Float.valueOf(tokens[11]) );
			ang_rate_z = round( Float.valueOf( tokens[12].substring(0, tokens[12].length() - 1 ) ) );

		}
		catch(Exception e){
			MissionController.logToFile(true, "Data : " + dataStr + " End" );
			MissionController.logToFile(true, "IMU : EXCEPTION OCCURRED");
			e.printStackTrace();
		}

		// printData();
	}

	public String getDataString(){
		String str = /*"Yaw: " + Float.toString(yaw) +
								 ", Pitch: " + Float.toString(pitch) +
								 ", Roll: " + Float.toString(roll) +
								 ", MagX: " + Float.toString(magnetometer_x) +
								 ", MagY: " + Float.toString(magnetometer_y) +
								 ", MagZ: " + Float.toString(magnetometer_z) +
								 ", AclX: " + Float.toString(accl_x) +
								 ", AclY: " + Float.toString(accl_y) +
								 ", AclZ: " + Float.toString(accl_z) +
								 ", AngX: " + Float.toString(ang_rate_x) +
								 ", AngY: " + Float.toString(ang_rate_y) +
								 ", AngZ: " + Float.toString(ang_rate_z) +
								 " \r\n" + */dataStr + "\r\n\r\n";
		return str;
	}

	public float getYaw(){
		if( MissionController.correctIMUDrift ){
			float dH = (float)(( System.currentTimeMillis() - MissionController.missionStartTime ) * MissionController.imuDriftRate);
			int cH = Math.round(yaw - dH);
			MissionController.logToFile(true,  "Actual : " + yaw + ", Corrected Heading : " + cH + ", Correction : -" + dH );
			return (yaw - dH);
		}

		return yaw;
	}

}
