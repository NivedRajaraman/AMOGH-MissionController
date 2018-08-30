import java.io.*;
import java.util.*;
import java.text.DateFormat;
import java.text.SimpleDateFormat;

public class FileLogger {
	public int parameterCount = 0;
	public String cwd, logDir, logFile = null;
	public String logTitle = null;
	public ArrayList<String> parameterNames = new ArrayList<String>();
	public ArrayList<Float> parameterValues = new ArrayList<Float>();
	public FileOutputStream out;
	
	private String newline = "\r\n";
	
	public void record(float val){
		parameterValues.add(val);
	}
	
	public boolean dump() throws FileNotFoundException{
		
		if( logFile == null ){
			int numLogs = new File(logDir).list().length;
			logFile = logDir+Integer.toString(numLogs)+".csv";
		}
		out = new FileOutputStream(logFile, true);
		
		/* Check parameter count */
		if( parameterCount == 0){
			MissionController.logToFile(true, "File Logger Error: 'parameterCount' is not set or set to zero");
			return false;
		}
		
		/* Log Title */
		if( logTitle == null || logTitle.isEmpty() ) logTitle = "<No Title>"+newline;
		
		/* System Date & Time */
	    DateFormat dateFormat = new SimpleDateFormat("yyyy/MM/dd HH:mm:ss");
	    Date date = new Date();
	    String dt = dateFormat.format(date)+newline;
	    
		try {
			out.write(logTitle.getBytes());
			out.write(newline.getBytes());
			out.write(dt.getBytes());
		} 
		catch (IOException e) {
			e.printStackTrace();
		}
		
		/* Parameter Names */
		if( !parameterNames.isEmpty() ){
			if( parameterNames.size() != parameterCount ){
				MissionController.logToFile(true, "File Logger WARNING: 'parameterCount' and 'parameterNames.size()' are not equal");
			}
			for(int count=0; count<parameterNames.size(); count++){
				String str = parameterNames.get(count)+",";
				if( count == parameterNames.size() - 1 ){
					str = parameterNames.get(count) + newline;
				}
				try{
					out.write(str.getBytes());
				}
				catch (IOException e){
					e.printStackTrace();
				}
			}
		}
		else{
			MissionController.logToFile(true, "File Logger WARNING: Parameter names not set");
		}
		
		/* Parameter Values */
		if ( !parameterValues.isEmpty() ){
			String str = null;
			int idx;
			for(idx=0; idx<parameterValues.size(); idx++){
				
				if( !((idx+1) % parameterCount == 0) )
					str = Float.toString(parameterValues.get(idx))+",";
				else
					str = Float.toString(parameterValues.get(idx))+newline;
				
				try{
					out.write(str.getBytes());
				}
				catch (IOException e){
					e.printStackTrace();
				}
			}
			
			/* Add commas for incomplete sets*/
			while((idx+1) % parameterCount != 0 ){
				try{
					out.write(",".getBytes());
				}
				catch (IOException e){
					e.printStackTrace();
				}
				idx++;
			}
			
			/* Add newline at the end */
			try{
				out.write(newline.getBytes());
			}
			catch (IOException e){
				e.printStackTrace();
			}
		}
		else{
			MissionController.logToFile(true, "File Logger: No parameter values to log");
		}
		
		/* End of log */
		String eol = "**************************************************"+newline;
		try{
			out.write(eol.getBytes());
		}
		catch (IOException e){
			e.printStackTrace();
		}
		return true;
	}
	
	public void createNewLog(){
		int numLogs = new File(logDir).list().length;
		logFile = logDir+Integer.toString(numLogs+1)+".csv";
	}
	
	public void flush(){
		parameterValues.clear();
	}
	
	public void initialize(){
		File dir = new File("logs/");
		cwd = System.getProperty("user.dir");
		logDir = cwd + "/logs/";
		if( !dir.exists() ){
			MissionController.logToFile(true, "Log files directory does not exist!");
			if( dir.mkdir() ){
				MissionController.logToFile(true, "Log directory successfully created at '"+logDir+"'");
			}
			else {
				MissionController.logToFile(true, "Error creating directory at '"+cwd+"/logs'");
			}
		}
		else{
			MissionController.logToFile(true, "FileLogger : Saving logs at "+logDir);
		}
	}
}
