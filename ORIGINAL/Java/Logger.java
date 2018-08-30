import java.net.*;
import java.io.*;

public class Logger {
	private ServerSocket serverSocket;
	private Socket server;
	private DataOutputStream out; 
	private static boolean isConnected = false;
	public static boolean log = false;
	
	
	public void consoleLog(String msg){
		System.out.println(msg);
	}

	public void start(){
		log = true;
	}
	
	public void stop(){
		log = false;
	}
	
	public void initialize(int port) throws IOException{
		serverSocket = new ServerSocket(port,4096);
		serverSocket.setSoTimeout(0);
		consoleLog("Log Server Started: " + this.serverSocket);
		server = serverSocket.accept();
		consoleLog("Connected To: " + server.getRemoteSocketAddress() );	
		out = new DataOutputStream(server.getOutputStream());
		isConnected = true;
	}
	
	public void closeServer(){
		if( isConnected ){
			try{
				System.out.println("Loggger::closeServer : Closing log TCP Server");
				server.close();
				serverSocket.close();
			}
			catch(Exception e){
				e.printStackTrace();
			}
		}
		isConnected = false;
	}
	
	public int sendData(String data) throws InterruptedException{
		try{
			if( isConnected ){
				out.write( data.getBytes() );
				return 0;
			}
			else
				return -1;
		}
		catch(IOException e){
			//e.printStackTrace();
			consoleLog("Connection Lost");
			return -1;
		}
		
		/*while(true){
			try{				
								
				while(true){
					if( newDataAvailable ){
						try{
							out.write(sendString.getBytes());
							newDataAvailable = false;
						}
						catch(IOException e){
							e.printStackTrace();
							break;
						}
					}
				}
				
				server.close();
			}
			catch(IOException e){
				e.printStackTrace();
				break;
			}
		}*/
	}

}
