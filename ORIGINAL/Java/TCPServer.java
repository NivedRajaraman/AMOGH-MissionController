import java.io.BufferedReader;
import java.io.BufferedWriter;
import java.io.IOException;
import java.io.InputStreamReader;
import java.io.PrintWriter;
import java.net.ServerSocket;
import java.net.Socket;
import java.util.Arrays;
import java.util.List;

public class TCPServer {

	ServerSocket listener;
	Socket clientSocket;
	int lock = 0;
	BufferedWriter outToClient;
	public boolean isRunning = false;
	
	public TCPServer() {
	}
	
	public void initialize(){
		String dataIn, temp;
		MissionController.logToFile(true, "TCPServer initialize: Server being setup at 9898 ");
		try {
			listener = new ServerSocket(9898);
		} catch (IOException e1) {
			e1.printStackTrace();
		}
		try {
			clientSocket = listener.accept();
			final BufferedReader inFromClient = new BufferedReader(
					new InputStreamReader(clientSocket.getInputStream()));
			outToClient = new BufferedWriter(new PrintWriter(
					clientSocket.getOutputStream()));
			// Need to write Debug here
			Thread listenerThread = new Thread(new Runnable() {

				@Override
				public void run() {
					// TODO Auto-generated method stub
					try {
						outToClient.write("Start/");
						outToClient.flush();
						String temp = inFromClient.readLine();
						if (temp.equals("Confirm/")) {
							MissionController.logToFile(true, "Hand-shake complete");
							isRunning = true;
						} else {
							MissionController.logToFile(true, "Hand-shake failed");
							System.exit(-1);
						}
						while (true) {
							String message = null;
							
							try{ message = inFromClient.readLine(); }
							catch (Exception e){}
							
							// Need to look for a way to make it atomic, not
							// sure how java handles, interrupts from other
							// threads.
							while (lock != 0);
							lock = 1;
							onRecieveMessage(message);
							lock = 0;
						}

					} catch (IOException e) {
						// TODO Auto-generated catch block
						e.printStackTrace();
						isRunning = false;
										
					}
				}
			});

			listenerThread.start();
		} catch (Exception e) {
			e.printStackTrace();
		}
	}

	public void onRecieveMessage(String mesg) {
		//MissionController.logToFile(true, "TCP Message => " + mesg);
		List<String> array = Arrays.asList(mesg.split(","));
		
		MissionController.taskId = array.get(0);
		MissionController.radius = Double.parseDouble(array.get(1));
		MissionController.theta = Double.parseDouble(array.get(2));
		
		if( array.size() == 4 )
			MissionController.slope = Double.parseDouble(array.get(3));
		
		// MissionController.logToFile(true, "Task ID: " + MissionController.taskId + ", Radius: " + MissionController.radius + ", Theta : " + MissionController.theta);
		// Call Handler in the corresponding IP-Mission Controller.
	}

	// Note have to make sure that only one thread at any-time can call this routine
	// Not handling concurrency issues
	// Have to check for the current lock, if it can lead to deadlocks.
	public void sendMessage(String mesg) {
		while (lock != 0);
		lock = -1;
		try {
			outToClient.write(mesg);
			outToClient.flush();
		} catch (IOException e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
		}
		lock = 0;
	}
}
