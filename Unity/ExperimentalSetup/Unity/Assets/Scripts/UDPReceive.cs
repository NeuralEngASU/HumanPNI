/****************************************************************************
 * UDPRecieve
 * 		This object runs on its own thread and will listen for UDP packets sent from LabView.
 * 		Requires special termination due to the thread
 * 
 * Author: Kevin O'Neill
 * Date: 2014.07.17
 * Version: Demo
 ****************************************************************************/


using UnityEngine;
using System.Collections;

using System;
using System.Text;
using System.Net;
using System.Net.Sockets;
using System.Threading;

public class UDPReceive {
	
	// Receiving Thread Object
	Thread m_receiveThread;
	
	// UDP Client Object
	UdpClient m_client;
	
	// IP Address and Port
	private string IP = "127.0.0.1";	// default local
	private int port; 					// define in init
	
	// Containers for the recieved UDP packets
	private string lastUDP = "";
	private string allUDP  = ""; // Make this empty every now and then

	// Volatile bool for termination of thread
	public volatile bool terminateFlag;

	// String Delimeters
	char[] delimiterChars;

	// Global Info
	GlobalInfo m_globalInfo;

	// start from unity3d
	void Start(){
		init();
	}

	// Initialization of variables
	public void init(){
		// Debug line
		Debug.Log("UDPSend.init()");
		
		// define port
		port = 9090;

		delimiterChars = new char[] { ' ', ',', '.', ':', '\t' };

		// Set termination boolean
		terminateFlag = false;

	}// END FUNCTION

	public void CreateThread(){
		// Create thread
		m_receiveThread = new Thread(new ThreadStart(ReceiveData)); // Set function to thread
		m_receiveThread.IsBackground = true;						// Run in background
		m_receiveThread.Start();	
	}// END FUNCTION

	public void SetGlobalInfo(GlobalInfo tempGlobalInfo){
		// Attach to existing handle
		m_globalInfo = tempGlobalInfo;
	}// END FUNCTION

	// Receive Thread
	private  void ReceiveData(){
		try{
			// Setup UDP Client
			this.m_client = new UdpClient(port);

			while (!terminateFlag)
			{
				try
				{
					// Creates new IP end point to be any that sent a UDP (127.0.0.1)
					IPEndPoint anyIP = new IPEndPoint(IPAddress.Any, 0);
					byte[] data = m_client.Receive(ref anyIP);
				
					// Bytes should be in UTF8 format
					string text = Encoding.UTF8.GetString(data);
				
					// Print UDP
					Debug.Log(">> " + text);
				
					// Update last UDP packet
					lastUDP = text;
				
					// Save new packet to list of pacekts
					allUDP = allUDP + text;
					
					string[] words = text.Split(delimiterChars);

//					if(String.Compare(words[1], (string)'FileName')){
//						m_globalInfo.SetFileName(words[2]);
//					}// END IF
				
						

					Thread.Sleep (1);
				}
				catch (SocketException e)
				{
					Debug.Log(e.ToString()); // Print error
				} // END TRY
			} // END WHILE
		}
		catch(Exception e) {
			Debug.Log(e.ToString()); // Print Error
		} // END TRY
	} // END FUNCTION
	
	// Returns the last UDP packet recieved and deletes all UDP
	public string GetUDP(){
		allUDP = "";
		return lastUDP;
	} // END FUNCTION

	// Terminates the UDP thread
	public void Terminate (){
		IPEndPoint remoteEndPoint = new IPEndPoint(IPAddress.Parse("127.0.0.1"), port); // Creates new end point
		string textEnd = "Abort";	// Data for UDP to carry
		byte[] dataEnd = Encoding.UTF8.GetBytes (textEnd); // Data transcoded to a byte array
		terminateFlag = true; // Set terminate boolean
		m_client.Send(dataEnd, dataEnd.Length, remoteEndPoint); // Send UDP packet to self in order to end the WHILE loop
	} // END FUNCTION

	// Runs when application is terminated
	void OnApplicationQuit () {
		if (m_receiveThread != null) {
			m_receiveThread.Abort (); // Tries to forcefully close the thread if not already terminated
		} // END IF
	} // END FUNCTION
} // END CLASS
// EOF