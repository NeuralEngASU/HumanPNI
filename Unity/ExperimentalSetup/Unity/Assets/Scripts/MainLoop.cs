/****************************************************************************
 * MainLoop
 * 		This object is used to control the Virtual Environment for the Human Peripherial Nerve System Project
 * 		This object is high level and is used to convey information and contol the other objects
 * 
 * Author: Kevin O'Neill
 * Date: 2014.07.17
 * Version: Demo
 ****************************************************************************/


using UnityEngine;
using System.Collections;
using System.Text.RegularExpressions;
using System.Diagnostics;

public class MainLoop : MonoBehaviour {
	
	// UDP Receive
	public UDPReceive m_UDPReceive;

	// Global Info
	public GlobalInfo m_globalInfo;

	// RotateBones
	public RotateBones m_rotateBones;	

	// TaskControl
	public TaskControl m_taskControl;

	// Control Variables
	private bool handFlag;
	private bool startFlag;
	private int  timeStamp;

	// Tests
	private float timeCount;
	private bool testFlag;
	private float[,] ecc;
	private int[] target;

	// Use this for any 'start' related Unity items
	void Start () {
		init ();
	} // END FUNCTION

	// Initialization of Variables
	void init(){
		timeCount = 0;
		testFlag = true;
		
		ecc = new float[5,4] // No Eccentricity
		   {{0,0,0,0},
			{0,0,0,0},
			{0,0,0,0},
			{0,0,0,0},
			{0,0,0,0}};

		// Global Info
		m_globalInfo = new GlobalInfo ();
		m_globalInfo.init ();

		// UDP Receive
		//		m_UDPReceive = new UDPReceive();
		//		m_UDPReceive.init ();
		//		m_UDPRecieve.SetGlobalInfo(m_globalInfo);
		
		// RotateBones
		m_rotateBones = new RotateBones ();
		m_rotateBones.init (); 
//		m_rotateBones.SetGlobalInfo(m_globalInfo);
		
		// TaskControl
		m_taskControl = new TaskControl ();
		m_taskControl.init ();// m_taskControl.init (m_globalInfo);
	} // END FUCNTION

	// Update is called once per frame
	void Update () {

//		inst = m_UDPRecieve.GetLastUDP ();
//		m_rotateBones.UpdateRot ();
//		m_taskControl.TrialUpdate ();

		if (timeCount >= 6 && testFlag) {
			target = new int[5]{0,1,0,1,0};
			m_taskControl.Instruction (0, target, ecc);
			testFlag = false;
			timeCount = 0;
		} else if (timeCount >= 6 && !testFlag) {
			target = new int[5]{1,0,1,0,1};
			m_taskControl.Instruction (1, target, ecc);
			testFlag = true;
			timeCount = 0;
		} else {
			timeCount = timeCount + Time.deltaTime;
		} // END IF


		m_rotateBones.UpdateRot ();

		if (Input.GetKeyDown ("q")) {
			Application.Quit ();
		} // END IF


	} // END FUNCTION

	// OnApplicationQuit is called when the application is closed or terminated.
	void OnApplicationQuit() {

		if (m_UDPReceive != null) {
				m_UDPReceive.Terminate ();
		} // END IF
		print ("Quitting...");
	} // END FUNCTION
} // END CLASS
// EOF