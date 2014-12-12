/****************************************************************************
 * RotateBones
 * 		This object will control the human model fingers using data form LEAP Motion
 * 
 * Author: Kevin O'Neill
 * Date: 2014.07.17
 * Version: Demo
 ****************************************************************************/

using UnityEngine;
using System.Collections;
//using LeapMotion;


public class RotateBones {

	// ***** Define Variables ***** //
	// Definitions:
	// Adb (Abduction / Aduction)
	// PIP (Proximal Interphalangeal joint)
	// IIP (Intermediate Interphalangeal joint)
	// DIP (Distal Interphalangeal joint)
	
	// *** THUMB *** //
	// Thumb GameObjects
	// Left Hand						// Right Hand
	private GameObject LTFingerPIP;		private GameObject RTFingerPIP;
	private GameObject LTFingerIIP;		private GameObject RTFingerIIP;
	private GameObject LTFingerDIP;		private GameObject RTFingerDIP;
	
	// *** INDEX *** //
	// Index GameObjects
	// Left Hand						// Right Hand
	private GameObject LIFingerPIP;		private GameObject RIFingerPIP;
	private GameObject LIFingerIIP;		private GameObject RIFingerIIP;
	private GameObject LIFingerDIP;		private GameObject RIFingerDIP;
	
	// *** MIDDLE *** //
	// Middle GameObjects
	// Left Hand						// Right Hand
	private GameObject LMFingerPIP;		private GameObject RMFingerPIP;
	private GameObject LMFingerIIP;		private GameObject RMFingerIIP;
	private GameObject LMFingerDIP;		private GameObject RMFingerDIP;
	
	// *** RING *** //
	// Ring GameObjects
	// Left Hand						// Right Hand
	private GameObject LRFingerPIP;		private GameObject RRFingerPIP;
	private GameObject LRFingerIIP;		private GameObject RRFingerIIP;
	private GameObject LRFingerDIP;		private GameObject RRFingerDIP;
	
	// *** LITTLE *** //
	// Little GameObjects
	// Left Hand						// Right Hand
	private GameObject LLFingerPIP;		private GameObject RLFingerPIP;
	private GameObject LLFingerIIP;		private GameObject RLFingerIIP;
	private GameObject LLFingerDIP;		private GameObject RLFingerDIP;
	
	// Leap Motion Controller
	private LeapMotion m_Leap;

	// GlobalInfo
	GlobalInfo m_globalInfo;

	// SaveToFile
	// private m_saveToFile;
	
	// Rotation Variables
	private float[,] newRot;
	private float[,] maxFingerRot;
	private float[,] minFingerRot;
	private float[,] baseLine;

	// Use this for any 'start' related Unity items
	void Start(){
		init ();
	} // END FUNCTION

	// Initilization of Variables
	public void init () {
	
		// ***** Initialize Variables ***** //
		
		// *** THUMB *** //
		LTFingerPIP = GameObject.Find("LeftHandThumb1");
		LTFingerIIP = GameObject.Find("LeftHandThumb2");
		LTFingerDIP = GameObject.Find("LeftHandThumb3");
		
		RTFingerPIP = GameObject.Find("RightHandThumb1");
		RTFingerIIP = GameObject.Find("RightHandThumb2");
		RTFingerDIP = GameObject.Find("RightHandThumb3");

		// *** INDEX *** //		
		// Index GameObjects
		LIFingerPIP = GameObject.Find("LeftHandIndex1");
		LIFingerIIP = GameObject.Find("LeftHandIndex2");
		LIFingerDIP = GameObject.Find("LeftHandIndex3");
		
		RIFingerPIP = GameObject.Find("RightHandIndex1");
		RIFingerIIP = GameObject.Find("RightHandIndex2");
		RIFingerDIP = GameObject.Find("RightHandIndex3");
		
		// *** MIDDLE *** //
		// Middle GameObjects
		LMFingerPIP = GameObject.Find("LeftHandMiddle1");
		LMFingerIIP = GameObject.Find("LeftHandMiddle2");
		LMFingerDIP = GameObject.Find("LeftHandMiddle3");
		
		RMFingerPIP = GameObject.Find("RightHandMiddle1");
		RMFingerIIP = GameObject.Find("RightHandMiddle2");
		RMFingerDIP = GameObject.Find("RightHandMiddle3");
		
		// *** RING *** //
		// Ring GameObjects
		LRFingerPIP = GameObject.Find("LeftHandRing1");
		LRFingerIIP = GameObject.Find("LeftHandRing2");
		LRFingerDIP = GameObject.Find("LeftHandRing3");
		
		RRFingerPIP = GameObject.Find("RightHandRing1");
		RRFingerIIP = GameObject.Find("RightHandRing2");
		RRFingerDIP = GameObject.Find("RightHandRing3");
		
		// *** LITTLE *** //
		// Little GameObjects
		LLFingerPIP = GameObject.Find("LeftHandPinky1");
		LLFingerIIP = GameObject.Find("LeftHandPinky2");
		LLFingerDIP = GameObject.Find("LeftHandPinky3");
		
		RLFingerPIP = GameObject.Find("RightHandPinky1");
		RLFingerIIP = GameObject.Find("RightHandPinky2");
		RLFingerDIP = GameObject.Find("RightHandPinky3");
		
		// Leap Motion Controller
		m_Leap = new LeapMotion ();
		m_Leap.init ();

		// SaveToFile
		// m_saveObj = new SaveToFile ();
		// m_saveObj.init ();
		// m_saveObj.SetPathName(m_info.savePath);
		// m_saveObj.SetFileName("DemoTest");
		// m_saveObj.SetExtName(".csv");
		// m_saveobj.SetPatientName(m_info.patinetName);
		// m_saveObj.SetExpDate(DateTime.Now.ToString("yyyyMMdd", System.Globalization.CultureInfo.GetCultureInfo("en-US"))));
		// m_saveObj.SetExpDate(m_info.date);

		// *** ROTATIONS *** //
		// Pattern: {Abd, PIP, IIP, DIP}

		// New Rotations
		newRot = new float[5, 4] 
		{{0, 0, 0, 0},
			{ 0, 0, 0, 0},
			{ 0, 0, 0, 0},
			{ 0, 0, 0, 0},
			{ 0, 0, 0, 0}};
		
		// Maximum Rotation 
		maxFingerRot = new float[5, 4]
		{{20, 90, 60, 60},
			{ 20, 90, 60, 60},
			{ 20, 90, 60, 60},
			{ 20, 90, 60, 60},
			{ 20, 90, 60, 60}};
		
		// Minumin Rotation (Really it is the maximum deflection in the negative direction)
		minFingerRot = new float[5, 4]
		{{-20, -10, 0, 0},
			{ -20, -10, 0, 0},
			{ -20, -10, 0, 0},
			{ -20, -10, 0, 0},
			{ -20, -10, 0, 0}};
		
		// Rest Position
		baseLine = new float[5, 4]
		{{0, 0, 0, 0},
			{ 0, 0, 0, 0},
			{ 0, 0, 0, 0},
			{ 0, 0, 0, 0},
			{ 0, 0, 0, 0}};
	} // END FUNCTION

	public void SetGlobalInfo(GlobalInfo tempGlobalInfo){
		// Attach to existing handle
		m_globalInfo = tempGlobalInfo;
	}// END FUNCTION

	// Updates the rotation of all bones
	public void UpdateRot () {

		newRot = m_Leap.GetRot();

		// *** THUMB *** //
		LTFingerPIP.transform.localEulerAngles = new Vector3 ((float)51.9, (float)-73.09+newRot [0, 1], (float)-96.35+newRot [0, 0]);
		LTFingerIIP.transform.localEulerAngles = new Vector3 (0, newRot [0, 2], 0);
		LTFingerDIP.transform.localEulerAngles = new Vector3 (0, newRot [0, 3], 0);
		
		RTFingerPIP.transform.localEulerAngles = new Vector3 ((float)-51.99927, (float)-73.09084, (float)96.35837);
//		RTFingerIIP.transform.localEulerAngles = new Vector3 (0, newRot [0, 2], 0);
//		RTFingerPIP.transform.localEulerAngles = new Vector3 ((float)-51.99927, (float)-73.09084+newRot [0, 1], (float)96.35837-0.5f*newRot [0, 0]);
		RTFingerIIP.transform.localEulerAngles = new Vector3 (0, newRot [0, 2], newRot [0, 1]);
		RTFingerDIP.transform.localEulerAngles = new Vector3 (0, newRot [0, 3], 0);
		
		// *** INDEX *** 
		LIFingerPIP.transform.localEulerAngles = new Vector3 (0, newRot [1, 1], newRot [1, 0]);
		LIFingerIIP.transform.localEulerAngles = new Vector3 (0, newRot [1, 2], 0);
		LIFingerDIP.transform.localEulerAngles = new Vector3 (0, newRot [1, 3], 0);
		
		RIFingerPIP.transform.localEulerAngles = new Vector3 (0, newRot [1, 1], newRot [1, 0]);
		RIFingerIIP.transform.localEulerAngles = new Vector3 (0, newRot [1, 2], 0);
		RIFingerDIP.transform.localEulerAngles = new Vector3 (0, newRot [1, 3], 0);
				
//		RIFingerPIP.transform.rotation = m_Leap.GetQuaternion (2);
//		RIFingerIIP.transform.rotation = m_Leap.GetQuaternion (1);
//		RIFingerDIP.transform.rotation = m_Leap.GetQuaternion (0);
		
		// *** MIDDLE *** //
		LMFingerPIP.transform.localEulerAngles = new Vector3 (0, newRot [2, 1], newRot [2, 0]);
		LMFingerIIP.transform.localEulerAngles = new Vector3 (0, newRot [2, 2], 0);
		LMFingerDIP.transform.localEulerAngles = new Vector3 (0, newRot [2, 3], 0);
		
		RMFingerPIP.transform.localEulerAngles = new Vector3 (0, newRot [2, 1], newRot [2, 0]);
		RMFingerIIP.transform.localEulerAngles = new Vector3 (0, newRot [2, 2], 0);
		RMFingerDIP.transform.localEulerAngles = new Vector3 (0, newRot [2, 3], 0);
		
		// *** RING *** //
		LRFingerPIP.transform.localEulerAngles = new Vector3 (0, newRot [3, 1], newRot [3, 0]);
		LRFingerIIP.transform.localEulerAngles = new Vector3 (0, newRot [3, 2], 0);
		LRFingerDIP.transform.localEulerAngles = new Vector3 (0, newRot [3, 3], 0);
		
		RRFingerPIP.transform.localEulerAngles = new Vector3 (0, newRot [3, 1], newRot [3, 0]);
		RRFingerIIP.transform.localEulerAngles = new Vector3 (0, newRot [3, 2], 0);
		RRFingerDIP.transform.localEulerAngles = new Vector3 (0, newRot [3, 3], 0);
		
		// *** LITTLE *** //
		LLFingerPIP.transform.localEulerAngles = new Vector3 (0, newRot [4, 1], newRot [4, 0]);
		LLFingerIIP.transform.localEulerAngles = new Vector3 (0, newRot [4, 2], 0);
		LLFingerDIP.transform.localEulerAngles = new Vector3 (0, newRot [4, 3], 0);
		
		RLFingerPIP.transform.localEulerAngles = new Vector3 (0, newRot [4, 1], newRot [4, 0]);
		RLFingerIIP.transform.localEulerAngles = new Vector3 (0, newRot [4, 2], 0);
		RLFingerDIP.transform.localEulerAngles = new Vector3 (0, newRot [4, 3], 0);



	} // END FUNCTION
} // END CLASS
// EOF