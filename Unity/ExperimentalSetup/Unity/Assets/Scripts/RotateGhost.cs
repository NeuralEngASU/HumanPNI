/****************************************************************************
 * RotateGhost
 * 		This object controls the positioning of the targets
 * 
 * Author: Kevin O'Neill
 * Date: 2014.07.17
 * Version: Demo
 ****************************************************************************/

using UnityEngine;
using System.Collections;

public class RotateGhost {
	
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
	public void init(){

		// ***** Initialize Variables ***** //
		
		// *** THUMB *** //
		// Thumb GameObjects
		LTFingerPIP = GameObject.Find("LeftHandThumb1Ghost");
		LTFingerIIP = GameObject.Find("LeftHandThumb2Ghost");
		LTFingerDIP = GameObject.Find("LeftHandThumb3Ghost");
		
		RTFingerPIP = GameObject.Find("RightHandThumb1Ghost");
		RTFingerIIP = GameObject.Find("RightHandThumb2Ghost");
		RTFingerDIP = GameObject.Find("RightHandThumb3Ghost");

		// *** INDEX *** //
		// Index GameObjects
		LIFingerPIP = GameObject.Find("LeftHandIndex1Ghost");
		LIFingerIIP = GameObject.Find("LeftHandIndex2Ghost");
		LIFingerDIP = GameObject.Find("LeftHandIndex3Ghost");
		
		RIFingerPIP = GameObject.Find("RightHandIndex1Ghost");
		RIFingerIIP = GameObject.Find("RightHandIndex2Ghost");
		RIFingerDIP = GameObject.Find("RightHandIndex3Ghost");
		
		// *** MIDDLE *** //
		// Middle GameObjects
		LMFingerPIP = GameObject.Find("LeftHandMiddle1Ghost");
		LMFingerIIP = GameObject.Find("LeftHandMiddle2Ghost");
		LMFingerDIP = GameObject.Find("LeftHandMiddle3Ghost");
		
		RMFingerPIP = GameObject.Find("RightHandMiddle1Ghost");
		RMFingerIIP = GameObject.Find("RightHandMiddle2Ghost");
		RMFingerDIP = GameObject.Find("RightHandMiddle3Ghost");
		
		// *** RING *** //
		// Ring GameObjects
		LRFingerPIP = GameObject.Find("LeftHandRing1Ghost");
		LRFingerIIP = GameObject.Find("LeftHandRing2Ghost");
		LRFingerDIP = GameObject.Find("LeftHandRing3Ghost");
		
		RRFingerPIP = GameObject.Find("RightHandRing1Ghost");
		RRFingerIIP = GameObject.Find("RightHandRing2Ghost");
		RRFingerDIP = GameObject.Find("RightHandRing3Ghost");
		
		// *** LITTLE *** //
		// Little GameObjects
		LLFingerPIP = GameObject.Find("LeftHandPinky1Ghost");
		LLFingerIIP = GameObject.Find("LeftHandPinky2Ghost");
		LLFingerDIP = GameObject.Find("LeftHandPinky3Ghost");
		
		RLFingerPIP = GameObject.Find("RightHandPinky1Ghost");
		RLFingerIIP = GameObject.Find("RightHandPinky2Ghost");
		RLFingerDIP = GameObject.Find("RightHandPinky3Ghost");

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

		UpdateRot ();
	} // END FUNCTION
	
	// Updates the position of all targets
	public void UpdateRot () {

		// *** THUMB *** //
		LTFingerPIP.transform.localEulerAngles = new Vector3 ((float)51.9, (float)-73.09, (float)-96.35);
		LTFingerIIP.transform.localEulerAngles = new Vector3 (0, newRot [0, 2], 0);
		LTFingerDIP.transform.localEulerAngles = new Vector3 (0, newRot [0, 3], 0);
		
		RTFingerPIP.transform.localEulerAngles = new Vector3 ((float)-51.99927, (float)-73.09084, (float)96.35837);
		RTFingerIIP.transform.localEulerAngles = new Vector3 (0, newRot [0, 2], 0);
		RTFingerDIP.transform.localEulerAngles = new Vector3 (0, newRot [0, 3], 0);

		// *** INDEX *** //
		LIFingerPIP.transform.localEulerAngles = new Vector3 (0, newRot [1, 1], newRot [1, 0]);
		LIFingerIIP.transform.localEulerAngles = new Vector3 (0, newRot [1, 2], 0);
		LIFingerDIP.transform.localEulerAngles = new Vector3 (0, newRot [1, 3], 0);
		
		RIFingerPIP.transform.localEulerAngles = new Vector3 (0, newRot [1, 1], newRot [1, 0]);
		RIFingerIIP.transform.localEulerAngles = new Vector3 (0, newRot [1, 2], 0);
		RIFingerDIP.transform.localEulerAngles = new Vector3 (0, newRot [1, 3], 0);

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

	// Sets new rotations
	public void SetRot(float[,] rot){
		newRot = rot;
	} // END FUNCTION

	// Sets rotations to baseline
	public void SetRotBase(){
		newRot = baseLine;
	} // END FUNCTION
} // END CLASS
// EOF