/****************************************************************************
 * TaskControl
 * 		This object controls the display of the task
 * 
 * Author: Kevin O'Neill
 * Date: 2014.07.17
 * Version: Demo
 ****************************************************************************/

using UnityEngine;
using System.Collections;

public class TaskControl {

	// **** Variable Definitions **** //
	// Task Control Variables
	private int currTask;			// Current Task: (Single Finger [SF], Multi Finger [MF], Finger Posture [FP], Intrinsic Finger [IF])
	private int[] currTarget;		// Current Target: Which finger(s) [SF, MF, IF]/posture [FP]
	private float[,] currEcc;		// Current Eccentricity: What joint angles [SF, MF, IF]/ eccentricity of posture [FP]
	private int currTimeOut;		// Current Timeout for trial
	private float currTimeTrial;	// Total time for trial
	private bool trialFlag;			// Boolean to say that the trial has started

	// Target Object Variables
	private GameObject[] gameObj;	// Collection of game objects: x10 finger tips. Need to control meshes and mesh renderers
	private int gameObjFlag;		// Controls which 'row' of the game object array to use [0: target_sphere, 1: target:torus]

	private Material greenTarget;	// Green material object
	private Material redTarget;		// Red material object
	private Material nullMat;		// Null material object
	private Mesh sphereMesh;		// Sphere mesh object
	private Mesh ringMesh;			// Ring mesh object
	private Mesh nullMesh;			// Null Mesh object

	// Rotate Ghost
	private RotateGhost m_rotateGhost;	// Rotate Ghost Object

	// Trial Success
	private TaskSuccess m_taskSuccess; 	// Task Success Object 
	
	// Use this for any 'start' related Unity items
	void Start () {
		init ();
	} // END FUNCTION

	// Initialization of Variables
	public void init(){

		// Task Control Variables
		currTask = -1;      					// No Task
		currTarget = new int[5] {0,0,0,0,0};	// No Target (Thumb, Index, Middle, Ring, Little)
		currEcc = new float[4,5]				// No Eccentricity
		   {{0,0,0,0,0},
			{0,0,0,0,0},
			{0,0,0,0,0},
			{0,0,0,0,0}};							
		currTimeTrial = 0;						// No time elapsed for trial (seconds)
		currTimeOut = 5000; 					// Trial timeout (miliseconds)

		// Rotate Ghost
		m_rotateGhost = new RotateGhost();
		m_rotateGhost.init();

		// Task Success
		m_taskSuccess = new TaskSuccess ();
		m_taskSuccess.init ();

		// Set up game objects
		Debug.Log("Task Objects Setup");
		gameObj = new GameObject[10];

		// Left hand
		gameObj[0] = GameObject.Find("LeftHandThumbGhostTarget");
		gameObj[1] = GameObject.Find("LeftHandIndexGhostTarget");
		gameObj[2] = GameObject.Find("LeftHandMiddleGhostTarget");
		gameObj[3] = GameObject.Find("LeftHandRingGhostTarget");
		gameObj[4] = GameObject.Find("LeftHandPinkyGhostTarget");

		// Right Hand
		gameObj[5] = GameObject.Find("RightHandThumbGhostTarget");
		gameObj[6] = GameObject.Find("RightHandIndexGhostTarget");
		gameObj[7] = GameObject.Find("RightHandMiddleGhostTarget");
		gameObj[8] = GameObject.Find("RightHandRingGhostTarget");
		gameObj[9] = GameObject.Find("RightHandPinkyGhostTarget");

		greenTarget = Resources.Load("GreenTarget", typeof(Material)) as Material;
		redTarget   = Resources.Load("redTarget"  , typeof(Material)) as Material;
		nullMat     = null;

		sphereMesh  = Resources.Load("IcosphereTarget", typeof(Mesh)) as Mesh;
		ringMesh    = Resources.Load("TorusTarget"    , typeof(Mesh)) as Mesh;
		nullMesh    = null;

		// Check to see if the mesh was found
		if (sphereMesh == null || ringMesh == null) {
			Debug.Log ("Null Error. Missing Resources from Resources folder.");
		} // END IF

		// Loop over each finger
		for (int i = 0; i <= 9; i++) {
			gameObj [i].GetComponent<MeshFilter>   ().mesh     = sphereMesh;	// Assign Mesh
			gameObj [i].GetComponent<MeshRenderer> ().material = redTarget;		// Assign Material
			gameObj [i].GetComponent<MeshRenderer> ().enabled  = false;			// Enable Render
		} // END FOR

		Debug.Log("Task Objects Initialized");

	} // END FUNCTION

	// Sets the timeout for all trials
	public void SetTimeOut(int timeOut){
			currTimeOut = timeOut;
	} // END FUNCTION

	// Instruction contronls the task, target, and eccentricity display
	public void Instruction(int task, int[] target, float[,] ecc){

		// Sets trial values
		currTarget = target;
		currEcc    = ecc;

		// IF new task is different from the old task, delete scene and regenerate
		// ELSE reset the current scene
		if (currTask != task) {
			currTask = task;	// Assign current Task
			DestroyScene ();	// Destroy Scene
			ConstructScene ();	// Construct Scene
		} else {
			ResetScene();		// Resets scene
		}// END IF

		StartScene();	// Starts the scene

	} // END FUNCTION

	// Turns the scene off from view and delets targets
	void DestroyScene(){

		// Loop over each finger
		for (int i = 0; i <= 9; i++) {
			gameObj [i].GetComponent<MeshFilter>   ().mesh     = nullMesh;		// Assign Mesh
			gameObj [i].GetComponent<MeshRenderer> ().material = redTarget;		// Assign Material
			gameObj [i].GetComponent<MeshRenderer> ().enabled  = false;			// Enable Render
		} // END FOR

		//Move Ghost targets to Rest
		m_rotateGhost.SetRotBase();
		m_rotateGhost.UpdateRot();

	} // END FUNCTION
	
	// Sets targets to red and sends to rest position
	void ResetScene(){
		
		// Loop over each finger
		for (int i = 0; i <= 9; i++) {
			gameObj [i].GetComponent<MeshRenderer> ().material = redTarget;		// Assign Material
			gameObj [i].GetComponent<MeshRenderer> ().enabled  = true;			// Enable Render
		} // END FOR
		
		//Move Ghost targets to Rest
		m_rotateGhost.SetRotBase();
		m_rotateGhost.UpdateRot();
		
	} // END FUNCTION

	// Builds new scene with correct targets and colors
	void ConstructScene(){

		// Use the correct mesh
		switch (currTask) {
			case 0: // Finger Task
				// Loop over each finger
				for (int i = 0; i <= 9; i++) {
					gameObj [i].GetComponent<MeshFilter>   ().mesh     = sphereMesh;	// Assign Mesh
				} // END FOR
				break;

			case 1: // Ring Task
				// Loop over each finger
				for (int i = 0; i <= 9; i++) {
					gameObj [i].GetComponent<MeshFilter>   ().mesh     = ringMesh;		// Assign Mesh
				} // END FOR
				break;

			default: // Default
				Debug.Log ("Task number not recognized _ ConstructScene");
				break;
		} // END SWITCH
	} // END FUNCTION

	// Starts the scene and displays the targets
	void StartScene(){
			
		// Change color on selected targets for both hands
		for (int i = 0; i<=4; i++) {
			if(currTarget[i] == 1){
				gameObj [i  ].GetComponent<MeshRenderer> ().material = greenTarget;		// Assign Material
				gameObj [i+5].GetComponent<MeshRenderer> ().material = greenTarget;		// Assign Material
			} // END IF
		} // END FOR

		// Move Targets
		m_rotateGhost.SetRot(currEcc); 	// Set Rotations
		m_rotateGhost.UpdateRot();		// Apply rotations

		// Display Targets
		// Loop over each finger
		for (int i = 0; i <= 9; i++) {
			gameObj [i].GetComponent<MeshRenderer> ().enabled  = true;		// Enables render
		} // END FOR

		// Start Timer
		currTimeTrial = 0;

		// StartScene
		trialFlag = true;

	} // END FUNCTION

	// Called every frame in order to update the time
	public void TrialUpdate(){

//		m_taskSuccess.Check ();

		currTimeTrial = currTimeTrial + Time.deltaTime;

		// Decides if the patient has taken too long
		if (currTimeTrial * 1000 >= currTimeOut && trialFlag) {
			ResetScene();	// Resets scene
//			m_taskSuccess.TimeOut(); // Tells the 
		}

	
	} // END FUNCTION
	
}
