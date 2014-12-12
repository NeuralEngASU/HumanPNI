/****************************************************************************
 * GlobalInfo
 * 		This object stores and controls information between threads.
 * 		Uses the Semaphore class.
 * 
 * Author: Kevin O'Neill
 * Date: 2014.07.17
 * Version: Demo
 ****************************************************************************/

using UnityEngine;
using System.Collections;
using System.Threading;

public class GlobalInfo {

	// Creates the pool of semaphores
	private static Semaphore pool;

	private int[] taskInfo;
	private string patientName;
	private string pathName;
	private string fileName;
	private float[,] neuralDecode;

	private bool handFlag;
	private bool expFlag;
	private float timeStamp;	

	// Use this for any 'start' related Unity items
	void Start () {
		init ();
	} // END FUNCTION

	// Initilization of Variables
	public void init(){

		// Task info: TaskType, targets, eccentricity
		taskInfo = new int[3]{0,0,0};

		patientName = "P197001";
		pathName = @"C:\";
		fileName = "Demo";

		neuralDecode = new float[5,4]
		{{0, 0, 0, 0},
			{ 0, 0, 0, 0},
			{ 0, 0, 0, 0},
			{ 0, 0, 0, 0},
			{ 0, 0, 0, 0}};

		handFlag = true;
		expFlag = false;
		timeStamp = -1f;

		// Creates a new semaphore pool with only 1 thread availiable
		pool = new Semaphore(0, 1);
		pool.Release (1);
	} // END FUNCTION

	// Set Task Information
	public void SetTaskInfo (int[] tempTaskInfo) {
		pool.WaitOne();
		taskInfo = tempTaskInfo;
		pool.Release ();
	} // END FUNCTION

	// Get Task Information
	public int[] GetTaskInfo () {
		pool.WaitOne();
		int[] returnVar = taskInfo;
		pool.Release ();
		return returnVar;
	} // END FUNCTION

	// Set Patient Name 
	public void SetPatientName (string tempPatientName) {
		pool.WaitOne();
		patientName = tempPatientName;
		pool.Release ();
	} // END FUNCTION
	
	// Get Patient Name 
	public string GetPatientName () {
		pool.WaitOne();
		string returnVar = patientName;
		pool.Release ();
		return returnVar;
	} // END FUNCTION

	// Set Path Name 
	public void SetPathName (string tempPathName) {
		pool.WaitOne();
		pathName = tempPathName;
		pool.Release ();
	} // END FUNCTION
	
	// Get Path Name 
	public string GetPathName () {
		pool.WaitOne();
		string returnVar = pathName;
		pool.Release ();
		return returnVar;
	} // END FUNCTION

	// Set File Name 
	public void SetFileName (string tempFileName) {
		pool.WaitOne();
		fileName = tempFileName;
		pool.Release ();
	} // END FUNCTION
	
	// Get File Name 
	public string GetFileName () {
		pool.WaitOne();
		string returnVar = fileName;
		pool.Release ();
		return returnVar;
	} // END FUNCTION

	// Set Neural Decode 
	public void SetNeuralDecode (float[,] tempNeuralDecode) {
		pool.WaitOne();
		neuralDecode = tempNeuralDecode;
		pool.Release ();
	} // END FUNCTION
	
	// Get Neural Decode
	public float[,] GetNeuralDecode () {
		pool.WaitOne();
		float[,] returnVar = neuralDecode;
		pool.Release ();
		return returnVar;
	} // END FUNCTION

	// Set Hand Flag
	public void SetHandFlag (bool tempHandFlag) {
		pool.WaitOne();
		handFlag = tempHandFlag;
		pool.Release ();
	} // END FUNCTION
	
	// Get Hand Flag
	public bool GetHandFlag () {
		pool.WaitOne();
		bool returnVar = handFlag;
		pool.Release ();
		return returnVar;
	} // END FUNCTION
} // END CLASS
// EOF