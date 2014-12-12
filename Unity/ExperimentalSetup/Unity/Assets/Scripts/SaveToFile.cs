/****************************************************************************
 * SaveToFile
 * 		This object stream data to a CSV every frame.
 * 
 * Author: Kevin O'Neill
 * Date: 2014.07.17
 * Version: Demo
 ****************************************************************************/

using UnityEngine;
using System.Collections;
using System.IO;

public class SaveToFile {

	// File variables
	private string pathName;	// Path to save location
	private string fileName;	// File name
	private string extName;		// Extension (default: .csv)
	private string expDate;		// Date of the experiment
	private string patientName;	// Patient name (ex: P201401)

	private string fullFile;	// Full path/file name for the new file.
	
	// Use this for any 'start' related Unity items
	void Start(){
		init ();
	} // END FUNCTION
	
	// Initilization of Variables
	public void init() {
		pathName    = @"C:\";  	// Use @ symbol so you don't have to escape your backslashes.
		fileName    = "";		// Default fileName. Must be issued by other objects or LabView
		extName     = ".csv";	// Defaule extension. Other extensions can be used for different data.
		expDate     = "";		// Experiment date to the data is attached to the correct day/time.
		patientName = "";		// Patient name so the data is attached to the right patient.


	} // END FUNCTION

	// Sets the path to a new path
	public void SetPathName(string tempPath){
		pathName = tempPath;
		pathName.Replace('/', '\\'); // Use backslashes for folder deliminators

		// Ensure that the pathName ends with a \
		if (!pathName.EndsWith ("\\")) {
				pathName = pathName + "\\";
		} // END IF
	} // END FUNCTION

	// Sets the file name to a new file name
	public void SetFileName(string tempFile){
		fileName = tempFile;
	} // END FUNCTION

	// Sets the extension to a new extension
	public void SetExtName(string tempExt){
		extName = tempExt;
	} // END FUNCTION

	// Sets the path to a new path
	public void SetExpDate(string tempDate){
		expDate = tempDate;
	} // END FUNCTION

	// Sets the path to a new path
	public void SetPatientName(string tempPatient){
		patientName = tempPatient;
	} // END FUNCTION

	public void MakeFullFile(){
		fullFile = pathName + patientName + "_" + expDate + "_" + fileName + extName;
	} // END FUNCTION

	// Makes the file/path for the saved data.
	public void MakePath(){
		// Checks if file exists, if not create a new path and file
		if (!File.Exists (pathName)) {
			File.Create(pathName);
		} // END IF
	} // END FUNCTION

	public void WriteCSV(string[] writeData) {
		File.AppendAllText(fullFile, string.Join (",", writeData)+"\n");
	} // END FUNCTION
} // END CLASS
// EOF