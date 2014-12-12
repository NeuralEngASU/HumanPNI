/****************************************************************************
 * LeapMotion
 * 		Calculates and tracks the success of a task.
 * 
 * 		Success: - Patient reached target and held for set duration
 * 
 * 		Faulure: - Patient 'did not reach target' (DNRT)
 * 				 - Patient reached target, but 'did not hold' for set duration (DNH)
 * 
 * 
 * Author: Barrett Andries, Kevin O'Neill
 * Date: 2014.07.17
 * Version: Demo
 ****************************************************************************/

using UnityEngine;
using System.Collections;
using Leap;

public class LeapMotion {

	private Controller controller;
	private Hand hand;
	private float[,] fingerRot;
	private int rightHandID, leftHandID;

	private GameObject HandObject;
	private GameObject RIFingerPIP;
	private GameObject RIFingerIIP;
	private GameObject RIFingerDIP;

	// Use this for any 'start' related Unity items
	void Start(){
		init ();
	} // END FUNCTION
	
	// Initilization of Variables
	public void init(){
		controller = new Controller ();

		fingerRot = new float[5,4]
			{{0, 0, 0, 0},
			{ 0, 0, 0, 0},
			{ 0, 0, 0, 0},
			{ 0, 0, 0, 0},
			{ 0, 0, 0, 0}};

		HandObject = GameObject.Find("RightHand");

		RIFingerPIP = GameObject.Find("RightHandIndex1");
		RIFingerIIP = GameObject.Find("RightHandIndex2");
		RIFingerDIP = GameObject.Find("RightHandIndex3");

	} // END FUNCTION

	// Returns the rotation matrix for the fingers
	public float[,] GetRot () {
		Frame frame = controller.Frame();

		//Debug.Log (frame.Hands[0].IsValid);

		if (frame.IsValid) {
			if(frame.Hands[0].IsValid || frame.Hands[1].IsValid ){
				if(frame.Hands.Count > 1){
					if (frame.Hand(0).IsRight) {
						rightHandID = frame.Hands[0].Id;
						leftHandID  = frame.Hands[1].Id;
					}else{
						rightHandID = frame.Hands[1].Id;
						leftHandID  = frame.Hands[0].Id;
					} // END IF Hands.IsRight
				}else{
					if (frame.Hand(0).IsRight) {
						rightHandID = frame.Hands[0].Id;
					} else {
						leftHandID  = frame.Hands[0].Id;
					} // END IF Hand.IsRight
				} // END IF Hand.Count
			} // END IF Hand.IsValid

			//Debug.Log(rightHandID.ToString());

			if(!frame.Hand(rightHandID).IsValid){
				rightHandID = 0;
			}else if (!frame.Hand(leftHandID).IsValid){
				leftHandID  = 0;
			}

			//Debug.Log(rightHandID.ToString());

			//int i = 1;
			//int j = 1;

			// *** CALCULATE ROTATIONS *** //
//			for(int i = 0; i <= 4; i++){
//				for(int j = 1; j <= 3; j++){
//
//					Bone.BoneType type = (Bone.BoneType)j;
//
//					//Debug.Log(type);
//					Quaternion localRot = frame.Hand(rightHandID).Fingers[i].Bone(type).Basis.Rotation();
//					Quaternion handRot = frame.Hand (rightHandID).Basis.Rotation();
//					//Debug.Log (localRot);
//					//Debug.Log(frame.Hand(rightHandID).Fingers[1]);
//					// Maybe add transform here if coordinate frame is wrong
//
//					fingerRot[i,j] = (localRot.x - handRot.x) * 180/3.14159f;
//
//					if(j == 1){
//						fingerRot[i,j-1] = (localRot.y - handRot.y) * 180/3.14159f;;
//					} // END IF
//				} // END FOR Bone loop
//			}

//			for(int i = 0; i <= 4; i++){
//				for(int j = 0; j <= 3; j++){
//
//					Bone.BoneType type = (Bone.BoneType)j;
//
//					//Debug.Log(type);
//					Quaternion localRot = frame.Hand(rightHandID).Fingers[i].Bone(type).Basis.Rotation();
//					Quaternion handRot = frame.Hand (rightHandID).Basis.Rotation();
//					//Debug.Log (localRot);
//					//Debug.Log(frame.Hand(rightHandID).Fingers[1]);
//					// Maybe add transform here if coordinate frame is wrong
//
//
//					//fingerRot[i,j] = (localRot.x - handRot.x) * 180/3.14159f;
//
//					if(i == 0){
//						fingerRot[i,j] = (localRot.x - handRot.x) * 180/3.14159f;
//					}
//
//					if(j == 1){
//						fingerRot[i,j-1] = (localRot.y - handRot.y) * -180/3.14159f;;
//					} // END IF
//				} // END FOR Bone loop
//			}


			//Debug.Log(fingerRot[1,1].ToString());

//			fingerRot[0][1][0] = frame.hand(rightHandID).Direction().angleTo(controller.frame().hand(rightHandID).fingers().fingerType(Leap::Finger::TYPE_INDEX)[0].bone(Bone::Type::TYPE_PROXIMAL).direction()); // Metacarpophalangeal adduction/abduction
//			fingerRot[0][1][1] = frame.hand(rightHandID).palmNormal().angleTo(controller.frame().hand(rightHandID).fingers().fingerType(Leap::Finger::TYPE_INDEX)[0].bone(Bone::Type::TYPE_PROXIMAL).direction()); // Metacarpophalangeal flexion/extension
//			fingerRot[0][1][2] = frame.hand(rightHandID).fingers().fingerType()[0].bone().direction().angleTo(controller.frame().hand(rightHandID).fingers().fingerType(Leap::Finger::TYPE_INDEX)[0].bone(Bone::Type::TYPE_INTERMEDIATE).direction()); // Proximal interphalangeal flexion/extension
//			fingerRot[0][1][3] = frame.hand(rightHandID).fingers().fingerType()[0].bone().direction().angleTo(controller.frame().hand(rightHandID).fingers().fingerType(Leap::Finger::TYPE_INDEX)[0].bone(Bone::Type::TYPE_DISTAL).direction()); // Distal interphalangeal flexion/extension

			for(int k = 0; k <= 4; k++){
				fingerRot[k,1] = frame.Hand(rightHandID).Fingers[k].Bone((Bone.BoneType)1).Direction.AngleTo(frame.Hand(rightHandID).Fingers[k].Bone((Bone.BoneType)0).Direction) * 180/3.14159f;
				fingerRot[k,2] = frame.Hand(rightHandID).Fingers[k].Bone((Bone.BoneType)2).Direction.AngleTo(frame.Hand(rightHandID).Fingers[k].Bone((Bone.BoneType)1).Direction) * 180/3.14159f;
				fingerRot[k,3] = frame.Hand(rightHandID).Fingers[k].Bone((Bone.BoneType)3).Direction.AngleTo(frame.Hand(rightHandID).Fingers[k].Bone((Bone.BoneType)2).Direction) * 180/3.14159f;
				///fingerRot[k,0] = frame.Hand(rightHandID).Fingers[k].Bone((Bone.BoneType)1).Direction.AngleTo(frame.Hand(rightHandID).Direction) * 180/3.14159f;
				fingerRot[k,0] = (frame.Hand(rightHandID).Fingers[k].Bone((Bone.BoneType)1).Direction.Yaw - frame.Hand(rightHandID).Fingers[k].Bone((Bone.BoneType)0).Direction.Yaw) * -180/3.14159f;

			}




		} // END IF Frame.Hands.IsValid

		return fingerRot;

	} // END FUNCTION

//	public Quaternion GetQuaternion(int boneID){

//		Frame frame = controller.Frame();
//
//			if(frame.Hands[0].IsValid || frame.Hands[1].IsValid ){
//				if(frame.Hands.Count > 1){
//					if (frame.Hand(0).IsRight) {
//						rightHandID = frame.Hands[0].Id;
//						leftHandID  = frame.Hands[1].Id;
//					}else{
//						rightHandID = frame.Hands[1].Id;
//						leftHandID  = frame.Hands[0].Id;
//					} // END IF Hands.IsRight
//				}else{
//					if (frame.Hand(0).IsRight) {
//						rightHandID = frame.Hands[0].Id;
//					} else {
//						leftHandID  = frame.Hands[0].Id;
//					} // END IF Hand.IsRight
//				} // END IF Hand.Count
//			} // END IF Hand.IsValid
//			
//			//Debug.Log(rightHandID.ToString());
//			
//			if(!frame.Hand(rightHandID).IsValid){
//				rightHandID = 0;
//			}else if (!frame.Hand(leftHandID).IsValid){
//				leftHandID  = 0;
//			}
//
//
//
//
//			Bone.BoneType type = (Bone.BoneType)boneID;
//					
//			//Debug.Log(type);
//			Quaternion localRot = frame.Hand(rightHandID).Fingers[1].Bone(type).Basis.Rotation();
//
//		return RIFingerIIP.transform.rotation * localRot;

//	} // END FUNCTION

} // END CLASS
// EOF