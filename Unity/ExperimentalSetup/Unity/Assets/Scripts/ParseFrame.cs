/*******************************************************************************\
 * ParseFrame																 	*
 * 		This object parses and calculates the quaternions for each bone in the 	*
 * 		hand as well as the palm, arm, and hand positions and orientations.	 	*
 * 																			 	*
 * Author: Kevin O'Neill													 	*
 * Date: 2014.12.02													 		 	*
 * Version: Demo															 	*
 \******************************************************************************/

using UnityEngine;
using System.Collections;
using Leap;

public class ParseFrame : MonoBehaviour {

	protected const float GIZMO_SCALE = 5.0f;

	private int NUM_FINGERS = 5;
	private int NUM_BONES = 4;

	private Transform palm;
	private Transform foreArm;

	protected Hand hand;
	protected Finger[] finger_ = new Finger[5];
	protected bool mirror_z_axis_ = true;

	public Transform[] bones = new Transform[4];
	
	public Vector3 modelFingerPointing = Vector3.forward;
	public Vector3 modelPalmFacing = -Vector3.up;

	public bool isHeadMounted = true;
	
	public Vector3 handMovementScale = Vector3.one;

	public Quaternion[] fingerRot = new Quaternion[20]; // Thumb x4, Index x4, ...

	Frame currentFrame;

	Controller controller_;

	private GameObject RIFingerPIP;
		



	void OnDrawGizmos() {
		// Draws the little Leap Motion Controller in the Editor view.
		Gizmos.matrix = Matrix4x4.Scale(GIZMO_SCALE * Vector3.one);
		Gizmos.DrawIcon(transform.position, "leap_motion.png");
	}

	void Awake() {
		controller_ = new Controller();
		
		// Optimize for top-down tracking if on head mounted display.
		Controller.PolicyFlag policy_flags = controller_.PolicyFlags;
		if (isHeadMounted)
			policy_flags |= Controller.PolicyFlag.POLICY_OPTIMIZE_HMD;
		else
			policy_flags &= ~Controller.PolicyFlag.POLICY_OPTIMIZE_HMD;
		
		controller_.SetPolicyFlags(policy_flags);
	}

	// Use this for initialization
	void Start () {
		if (controller_ == null) {
			Debug.LogWarning(
				"Cannot connect to controller. Make sure you have Leap Motion v2.0+ installed");
		}
		RIFingerPIP = GameObject.Find("Bip01 R Finger1");
	}

	void Update(){
		hand = controller_.Frame().Hands[0];
		SetLeapHand ();

		Quaternion[] tmpFingers = UpdateFingers ();

		RIFingerPIP.transform.rotation = tmpFingers [6];
	}

	public void SetLeapHand(){
		if (hand != null){		
			for (int i = 0; i < NUM_FINGERS; ++i) {
				finger_[i] = hand.Fingers[i];
			}
		}
	}

	public Quaternion GetBoneRotation(int fingerNum, int bone_type) {
		Quaternion local_rotation =
			finger_[fingerNum].Bone((Bone.BoneType)(bone_type)).Basis.Rotation(mirror_z_axis_);
		return transform.rotation * local_rotation;
	}

	public Quaternion Reorientation() {
		return Quaternion.Inverse(Quaternion.LookRotation(modelFingerPointing, -modelPalmFacing));
	}

	public Quaternion[] UpdateFingers(){
		if (hand != null || finger_[0] == null) {
			for (int i = 0; i < NUM_FINGERS; ++i) {
				for (int j =0; j < NUM_BONES; ++j) {
					fingerRot [i*4 + j] = GetBoneRotation (i, j) * Reorientation ();
				}
			}
			return fingerRot;
		}

		Quaternion[] tmpQuat = new Quaternion[20];

		tmpQuat[6] = RIFingerPIP.transform.localRotation;

		return tmpQuat;
	}

	public Vector3 GetPalmOffset() {
		if (controller_ == null || hand == null)
			return Vector3.zero;
		
		Vector3 additional_movement = handMovementScale - Vector3.one;
		Vector3 scaled_palm_position = Vector3.Scale(additional_movement,
		                                             hand.PalmPosition.ToUnityScaled(mirror_z_axis_));
		
		return transform.TransformPoint(scaled_palm_position) - transform.position;
	}

	public Vector3 GetPalmPosition() {
		return transform.TransformPoint(hand.PalmPosition.ToUnityScaled(mirror_z_axis_)) + GetPalmOffset();
	}

	// Returns the palm direction of the hand in relation to the controller.
	public Vector3 GetPalmDirection() {
		return transform.TransformDirection(hand.Direction.ToUnity(mirror_z_axis_));
	}

	// Returns the palm rotation of the hand in relation to the controller.
	public Quaternion GetPalmRotation() {
		return transform.rotation * hand.Basis.Rotation(mirror_z_axis_);
	}

	// Returns the rotation quaternion of the arm in relation to the controller.
	public Quaternion GetArmRotation() {
		Quaternion local_rotation = hand.Arm.Basis.Rotation(mirror_z_axis_);
		return transform.rotation * local_rotation;
	}

	// Returns the lower arm elbow position in relation to the controller.
	public Vector3 GetElbowPosition() {
		Vector3 local_position = hand.Arm.ElbowPosition.ToUnityScaled(mirror_z_axis_);
		return transform.TransformPoint(local_position);
	}

	public void UpdateHand() {
		if (palm != null) {
			palm.position = GetPalmPosition();
			palm.rotation = GetPalmRotation() * Reorientation();
		}
		
		if (foreArm != null)
			foreArm.rotation = GetArmRotation();
	}

//	 // Update is called once per frame
//	void Update () {
//		currentFrame = leap_controller_.Frame();
//	}

//	public Quaternion GetBoneRotation(int bone_type) {
//		Quaternion local_rotation =
//			finger_.Bone((Bone.BoneType)(bone_type)).Basis.Rotation(mirror_z_axis_);
//		return controller_.transform.rotation * local_rotation;
//	}
}

// EOF