using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class cameraControl : MonoBehaviour {

	// Use this for initialization
	void Start () {
		
	}
	
	// Update is called once per frame
	void Update () {

		ulong time = GameObject.Find ("Drone").GetComponent<receivePose> ().time;
		float[] pos = GameObject.Find ("Drone").GetComponent<receivePose> ().pos;
		float[] ori = GameObject.Find ("Drone").GetComponent<receivePose> ().ori;

		// Let's tell the world what we are setting the position and orientation to
		string timeStr = "Time :" + time.ToString () + "\n";
		//Debug.Log (timeStr);
		string positionStr = "Position: (" + pos [0].ToString () + "," + pos [1].ToString () + "," + pos [2].ToString () + ")\n";
		//Debug.Log (positionStr);
		string orientationStr = "Orientation: (" + ori [0].ToString () + "," + ori [1].ToString () + "," + ori [2].ToString () + "," + ori [3].ToString() + ")\n";
		//Debug.Log (orientationStr);

		// Now we transform to the stupid left handed frame that unity uses (this is bad..)
		Vector3 positionNew = new Vector3 (pos[0], pos[1], -pos[2]);
		Quaternion orientationNew = new Quaternion (ori[0], ori[1], ori[2], ori[3]);
		Vector3 orientationEuler = new Vector3(0, 0, 0); // Get Euler angles
		orientationEuler = orientationNew.eulerAngles;
		Vector3 orientationSwap = new Vector3 (0, 0, 0); // Swap the angles, et cetera...
		orientationSwap [0] = -orientationEuler [2];
		orientationSwap [1] = orientationEuler [1];
		orientationSwap [2] = orientationEuler [0];
		Quaternion orientationTrans = new Quaternion (0, 0, 0, 1); // Get Quaternion from these euler angles
		orientationTrans.eulerAngles = orientationSwap;

		// Set the transform to its new value
		transform.position = positionNew;
		transform.rotation = orientationTrans;

	}
}
