using System.Collections;
using System.Collections.Generic;
using UnityEngine;

// Libraries to catch exceptions
using System;

// Libraries for TCP
using System.Net;
using System.IO;
using System.Net.Sockets;


// Establish a TCP connection to a pose server; get pose.
public class receivePose : MonoBehaviour {

	public string host = "127.0.0.1";
	public int port = 10253;

	// Private variables for TCP connection
	private TcpClient socket;
	private NetworkStream stream;
	private StreamWriter writer;
	private StreamReader reader;
	private bool socketReady = false;
	private int poseN = 0;


    private Transform candidate_trans;





       // Last time and pose
       [HideInInspector]
	public ulong time = 0;
	[HideInInspector]
	public float[] pos = new float[3];
	[HideInInspector]
	public float[] ori = new float[4];

	[HideInInspector]
	public ulong time_receive_pose;

	ulong GetDateTimeInMicroseconds()
	{
		DateTime datetime = DateTime.Now;
		return (ulong)Math.Floor(datetime.Ticks / (double)10.0);
	}

	// Try to initiate connection
	void setupSocket () {
		
		try {
			socket = new TcpClient (host, port);
			stream = socket.GetStream ();
			writer = new StreamWriter (stream);
			reader = new StreamReader (stream);
			socketReady = true;
		} 
		catch (Exception e) {
			Debug.Log ("Socket error: " + e);
		}
	}

	// Write string on socket 
	void writeSocket (string message) {

		if (!socketReady)
			return;

		writer.Write (message);
		writer.Flush ();
	}

	//Read string from socket
	string readSocket () {

		string result = "";
		if (stream.DataAvailable) {
			byte[] inStream = new byte[socket.ReceiveBufferSize];
			stream.Read (inStream, 0, inStream.Length);
			result += System.Text.Encoding.UTF8.GetString (inStream);

			// Cut anything after a new line
			// TODO: This is not the best place to cut the new line.
			//       probably cut the new line after we receive the data.
			int idx = result.LastIndexOf ("\n");
			if (idx > 0)
				result = result.Substring (0, idx);
		}
		return result;
	}

	// Close connection
	// TODO: Actually run this function before quiting
	void closeSocket () {

		if (!socketReady)
			return;
		writer.Close ();
		reader.Close ();
		socket.Close ();
		socketReady = false;
	}


	// Use this for initialization
	void Start () {

        // Create our new candidate position
        candidate_trans = new GameObject().transform;

        time = 0;

/*		pos [0] = 0.0f;
		pos [1] = 0.0f;
		pos [2] = 0.0f;

		ori [0] = 1.0f;
		ori [1] = 0.0f;
		ori [2] = 0.0f;
		ori [3] = 0.0f;
        */
		setupSocket ();
	}
	
	// Update is called once per frame
	void Update () {
	
		if (socketReady) {

			time_receive_pose = GetDateTimeInMicroseconds ();

			// Write 'A' to socket to ask for the current pose
			string message = "A";
			writeSocket (message);

			// Read current pose from the socket 
			message = "";
			message = readSocket ();


			if (message.Length > 0) { // TODO: Appropriate checks to make sure that we have a pose message

				/*
				for (int i = 0; i < 15; i++) {
					Debug.Log ("str[" + i.ToString () + "]: " + message [i].GetHashCode ().ToString () + "\n");
					if (message [i] != '\0')
						Debug.Log ("str[" + i.ToString () + "] value : " + message [i] + "\n");
				}
				Debug.Log ("str[55]: " + message[55].GetHashCode().ToString() + "\n");

				Debug.Log ("Received: " + message + "\n");
				Debug.Log ("Received length: " + message.Length + "\n");
				*/

				// Cut the first P letter
				message = message.Substring (20);


				/*
				// Here is the current message
				Debug.Log ("Received cut: " + message + "\n");
				Debug.Log ("Received length: " + message.Length + "\n");
				*/ 

				string[] poseString = message.Split (',');

				/*
				// Let's see what we received and parsed
				Debug.Log ("Number of components: " + poseString.Length.ToString () + "\n");
				Debug.Log ("Received Time: " + poseString[0] + "\n");
				Debug.Log ("Received Position: (" + poseString[1] + "," + poseString[2] + "," + poseString[3] + ")\n");
				Debug.Log ("Received Orientation: (" + poseString[4] + "," + poseString[5] + "," + poseString[6] + "," + poseString[7] + ")\n");
				*/

				// Parse string into integer and floats
				ulong.TryParse (poseString [0], out time);
				float.TryParse (poseString [1], out pos [0]);
				float.TryParse (poseString [2], out pos [1]);
				float.TryParse (poseString [3], out pos [2]);
				float.TryParse (poseString [4], out ori [0]);
				float.TryParse (poseString [5], out ori [1]);
				float.TryParse (poseString [6], out ori [2]);
				float.TryParse (poseString [7], out ori [3]);






                // Set the transform to its new value
                //transform.position = new Vector3(pos[0], pos[1], pos[2]);
                //transform.rotation = new Quaternion(ori[0], ori[1], ori[2], ori[3]);
                
                candidate_trans.position = new Vector3(pos[0], pos[1], pos[2]);
                candidate_trans.rotation = new Quaternion(ori[0], ori[1], ori[2], ori[3]);
                // Rotate about the local X axis to face in the correct dir.
                candidate_trans.RotateAround(Vector3.zero, Vector3.right, -90);
                // Rotate along the Y axis to face in the correct dir."
                candidate_trans.RotateAround(Vector3.zero, Vector3.up, -90);

                // Fix fact that camera is facing backwards
                candidate_trans.Rotate(0, 180, 0);

                //transform.Rotate(Vector3.up, -90, Space.World);

                // Do error checking by checking if the current transform is crazy different
                float angle_deg_error = Quaternion.Angle(transform.rotation, candidate_trans.rotation);
                if (poseN == 0)
                {
                    // First pose appears to be gibberish so skip it
                    /// @todo Why is it gibberish??
                    angle_deg_error = 1000.0f; // Set stupidly high value to skip
                    ++poseN;
                }
                else if (poseN == 1)
                {
                    // We actually want to keep the second pose
                    angle_deg_error = 0.0f;
                    ++poseN;
                }
                // Ignore this update if the angle error is more than 20 deg
                if (Math.Abs(angle_deg_error) <= 20.0 )
                {
                    transform.position = candidate_trans.position;
                    transform.rotation = candidate_trans.rotation;
                    Debug.Log("Updated position \n");
                } else
                {
                    Debug.Log("Threw away a bad position!! \n");
                }

         

            }
		}
	}

	void OnGUI (){

		/*
		if (socketReady)
			GUI.Label (new Rect (0, 0, 500, 500), "Connected!");
		else
			GUI.Label (new Rect (0, 0, 500, 500), "No Connection...");

		string posetime = "Time: " + time;
		GUI.Label (new Rect (0, 20, 500, 500), posetime);

		string position = "Position: (" + pos [0].ToString () + "," + pos [1].ToString () + "," + pos [2].ToString () + ")";
		GUI.Label (new Rect (0, 40, 500, 500), position);

		string orientation = "Orientation: (" + ori [0].ToString () + "," + ori [1].ToString () + "," + ori [2].ToString () + "," + ori [3].ToString() + ")";
		GUI.Label (new Rect (0, 60, 500, 500), orientation);
		*/
	}

}
