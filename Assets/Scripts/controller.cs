using UnityEngine;
using System;
using System.Collections;
using System.Net;
using System.IO;
using System.Net.Sockets;


public class controller : MonoBehaviour {

	// Parameters for connection 
	//public string name = "LocalHost";
	public string host = "127.0.0.1";
	public int port = 10253;

	// TODO: Remove this parameters
	// Parameter for movement speed 
	public float speed = 10.0F;


	// Connection variables
	private TcpClient socket; 
	private NetworkStream stream; 
	private StreamWriter writer; 
	private StreamReader reader;  // TODO: Why dont we use this variable?

	private bool socketReady = false;

	// Position and orientation
	float[] pos = new float[3];
	float[] ori = new float[4];


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


	void writeSocket (string message) {
		
		if (!socketReady)
			return;
		
		String tmpMessage = message; // + "\r\n";
		writer.Write (tmpMessage);
		writer.Flush ();

	}


	string readSocket () {

		String result = "";
		if (stream.DataAvailable) {
			Byte[] inStream = new Byte[socket.SendBufferSize];
			stream.Read (inStream, 0, inStream.Length);
			result += System.Text.Encoding.UTF8.GetString (inStream);
		}
		return result;
	}


	// Directly writes into the NetworkStream
	void writeSocketDirect (byte[] message, Int32 size) {

		if (!socketReady)
			return;

		// Old way of writing via StreamWriter
		/*
		String tmpMessage = message; // + "\r\n";
		writer.Write (tmpMessage);
		writer.Flush ();
		*/

		stream.Write (message, 0, size);
	}


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
		Cursor.lockState = CursorLockMode.Locked;
		setupSocket ();

	}
	
	// Update is called once per frame
	void Update () {

		float translation = Input.GetAxis ("Vertical") * speed;
		float straffe = Input.GetAxis ("Horizontal") * speed;

		translation *= Time.deltaTime;
		straffe *= Time.deltaTime;

		transform.Translate (straffe, 0, translation);

		if (Input.GetKeyDown ("escape"))
			Cursor.lockState = CursorLockMode.Confined;

		//transform.position = new Vector3 (0, 0, 0);
		//transform.rotation = new Quaternion (0, 0, 0, 1);

		/*
		if (socketReady) {
			// Write socket
			string message = "A";
			writeSocket (message);

			// Read socket
			message = "";
			message = readSocket ();

			if (message.Length > 0) {

				// Cut the first P letter
				message = message.Substring (1);

				// Cut anything else after a P
				int idx = message.LastIndexOf ("P");
				if (idx > 0)
					message = message.Substring (0, idx);

				Debug.Log ("Received: " + message);

				string[] poseString = message.Split (',');
				//Debug.Log ("Size :" + poseString.Length.ToString ());
				//Debug.Log ("Position: (" + poseString[0] + "," + poseString[1] + "," + poseString[2] + ")");
				//Debug.Log ("Orientation: (" + poseString[3] + "," + poseString[4] + "," + poseString[5] + "," + poseString[6] + ")");

				float.TryParse (poseString [0], out pos [0]);
				float.TryParse (poseString [2], out pos [1]);
				float.TryParse (poseString [3], out pos [2]);
				float.TryParse (poseString [3], out ori [0]);
				float.TryParse (poseString [4], out ori [1]);
				float.TryParse (poseString [5], out ori [2]);
				float.TryParse (poseString [6], out ori [3]);

			}
		}
		*/
	}

	// Lateupdate function
	void LateUpdate () {


		Camera mainCamera = GameObject.FindGameObjectWithTag ("MainCamera").GetComponent<Camera> ();

		RenderTexture rt = RenderTexture.active;
		RenderTexture.active = mainCamera.targetTexture;
		int width = mainCamera.targetTexture.width;
		int height = mainCamera.targetTexture.height;

		Texture2D texture = new Texture2D (width, height);
		texture.ReadPixels (new Rect (0, 0, texture.width, texture.height), 0, 0);
		texture.Apply ();

		byte[] textureBytes = texture.EncodeToJPG ();

		// Debug.Log ("(" + counter++ + ") "+  "Width: " + width + " - Height: " + height + " -- Size: " + textureBytes.Length.ToString ());

		// File.WriteAllBytes ("screen.jpg", textureBytes);


		if (socketReady) {



			// Write the message size in the beginning of the message

			/*
			// Get the message into a C# string
			string message;
			message = System.Text.Encoding.Default.GetString (textureBytes);
			string messageSize;
			messageSize = message.Length.ToString () + "\0";
			writeSocket (messageSize);
			writeSocket (message);
			*/

			// Direct write
			string messageSize;
			messageSize = textureBytes.Length.ToString () + '\0';
			writeSocket (messageSize);
			writeSocketDirect (textureBytes, textureBytes.Length);

			//socketReady = false;
		}

		Destroy (texture);
	}

	void OnGUI (){

		if (socketReady)
			GUI.Label (new Rect (0, 0, 500, 500), "Connected!");
		else
			GUI.Label (new Rect (0, 0, 500, 500), "No Connection...");

		string position = "Position: (" + pos [0].ToString () + "," + pos [1].ToString () + "," + pos [2].ToString () + ")";
		GUI.Label (new Rect (0, 20, 500, 500), position);

		string orientation = "Orientation: (" + ori [0].ToString () + "," + ori [1].ToString () + "," + ori [2].ToString () + "," + ori [3].ToString() + ")";
		GUI.Label (new Rect (0, 40, 500, 500), orientation);
	}
}
