using UnityEngine;
using System;
using System.Collections;
using System.IO;
using System.Net;
using System.Net.Sockets;

public class renderToLCM : MonoBehaviour {

	// Parameters for connection 
	public string host = "192.168.2.2";
	public int port = 10254;
    public string debugRenderLocation = "C:\\Users\\Icarus\\Desktop\\debugRenders\\";
    public int JPGCompressionLevel = 75;
    public int maxFrameRate = 25;

    // Connection variables
    private TcpClient socket; 
	private NetworkStream stream; 
	private StreamWriter writer; 
	private StreamReader reader;  // TODO: Why dont we use this variable?
	private bool socketReady = false;

	private int counter = 0; // Frame counter
    

	ulong GetDateTimeInMicroseconds()
	{
		DateTime datetime = DateTime.Now;
		return (ulong)Math.Floor(datetime.Ticks / (double)10.0);
	}

    // Try to initiate connection
    bool setupSocket()
    {

        try
        {
            socket = new TcpClient(host, port);
            stream = socket.GetStream();
            writer = new StreamWriter(stream);
            reader = new StreamReader(stream);
            socketReady = true;
        }
        catch (Exception e)
        {
            Debug.Log("Socket error: " + e);
            closeSocket();
            return false;
        }
        return true;
    }

    // Write string on socket 
    bool writeSocket(string message)
    {

        if (!socketReady)
            return false;

        try
        {
            writer.Write(message);
            writer.Flush();

        }
        catch (Exception e)
        {
            Debug.Log("Socket error: " + e);
            closeSocket();
            return false;
        }
        return true;

    }

    // Writes bytes directly into the NetworkStream
    bool writeSocketDirect(byte[] message, Int32 size)
    {

        if (!socketReady)
            return false;;
        try
        {
            stream.Write(message, 0, size);
        } catch (Exception e)
        {
            Debug.Log("Socket error: " + e);
            closeSocket();
            return false;
        }
        return true;
    }

    //Read string from socket
    string readSocket()
    {

        string result = "";
        if (stream.DataAvailable)
        {
            byte[] inStream = new byte[socket.ReceiveBufferSize];
            stream.Read(inStream, 0, inStream.Length);
            result += System.Text.Encoding.UTF8.GetString(inStream);

            // Cut anything after a new line
            // TODO: This is not the best place to cut the new line.
            //       probably cut the new line after we receive the data.
            int idx = result.LastIndexOf("\n");
            if (idx > 0)
                result = result.Substring(0, idx);
        }
        return result;
    }

    // Close connection
    // TODO: Actually run this function before quiting
    void closeSocket()
    {

        writer.Close();
        reader.Close();
        socket.Close();
        socketReady = false;
    }

    // Dumps Render Texture into a Texture2D
    public static Texture2D DumpRenderTexture(RenderTexture rt)
    {
        var oldRT = RenderTexture.active;

        var tex = new Texture2D(rt.width, rt.height);
        RenderTexture.active = rt;
        tex.ReadPixels(new Rect(0, 0, rt.width, rt.height), 0, 0);
        tex.Apply();

        RenderTexture.active = oldRT;
        return tex;
    }

    void Awake()
    {
        QualitySettings.vSyncCount = 0;  // VSync must be disabled
        Application.targetFrameRate = maxFrameRate;
    }

    // Use this for initialization
    void Start () {
        // Setup connection.
        setupSocket();
	}

    // Called after all rendering is done.
    void OnRenderImage(RenderTexture src, RenderTexture dest) {


		ulong time_start = GetDateTimeInMicroseconds ();


		// Find the camera that this script is attached to
		Camera thisCamera = this.GetComponent<Camera> ();
		if (!thisCamera)
			Debug.Log ("ERROR: Null camera. Please make sure this camera actually exists.\n");

        // Get the render as a Texture2D
        Texture2D tex = DumpRenderTexture(src);

        // Get current timestamp
        ulong time = thisCamera.GetComponent<receivePose> ().time;

		// Get the width and height of this texture
		int width = tex.width;
		int height = tex.height;
	
		//Debug.Log ("Compressing the image...\n");
		// Encode the texture to JPG
		ulong time_start_jpeg_compression = GetDateTimeInMicroseconds ();
		byte[] textureBytes = tex.EncodeToJPG (JPGCompressionLevel);
		ulong time_finish_jpeg_compression = GetDateTimeInMicroseconds ();
		ulong time_elapsed_jpeg_compression = time_finish_jpeg_compression - time_start_jpeg_compression;
		//Debug.Log ("Time elapsed JPEG compression: " + time_elapsed_jpeg_compression.ToString() + "\n");
			

		// Increment the counter
		counter++;

		// Report out the frame number, width, height, and size
		Debug.Log ("Image(" + counter + ") "+  "; Image Size: " + textureBytes.Length.ToString () + "\n");

        // Pretend to be a shader by passing our image along
        Graphics.Blit(src, dest);


		// Send JPG through the socket connection
		if (socketReady) { // Check whether the socket is ready

			//Debug.Log ("Sending the image...\n");


			// Write the timetstamp to socket
			string timestamp_str;
			timestamp_str = time.ToString() + '\0';
			writeSocket (timestamp_str);

			// Write the length of the image to socket
			string imageSize_str;
			imageSize_str = textureBytes.Length.ToString () + '\0';
			writeSocket (imageSize_str); // Attach the size to the beginning of the message

			// Write the image to the socket
			writeSocketDirect (textureBytes, textureBytes.Length); // Place the image to the rest of the message

			//Debug.Log ("Sent! Size: " + textureBytes.Length.ToString () + "\n");

			//TODO: Send the timestamp of the pose that was used for rendering this scene

			ulong time_finish = GetDateTimeInMicroseconds ();
			ulong time_elapsed = time_finish - time_start;

			//Debug.Log ("Time elapsed transmit: " + time_elapsed.ToString () + "\n");


			ulong time_receive_pose = thisCamera.GetComponent<receivePose> ().time_receive_pose;

			ulong time_elapsed_total = time_finish - time_receive_pose;
			//Debug.Log ("Time elapsed since pose acquision: " + time_elapsed_total.ToString () + "\n");


			//socketReady = false;
		}

		// Write the JPG into a file
		//File.WriteAllBytes (debugRenderLocation + counter.ToString() + ".jpg", textureBytes);

		Destroy (tex);
	}
}
