
using System.IO;
using System.IO.Compression;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using System.Runtime.InteropServices;
using System.Threading;
using System.Threading.Tasks;

// Array ops
using System.Linq;

/*
 * Camera Controller
 * Listens for ZMQ camera poses on a separate thread and creates/moves cameras to match.
 *
 */

// ZMQ
using NetMQ;
//using ImageSharp;

// Camera class for decoding the ZMQ messages.
class CameraObj
{
	public string ID { get; set; }
	public int render_order { get; set; }
//	public Vector3 position { get; set; }
//	public Quaternion rotation { get; set; }
	//public long timestamp { get; set; }
	public GameObject obj { get; set; }

//	public string ToString(){
//		return string.Format("Camera: [{0}; {1}; {2}; {3}]", ID, timestamp, position.ToString(), rotation.ToString());
//	}
}

public class CameraController : MonoBehaviour
{
	// Parameters
	public string pose_host = "@tcp://localhost:10253";
	public string video_host = "@tcp://localhost:10254";
	public bool DEBUG = true;
	public int num_cameras = 3;
	public int width = 1024;
	public int height = 768;

	public GameObject camera_template;

	// instance vars
	private NetMQ.Sockets.PullSocket pull_socket;
	//private NetMQ.Sockets.PushSocket push_socket;
	private int camera_frame_length = 8;
	private long timestamp = 0;
	private Dictionary<string, CameraObj> camera_objects;
	private Texture2D rendered_frame;

    /// <summary>
    /// Compresses byte array to new byte array.
    /// </summary>
    public static byte[] Compress(byte[] raw)
    {
        using (MemoryStream memory = new MemoryStream())
        {
            using (GZipStream gzip = new GZipStream(memory,
                CompressionMode.Compress, true))
            {
                gzip.Write(raw, 0, raw.Length);
            }
            return memory.ToArray();
        }
    }



/* Use this for initialization
 * Should create a worker thread that listens for ZMQ messages and saves the most recent one.
 */

public IEnumerator Start()
	{
        // Fixes for Unity/NetMQ conflict stupidity.
        AsyncIO.ForceDotNet.Force();


        // Connect sockets
        Debug.Log("Creating sockets.");
		pull_socket = new NetMQ.Sockets.PullSocket (pose_host);
		//push_socket = new NetMQ.Sockets.PushSocket (video_host);
		Debug.Log("Sockets bound.");

		// Initialize Objects
		camera_objects = new Dictionary<string, CameraObj> (){};

		// initialize the display to fit all cameras
		Display.main.SetRenderingResolution(width*num_cameras, height);
		rendered_frame = new Texture2D(width*num_cameras, height, TextureFormat.RGBA32, false);



		// Wait until end of frame to transmit images
		while (true){
			// Wait until all rendering + UI is done.
			// Blocks until the frame is rendered.
			yield return new WaitForEndOfFrame();
     
            // Read pixels from the display.
            rendered_frame.ReadPixels(new Rect(0,0,1,1), 0, 0);
            rendered_frame.Apply();

            byte[] raw = rendered_frame.GetRawTextureData();

            // Spawn a new thread
            new Thread(() =>
            {
                byte[] gzipped = Compress(raw);
                Debug.LogFormat("Frame length {0}", gzipped.Length);
            }).Start();
            // Compress the image.
            //Image<Rgba32> image = Image<Rgba32>.Load<Rgba32>(rendered_frame.GetRawTextureData());

            //Color32[] ColorBuffer = Display.main.colorBuffer.GetNativeRenderBufferPtr();

            //Debug.LogFormat ("Got pointer to buffer of size {0}", ColorBuffer.Length);

            ////			// Transmit new message back to LCM
            //			var msg = new NetMQMessage();
            ////
            ////			/* Message format:
            ////			 * timestamp
            ////			 * <foreach camera>
            ////			 * camera_id
            ////			 * image_data
            ////			 * <end foreach>
            ////			 * */
            //			msg.Append (timestamp);
            ////
            ////			// Add the camera images to the message.
            //			foreach(KeyValuePair<string, CameraObj> cam in camera_objects) {
            //				msg.Append (cam.Value.ID);
            //				// Encode the texture to JPG
            //				byte[] textureBytes = cam.Value.obj.GetComponent<renderToMemory> ().last_render.EncodeToJPG (50);
            //
            //				msg.Append (textureBytes);
            //			}
            ////
            ////			// try to send, but drop the message if unsuccessful (don't block).
            //			push_socket.TrySendMultipartMessage(msg);
            //			Debug.Log ("Sent Frame to LCM!");
            //

        }
		yield return null;
	}

    private void OnApplicationQuit()
    {
        // Close ZMQ sockets
        pull_socket.Close();
        Debug.Log("Terminated ZMQ sockets.");
        NetMQConfig.Cleanup();
    }




    /* 
	 * Update is called once per frame
	 * Take the most recent ZMQ message and use it to position the cameras.
	 * If there has not been a recent message, the renderer should probably pause rendering until a new request is received. 

Pose Message format for ZMQ
-- ZMQ Message start
timestamp
[for each camera]
Camera ID string.
pose[0]
pose[1]
pose[2]
quat[0]
quat[1]
quat[2]
quat[3]
[end for each camera]
-- ZMQ Message end.

	*/

    void Update()
    {
        //Debug.Log ("Entering Update");

        // Receive message
        var msg = new NetMQMessage();
        msg = pull_socket.ReceiveMultipartMessage();
            Debug.LogFormat (" Received ZMQ message with {0} frames!", msg.FrameCount);

            // Split message into camera objects
            // Decode timestamp
            long new_timestamp = long.Parse(msg[0].ConvertToString());
            // sanity check the timestamp
            if (!(new_timestamp > timestamp) && !DEBUG)
            {
                // Skip this message
                return;
            }
            // Update the timestamp
            timestamp = new_timestamp;
            // sanity check the message
            int zmq_num_cameras = (msg.FrameCount - 1) / 8;
            Debug.AssertFormat(zmq_num_cameras == num_cameras, "Number of cameras in ZMQ message {0} does not match Unity settings.", zmq_num_cameras);

            // split the message into batches of 8 (eg. for each camera).
            for (int i = 1; i < msg.FrameCount; i += camera_frame_length)
            {

                // Decode the camera message
                string ID = msg[i].ConvertToString();
                Vector3 position = new Vector3(
                                        float.Parse(msg[i + 1].ConvertToString()),
                                        float.Parse(msg[i + 2].ConvertToString()),
                                        float.Parse(msg[i + 3].ConvertToString()));
                Quaternion rotation = new Quaternion(
                    float.Parse(msg[i + 4].ConvertToString()),
                    float.Parse(msg[i + 5].ConvertToString()),
                    float.Parse(msg[i + 6].ConvertToString()),
                    float.Parse(msg[i + 7].ConvertToString()));

                // Check if this camera exists.
                if (camera_objects.ContainsKey(ID))
                {
                    // transform the gameobject
                    GameObject camera_obj = camera_objects[ID].obj;
                    camera_obj.transform.SetPositionAndRotation(position, rotation);
                    // Update timestamp

                    // Create the object if it does not exist.
                }
                else
                {
                    GameObject camera_obj = Instantiate(camera_template, position, rotation);
                    // Set the game object name to the camera ID.
                    camera_obj.name = ID;
                    int render_order = ((i - 1) / 8);
                    // Setup camera's position on screen.
                    camera_obj.GetComponent<Camera>().pixelRect = new Rect(width * render_order, 0, width * (render_order + 1), height);

                    // enable Camera.
                    camera_obj.SetActive(true);
                    // Add the new camera to our dictionary of gameobjects
                    camera_objects.Add(ID, new CameraObj() { ID = ID, obj = camera_obj, render_order = render_order });
                }


            };

        
    }
				
}

