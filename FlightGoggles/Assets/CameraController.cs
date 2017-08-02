
using System;
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
using Snappy;

// Camera class for decoding the ZMQ messages.
class CameraObj
{
    public string ID { get; set; }
    public int render_order { get; set; }
    public GameObject obj { get; set; }
}

public class CameraController : MonoBehaviour
{
	// Parameters
	public string pose_host = ">tcp://192.168.0.103:10253";
	public string video_host = ">tcp://192.168.0.103:10254";
	public bool DEBUG = true;
	public int num_cameras = 3;
	public int width = 1024;
	public int height = 768;
    public int max_framerate = 80;

    public GameObject camera_template;

	// instance vars
	private NetMQ.Sockets.PullSocket pull_socket;
	private NetMQ.Sockets.PushSocket push_socket;
	private int camera_frame_length = 8;
	private long timestamp = 0;
	private Dictionary<string, CameraObj> camera_objects;
	private Texture2D rendered_frame;

    // Helper function for getting command line arguments
    private static string GetArg(string name, string default_return)
    {
        var args = System.Environment.GetCommandLineArgs();
        for (int i = 0; i < args.Length; i++)
        {
            if (args[i] == name && args.Length > i + 1)
            {
                return args[i + 1];
            }
        }
        return default_return;
    }

    /* Use this for initialization
     * Should create a worker thread that listens for ZMQ messages and saves the most recent one.
     */

    public IEnumerator Start()
	{
        // Check if the program should use CLI arguments (with defaults)
        if (! Application.isEditor)
        {
            pose_host = GetArg("-pose-host", pose_host);
            video_host = GetArg("-video-host", video_host);
            width = int.Parse(GetArg("-screen-width", width.ToString()));
            height = int.Parse(GetArg("-screen-height", height.ToString()));
            max_framerate = int.Parse(GetArg("-max-framerate", max_framerate.ToString()));
        }

        // Set the max framerate
        Application.targetFrameRate = max_framerate;

        // Fixes for Unity/NetMQ conflict stupidity.
        AsyncIO.ForceDotNet.Force();


        // Connect sockets
        Debug.Log("Creating sockets.");
		pull_socket = new NetMQ.Sockets.PullSocket (pose_host);
		push_socket = new NetMQ.Sockets.PushSocket (video_host);
		Debug.Log("Sockets bound.");

		// Initialize Objects
		camera_objects = new Dictionary<string, CameraObj> (){};

		// initialize the display to fit all cameras
		Display.main.SetRenderingResolution(width*num_cameras, height);
		rendered_frame = new Texture2D(width*num_cameras, height, TextureFormat.RGB24, false, true);




		// Wait until end of frame to transmit images
		while (true){
			// Wait until all rendering + UI is done.
			// Blocks until the frame is rendered.
			yield return new WaitForEndOfFrame();

     
            // Read pixels from the display.
            rendered_frame.ReadPixels(new Rect(0,0, width * num_cameras, height), 0, 0);

            //Color32[] raw_colors = rendered_frame.GetPixels32();
            rendered_frame.Apply();
            byte[] raw = rendered_frame.GetRawTextureData();

            //Debug.Log(raw[1030]);

            // Compress and send the image in a different thread.
            Task.Run( () =>
            {
                //byte[] rgba = Color32ArrayToByteArray(raw_colors);
                // Reorder the byte array such that the array is first all, R, then B, then G.
                // This is probably easier to compress.
                int raw_length = raw.Length;
                int channel_length = raw_length / 3;


                //byte[] reordered_raw = new byte[raw_length];
                //for (int c = 0; c < 3; c++)
                //{
                //    for (int i = 0; i < channel_length; i++)
                //    {
                //        reordered_raw[c * channel_length + i] = raw[c + i * 3];
                //    }
                //}


                // Get the grayscale image, flipping the rows such that the image is transmitted rightside up
                byte[] raw_flipped_r = new byte[channel_length];
                int row_length_px = width * num_cameras;
                for (int y = 0; y < height; y++)
                {
                    int y_inv = height - y - 1;
                    for (int x = 0; x < row_length_px; x++)
                    {
                        raw_flipped_r[(y_inv*row_length_px) + x] = raw[(y*row_length_px)*3 + (x*3)];
                    }
                }



                byte[] compressed = SnappyCodec.Compress(raw_flipped_r);

                //byte[] compressed = compressor.Compress(raw, width * num_cameras * 3, width * num_cameras, height, TJPF_RGB, TJSubsamplingOptions.TJSAMP_444, 50);
                //Debug.Log(compressed[1020]);
                var msg = new NetMQMessage();
                /* Message format:
                    * timestamp
                    * compressed_image_data
                    * <end foreach>
                    * */
                msg.Append(timestamp);
                msg.Append(compressed);
                push_socket.TrySendMultipartMessage(msg);

            }
            );

        }
		yield return null;
	}

    private void OnApplicationQuit()
    {
        // Close ZMQ sockets
        pull_socket.Close();
        push_socket.Close();
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

