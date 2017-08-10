
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

// ZMQ/LCM
using NetMQ;

// Include the fastest version of Snappy available for the arch
using Snappy;
using Snappy.Sharp;


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
    public string pose_host = "tcp://192.168.0.102:10253";
    public string video_host = "tcp://192.168.0.102:10254";
    public bool DEBUG = true;
    public int num_cameras = 3;
    public int width = 1024;
    public int height = 768;
    public int max_framerate = 80;
    public bool should_compress_video = true;

    public GameObject camera_template;

    // instance vars
    private NetMQ.Sockets.SubscriberSocket pull_socket;
    private NetMQ.Sockets.PublisherSocket push_socket;
    private int camera_frame_length = 8;
    private int frame_headers = 2;
    private long timestamp = 0;
    private Dictionary<string, CameraObj> camera_objects;
    private Texture2D rendered_frame;
    private object socket_lock;
    private bool system_initialized;

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

    private void ensureScreenSize()
    {
        if (!system_initialized)
        {
            // Set the max framerate
            Application.targetFrameRate = max_framerate;
            // Set number of cameras
            num_cameras = camera_objects.Count;
            // initialize the display to a window that fits all cameras
            Screen.SetResolution(width * num_cameras, height, false);
            // Set render texture to the correct size
            rendered_frame = new Texture2D(width * num_cameras, height, TextureFormat.RGB24, false, true);
            // Make sure that all cameras are drawing to the correct portion of the screen.

            // Do not run this function again since we are initialized
            system_initialized = true;
        }
    }

    public IEnumerator Start()
    {
        // Check if the program should use CLI arguments (with defaults)
        if (!Application.isEditor)
        {
            pose_host = GetArg("-pose-host", pose_host);
            video_host = GetArg("-video-host", video_host);
            width = int.Parse(GetArg("-width", width.ToString()));
            height = int.Parse(GetArg("-height", height.ToString()));
            max_framerate = int.Parse(GetArg("-max-framerate", max_framerate.ToString()));
            should_compress_video = bool.Parse(GetArg("-should-compress-video", (!video_host.Contains("localhost")).ToString()));
        }


        // Fixes for Unity/NetMQ conflict stupidity.
        AsyncIO.ForceDotNet.Force();
        socket_lock = new object();


        // Connect sockets
        Debug.Log("Creating sockets.");
        pull_socket = new NetMQ.Sockets.SubscriberSocket();
        //pull_socket.Options.ReceiveBuffer = 1000;
        pull_socket.Options.ReceiveHighWatermark = 90;
        pull_socket.Connect(pose_host);
        pull_socket.Subscribe("Pose");

        push_socket = new NetMQ.Sockets.PublisherSocket();
        ////push_socket.Options.SendHighWatermark = 10;
        push_socket.Connect(video_host);
        
        Debug.Log("Sockets bound.");

        // Initialize Objects
        camera_objects = new Dictionary<string, CameraObj>() { };




        // Wait until end of frame to transmit images
        while (true)
        {
            // Wait until all rendering + UI is done.
            // Blocks until the frame is rendered.
            yield return new WaitForEndOfFrame();


            // Read pixels from the display.
            rendered_frame.ReadPixels(new Rect(0, 0, width * num_cameras, height), 0, 0);

            //Color32[] raw_colors = rendered_frame.GetPixels32();
            rendered_frame.Apply();
            byte[] raw = rendered_frame.GetRawTextureData();


            // Compress and send the image in a different thread (if using windows).
            Task.Run(() =>
            {
                //byte[] rgba = Color32ArrayToByteArray(raw_colors);
                // Reorder the byte array such that the array is first all, R, then B, then G.
                // This is probably easier to compress.
                int raw_length = raw.Length;
                int channel_length = raw_length / 3;


                // Get the grayscale image, flipping the rows such that the image is transmitted rightside up
                byte[] raw_flipped_r = new byte[channel_length];
                int row_length_px = width * num_cameras;
                for (int y = 0; y < height; y++)
                {
                    int y_inv = height - y - 1;
                    for (int x = 0; x < row_length_px; x++)
                    {
                        raw_flipped_r[(y_inv * row_length_px) + x] = raw[(y * row_length_px) * 3 + (x * 3)];
                    }
                }

                // Create the image data holder
                byte[] image;
                if (should_compress_video) {

                    // Compress the image
#if (UNITY_EDITOR_WINDOWS || UNITY_STANDALONE_WINDOWS)
                    image =  SnappyCodec.Compress(raw_flipped_r);
#else
                    image = Snappy.Sharp.Snappy.Compress(raw_flipped_r);
#endif
                } else
                {
                    image = raw_flipped_r;
                }
                var msg = new NetMQMessage();
                /* Message format:
                    * timestamp
                    * is_compressed?
                    * image_data
                    * <end foreach>
                    * */
                msg.Append(timestamp);
                msg.Append(Convert.ToInt16(should_compress_video));
                msg.Append(image);
                lock (socket_lock)
                {
                    push_socket.TrySendMultipartMessage(msg);
                }
                //bot_core.image_t msg = new bot_core.image_t();
            }
            );

        }
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
"Pose"
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

        // Receive most recent message
        var msg = new NetMQMessage();
        var new_msg = new NetMQMessage();
        // Blocking receive for a message
        msg = pull_socket.ReceiveMultipartMessage();
        // Check if this is the latest message
        while (pull_socket.TryReceiveMultipartMessage(ref new_msg));

        if (new_msg.FrameCount >= msg.FrameCount)
        {
            msg = new_msg;
        }

        //msg = pull_socket.ReceiveMultipartMessage();
        Debug.LogFormat(" Received ZMQ message with {0} frames!", msg.FrameCount);
        // Check for errors
        if (msg.FrameCount == 0)
        {
            return;
        }


        // Split message into camera objects
        // Decode timestamp
        long new_timestamp = long.Parse(msg[1].ConvertToString());
        // sanity check the timestamp
        if (!(new_timestamp > timestamp) && !DEBUG)
        {
            // Skip this message
            return;
        }
        // Update the timestamp
        timestamp = new_timestamp;
        // sanity check the message
        int zmq_num_cameras = (msg.FrameCount - frame_headers) / camera_frame_length;
        Debug.AssertFormat(zmq_num_cameras == num_cameras, "Number of cameras in ZMQ message {0} does not match Unity settings.", zmq_num_cameras);

        // split the message into batches of 8 (eg. for each camera).
        for (int i = frame_headers; i < msg.FrameCount; i += camera_frame_length)
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

                int render_index = ((i - frame_headers) / camera_frame_length); // 0,1,2...
                // Setup camera's position on screen.
                camera_obj.GetComponent<Camera>().pixelRect = new Rect(width * render_index, 0, width, height);

                // enable Camera.
                camera_obj.SetActive(true);
                // Add the new camera to our dictionary of gameobjects
                camera_objects.Add(ID, new CameraObj() { ID = ID, obj = camera_obj, render_order = render_index });
            }


        };

        // After decoding the packet, initialize the screen size and render textures (if necessary)
        ensureScreenSize();


    }

}
