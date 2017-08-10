
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
using LCM.LCM;

// LCM types
using bot_core;
using agile;


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
    public int cam_width = 1024;
    public int cam_height = 768;
    public int max_framerate = 80;
    public bool should_compress_video = true;
    public GameObject camera_template;

    // instance vars
    private NetMQ.Sockets.SubscriberSocket pull_socket;
    private NetMQ.Sockets.PublisherSocket push_socket;
    private int camera_frame_length = 8;
    private int frame_headers = 2;
    private int num_cameras = 3;
    private int rendered_image_width;
    private int rendered_image_height;
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
            // Calculate dimensions of rendered image <3H, W, C>
            rendered_image_width = cam_width;
            rendered_image_height = num_cameras * cam_height;
            // initialize the display to a window that fits all cameras
            Screen.SetResolution(rendered_image_width, rendered_image_height, false);
            // Set render texture to the correct size
            rendered_frame = new Texture2D(rendered_image_width, rendered_image_height, TextureFormat.RGB24, false, true);
            // Make sure that all cameras are drawing to the correct portion of the screen.
            foreach (KeyValuePair<string, CameraObj> entry in camera_objects)
            {
                GameObject camera_obj = entry.Value.obj;
                // Make sure camera renders to the correct portion of the screen.
                camera_obj.GetComponent<Camera>().pixelRect = new Rect(0, cam_width*entry.Value.render_order, cam_width, cam_height);
                // enable Camera.
                camera_obj.SetActive(true);
            }

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
            cam_width = int.Parse(GetArg("-cam-width", cam_width.ToString()));
            cam_height = int.Parse(GetArg("-cam-height", cam_height.ToString()));
            max_framerate = int.Parse(GetArg("-max-framerate", max_framerate.ToString()));
            should_compress_video = bool.Parse(GetArg("-should-compress-video", (!video_host.Contains("localhost")).ToString()));
        }


        // Fixes for Unity/NetMQ conflict stupidity.
        AsyncIO.ForceDotNet.Force();
        socket_lock = new object();


        // Connect sockets
        Debug.Log("Creating sockets.");
        pull_socket = new NetMQ.Sockets.SubscriberSocket();
        pull_socket.Options.ReceiveHighWatermark = 90;
        pull_socket.Connect(pose_host);
        pull_socket.Subscribe("Pose");

        push_socket = new NetMQ.Sockets.PublisherSocket();
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

            if (system_initialized)
            {

                // Read pixels from the display.
                rendered_frame.ReadPixels(new Rect(0, 0, rendered_image_width, rendered_image_height), 0, 0);
                rendered_frame.Apply();
                byte[] raw = rendered_frame.GetRawTextureData();


                // Compress and send the image in a different thread (if using windows).
                Task.Run(() =>
                {
                    const int channels = 3;

                    // Get the grayscale image, flipping the rows such that the image is transmitted rightside up
                    byte[] raw_flipped_r = new byte[raw.Length/channels];

                    for (int y = 0; y < rendered_image_height; y++)
                    {
                        int y_inv = rendered_image_height - y - 1;
                        for (int x = 0; x < rendered_image_width; x++)
                        {
                            raw_flipped_r[(y_inv * rendered_image_width) + x] = raw[((y * rendered_image_width) + x) * channels];
                        }
                    }

                    // Create the image data holder
                    byte[] image;
                    if (should_compress_video)
                    {

                        // Compress the image using the fastest system package available.
#if (UNITY_EDITOR_WINDOWS || UNITY_STANDALONE_WINDOWS)
                        image =  SnappyCodec.Compress(raw_flipped_r);
#else
                        image = Snappy.Sharp.Snappy.Compress(raw_flipped_r);
#endif
                    }
                    else
                    {
                        image = raw_flipped_r;
                    }

                    // Send the image back using NetMQ
                    SendNetMQImage(timestamp, ref image, should_compress_video);
                });

            }
        }
    }

    /* Message format:
    * timestamp
    * is_compressed?
    * image_data
    * <end foreach>
    * */
    private void SendNetMQImage(long utime, ref byte[] im, bool is_compressed)
    {
        var msg = new NetMQMessage();
        msg.Append(utime);
        msg.Append(Convert.ToInt16(is_compressed));
        msg.Append(im);
        lock (socket_lock)
        {
            push_socket.TrySendMultipartMessage(msg);
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
            }
            else
            {
                // Instatiate the object
                GameObject camera_obj = Instantiate(camera_template, position, rotation);
                // Set the game object name to the camera ID.
                camera_obj.name = ID;
                int render_order = ((i - frame_headers) / camera_frame_length); // 0,1,2...
                camera_objects.Add(ID, new CameraObj() { ID = ID, obj = camera_obj, render_order = render_order });
            }


        };

        // After decoding the packet, initialize the screen size and render textures (if necessary)
        ensureScreenSize();
    }

}
