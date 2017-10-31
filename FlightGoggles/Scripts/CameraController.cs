/*
 * Camera Controller
 * Listens for ZMQ camera poses on a separate thread and creates/moves cameras to match.
 *
 */


using System;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using System.Threading.Tasks;

// Array ops
using System.Linq;

// ZMQ/LCM
using NetMQ;

// Include JSON
using Newtonsoft.Json;

// Include message types
using MessageSpec;

public class CameraController : MonoBehaviour
{

    // Public Parameters
    public string pose_host = "tcp://192.168.0.102:10253";
    public string video_host = "tcp://192.168.0.102:10254";
    public bool DEBUG = true;

    public int max_framerate = 80;
    public bool should_compress_video = true;
    public GameObject camera_template;
    public GameObject window_template;

    // instance vars
    // NETWORK
    private NetMQ.Sockets.SubscriberSocket pull_socket;
    private NetMQ.Sockets.PublisherSocket push_socket;
    private StateMessage_t state;

    // Internal state & storage variables
    private Dictionary<string, GameObject> objects;
    private Texture2D rendered_frame;
    private object socket_lock;
    private bool screen_initialized = false;
    private bool cameras_initialized = false;
    private bool connected = false;


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

    // Helper functions for converting list -> vector
    public static Vector3 ListToVector3(IList<float> list) { return new Vector3(list[0], list[1], list[2]); }
    public static Quaternion ListToQuaternion(IList<float> list) { return new Quaternion(list[0], list[1], list[2], list[3]); }

    /* Use this for initialization
     * Should create a worker thread that listens for ZMQ messages and saves the most recent one.
     */

    private void ensureScreenSize()
    {
        if (!screen_initialized && connected) {
            // Set the max framerate
            Application.targetFrameRate = max_framerate;
            // initialize the display to a window that fits all cameras
            Screen.SetResolution(state.rendered_image_width, state.rendered_image_height, false);
            // Set render texture to the correct size
            rendered_frame = new Texture2D(state.rendered_image_width, state.rendered_image_height, TextureFormat.RGB24, false, true);
            // Screen is initialized, but the cameras are not.
            screen_initialized = true;
            return;
        }
        // Initialize the cameras (on next frame)
        if (screen_initialized && connected && !cameras_initialized){
            // Make sure that all cameras are drawing to the correct portion of the screen.
            foreach (Camera_t cam in state.Cameras)
            {
                // Get object
                GameObject camera_obj = cam.get_gameobject(objects, camera_template);
                // Make sure camera renders to the correct portion of the screen.
                camera_obj.GetComponent<Camera>().pixelRect = new Rect(0, state.cam_height*(state.num_cameras - cam.output_index - 1), state.cam_width, state.cam_height);
                // Ensure FOV is set for camera.
                camera_obj.GetComponent<Camera>().fieldOfView = state.cam_vertical_fov;
                // enable Camera.
                camera_obj.SetActive(true);
            }
            cameras_initialized = true;

        }
    }

    public IEnumerator Start()
    {
        // Check if the program should use CLI arguments (with defaults)
        if (!Application.isEditor)
        {
            pose_host = GetArg("-pose-host", pose_host);
            video_host = GetArg("-video-host", video_host);
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

        // Setup subscriptions.
        pull_socket.Subscribe("Pose");

        push_socket = new NetMQ.Sockets.PublisherSocket();
        push_socket.Connect(video_host);
        Debug.Log("Sockets bound.");

        // Initialize Objects
        objects = new Dictionary<string, GameObject>() { };


        // Wait until end of frame to transmit images
        while (true)
        {
            // Wait until all rendering + UI is done.
            // Blocks until the frame is rendered.
            yield return new WaitForEndOfFrame();

            if (screen_initialized && cameras_initialized)
            {

                // Read pixels from the display.
                rendered_frame.ReadPixels(new Rect(0, 0, state.rendered_image_width, state.rendered_image_height), 0, 0);
                rendered_frame.Apply();
                byte[] raw = rendered_frame.GetRawTextureData();


                // Compress and send the image in a different thread (if using windows).
                Task.Run(() =>
                {
                    const int channels = 3;

                    // Get the grayscale image, flipping the rows such that the image is transmitted rightside up
                    // @TODO: Use REAL grayscale conversion.
                    byte[] raw_flipped_gray = new byte[raw.Length/channels];

                    // Temp vars
                    byte R, G, B;

                    for (int y = 0; y < state.rendered_image_height; y++)
                    {
                        int y_inv = state.rendered_image_height - y - 1;
                        for (int x = 0; x < state.rendered_image_width; x++)
                        {
                            R = raw[((y * state.rendered_image_width) + x) * channels];
                            G = raw[((y * state.rendered_image_width) + x) * channels + 1];
                            B = raw[((y * state.rendered_image_width) + x) * channels + 2];

                            // Grayscale conversion.
                            // https://www.mathworks.com/help/matlab/ref/rgb2gray.html
                            // Use of Math.Min for overflow handling without try/catch statement.
                            // NOTE: This does not round correctly (just truncates), since rounding is SLOW (-10FPS)
                            raw_flipped_gray[(y_inv * state.rendered_image_width) + x] = (byte) Math.Min(0.2989 * R + 0.5870 * G + 0.1140 * B, byte.MaxValue);
                        }
                    }

                    // Create the image data holder
                    byte[] image;
                    if (should_compress_video)
                    {

                        // Compress the image using the fastest system package available.
#if (UNITY_EDITOR_WINDOWS || UNITY_STANDALONE_WINDOWS)
                        image =  SnappyCodec.Compress(raw_flipped_gray);
#else
                        image = Snappy.Sharp.Snappy.Compress(raw_flipped_gray);
#endif
                    }
                    else
                    {
                        image = raw_flipped_gray;
                    }

                    // Send the image back using NetMQ
                    // SendNetMQImage(timestamp, ref image, should_compress_video);
                    var msg = new NetMQMessage();
                    msg.Append(timestamp);
                    msg.Append(Convert.ToInt16(should_compress_video));
                    msg.Append(image);
                    lock (socket_lock)
                    {
                        push_socket.TrySendMultipartMessage(msg);
                    }
                });

                
            } else {
                ensureScreenSize();
            }
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

        // Check that we got the whole message
        if (new_msg.FrameCount >= msg.FrameCount)
        {
            msg = new_msg;
        }

        if (msg.FrameCount == 0)
        {
            return;
        }

        // Get scene state from LCM
        StateMessage_t state =  JsonConvert.DeserializeObject<StateMessage_t>(msg[1].ConvertToString());
        // Update position of game objects.
        updateObjectPositions();
        // Make sure that all objects are initialized properly
        initializeObjects();

        // Make sure that the rest of the program knows that we are connected.
        connected = true;
    }


    void updateObjectPositions()
    {
        // Update camera positions
        foreach (Camera_t obj_state in state.Cameras)
        {
            // Get camera game object 
            GameObject obj = obj_state.get_gameobject(objects, camera_template);
            // Apply translation and rotation
            obj.transform.SetPositionAndRotation(ListToVector3(obj_state.position), ListToQuaternion(obj_state.rotation));

        }

        // Update Window positions
        foreach (Window_t obj_state in state.Windows)
        {
            // Get camera game object 
            GameObject obj = obj_state.get_gameobject(objects, window_template);
            // Apply translation and rotation
            obj.transform.SetPositionAndRotation(ListToVector3(obj_state.position), ListToQuaternion(obj_state.rotation));

        }
    }

    // Tries to initialize uninitialized objects multiple times until the object is initialized.
    // When everything is initialized, this function will NOP.
    void initializeObjects() {

        //foreach (Window_t obj_state in state.Windows where )


        //    if (is_window)
        //        {
        //            // Set window size
        //            obj.transform.localScale = window_dimensions;
        //            // set window color
        //            Color hsv_color = Color.HSVToRGB(window_hsv[0], window_hsv[1], window_hsv[2]);
        //            obj.GetComponentInChildren<MeshRenderer>().material.SetColor("_EmissionColor", hsv_color);
        //        }

        //        // Save the object.
        //        objects.Add(ID, new SimulationObj() { ID = ID, obj = obj, render_index = render_index });

        throw new NotImplementedException();

    }


}
