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

// Message definitions
class StateMessage_t
{
    // Metadata
    public double utime { get; set; }
    public int cam_width { get; set; }
    public int cam_height { get; set; }
    public float cam_vertical_fov { get; set; }
    // Object state update
    public IList<Camera_t> Cameras { get; set; }
    public IList<Window_t> Windows { get; set; }

    // Additional getters
    public int num_cameras { get {return Cameras.Count();} }
    public int rendered_image_width { get { return cam_width; } }
    public int rendered_image_height { get { return cam_height*num_cameras; } }
}

// Camera class for decoding the ZMQ messages.
class Camera_t
{
    public string ID { get; set; }
    public IList<double> translation { get; set; }
    public IList<double> quaternion { get; set; }
    // Metadata
    public bool has_depth { get; set; }
    public int is_grayscale { get; set; }
    // Additional getters
    public GameObject get_gameobject(Dictionary<string, GameObject> dict)
    {
        return dict[ID];
    }
}

// Window class for decoding the ZMQ messages.
class Window_t
{
    public string ID { get; set; }
    public IList<double> translation { get; set; }
    public IList<double> quaternion { get; set; }
    // Metadata
    public IList<double> color { get; set; }
    public IList<double> size { get; set; }
    public GameObject get_gameobject(Dictionary<string, GameObject> dict)
    {
        return dict[ID];
    }
}



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
    private Dictionary<string, GameObject> simulation_objects;
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
            foreach (KeyValuePair<string, SimulationObj> entry in simulation_objects.Where(kv => kv.Key.ToLower().Contains("cam")))
            {
                //Debug.LogFormat("Turning on camera ID: {0}", entry.Key);
                GameObject camera_obj = entry.Value.obj;
                // Make sure camera renders to the correct portion of the screen.
                camera_obj.GetComponent<Camera>().pixelRect = new Rect(0, cam_height*(num_cameras-entry.Value.render_index-1), cam_width, cam_height);
                // Ensure FOV is set for camera.
                camera_obj.GetComponent<Camera>().fieldOfView = cam_vertical_fov;
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

        // Setup subscriptions.
        pull_socket.Subscribe("Pose");

        push_socket = new NetMQ.Sockets.PublisherSocket();
        push_socket.Connect(video_host);
        Debug.Log("Sockets bound.");

        // Initialize Objects
        simulation_objects = new Dictionary<string, SimulationObj>() { };


        // Wait until end of frame to transmit images
        while (true)
        {
            // Wait until all rendering + UI is done.
            // Blocks until the frame is rendered.
            yield return new WaitForEndOfFrame();

            if (screen_initialized && cameras_initialized)
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
                    // @TODO: Use REAL grayscale conversion.
                    byte[] raw_flipped_gray = new byte[raw.Length/channels];

                    // Temp vars
                    byte R, G, B;

                    for (int y = 0; y < rendered_image_height; y++)
                    {
                        int y_inv = rendered_image_height - y - 1;
                        for (int x = 0; x < rendered_image_width; x++)
                        {
                            R = raw[((y * rendered_image_width) + x) * channels];
                            G = raw[((y * rendered_image_width) + x) * channels + 1];
                            B = raw[((y * rendered_image_width) + x) * channels + 2];

                            // Grayscale conversion.
                            // https://www.mathworks.com/help/matlab/ref/rgb2gray.html
                            // Use of Math.Min for overflow handling without try/catch statement.
                            // NOTE: This does not round correctly (just truncates), since rounding is SLOW (-10FPS)
                            raw_flipped_gray[(y_inv * rendered_image_width) + x] = (byte) Math.Min(0.2989 * R + 0.5870 * G + 0.1140 * B, byte.MaxValue);
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

        // Get metadata
        Object metadata = JsonConvert.DeserializeObject<Movie>(json)



        /*
        // Metadata index.
        int f = 1;

        // Decode timestamp
        long new_timestamp = long.Parse(msg[f++].ConvertToString());
        // sanity check the timestamp
        if (!(new_timestamp > timestamp) && !DEBUG)
        {
            // Skip this message
            return;
        }
        // Update the timestamp
        timestamp = new_timestamp;

        // Get metadata
        cam_width = int.Parse(msg[f++].ConvertToString());
        cam_height = int.Parse(msg[f++].ConvertToString());
        cam_vertical_fov = float.Parse(msg[f++].ConvertToString());
        // Window dimensions
        window_dimensions = new Vector3(
                                    float.Parse(msg[f++].ConvertToString()),
                                    float.Parse(msg[f++].ConvertToString()),
                                    float.Parse(msg[f++].ConvertToString()));

        window_hsv = new Vector3(
                            float.Parse(msg[f++].ConvertToString()),
                            float.Parse(msg[f++].ConvertToString()),
                            float.Parse(msg[f++].ConvertToString()));

        // split the message into batches of 8 (eg. poses for each object).
        for (int i = frame_headers; i < msg.FrameCount; i += camera_frame_length)
        {
            // Get the Object ID.
            string ID = msg[i].ConvertToString();
            // Decode the pose message
            Vector3 position = new Vector3(
                                    float.Parse(msg[i + 1].ConvertToString()),
                                    float.Parse(msg[i + 2].ConvertToString()),
                                    float.Parse(msg[i + 3].ConvertToString()));
            Quaternion rotation = new Quaternion(
                float.Parse(msg[i + 4].ConvertToString()),
                float.Parse(msg[i + 5].ConvertToString()),
                float.Parse(msg[i + 6].ConvertToString()),
                float.Parse(msg[i + 7].ConvertToString()));

            // Check if this object exists.
            if (simulation_objects.ContainsKey(ID))
            {
                // transform the gameobject
                GameObject obj = simulation_objects[ID].obj;
                obj.transform.SetPositionAndRotation(position, rotation);
            }
            else
            {
                // Check if we got a Window or Camera pose.
                bool is_camera = ID.ToLower().Contains("cam");
                bool is_window = !is_camera;

                GameObject template = is_camera ? camera_template : window_template;
                // Instantiate the object using the desired pose.
                GameObject obj = Instantiate(template, position, rotation);
                // Set the game object name
                obj.name = ID;
                // remember the order that this object was given to us in.
                int render_index = ((i - frame_headers) / camera_frame_length); // 0,1,2...
                if (is_window){
                    // Set window size
                    obj.transform.localScale = window_dimensions;
                    // set window color
                    Color hsv_color = Color.HSVToRGB(window_hsv[0], window_hsv[1], window_hsv[2]);
                    obj.GetComponentInChildren<MeshRenderer>().material.SetColor("_EmissionColor", hsv_color);
                }

                // Save the object.
                simulation_objects.Add(ID, new SimulationObj() { ID = ID, obj = obj, render_index = render_index });

            }


        };
        */

        // Make sure that the rest of the program knows that we are connected.
        connected = true;
    }


}
