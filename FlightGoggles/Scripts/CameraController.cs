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
    private UnityState_t internal_state;
    private Texture2D rendered_frame;
    private object socket_lock;


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
    public static Color ListHSVToColor(IList<float> list) { return Color.HSVToRGB(list[0], list[1], list[2]); }

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

        // Initialize Internal State
        internal_state = new UnityState_t();


        // Wait until end of frame to transmit images
        while (true)
        {
            // Wait until all rendering + UI is done.
            // Blocks until the frame is rendered.
            yield return new WaitForEndOfFrame();

            // Check if this frame should be rendered.
            if (internal_state.screen_initialized && internal_state.screen_skip_frames == 0)
            {

                // Read pixels from screen backbuffer (expensive).
                rendered_frame.ReadPixels(new Rect(0, 0, state.screen_width, state.screen_height), 0, 0);
                rendered_frame.Apply(); // Might not actually be needed since only applies setpixel changes.
                byte[] raw = rendered_frame.GetRawTextureData();


                // Compress and send the image in a different thread (if using windows).
                Task.Run(() =>
                {
                    // Get metadata
                    RenderMetadata_t metadata = new RenderMetadata_t(state);

                    // Create packet metadata
                    var msg = new NetMQMessage();
                    msg.Append(JsonConvert.SerializeObject(metadata));

                    // Process each camera's image, compress the image, and serialize the result.
                    // Compression is disabled for now...
                    List<byte[]> images = state.Cameras.AsParallel().Select(cam => get_raw_image(cam, raw)).ToList();

                    // Append images to message
                    images.ForEach(image => msg.Append(image));

                    // Send the message.
                    lock (socket_lock)
                    {
                        push_socket.TrySendMultipartMessage(msg);
                    }
                });

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

    // @TODO: Make allow for images to be RGB. Currently only allows for Gray output.
    public byte[] get_raw_image(Camera_t cam, byte[] raw)
    {
        // Allocate variables
        byte R, G, B;
        byte[] output = new byte[cam.channels * state.cam_width * state.cam_height];
        
        // Figure out where camera data starts and ends
        int y_start = cam.output_index * state.cam_height;
        int y_end = (cam.output_index+1) * state.cam_height;

        // Iterate through pixels in the source image
        for (int y = y_start; y < y_end; y++)
        {
            int y_inv = state.screen_height - y - 1;
            for (int x = 0; x < state.screen_width; x++)
            {
                // Get RGB values
                R = raw[((y * state.screen_width) + x) * 3];
                G = raw[((y * state.screen_width) + x) * 3 + 1];
                B = raw[((y * state.screen_width) + x) * 3 + 2];

                // Grayscale conversion.
                // https://www.mathworks.com/help/matlab/ref/rgb2gray.html
                // Use of Math.Min for overflow handling without try/catch statement.
                // NOTE: This does not round correctly (just truncates), since rounding is SLOW (-10FPS)
                output[(y_inv * state.screen_width) + x] = (byte)Math.Min(0.2989 * R + 0.5870 * G + 0.1140 * B, byte.MaxValue);
            }
        }

        return output;
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
        while (pull_socket.TryReceiveMultipartMessage(ref new_msg)) ;

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
        StateMessage_t state = JsonConvert.DeserializeObject<StateMessage_t>(msg[1].ConvertToString());

        // Update position of game objects.
        updateObjectPositions();
        // Make sure that all objects are initialized properly
        initializeObjects();
    }


    void updateObjectPositions()
    {
        // Update camera positions
        foreach (Camera_t obj_state in state.Cameras)
        {
            // Get camera game object 
            GameObject obj = internal_state.get_gameobject(obj_state.ID, camera_template);
            // Apply translation and rotation
            obj.transform.SetPositionAndRotation(ListToVector3(obj_state.position), ListToQuaternion(obj_state.rotation));

        }

        // Update Window positions
        foreach (Window_t obj_state in state.Windows)
        {
            // Get camera game object 
            GameObject obj = internal_state.get_gameobject(obj_state.ID, window_template);
            // Apply translation and rotation
            obj.transform.SetPositionAndRotation(ListToVector3(obj_state.position), ListToQuaternion(obj_state.rotation));

        }
    }

    // Tries to initialize uninitialized objects multiple times until the object is initialized.
    // When everything is initialized, this function will NOP.
    void initializeObjects()
    {

        // Initialize Screen & keep track of frames to skip
        internal_state.screen_skip_frames = Math.Max(0, internal_state.screen_skip_frames - 1);

        if (!internal_state.screen_initialized)
        {
            // Set the max framerate
            Application.targetFrameRate = max_framerate;
            // initialize the display to a window that fits all cameras
            Screen.SetResolution(state.screen_width, state.screen_height, false);
            // Set render texture to the correct size
            rendered_frame = new Texture2D(state.screen_width, state.screen_height, TextureFormat.RGB24, false, true);
            // Discard this frame, since changes do not take effect until the next frame.
            internal_state.screen_skip_frames += 1;
            internal_state.screen_initialized = true;
        }

        // Initialize windows
        state.Windows.Where(obj => !internal_state.is_initialized(obj.ID)).ToList().ForEach(
            obj_state =>
            {
                // Get objects
                ObjectState_t internal_object_state = internal_state.get_wrapper_object(obj_state.ID, window_template);
                GameObject obj = internal_object_state.game_obj;
                // Set window size
                obj.transform.localScale = ListToVector3(obj_state.size);
                // set window color
                obj.GetComponentInChildren<MeshRenderer>().material.SetColor("_EmissionColor", ListHSVToColor(obj_state.color));
                // Mark initialized.
                internal_object_state.initialized = true;
            }
        );

        // Initialize Camera objects if screen is ready to render.
        if (internal_state.screen_initialized && internal_state.screen_skip_frames == 0)
        {
            state.Cameras.Where(obj => !internal_state.is_initialized(obj.ID)).ToList().ForEach(
                obj_state =>
                {
                    // Get object
                    ObjectState_t internal_object_state = internal_state.get_wrapper_object(obj_state.ID, camera_template);
                    GameObject obj = internal_object_state.game_obj;
                    // Make sure camera renders to the correct portion of the screen.
                    obj.GetComponent<Camera>().pixelRect = new Rect(0, state.cam_height * (state.num_cameras - obj_state.output_index - 1), state.cam_width, state.cam_height);
                    // Ensure FOV is set for camera.
                    obj.GetComponent<Camera>().fieldOfView = state.cam_vertical_fov;
                    // @TODO: Ensure that post-processing profiles are correct (RGB vs Gray)

                    // enable Camera.
                    obj.SetActive(true);
                    // Log that camera is initialized
                    internal_object_state.initialized = true;
                }
            );
        }
    }

}
