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

// Include postprocessing
using UnityEngine.PostProcessing;
// TriLib dynamic model loader.
using TriLib;
using System.IO;

public class CameraController : MonoBehaviour
{

    // Public Parameters
    public string pose_host = "tcp://192.168.0.102:10253";
    public string video_host = "tcp://192.168.0.102:10254";
    public bool DEBUG = false;
    public bool should_compress_video = false;
    public GameObject camera_template;
    public GameObject window_template;
    public GameObject defaultSceneObject;

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
            //max_framerate = int.Parse(GetArg("-max-framerate", max_framerate.ToString()));
            //should_compress_video = bool.Parse(GetArg("-should-compress-video", (!video_host.Contains("localhost")).ToString()));
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
            if (internal_state.screenInitialized && internal_state.screenSkipFrames == 0)
            {

                // Read pixels from screen backbuffer (expensive).
                rendered_frame.ReadPixels(new Rect(0, 0, state.screenWidth, state.screenHeight), 0, 0);
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
                    // Stride is also disabled for now. Outputs RGB blocks with stride 3.
                    List<byte[]> images = state.cameras.AsParallel().Select(cam => get_raw_image(cam, raw, metadata.cameraIDs.Count())).ToList();

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

    // Get image block.
    public byte[] get_raw_image(Camera_t cam, byte[] raw, int numCams)
    {

        //int num_bytes_to_copy = cam.channels * state.camWidth * state.camHeight;
        int num_bytes_to_copy = 3 * state.camWidth * state.camHeight;

        byte[] output = new byte[num_bytes_to_copy];

        // Reverse camera indexing since the camera output is globally flipped on the Y axis.
        int outputIndex = numCams - 1 - cam.outputIndex;

        // Figure out where camera data starts and ends
        int y_start = outputIndex * state.camHeight;
        int y_end = (outputIndex+1) * state.camHeight;
        
        // Calculate start and end byte
        int byte_start = (y_start * state.screenWidth) * 3;
        int byte_end = (y_end * state.screenWidth) * 3;

        // Sanity check chunk length.
        if ((byte_end - byte_start) != num_bytes_to_copy)
        {
            throw new System.InvalidOperationException("Calculated image packet size is not correct.");
        }

        // Create a copy of the array
        Array.Copy(raw, byte_start, output, 0, num_bytes_to_copy);

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
        state = JsonConvert.DeserializeObject<StateMessage_t>(msg[1].ConvertToString());

        // Update position of game objects.
        updateObjectPositions();
        // Make sure that all objects are initialized properly
        initializeObjects();
        // Ensure that dynamic object settings such as depth-scaling and color are set correctly.
        ensureCorrectObjectSettings();
    }


    void updateObjectPositions()
    {

        // Update camera positions
        foreach (Camera_t obj_state in state.cameras)
        {
            // Get camera game object 
            GameObject obj = internal_state.getGameobject(obj_state.ID, camera_template);
            // Apply translation and rotation
            obj.transform.SetPositionAndRotation(ListToVector3(obj_state.position), ListToQuaternion(obj_state.rotation));

        }

        // Update Window positions
        foreach (Window_t obj_state in state.windows)
        {
            // Get camera game object 
            GameObject obj = internal_state.getGameobject(obj_state.ID, window_template);
            // Apply translation and rotation
            obj.transform.SetPositionAndRotation(ListToVector3(obj_state.position), ListToQuaternion(obj_state.rotation));

        }
    }

    // Tries to initialize uninitialized objects multiple times until the object is initialized.
    // When everything is initialized, this function will NOP.
    void initializeObjects()
    {

        // Initialize Screen & keep track of frames to skip
        internal_state.screenSkipFrames = Math.Max(0, internal_state.screenSkipFrames - 1);

        if (!internal_state.screenInitialized)
        {
            // Load external scene if necessary
            if (state.sceneFilename.Length != 0)
            {
                // Delete default scene objects
                foreach (Transform child in defaultSceneObject.transform)
                {
                    GameObject.Destroy(child.gameObject);
                }

                // Load in new scene model using TriLib
                using (var assetLoader = new AssetLoader())
                { // Initializes our Asset Loader.
                    // Creates an Asset Loader Options object.
                    var assetLoaderOptions = ScriptableObject.CreateInstance<AssetLoaderOptions>();
                    // Specify loading options.
                    assetLoaderOptions.DontLoadAnimations = false;
                    assetLoaderOptions.DontLoadCameras = true;
                    assetLoaderOptions.DontLoadLights = false;
                    assetLoaderOptions.DontLoadMaterials = false;
                    assetLoaderOptions.AutoPlayAnimations = true;
                    // Loads our model.
                    assetLoader.LoadFromFile(state.sceneFilename, assetLoaderOptions, defaultSceneObject);
                }

            }

            // Set our scene as static
            StaticBatchingUtility.Combine(defaultSceneObject);

            // Set the max framerate
            Application.targetFrameRate = state.maxFramerate;
            // initialize the display to a window that fits all cameras
            Screen.SetResolution(state.screenWidth, state.screenHeight, false);
            // Set render texture to the correct size
            rendered_frame = new Texture2D(state.screenWidth, state.screenHeight, TextureFormat.RGB24, false, true);
            // Discard this frame, since changes do not take effect until the next frame.
            internal_state.screenSkipFrames += 1;
            internal_state.screenInitialized = true;
        }

        // Initialize windows
        state.windows.Where(obj => !internal_state.isInitialized(obj.ID)).ToList().ForEach(
            obj_state =>
            {
                // Get objects
                ObjectState_t internal_object_state = internal_state.getWrapperObject(obj_state.ID, window_template);
                GameObject obj = internal_object_state.gameObj;
                // Set window size
                obj.transform.localScale = ListToVector3(obj_state.size);
                // set window color
                obj.GetComponentInChildren<MeshRenderer>().material.SetColor("_EmissionColor", ListHSVToColor(obj_state.color));
                // Mark initialized.
                internal_object_state.initialized = true;
            }
        );

        // Initialize Camera objects if screen is ready to render.
        if (internal_state.screenInitialized && internal_state.screenSkipFrames == 0)
        {
            state.cameras.Where(obj => !internal_state.isInitialized(obj.ID)).ToList().ForEach(
                obj_state =>
                {
                    // Get object
                    ObjectState_t internal_object_state = internal_state.getWrapperObject(obj_state.ID, camera_template);
                    GameObject obj = internal_object_state.gameObj;
                    // Make sure camera renders to the correct portion of the screen.
                    obj.GetComponent<Camera>().pixelRect = new Rect(0, state.camHeight * (state.numCameras - obj_state.outputIndex - 1), state.camWidth, state.camHeight);
                    // Ensure FOV is set for camera.
                    obj.GetComponent<Camera>().fieldOfView = state.camFOV;
                    
                    // Copy and save postProcessingProfile into internal_object_state.
                    var postBehaviour = obj.GetComponent<PostProcessingBehaviour>();
                    internal_object_state.postProcessingProfile = Instantiate(postBehaviour.profile);
                    postBehaviour.profile = internal_object_state.postProcessingProfile;
                    // Enable depth if needed.
                    if (obj_state.isDepth) {
                        var debugSettings = internal_object_state.postProcessingProfile.debugViews.settings;
                        debugSettings.mode = BuiltinDebugViewsModel.Mode.Depth;
                        debugSettings.depth.scale = state.camDepthScale;
                        internal_object_state.postProcessingProfile.debugViews.settings = debugSettings;
                    }

                    // enable Camera.
                    obj.SetActive(true);
                    // Log that camera is initialized
                    internal_object_state.initialized = true;
                }
            );
        }

    }

    void ensureCorrectObjectSettings()
    {

        // Check camera settings are correct.
        if (internal_state.screenInitialized && internal_state.screenSkipFrames == 0)
        {
            state.cameras.Where(obj => internal_state.isInitialized(obj.ID) && obj.isDepth).ToList().ForEach(
                obj_state =>
                {
                    // Get object
                    ObjectState_t internal_object_state = internal_state.getWrapperObject(obj_state.ID, camera_template);
                    GameObject obj = internal_object_state.gameObj;

                    // Scale depth.
                    if (internal_object_state.postProcessingProfile.debugViews.settings.depth.scale != state.camDepthScale)
                    {
                        var debugSettings = internal_object_state.postProcessingProfile.debugViews.settings;
                        debugSettings.mode = BuiltinDebugViewsModel.Mode.Depth;
                        debugSettings.depth.scale = state.camDepthScale;
                        internal_object_state.postProcessingProfile.debugViews.settings = debugSettings;
                    }
                
                }
            );
        }

    }
}
