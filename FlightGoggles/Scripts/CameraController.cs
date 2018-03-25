/*
 * FlightGoggles Camera Controller.
 * Top-level logic script for FlightGoggles. 
 * Listens for ZMQ camera poses and object positions creates/moves cameras and objects to match.
 * Pushes rendered RGBD frames back to requester over ZMQ with associated metadata.
 * 
 * Author: Winter Guerra <winterg@mit.edu> 
 * Date: January 2017.
 */

// To allow for OBJ file import, you must download and include the TriLib Library
// into this project folder. Once the TriLib library has been included, you can enable
// OBJ importing by commenting out the following line.
//#define TRILIB_DOES_NOT_EXIST

using System;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.UI;
using System.Threading;
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
#if !TRILIB_DOES_NOT_EXIST
using TriLib;
using System.IO;
#endif

// Dynamic scene management
using UnityEngine.SceneManagement;

public class CameraController : MonoBehaviour
{
    // Default Parameters
    [HideInInspector]
    public const string pose_host_default = "tcp://127.0.0.1:10253";
    [HideInInspector]
    public const string video_host_default = "tcp://127.0.0.1:10254";

    // Public Parameters
    public string flight_goggles_version = "v1.4.1";
    public string pose_host;
    public string video_host;
    public bool DEBUG = false;
    public bool should_compress_video = false;
    public GameObject camera_template;
    public GameObject window_template;
    public GameObject splashScreen;
    // default scenes and assets
    public string defaultScene;
    public string defaultLightingScene;

    // instance vars
    // NETWORK
    private NetMQ.Sockets.SubscriberSocket pull_socket;
    private NetMQ.Sockets.PublisherSocket push_socket;
    private StateMessage_t state;

    // Internal state & storage variables
    private UnityState_t internal_state;
    private bool is_first_frame;
    private Texture2D rendered_frame;
    private object socket_lock;

    /* =====================
     * UNITY PLAYER EVENT HOOKS 
     * =====================
     */

    // Function called when Unity Player is loaded.
    public IEnumerator Start()
    {
        // Check if the program should use CLI arguments (with defaults)
        if (!Application.isEditor)
        {
            pose_host = GetArg("-pose-host", pose_host_default);
            video_host = GetArg("-video-host", video_host_default);
        }

        // Init simple splash screen
        Text text_obj = splashScreen.GetComponentInChildren<Text>(true);
        text_obj.text = "FlightGoggles Simulation Environment" + Environment.NewLine +
            flight_goggles_version + Environment.NewLine + Environment.NewLine +
            "Waiting for client connection..." + Environment.NewLine + Environment.NewLine +
            "Pose input socket:" + Environment.NewLine + pose_host + Environment.NewLine + Environment.NewLine +
           "Video output socket:" + Environment.NewLine + video_host;

        splashScreen.SetActive(true);

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

        // Do not try to do any processing this frame so that we can render our splash screen.
        internal_state.screenSkipFrames = 1;

        // Wait until end of frame to transmit images
        while (true)
        {
            // Wait until all rendering + UI is done.
            // Blocks until the frame is rendered.
            yield return new WaitForEndOfFrame();

            // Check if this frame should be rendered.
            if (internal_state.readyToRender)
            {
                // Read the frame from the GPU backbuffer and send it via ZMQ.
                sendFrameOnWire();
            }
        }
    }

    // Function called when Unity player is killed.
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
        if (pull_socket.HasIn || internal_state.screenSkipFrames<=0)
        {
            // Receive most recent message
            var msg = new NetMQMessage();
            var new_msg = new NetMQMessage();
            // Blocking receive for a message
            msg = pull_socket.ReceiveMultipartMessage();
            // Make sure that splashscreen is disabled
            splashScreen.SetActive(false);

            // Check if this is the latest message
            while (pull_socket.TryReceiveMultipartMessage(ref new_msg)) ;

            // Check that we got the whole message
            if (new_msg.FrameCount >= msg.FrameCount) { msg = new_msg; }

            if (msg.FrameCount == 0) { return; }

            // Get scene state from LCM
            state = JsonConvert.DeserializeObject<StateMessage_t>(msg[1].ConvertToString());

            // Make sure that all objects are initialized properly
            initializeObjects();
            // Ensure that dynamic object settings such as depth-scaling and color are set correctly.
            updateDynamicObjectSettings();
            // Update position of game objects.
            updateObjectPositions();
        } else
        {
            // Throttle to 10hz when idle
            Thread.Sleep(100); // [ms]
        }
    }


    /* ==================================
     * FlightGoggles High Level Functions 
     * ==================================
     */


    // Tries to initialize uninitialized objects multiple times until the object is initialized.
    // When everything is initialized, this function will NOP.
    void initializeObjects()
    {

        // Initialize Screen & keep track of frames to skip
        internal_state.screenSkipFrames = Math.Max(0, internal_state.screenSkipFrames - 1);

        // NOP if Unity wants us to skip this frame.
        if (internal_state.screenSkipFrames > 0){
            return;
        }
        
        // Run initialization steps in order.
        switch (internal_state.initializationStep)
        {
            // Load scene if needed.
            case 0:
                loadScene();
                internal_state.initializationStep++;
                // Takes one frame to take effect.
                internal_state.screenSkipFrames++;
                // Skip the rest of this frame
                break;
                
            // Initialize screen if scene is fully loaded and ready.
            case 1:
                resizeScreen();
                internal_state.initializationStep++;
                // Takes one frame to take effect.
                internal_state.screenSkipFrames++;
                // Skip the rest of this frame
                break;

            // Initialize gameobjects if screen is ready to render.
            case 2:
                disableColliders();
                instantiateWindows();
                instantiateCameras();
                internal_state.initializationStep++;
                // Takes one frame to take effect.
                internal_state.screenSkipFrames++;
                // Skip the rest of this frame
                break;

            // Ensure cameras are rendering to correct portion of GPU backbuffer.
            // Note, this should always be run after initializing the cameras.
            case 3:
                setCameraViewports();
                // Go to next step.
                internal_state.initializationStep++;
                // Takes one frame to take effect.
                internal_state.screenSkipFrames++;
                // Skip the rest of this frame
                break;

            case 4:
                setCameraPostProcessSettings();
                // Set initialization to -1 to indicate that we're done initializing.
                internal_state.initializationStep=-1;
                // Takes one frame to take effect.
                internal_state.screenSkipFrames++;
                // Skip the rest of this frame
                break;

            
            // If initializationStep does not match any of the ones above
            // then initialization is done and we need do nothing more.
                
        }
    }

    void setCameraPostProcessSettings()
    {

        state.cameras.ToList().ForEach(
            obj_state =>
            {
                // Get object
                ObjectState_t internal_object_state = internal_state.getWrapperObject(obj_state.ID, camera_template);
                GameObject obj = internal_object_state.gameObj;


                // Copy and save postProcessingProfile into internal_object_state.
                var postBehaviour = obj.GetComponent<PostProcessingBehaviour>();
                internal_object_state.postProcessingProfile = Instantiate(postBehaviour.profile);
                postBehaviour.profile = internal_object_state.postProcessingProfile;

                // Enable depth if needed.
                if (obj_state.isDepth)
                {
                    var debugSettings = internal_object_state.postProcessingProfile.debugViews.settings;
                    debugSettings.mode = BuiltinDebugViewsModel.Mode.Depth;
                    debugSettings.depth.scale = state.camDepthScale;
                    internal_object_state.postProcessingProfile.debugViews.settings = debugSettings;

                    // Set RGB settings
                }
                else
                {

                    // TXAA settings
                    AntialiasingModel.Settings AASettings = internal_object_state.postProcessingProfile.antialiasing.settings;

                    AASettings.method = AntialiasingModel.Method.Taa; // TAA
                    AASettings.taaSettings.jitterSpread = state.jitterSpread;
                    AASettings.taaSettings.sharpen = state.sharpen;
                    AASettings.taaSettings.stationaryBlending = state.blendingStationary;
                    AASettings.taaSettings.motionBlending = state.blendingMotion;

                    internal_object_state.postProcessingProfile.antialiasing.settings = AASettings;
                }
            });
        // 

    }

    // Update window and camera positions based on the positions sent by ZMQ.
    void updateObjectPositions()
    {
        if (internal_state.readyToRender){
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
    }

    void updateDynamicObjectSettings()
    {
        // Update object settings.
        if (internal_state.readyToRender)
        {
            // Update depth cameras with dynamic depth scale.
            state.cameras.Where(obj => obj.isDepth).ToList().ForEach(
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

    /* =============================================
     * FlightGoggles Initialization Functions 
     * =============================================
     */

    void loadScene(){
        // Load a scene, either from internal selection or external .obj or .dae file.
        if (state.sceneIsInternal || state.sceneIsDefault){
            // Load scene from internal scene selection
            // Get the scene name.
            string sceneName = (state.sceneIsDefault)? defaultScene : state.sceneFilename;
            // Load the scene. 
            SceneManager.LoadScene(sceneName, LoadSceneMode.Additive);
            
        // Load external scene
        } else {
            // Throw error if trilib does not exist
#if TRILIB_DOES_NOT_EXIST
            throw new System.InvalidOperationException("Cannot import external 3D models without including TriLib in the project directory. Please read the FlightGoggles README for more information.");
#else
            // Load default lighting scene.
            // SceneManager.LoadScene(defaultLightingScene, LoadSceneMode.Additive);
            // Make new empty scene for holding the .obj data.
            Scene externallyLoadedScene = SceneManager.CreateScene("Externally_Loaded_Scene");
            // Tell Unity to put new objects into the newly created container scene.
            SceneManager.SetActiveScene(externallyLoadedScene);

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
                // Loads scene model into container scene.
                assetLoader.LoadFromFile(state.sceneFilename, assetLoaderOptions);
            }
            // Set our loaded scene as static
            // @TODO
            // StaticBatchingUtility.Combine(externallyLoadedScene);
#endif

        }
    }

    void resizeScreen(){
        // Set the max framerate
        Application.targetFrameRate = state.maxFramerate;
        // initialize the display to a window that fits all cameras
        Screen.SetResolution(state.screenWidth, state.screenHeight, false);
        // Set render texture to the correct size
        rendered_frame = new Texture2D(state.screenWidth, state.screenHeight, TextureFormat.RGB24, false, true);
    }

    void disableColliders(){
        // Disable object colliders in scene
        foreach(Collider c in FindObjectsOfType<Collider>())
        {
            c.enabled = false;
        }
    }

    void instantiateWindows(){
        // Initialize windows
        state.windows.ToList().ForEach(
            obj_state =>
            {
                // Get objects
                ObjectState_t internal_object_state = internal_state.getWrapperObject(obj_state.ID, window_template);
                GameObject obj = internal_object_state.gameObj;
                // Set window size
                obj.transform.localScale = ListToVector3(obj_state.size);
                // set window color
                obj.GetComponentInChildren<MeshRenderer>().material.SetColor("_EmissionColor", ListHSVToColor(obj_state.color));
            }
        );
    }

    void instantiateCameras(){
        // Initialize Camera objects.
        state.cameras.ToList().ForEach(
            obj_state =>
            {
                // Get object
                ObjectState_t internal_object_state = internal_state.getWrapperObject(obj_state.ID, camera_template);
                GameObject obj = internal_object_state.gameObj;
                // Ensure FOV is set for camera.
                obj.GetComponent<Camera>().fieldOfView = state.camFOV;
                
            }
        );
    }

    // Marked @TODO
    void setCameraViewports(){
        // Resize camera viewports.
        state.cameras.ToList().ForEach(
            obj_state =>
            {
                // Get object
                ObjectState_t internal_object_state = internal_state.getWrapperObject(obj_state.ID, camera_template);
                GameObject obj = internal_object_state.gameObj;
                // Make sure camera renders to the correct portion of the screen.
                obj.GetComponent<Camera>().pixelRect = new Rect(0, state.camHeight * (state.numCameras - obj_state.outputIndex - 1), state.camWidth, state.camHeight);
                // enable Camera.
                obj.SetActive(true);
            }
        );
    }


    /* ==================================
     * FlightGoggles Low Level Functions 
     * ==================================
     */


    // Cut image block for an individual camera from larger provided image.
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

    // Reads a scene frame from the GPU backbuffer and sends it via ZMQ.
    void sendFrameOnWire()
    {
        // Read pixels from screen backbuffer (expensive).
        rendered_frame.ReadPixels(new Rect(0, 0, state.screenWidth, state.screenHeight), 0, 0);
        rendered_frame.Apply(); // Might not actually be needed since only applies setpixel changes.
        byte[] raw = rendered_frame.GetRawTextureData();


        // Compress and send the image in a different thread.
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

    /* ==================================
     * FlightGoggles Helper Functions 
     * ==================================
     */

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

}
