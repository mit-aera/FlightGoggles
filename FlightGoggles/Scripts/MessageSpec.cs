using System;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.PostProcessing;

//[RequireComponent(typeof(PostProcessingBehaviour))]

// Array ops
using System.Linq;

namespace MessageSpec
{

    // =============================
    // INTERNAL Message definitions
    // =============================
    // For storing unity's internal state
    // E.g. are objects initialized, should this frame be rendered, etc.
    public class UnityState_t
    {
        private Dictionary<string, ObjectState_t> objects;

        // Initialization status
        public int initializationStep { get; set; } = 0;
        public int screenSkipFrames { get; set; } = 0;
        // Convenience getter function.
        public bool initialized { get { return (initializationStep < 0); } }
        public bool readyToRender { get { return (initialized && (screenSkipFrames == 0)); } }

        // Advanced getters/setters
        
        // Ensure object exists.
        public void ensureObjectExists(string ID, GameObject template)
        {
            if (!objects.ContainsKey(ID)) {
                // Create and save object from template
                objects[ID] = new ObjectState_t(template);
            }
        }

        // Get Wrapper object
        public ObjectState_t getWrapperObject(string ID, GameObject template)
        {
            ensureObjectExists(ID, template);
            return objects[ID];
        }
        // Get Wrapper object
        public GameObject getGameobject(string ID, GameObject template)
        {
            return getWrapperObject(ID, template).gameObj;
        }
        // Check if object is initialized
        public bool isInitialized(string ID)
        {
            bool isInitialized = false;
            if (objects.ContainsKey(ID))
            {
                isInitialized = objects[ID].initialized;
            }
            return isInitialized;
        }
        // Constructor
        public UnityState_t()
        {
            objects = new Dictionary<string, ObjectState_t>() { };
        }
    }

    // Keeps track of gameobjects and their initialization and instantiation.
    public class ObjectState_t
    {
        public bool initialized { get; set; } = false;
        public GameObject gameObj { get; set; }
        public GameObject template { get; set; }
        public PostProcessingProfile postProcessingProfile { get; set; }
        // Constructor
        public ObjectState_t(GameObject template)
        {
            this.gameObj = GameObject.Instantiate(template);
            this.template = template;
        }

    }

    // =============================
    // INCOMING Message definitions
    // =============================
    public class StateMessage_t
    {
        // Startup parameters. They should only change once.
        public int maxFramerate { get; set; }
        public bool sceneIsInternal { get; set; }
        public string sceneFilename { get; set; }
        public bool compressImage { get; set; }
        // Frame Metadata
        public Int64 utime { get; set; }
        public int camWidth { get; set; }
        public int camHeight { get; set; }
        public float camFOV   { get; set; }
        public float camDepthScale { get; set; }
        // Object state update
        public IList<Camera_t> cameras { get; set; }
        public IList<Window_t> windows { get; set; }
        // Additional getters (for convenience)
        public int numCameras { get { return cameras.Count(); } }
        public int screenWidth { get { return camWidth; } }
        public int screenHeight { get { return camHeight * numCameras; } }
        public bool sceneIsDefault { get { return sceneFilename.Length == 0; } }
        
    }

    // Camera class for decoding the ZMQ messages.
    public class Camera_t
    {
        public string ID { get; set; }
        public IList<float> position { get; set; }
        public IList<float> rotation { get; set; }
        // Metadata
        public int channels { get; set; }
        public bool isDepth { get; set; }
        public int outputIndex { get; set; }
        public bool useAA { get; set; } = false;

        // Additional getters
        public bool isGrayscale { get { return (channels == 1) && (!isDepth); } }

    }

    // Window class for decoding the ZMQ messages.
    public class Window_t
    {
        public string ID { get; set; }
        public IList<float> position { get; set; }
        public IList<float> rotation { get; set; }
        // Metadata
        public IList<float> color { get; set; }
        public IList<float> size { get; set; }
    }
    


    // =============================
    // OUTGOING Message definitions
    // =============================

    public class RenderMetadata_t
    {
        // Metadata
        public Int64 utime { get; set; }
        public bool isCompressed { get; set; }
        public int camWidth { get; set; }
        public int camHeight { get; set; }
        public float camDepthScale { get; set; }
        // Additional metadata for helping with the deserialization process.
        public IList<string> cameraIDs { get; set; }
        public IList<int> channels { get; set; }

        public RenderMetadata_t(StateMessage_t state)
        {
            utime = state.utime;
            isCompressed = state.compressImage;
            camWidth = state.camWidth;
            camHeight = state.camHeight;
            camDepthScale = state.camDepthScale;
            cameraIDs = state.cameras.Select(obj => obj.ID).ToList();
            channels = state.cameras.Select(obj => obj.channels).ToList();
        }
    }

}