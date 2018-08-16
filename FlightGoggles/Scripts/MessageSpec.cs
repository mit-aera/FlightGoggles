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

        // Get Wrapper object, defaulting to a passed in template if it does not exist.
        public ObjectState_t getWrapperObject(string ID, GameObject template)
        {
            if (!objects.ContainsKey(ID))
            {
                // Create and save object from template
                objects[ID] = new ObjectState_t(template);
            }
            return objects[ID];
        }

        // Get Wrapper object, defaulting to a passed in template if it does not exist.
        public ObjectState_t getWrapperObject(string ID, string prefab_ID)
        {
            if (!objects.ContainsKey(ID))
            {
                // Create and save object from template
                GameObject template = Resources.Load(prefab_ID) as GameObject;
                objects[ID] = new ObjectState_t(template);
            }
            return objects[ID];
        }

        // Get gameobject from wrapper object
        public GameObject getGameobject(string ID, GameObject template)
        {
            return getWrapperObject(ID, template).gameObj;
        }
        public GameObject getGameobject(string ID, string templateID)
        {
            return getWrapperObject(ID, templateID).gameObj;
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
    [Serializable]
    public class StateMessage_t
    {
        // Startup parameters. They should only change once.
        public int maxFramerate;
        public bool sceneIsInternal;
        public string sceneFilename ;
        public bool compressImage ;
        // AA settings
        public float temporalJitterScale ;
        public int temporalStability ;
        public float hdrResponse ;
        public float sharpness ;
        public float adaptiveEnhance ;
        public float microShimmerReduction ;
        public float staticStabilityPower ;

        // Frame Metadata
        public Int64 utime ;
        public int camWidth ;
        public int camHeight ;
        public float camFOV   ;
        public float camDepthScale ;
        public bool forceFrameRender ;
        // Additional metadata that will be passed through the render process.
        public Dictionary<string, string> additionalMetadata ;
        // Object state update
        public List<Camera_t> cameras ;
        public List<Object_t> objects ;


        // Additional getters (for convenience)
        public int numCameras { get { return cameras.Count(); } }
        public int screenWidth { get { return camWidth; } }
        public int screenHeight { get { return camHeight * numCameras; } }
        public bool sceneIsDefault { get { return sceneFilename.Length == 0; } }

    }

    //public class utilities
    //{
    //    //public int numCameras { get { return cameras.Count(); } }
    //    //public int screenWidth { get { return camWidth; } }
    //    //public int screenHeight { get { return camHeight * numCameras; } }
    //    //public bool sceneIsDefault { get { return sceneFilename.Length == 0; } }
    //}

    // Camera class for decoding the ZMQ messages.
    [Serializable]
    public class Camera_t
    {
        public string ID ;
        public List<float> position ;
        public List<float> rotation ;
        // Metadata
        public int channels ;
        public bool isDepth ;
        public int outputIndex ;
        public bool useAA = false;

        // Additional getters
        public bool isGrayscale { get { return (channels == 1) && (!isDepth); } }

    }

    // Generic object class for decoding the ZMQ messages.
    [Serializable]
    public class Object_t
    {
        public string prefabID ;
        public string ID ; // The instance ID
        public List<float> position ;
        public List<float> rotation ;
        // Metadata
        public List<float> size ;
    }



    // =============================
    // OUTGOING Message definitions
    // =============================
    [Serializable]
    public class RenderMetadata_t
    {
        // Metadata
        public Int64 utime ;
        public bool isCompressed ;
        public int camWidth ;
        public int camHeight ;
        public float camDepthScale;
        // Additional metadata for helping with the deserialization process.
        public List<string> cameraIDs;
        public List<int> channels;
        // Additional unstructured metadata given to us by the render request.
        public Dictionary<string, string> additionalMetadata;
        // To make sure that both client and renderer are on the same page
        public string apiVersion;


        public RenderMetadata_t(StateMessage_t state, string apiVersion_)
        {
            utime = state.utime;
            isCompressed = state.compressImage;
            camWidth = state.camWidth;
            camHeight = state.camHeight;
            camDepthScale = state.camDepthScale;
            cameraIDs = state.cameras.Select(obj => obj.ID).ToList();
            channels = state.cameras.Select(obj => obj.channels).ToList();
            additionalMetadata = state.additionalMetadata;
            apiVersion = apiVersion_;
        }
    }

}