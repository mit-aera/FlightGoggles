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
    public class StateMessage_t
    {
        // Startup parameters. They should only change once.
        public int maxFramerate { get; set; }
        public bool sceneIsInternal { get; set; }
        public string sceneFilename { get; set; }
        public bool compressImage { get; set; }
        // AA settings
        public float temporalJitterScale { get; set; }
        public int temporalStability { get; set; }
        public float hdrResponse { get; set; }
        public float sharpness { get; set; }
        public float adaptiveEnhance { get; set; }
        public float microShimmerReduction { get; set; }
        public float staticStabilityPower { get; set; }

        // Frame Metadata
        public Int64 utime { get; set; }
        public int camWidth { get; set; }
        public int camHeight { get; set; }
        public float camFOV   { get; set; }
        public float camDepthScale { get; set; }
        public bool forceFrameRender { get; set; }
        // Additional metadata that will be passed through the render process.
        public Dictionary<string, string> additionalMetadata { get; set; }
        // Object state update
        public IList<Camera_t> cameras { get; set; }
        public IList<Object_t> objects { get; set; }
        
        
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

    // Generic object class for decoding the ZMQ messages.
    public class Object_t
    {
        public string prefabID { get; set; }
        public string ID { get; set; } // The instance ID
        public IList<float> position { get; set; }
        public IList<float> rotation { get; set; }
        // Metadata
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
        // Additional unstructured metadata given to us by the render request.
        public Dictionary<string, string> additionalMetadata {get; set;}
        // To make sure that both client and renderer are on the same page
        public string apiVersion { get; set; }


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