using System;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;

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

        // Screen state
        public bool screen_initialized { get; set; } = false;
        public int screen_skip_frames { get; set; } = 0;

        // Advanced getters/setters
        // Ensure object exists.
        public void ensure_object_exists(string ID, GameObject template)
        {
            if (!objects.ContainsKey(ID)) {
                // Create and save object from template
                objects[ID] = new ObjectState_t(template);
            }
        }

        // Get Wrapper object
        public ObjectState_t get_wrapper_object(string ID, GameObject template)
        {
            ensure_object_exists(ID, template);
            return objects[ID];
        }
        // Get Wrapper object
        public GameObject get_gameobject(string ID, GameObject template)
        {
            return get_wrapper_object(ID, template).game_obj;
        }
        // Check if object is initialized
        public bool is_initialized(string ID)
        {
            bool is_initialized = false;
            if (objects.ContainsKey(ID))
            {
                is_initialized = objects[ID].initialized;
            }
            return is_initialized;
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
        public GameObject game_obj { get; set; }
        public GameObject template { get; set; }
        // Constructor
        public ObjectState_t(GameObject template)
        {
            this.game_obj = GameObject.Instantiate(template);
            this.template = template;
        }

    }

    // =============================
    // INCOMING Message definitions
    // =============================
    public class StateMessage_t
    {
        // Metadata
        public double utime { get; set; }
        public int cam_width { get; set; }
        public int cam_height { get; set; }
        public float cam_vertical_fov { get; set; }
        public bool compress_image { get; set; }
        // Object state update
        public IList<Camera_t> Cameras { get; set; }
        public IList<Window_t> Windows { get; set; }
        // Additional getters
        public int num_cameras { get { return Cameras.Count(); } }
        public int screen_width { get { return cam_width; } }
        public int screen_height { get { return cam_height * num_cameras; } }


    }

    // Camera class for decoding the ZMQ messages.
    public class Camera_t
    {
        public string ID { get; set; }
        public IList<float> position { get; set; }
        public IList<float> rotation { get; set; }
        // Metadata
        public int channels { get; set; }
        public bool has_depth { get; set; }
        public int output_index { get; set; }

        // Additional getters
        public bool is_grayscale { get { return (channels == 1) && (!has_depth); } }

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
        public double utime { get; set; }
        public bool is_compressed { get; set; }
        public int cam_width { get; set; }
        public int cam_height { get; set; }
        // Additional metadata for helping with the deserialization process.
        public IList<string> camera_IDs { get; set; }
        public IList<int> channels { get; set; }

        public RenderMetadata_t(StateMessage_t state)
        {
            utime = state.utime;
            is_compressed = state.compress_image;
            cam_width = state.cam_width;
            cam_height = state.cam_height;
            camera_IDs = state.Cameras.Select(obj => obj.ID).ToList();
            channels = state.Cameras.Select(obj => obj.channels).ToList();
        }
    }

}