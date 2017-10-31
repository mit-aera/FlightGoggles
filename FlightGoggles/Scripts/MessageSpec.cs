using System;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;

// Array ops
using System.Linq;

namespace MessageSpec
{
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
        // Object state update
        public IList<Camera_t> Cameras { get; set; }
        public IList<Window_t> Windows { get; set; }

        // Additional getters
        public int num_cameras { get { return Cameras.Count(); } }
        public int rendered_image_width { get { return cam_width; } }
        public int rendered_image_height { get { return cam_height * num_cameras; } }
    }

    // Camera class for decoding the ZMQ messages.
    public class Camera_t
    {
        public string ID { get; set; }
        public IList<float> position { get; set; }
        public IList<float> rotation { get; set; }
        public bool initialized { get; set; } = false; // default to false.
        // Metadata
        public int channels { get; set; }
        public bool has_depth { get; set; }
        public int output_index { get; set; }

        // Additional getters
        public bool is_grayscale { get { return (channels == 1) && (!has_depth); } }

        // Get Object, creating a new one if neccessary.
        public GameObject get_gameobject(Dictionary<string, GameObject> objects, GameObject template) {
            GameObject obj;
            if (objects.ContainsKey(ID))
            {
                obj = objects[ID];
            } else
            {
                // Create object from template
                obj = GameObject.Instantiate(template);
                
            }
            return obj;
        }
    }

    // Window class for decoding the ZMQ messages.
    public class Window_t
    {
        public string ID { get; set; }
        public IList<float> position { get; set; }
        public IList<float> rotation { get; set; }
        public bool initialized { get; set; } = false; // default to false.
        // Metadata
        public IList<float> color { get; set; }
        public IList<float> size { get; set; }
        // Get Object, creating a new one if neccessary.
        public GameObject get_gameobject(Dictionary<string, GameObject> objects, GameObject template)
        {
            GameObject obj;
            if (objects.ContainsKey(ID))
            {
                obj = objects[ID];
            }
            else
            {
                // Create object from template
                obj = GameObject.Instantiate(template);

            }
            return obj;
        }
    }
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
}

}