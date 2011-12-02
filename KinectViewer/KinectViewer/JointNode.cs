﻿using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using Microsoft.Xna.Framework;

namespace KinectViewer
{
    class JointNode
    {
        public string name { get; set; }
        public double mass { get; set; }
        public Vector3 orientation { get; set; }
        public JointNode next { get; set; }
        public Matrix localPosition { get; set; }
        public Vector3 com { get; set; }
        public float initialAngle { get; set; }
        public Matrix torsoSpacePosition;
        

        public JointNode(string name, Vector3 orientation)
        {
            this.orientation = orientation;
            this.name = name;
            this.next = null;
        }

        public JointNode()
        {

        }
        
        public Matrix MakeRotation(float angle) {
            return Matrix.CreateFromAxisAngle(orientation, initialAngle - angle);
        }
    }

    
    
}