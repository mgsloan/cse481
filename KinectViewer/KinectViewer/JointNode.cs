using System;
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
        public Vector3 position { get; set; }

        public JointNode(string name, double mass)
        {
            this.name = name;
            this.mass = mass;
        }

        


    }

    
    
}
