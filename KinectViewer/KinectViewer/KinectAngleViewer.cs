using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using Microsoft.Research.Kinect.Nui;
using Microsoft.Xna.Framework;
using Microsoft.Xna.Framework.Graphics;

namespace KinectViewer
{
    class KinectAngleViewer : KinectViewer
    {

        protected override void updateSkeleton(SkeletonData skeleton)
        {
            Vector3 elbowRight     = getLoc(skeleton.Joints[JointID.ElbowRight]),
                    handRight      = getLoc(skeleton.Joints[JointID.HandRight]),
                    shoulderRight  = getLoc(skeleton.Joints[JointID.ShoulderRight]),
                    shoulderLeft   = getLoc(skeleton.Joints[JointID.ShoulderLeft]),
                    shoulderCenter = getLoc(skeleton.Joints[JointID.ShoulderCenter]),
                    spine          = getLoc(skeleton.Joints[JointID.Spine]),
                    elbowLeft      = getLoc(skeleton.Joints[JointID.ElbowLeft]);

            lines.Clear();
            
            // Compute angular reference frame used for pitch
            Vector3 dx = Vector3.Subtract(shoulderLeft, shoulderRight);
            Vector3 dy = Vector3.Subtract(shoulderCenter, spine);
            dx.Normalize(); dy.Normalize();
            Vector3 dz = Vector3.Cross(dx, dy);
            Matrix srRef = Matrix.CreateWorld(Vector3.Zero, dz, dy);

            // Compute pitch by transforming into body frame, and projecting onto Y-Z plane.
            Vector3 s_e = Vector3.Subtract(elbowRight, shoulderRight);
            Vector3 elocal = Vector3.Transform(s_e, Matrix.Invert(srRef));
            // elocal.Z and elocal.Y both near 0 at once has wierd singularities - pick a default pitch
            float pitch = (float)(Math.Abs(elocal.Z) < 0.1 ? 0 : (Math.Atan2(elocal.Z, elocal.Y) + Math.PI));

            // Compute angular reference frame used for roll, by pitching our original frame forward.
            Matrix srRef2 = Matrix.Multiply(Matrix.CreateRotationX(pitch), srRef);

            // Compute roll by transforming into this frame, and projecting onto X-Y plane.
            Vector3 elocal2 = Vector3.Transform(s_e, Matrix.Invert(srRef2));
            float roll = (float)(Math.Atan2(elocal2.X, elocal2.Y));

            // Compute angular reference frame used for elbow pitch by rolling the previous frame.
            Matrix eRef = Matrix.Multiply(Matrix.CreateRotationZ((float)Math.PI - roll), srRef2);

            // Compute elbow pitch by transforming into this frame, and projecting onto X-Z plane.
            Vector3 e_h = Vector3.Subtract(handRight, elbowRight);
            Vector3 hlocal = Vector3.Transform(e_h, Matrix.Invert(eRef));
            float epitch = (float)(Math.Atan2(hlocal.X, hlocal.Z));

            // Compute angular reference frame used for elbow roll by rotating the previous frame.
            Matrix eRef2 = Matrix.Multiply(Matrix.CreateRotationY((float)Math.PI + epitch), eRef);

            // visualizations of values involved

            Vector3 offset = Vector3.Add(spine, new Vector3(5, 0, 0));
            //lines.Add(new LabelledVector(offset, Vector3.Add(offset, Vector3.Multiply(new Vector3(elocal2.X, elocal2.Y, elocal2.Z), 5)), Color.Black, "pitch = " + pitch.ToString()));
            //lines.Add(new LabelledVector(offset, Vector3.Add(offset, Vector3.Multiply(new Vector3(0, elocal.Y, elocal.Z), 5)), Color.Black, "pitch = " + pitch.ToString()));
            //lines.Add(new LabelledVector(offset, Vector3.Add(offset, Vector3.Multiply(new Vector3(elocal2.X, elocal2.Y, 0), 5)), Color.Black, "roll = " + roll.ToString()));
            debugReferenceFrame(roll.ToString(), srRef, 3, shoulderRight);
            //debugReferenceFrame("sr2", srRef2, 3);
            //debugReferenceFrame(epitch.ToString(), eRef, 3, elbowRight);
            debugReferenceFrame(epitch.ToString(), eRef2, 3, elbowRight);
        }
        
        private void debugReferenceFrame(String str, Matrix m, float sz)
        {
            debugReferenceFrame(str, m, sz, m.Translation);
        }

        private void debugReferenceFrame(String str, Matrix m, float sz, Vector3 origin)
        {
            lines.Add(new LabelledVector(origin, origin + m.Right * sz, Color.Red, str));
            lines.Add(new LabelledVector(origin, origin + m.Up * sz, Color.Green, ""));
            lines.Add(new LabelledVector(origin, origin + m.Forward * sz, Color.Blue, ""));
        }
    }
}
