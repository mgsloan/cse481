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
                    handLeft       = getLoc(skeleton.Joints[JointID.HandLeft]),
                    shoulderLeft   = getLoc(skeleton.Joints[JointID.ShoulderLeft]),
                    shoulderCenter = getLoc(skeleton.Joints[JointID.ShoulderCenter]),
                    spine          = getLoc(skeleton.Joints[JointID.Spine]),
                    elbowLeft      = getLoc(skeleton.Joints[JointID.ElbowLeft]);

            lines.Clear();
            
            Vector3 X = Vector3.Subtract(shoulderLeft, shoulderRight);
            Vector3 Y = Vector3.Subtract(shoulderCenter, spine);
            Vector3 RUA = Vector3.Subtract(elbowRight, shoulderRight);
            Vector3 RLA = Vector3.Subtract(handRight, elbowRight);
            calculateAngles(true, X, Y, RUA, RLA);

            Vector3 LUA = Vector3.Subtract(elbowLeft, shoulderLeft);
            Vector3 LLA = Vector3.Subtract(handLeft, elbowLeft);
            calculateAngles(false, X, Y, LUA, LLA);
        }

        private void calculateAngles(bool right, Vector3 X, Vector3 Y, Vector3 UA, Vector3 LA)
        {
            // Compute angular reference frame used for pitch

            X.Normalize(); Y.Normalize(); UA.Normalize(); LA.Normalize(); 
            Vector3 dz = Vector3.Cross(X, Y);
            Vector3 dy2 = Vector3.Cross(dz, X);
            Matrix srRef = Matrix.CreateWorld(Vector3.Zero, dz, dy2);

            // reflect across YZ plane 
            if (!right)
            {
                Vector3 UA_t = Vector3.Transform(UA, Matrix.Invert(srRef));
                Vector3 LA_t = Vector3.Transform(LA, Matrix.Invert(srRef));
                LA_t.X = -LA_t.X;
                UA_t.X = -UA_t.X;

                //lines.Add(new LabelledVector(Vector3.Zero, LA * 3, Color.Gold, "LA"));
                //lines.Add(new LabelledVector(Vector3.Zero, Y * 3, Color.Red, "Y"));
                //lines.Add(new LabelledVector(Vector3.Zero, dz * 3, Color.Green, "Z"));

                LA = Vector3.Transform(LA_t, srRef);
                UA = Vector3.Transform(UA_t, srRef);

                LA.Normalize();
                UA.Normalize();
                //lines.Add(new LabelledVector(Vector3.Zero, LA * 3, Color.Blue, "LA_T"));
            }
            // end new

            // Compute pitch by transforming into body frame, and projecting onto Y-Z plane.
            
            Vector3 elocal = Vector3.Transform(UA, Matrix.Invert(srRef));
            // elocal.Z and elocal.Y both near 0 at once has wierd singularities - pick a default pitch
            float pitch = (float)(Math.Abs(Math.Sqrt(Math.Pow(elocal.Z, 2) + Math.Pow(elocal.Y, 2))) < .2 ? 0
                        : Math.Atan2(-elocal.Y, -elocal.Z));

            //Console.WriteLine("UA_PITCH: " + pitch);

            // Compute angular reference frame used for roll, by pitching our original frame forward.
            Matrix srRef2 = Matrix.Multiply(Matrix.CreateRotationX((float)Math.PI / 2 - pitch), srRef);

            // Compute roll by transforming into this frame, and projecting onto X-Y plane.
            Vector3 elocal2 = Vector3.Transform(UA, Matrix.Invert(srRef2));
            float roll = (float)(Math.Atan2(elocal2.X, elocal2.Y));

            elocal2.Normalize();
            lines.Add(new LabelledVector(Vector3.Zero, elocal2 * 3, Color.Gold, "e_local"));

            // Compute angular reference frame used for elbow pitch by rolling the previous frame.
            Matrix eRef = Matrix.Multiply(Matrix.CreateRotationZ((float)Math.PI - roll), srRef2);

            // Compute elbow pitch by transforming into this frame, and projecting onto X-Z plane.
            Vector3 hlocal = Vector3.Transform(LA, Matrix.Invert(eRef));
            float eyaw = (float)(Math.Atan2(hlocal.X, -hlocal.Z));

            // Compute angular reference frame used for elbow roll by rotating the previous frame.
            Matrix eRef2 = Matrix.Multiply(Matrix.CreateRotationY(-eyaw), eRef);
            Vector3 hlocal2 = Vector3.Transform(LA, Matrix.Invert(eRef2));
            float eroll = (float)(Math.Atan2(hlocal2.Z, hlocal2.Y));

            hlocal2.Normalize();
            lines.Add(new LabelledVector(Vector3.Zero, hlocal2 * 3, Color.Gold, "h_local"));

            if (right)
            {
                nao.RSUpdatePitch(pitch);
                nao.RSUpdateRoll(roll - (float)Math.PI);
                nao.REUpdateYaw(eyaw + (float)(Math.PI / 2));
                nao.REUpdateRoll(eroll + (float)Math.PI);
            }
            else
            {
                nao.LSUpdatePitch(pitch);
                nao.LSUpdateRoll(-(roll - (float)Math.PI));
                nao.LEUpdateYaw(-(eyaw + (float)(Math.PI / 2)));
                nao.LEUpdateRoll(-(eroll + (float)Math.PI));
            }

            // visualizations of values involved
            //Vector3 offset = Vector3.Add(spine, new Vector3(5, 0, 0));
            //lines.Add(new LabelledVector(offset, Vector3.Add(offset, Vector3.Multiply(new Vector3(elocal2.X, elocal2.Y, elocal2.Z), 5)), Color.Black, "pitch = " + pitch.ToString()));
            //lines.Add(new LabelledVector(offset, Vector3.Add(offset, Vector3.Multiply(new Vector3(0, elocal.Y, elocal.Z), 5)), Color.Black, "pitch = " + pitch.ToString()));
            //lines.Add(new LabelledVector(offset, Vector3.Add(offset, Vector3.Multiply(new Vector3(elocal2.X, elocal2.Y, 0), 5)), Color.Black, "roll = " + roll.ToString()));
            //debugReferenceFrame(roll.ToString(), srRef, 3, shoulderRight);
            //debugReferenceFrame("sr2", srRef2, 3);
            //debugReferenceFrame(eyaw.ToString(), eRef, 3, elbowRight);
            //debugReferenceFrame(eroll.ToString(), eRef2, 3, elbowRight);
        }

        private void debugReferenceFrame(String str, Matrix m, float sz)
        {
            debugReferenceFrame(str, m, sz, m.Translation);
        }

        private void debugReferenceFrame(String str, Matrix m, float sz, Vector3 origin)
        {
            lines.Add(new LabelledVector(origin, origin + m.Right * sz, Color.Black, str));
            lines.Add(new LabelledVector(origin, origin + m.Up * sz, Color.Green, ""));
            lines.Add(new LabelledVector(origin, origin + m.Forward * sz, Color.Blue, ""));
        }
    }
}
