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
                    wristRight     = getLoc(skeleton.Joints[JointID.WristRight]),
                    elbowLeft      = getLoc(skeleton.Joints[JointID.ElbowLeft]),
                    handLeft       = getLoc(skeleton.Joints[JointID.HandLeft]),
                    shoulderLeft   = getLoc(skeleton.Joints[JointID.ShoulderLeft]),
                    wristLeft      = getLoc(skeleton.Joints[JointID.WristLeft]),
                    shoulderCenter = getLoc(skeleton.Joints[JointID.ShoulderCenter]),
                    spine          = getLoc(skeleton.Joints[JointID.Spine]);

            lines.Clear();
            
            Vector3 X = Vector3.Subtract(shoulderLeft, shoulderRight);
            Vector3 Y = Vector3.Subtract(shoulderCenter, spine);

            X.Normalize(); Y.Normalize(); 
            Vector3 dz = Vector3.Cross(X, Y);
            Vector3 dy2 = Vector3.Cross(dz, X);
            Matrix srRef = Matrix.CreateWorld(Vector3.Zero, dz, dy2);
            Matrix srRefInv = Matrix.Invert(srRef);

            Vector3 RUA = Vector3.Subtract(elbowRight, shoulderRight);
            Vector3 RLA = Vector3.Subtract(wristRight, elbowRight);
            Vector3 RH = Vector3.Subtract(handRight, wristRight);
            RUA.Normalize(); RLA.Normalize(); RH.Normalize();
            calculateAngles(skeleton, true, srRef, srRefInv, RUA, RLA, RH);

            R1b = shoulderRight;
            R2b = elbowRight;
            R3b = handRight;

            // reflect across YZ plane
            Vector3 LUA = Vector3.Transform(Vector3.Subtract(elbowLeft, shoulderLeft), srRefInv);
            Vector3 LLA = Vector3.Transform(Vector3.Subtract(wristLeft, elbowLeft), srRefInv);
            Vector3 LH = Vector3.Transform(Vector3.Subtract(handLeft, wristLeft), srRefInv);

            LUA.X = -LUA.X;
            LLA.X = -LLA.X;
            LH.X = -LH.X;
            
            LUA = Vector3.Transform(LUA, srRef);
            LLA = Vector3.Transform(LLA, srRef);
            LH = Vector3.Transform(LH, srRef);

            LLA.Normalize();
            LUA.Normalize();
            LH.Normalize();

            calculateAngles(skeleton, false, srRef, srRefInv, LUA, LLA, LH);
            nao.RSSend();
        }

        bool rhand, lhand;
        Vector3 R1b, R2b, R3b;
        Matrix R1, R2;

        private void calculateAngles(SkeletonData skeleton, bool right, Matrix srRef, Matrix srRefInv, Vector3 UA, Vector3 LA, Vector3 H)
        {
            // Compute angular reference frame used for pitch

            // Compute pitch by transforming into body frame, and projecting onto Y-Z plane.
            
            Vector3 elocal = Vector3.Transform(UA, srRefInv);
            // elocal.Z and elocal.Y both near 0 at once has wierd singularities - pick a default pitch
            float pitch = (float)(Math.Abs(Math.Sqrt(Math.Pow(elocal.Z, 2) + Math.Pow(elocal.Y, 2))) < .2 ? Math.PI / 2
                        : Math.Atan2(-elocal.Y, -elocal.Z));

            //Console.WriteLine("UA_PITCH: " + pitch);

            // Compute angular reference frame used for roll, by pitching our original frame forward.
            Matrix srRef2 = Matrix.Multiply(Matrix.CreateRotationX((float)Math.PI / 2 - pitch), srRef);
            Matrix srRef2Inv = Matrix.Invert(srRef2);
            // Compute roll by transforming into this frame, and projecting onto X-Y plane.
            Vector3 elocal2 = Vector3.Transform(UA, srRef2Inv);
            float roll = (float)(Math.Atan2(elocal2.X, elocal2.Y));

            //elocal2.Normalize();
            //lines.Add(new LabelledVector(Vector3.Zero, elocal2 * 3, Color.Gold, "e_local"));

            // Compute angular reference frame used for elbow pitch by rolling the previous frame.
            Matrix eRef = Matrix.Multiply(Matrix.CreateRotationZ((float)Math.PI - roll), srRef2);

            // Compute elbow yaw by transforming into this frame, and projecting onto X-Z plane.
            Vector3 wlocal = Vector3.Transform(LA, Matrix.Invert(eRef));
            float eyaw = (float)(Math.Atan2(wlocal.X, -wlocal.Z));

            // Compute angular reference frame used for elbow roll by rotating the previous frame.
            Matrix eRef2 = Matrix.Multiply(Matrix.CreateRotationY(-eyaw), eRef);
            Matrix eRef2Inv = Matrix.Invert(eRef2);
            Vector3 wlocal2 = Vector3.Transform(LA, eRef2Inv);
            float eroll = (float)(Math.Atan2(wlocal2.Z, wlocal2.Y));
            
            /*
            Matrix wRef = Matrix.Multiply(Matrix.CreateRotationX(eroll), eRef2);
            Vector3 hlocal = Vector3.Transform(H, Matrix.Invert(wRef));
            float wroll = (float)(Math.Atan2(hlocal.Z, hlocal.X));

            Matrix wRef2 = Matrix.Multiply(Matrix.CreateRotationY(wroll), wRef);
            Vector3 hlocal2 = Vector3.Transform(H, Matrix.Invert(wRef2));
            float hand = (float)(Math.Atan2(hlocal2.Y, hlocal2.Z));
            */

            //hlocal2.Normalize();
            //lines.Add(new LabelledVector(Vector3.Zero, hlocal2 * 3, Color.Gold, "h_local"));

            if (right)
            {
                R1 = srRefInv;
                R2 = eRef2Inv;
                //if (hand < 1.4 && rhand) nao.SetRHand(rhand = false);
                //if (hand > 1.7 && !rhand) nao.SetRHand(rhand = true);
              //  debugReferenceFrame("wr = " + hand.ToString(), wRef, 3, getLoc(skeleton.Joints[JointID.WristRight]));
                nao.RSUpdatePitch(pitch);
                nao.RSUpdateRoll(roll - (float)Math.PI);
                nao.REUpdateYaw(eyaw + (float)(Math.PI / 2));
                nao.REUpdateRoll(eroll + (float)Math.PI);
                //debugReferenceFrame("", srRef2, 3, getLoc(skeleton.Joints[JointID.ShoulderRight]));
                //debugReferenceFrame("", eRef2, 3, getLoc(skeleton.Joints[JointID.ElbowRight]));
                //debugReferenceFrame("", wRef2, 3, getLoc(skeleton.Joints[JointID.WristRight]));
            }
            else
            {
                //if (hand < 1.4 && lhand) nao.SetLHand(lhand = false);
                //if (hand > 1.7 && !lhand) nao.SetLHand(lhand = true);
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
            //debugReferenceFrame("", srRef, 3, getLoc(shoulderRight));
            //debugReferenceFrame(, srRef, 3, shoulderRight);
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

        public Vector toNuiVec(Vector3 vec)
        {
            Vector result = new Vector();
            result.X = vec.X;
            result.Y = vec.Y;
            result.Z = vec.Z;
            return result;
        }
        /*
        public void localCloud(PlanarImage image)
        {
            float x, y;
            short val;
            nui.SkeletonEngine.SkeletonToDepthImage(R3b, out x, out y, out val);
        }*/
    }
}
