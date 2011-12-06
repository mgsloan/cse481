using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using Microsoft.Research.Kinect.Nui;
using Microsoft.Xna.Framework;
using Microsoft.Xna.Framework.Graphics;
using Microsoft.Xna.Framework.Input;

namespace KinectViewer
{
    class KinectAngleViewer : KinectViewer
    {

        protected override void UpdateSkeleton(SkeletonData skeleton)
        {
            Vector3 elbowRight     = getLoc(skeleton.Joints[JointID.ElbowRight    ]),
                    handRight      = getLoc(skeleton.Joints[JointID.HandRight     ]),
                    shoulderRight  = getLoc(skeleton.Joints[JointID.ShoulderRight ]),
                    wristRight     = getLoc(skeleton.Joints[JointID.WristRight    ]),
                    elbowLeft      = getLoc(skeleton.Joints[JointID.ElbowLeft     ]),
                    handLeft       = getLoc(skeleton.Joints[JointID.HandLeft      ]),
                    shoulderLeft   = getLoc(skeleton.Joints[JointID.ShoulderLeft  ]),
                    wristLeft      = getLoc(skeleton.Joints[JointID.WristLeft     ]),
                    shoulderCenter = getLoc(skeleton.Joints[JointID.ShoulderCenter]),
                    spine          = getLoc(skeleton.Joints[JointID.Spine         ]),
                    hipLeft        = getLoc(skeleton.Joints[JointID.HipLeft       ]),
                    hipRight       = getLoc(skeleton.Joints[JointID.HipRight      ]),
                    kneeLeft       = getLoc(skeleton.Joints[JointID.KneeLeft      ]),
                    kneeRight      = getLoc(skeleton.Joints[JointID.KneeRight     ]),
                    ankleLeft      = getLoc(skeleton.Joints[JointID.AnkleLeft     ]),
                    ankleRight     = getLoc(skeleton.Joints[JointID.AnkleRight    ]),
                    footLeft       = getLoc(skeleton.Joints[JointID.FootLeft      ]),
                    footRight      = getLoc(skeleton.Joints[JointID.FootRight     ]);

            // legs
            Vector3 Xlegs = Vector3.Subtract(hipLeft, hipRight);
            Vector3 Ylegs = Vector3.Subtract(shoulderCenter, spine);

            Xlegs.Normalize(); Ylegs.Normalize();
            Vector3 dZlegs = Vector3.Cross(Xlegs, Ylegs);
            Vector3 dY2legs = Vector3.Cross(dZlegs, Xlegs);
            Matrix srReflegs = Matrix.CreateWorld(Vector3.Zero, dZlegs, dY2legs);
            Matrix srRefInvlegs = Matrix.Invert(srReflegs);

            curHl = hipLeft;
            curHr = hipRight;
            curFl = footLeft;
            curFr = footRight;

            // right leg
            Vector3 RUAlegs = Vector3.Subtract(kneeRight, hipRight);
            Vector3 RLAlegs = Vector3.Subtract(ankleRight, kneeRight);
            Vector3 RHlegs = Vector3.Subtract(footRight, ankleRight);
            RUAlegs.Normalize(); RLAlegs.Normalize(); RHlegs.Normalize();
            calculateAngles(skeleton, "rl", srReflegs, srRefInvlegs, RUAlegs, RLAlegs, RHlegs);

            // left leg
            Vector3 LUAlegs = flipXInRef(srReflegs, srRefInvlegs, Vector3.Subtract(kneeLeft, hipLeft));
            Vector3 LLAlegs = flipXInRef(srReflegs, srRefInvlegs, Vector3.Subtract(ankleLeft, kneeLeft));
            Vector3 LHlegs  = flipXInRef(srReflegs, srRefInvlegs, Vector3.Subtract(footLeft, ankleLeft));
            LUAlegs.Normalize(); LLAlegs.Normalize(); LHlegs.Normalize();
            calculateAngles(skeleton, "ll", srReflegs, srRefInvlegs, LUAlegs, LLAlegs, LHlegs);

            //find ankle roll based on current hip and foot positions relative to a starting
            //value (currently the point when both feet are touching the ground)
            //assume that the feet are both at the same Y and Z value WRT torso space
            //this does not change anything yet but just prints out what the results should be
            //they would be fed in the simulater, which would then do it's best to balance
            if (TwoLegStand)
            {
                //current left & right hip roll (take angle between hips and from hip to foot)
                double curRHRoll = MathUtils.AngleBetween((curFr - curHr), (curHl - curHr));
                double curLHRoll = MathUtils.AngleBetween((curFl - curHl), (curHr - curHl));

                //initial left and right hip roll (when the two legged stance was started)
                double initRHRoll = MathUtils.AngleBetween((initFr - initHr), (initHl - initHr));
                double initLHRoll = MathUtils.AngleBetween((initFl - initHl), (initHr - initHl));

                //change in left & right hip roll
                double deltaRH = curRHRoll - initRHRoll;
                double deltaLH = curLHRoll - initLHRoll;

                //initial left & right ankle roll (take angle between feet and from foot to hip)
                double initLARoll = MathUtils.AngleBetween((initHl - initFl), (initFr - initFl));
                double initRARoll = MathUtils.AngleBetween((initHr - initFr), (initFl - initFr));

                //ankles counterrotate with respect to hip
                double curLARoll = initLARoll - deltaLH;
                double curRARoll = initRARoll - deltaRH;

                Console.WriteLine("LRoll: " + curLARoll);
                Console.WriteLine("RRoll: " + curRARoll);
            }

            // arms
            Vector3 X = Vector3.Subtract(shoulderLeft, shoulderRight);
            Vector3 Y = Vector3.Subtract(shoulderCenter, spine);

            X.Normalize(); Y.Normalize();
            Vector3 dz = Vector3.Cross(X, Y);
            Vector3 dy2 = Vector3.Cross(dz, X);
            this.srRef = Matrix.CreateWorld(Vector3.Zero, dz, dy2);
            Matrix srRefInv = Matrix.Invert(srRef);

            // right arm
            Vector3 RUA = Vector3.Subtract(elbowRight, shoulderRight);
            Vector3 RLA = Vector3.Subtract(wristRight, elbowRight);
            Vector3 RH = Vector3.Subtract(handRight, wristRight);
            RUA.Normalize(); RLA.Normalize(); RH.Normalize();
            calculateAngles(skeleton, "ra", srRef, srRefInv, RUA, RLA, RH);
            
            // left arm
            Vector3 LUA = flipXInRef(srRef, srRefInv, Vector3.Subtract(elbowLeft, shoulderLeft));
            Vector3 LLA = flipXInRef(srRef, srRefInv, Vector3.Subtract(wristLeft, elbowLeft));
            Vector3 LH  = flipXInRef(srRef, srRefInv, Vector3.Subtract(handLeft, wristLeft));
            LLA.Normalize(); LUA.Normalize(); LH.Normalize();
            calculateAngles(skeleton, "la", srRef, srRefInv, LUA, LLA, LH);

            foreach (KeyValuePair<String, float> entry in kinectAngles) naoSim.UpdateAngle(entry.Key, entry.Value, .3f);

            //important to call this last (it actually sends the angles to the NAO)
            base.UpdateSkeleton(skeleton);

            // DEBUG
            //ParallelFoot(srReflegs, RUAlegs, RLAlegs);
        }

        override protected void SetTwoLegStance()
        {
            TwoLegStand = true;
            initHl = curHl;
            initHr = curHr;
            initFl = curFl;
            initFr = curFr;
            naoSim.InitializeTwoLegStance(FromKinectSpace(cur_skeleton.Joints[JointID.HipCenter].Position));
        }

        override protected void SetOneLegStance()
        {
            TwoLegStand = false;
                        
        }

        private Vector3 flipXInRef(Matrix forward, Matrix back, Vector3 vec)
        {
            Vector3 vec2 = Vector3.Transform(vec, back);
            vec2.X = -vec2.X;
            return Vector3.Transform(vec2, forward);
        }

        private void calculateAngles(SkeletonData skeleton, string extremity, Matrix srRef, Matrix srRefInv, Vector3 UA, Vector3 LA, Vector3 H)
        {
            // Compute pitch by transforming into body frame, and projecting onto Y-Z plane.
            Vector3 elocal = Vector3.Transform(UA, srRefInv);
            float pitch = (float) Math.Atan2(-elocal.Y, -elocal.Z);

            // Compute angular reference frame used for roll, by pitching our original frame forward.
            Matrix srRef2 = Matrix.Multiply(Matrix.CreateRotationX((float)Math.PI / 2 - pitch), srRef);
            Matrix srRef2Inv = Matrix.Invert(srRef2);

            // Compute roll by transforming into this frame, and projecting onto X-Y plane.
            Vector3 elocal2 = Vector3.Transform(UA, srRef2Inv);
            float roll = (float)(Math.Atan2(elocal2.X, elocal2.Y));

            // Compute angular reference frame used for elbow yaw by rolling the previous frame.
            Matrix eRef = Matrix.Multiply(Matrix.CreateRotationZ((float)Math.PI - roll), srRef2);

            float eyaw = 0f, eroll = 0f, knee = 0f, anklePitch = 0f, ankleRoll = 0f;
            bool arm = extremity == "ra" || extremity == "la";

            if (arm)
            {
                // Compute elbow yaw by transforming into this frame, and projecting onto X-Z plane.
                Vector3 wlocal = Vector3.Transform(LA, Matrix.Invert(eRef));
                eyaw = (float)(Math.Atan2(wlocal.X, -wlocal.Z));

                // Compute angular reference frame used for elbow roll by rotating the previous frame.
                Matrix eRef2 = Matrix.Multiply(Matrix.CreateRotationY(-eyaw), eRef);
                Matrix eRef2Inv = Matrix.Invert(eRef2);
                Vector3 wlocal2 = Vector3.Transform(LA, eRef2Inv);
                eroll = (float)(Math.Atan2(wlocal2.Z, wlocal2.Y));
            }
            else
            {
                knee = (float)Math.Acos(Vector3.Dot(UA, LA));

                Matrix fRef = Matrix.Multiply(Matrix.CreateRotationX(knee), eRef);
                Matrix fRefInv = Matrix.Invert(fRef);
                Vector3 fLocal = Vector3.Transform(H, fRefInv);
                anklePitch = (float)(Math.Atan2(fLocal.Z, fLocal.Y));

                
                Matrix fRef2 = Matrix.Multiply(Matrix.CreateRotationX(anklePitch), fRef);
                Matrix fRefInv2 = Matrix.Invert(fRef2);
                Vector3 fLocal2 = Vector3.Transform(H, fRefInv2);
                ankleRoll = (float)(Math.Atan2(fLocal2.X, fLocal2.Y));
            }

            float hand = (float)Math.Acos(Vector3.Dot(LA, H));

            switch (extremity)
            {
                case "ra":
                    {
                        //if (hand < 1.4 && rhand) nao.SetRHand(rhand = false);
                        //if (hand > 1.7 && !rhand) nao.SetRHand(rhand = true);
                        kinectAngles["RShoulderPitch"] = pitch;
                        kinectAngles["RShoulderRoll"] = roll - (float)Math.PI;
                        kinectAngles["RElbowYaw"] = eyaw + (float)(Math.PI / 2);
                        kinectAngles["RElbowRoll"] = eroll + (float)Math.PI;
                        break;
                    }
                case "la":
                    {
                        //if (hand < 1.4 && lhand) nao.SetLHand(lhand = false);
                        //if (hand > 1.7 && !lhand) nao.SetLHand(lhand = true);
                        kinectAngles["LShoulderPitch"] = pitch;
                        kinectAngles["LShoulderRoll"] = -(roll - (float)Math.PI);
                        kinectAngles["LElbowYaw"] = -(eyaw + (float)(Math.PI / 2));
                        kinectAngles["LElbowRoll"] = -(eroll + (float)Math.PI);
                        break;
                    }
                case "rl":
                    {
                        roll = roll - (float)Math.PI;
                        if (roll < -(float)Math.PI) roll += 2 * (float)Math.PI;
                        kinectAngles["RHipRoll"] = roll;
                        kinectAngles["RHipPitch"] = pitch - (float)Math.PI / 2;
                        kinectAngles["RKneePitch"] = knee;
                        break;
                    }
                case "ll":
                    {
                        roll = roll - (float)Math.PI;
                        if (roll < -(float)Math.PI) roll += 2 * (float)Math.PI;
                        kinectAngles["LHipRoll"] = roll;
                        kinectAngles["LHipPitch"] = pitch - (float)Math.PI / 2;
                        kinectAngles["LKneePitch"] = knee;
                        break;
                    }
            }
        }

        public Vector toNuiVec(Vector3 vec)
        {
            Vector result = new Vector();
            result.X = vec.X;
            result.Y = vec.Y;
            result.Z = vec.Z;
            return result;
        }

        //find angles that would make the flat of the foot parallel with the ground
        //UL = upper leg vector, LL = lower leg vector
        public void ParallelFoot(Matrix BodyTxform, Vector3 UL, Vector3 LL) 
        {
            Vector3 X = BodyTxform.Left;
            Vector3 X_ref = new Vector3(X.X, 0, X.Z);
            Vector3 Y_ref = new Vector3(0, 1, 0);
            Vector3 Z_ref = Vector3.Cross(X_ref, Y_ref);
            Matrix AnkleRef = Matrix.CreateWorld(new Vector3(0), Z_ref, Y_ref);            

            Vector3 origin = new Vector3(4, 0, 0);
            /*
            lines.Add(new LabelledVector(origin, origin + X_ref * 2, Color.BlanchedAlmond, "XR"));
            lines.Add(new LabelledVector(origin, origin + Y_ref * 2, Color.BurlyWood, "YR"));
            lines.Add(new LabelledVector(origin, origin + Z_ref * 2, Color.Chartreuse, "ZR"));
            */
            Vector3 Xw = new Vector3(1, 0, 0);
            Vector3 Yw = new Vector3(0, 1, 0);
            Vector3 Zw = new Vector3(0, 0, 1);
            lines.Add(new LabelledVector(origin, origin + Xw * 2, Color.BlanchedAlmond, "XR"));
            lines.Add(new LabelledVector(origin, origin + Yw * 2, Color.BurlyWood, "YR"));
            lines.Add(new LabelledVector(origin, origin + Zw * 2, Color.Chartreuse, "ZR"));

            Matrix b2W = Matrix.Invert(BodyTxform); //body 2 world transform
            Vector3 UL_tx = UL; // Vector3.Transform(UL, b2W);
            Vector3 LL_tx = LL; // Vector3.Transform(LL, b2W);

            Vector3 UL_ref = Vector3.Transform(UL_tx, AnkleRef);
            Vector3 LL_ref = Vector3.Transform(LL_tx, AnkleRef);

            lines.Add(new LabelledVector(origin, origin + UL_ref * 2, Color.DarkMagenta, "UL"));
            lines.Add(new LabelledVector(origin, origin + LL_ref * 2, Color.DarkMagenta, "LL"));

            float ankleRoll = (float) Math.Atan2(UL_ref.X, UL_ref.Y);
            float anklePitch = (float) Math.Atan2(LL_ref.Z, LL_ref.Y);

            Console.WriteLine("p: " + anklePitch);
            Console.WriteLine("r: " + ankleRoll);
        }
    }
}
