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
            

            


            //TODO: send the information to the NAO 
            naoSim.UpdatePositions();
            base.updateSkeleton(skeleton);


            // DEBUG

            ParallelFoot(srReflegs, RUAlegs, RLAlegs);

            /*
            //float UL_len, LL_len;
            //Vector3.Distance(ref hipLeft, ref kneeLeft, out UL_len);
            //Vector3.Distance(ref kneeLeft, ref footLeft, out LL_len);

            //double[] legAngles = LegIK(srRefInv, new Vector3(0, 0, 0), footLeft - hipLeft, 5, 5);
            //nao.LHUpdateRoll((float) (legAngles[1] + Math.PI / 2));
            //nao.LHUpdatePitch((float) (legAngles[0] + Math.PI / 2));
            //nao.LKUpdatePitch((float) (Math.PI - legAngles[2]));

            Vector3 origin = new Vector3(4, 0, 0);
            Vector3 displayHipRoll = new Vector3(-2, 10, 0);
            lines.Add(new LabelledVector(origin + displayHipRoll, origin + displayHipRoll, Color.Black, "HipRoll: " + legAngles[1]));

            Vector3 displayHipPitch = new Vector3(-2, 12, 0);
            lines.Add(new LabelledVector(origin + displayHipPitch, origin + displayHipPitch, Color.Black, "HipPitch: " + (legAngles[0])));

            Vector3 displayKneePitch = new Vector3(-2, 14, 0);
            lines.Add(new LabelledVector(origin + displayKneePitch, origin + displayKneePitch, Color.Black, "KneePitch: " + legAngles[2]));

            //legAngles = LegIK(srRefInv, hipRight, footRight, UL_len, LL_len);
            //nao.RHUpdateRoll((float) (legAngles[1] + Math.PI / 2));
            //nao.RHUpdatePitch((float) (legAngles[0] + Math.PI / 2));
            //nao.RKUpdatePitch((float) (Math.PI - legAngles[2]));
            */
            // END DEBUG
        }

        private Vector3 flipXInRef(Matrix forward, Matrix back, Vector3 vec)
        {
            Vector3 vec2 = Vector3.Transform(vec, back);
            vec2.X = -vec2.X;
            return Vector3.Transform(vec2, forward);
        }

        private void calculateAngles(SkeletonData skeleton, string extremity, Matrix srRef, Matrix srRefInv, Vector3 UA, Vector3 LA, Vector3 H)
        {
            // Compute angular reference frame used for pitch

            // Compute pitch by transforming into body frame, and projecting onto Y-Z plane.
            
            Vector3 elocal = Vector3.Transform(UA, srRefInv);
            // elocal.Z and elocal.Y both near 0 at once has wierd singularities - pick a default pitch
            //float pitch = (float)(Math.Abs(Math.Sqrt(Math.Pow(elocal.Z, 2) + Math.Pow(elocal.Y, 2))) < .2 ? (Math.PI / 2)
            //            : Math.Atan2(-elocal.Y, -elocal.Z));
            float pitch;
            //if (Math.Sqrt(Math.Pow(elocal.Z, 2) + Math.Pow(elocal.Y, 2)) < .5) 
            //    pitch = 0;
            //else 
                pitch = (float) Math.Atan2(-elocal.Y, -elocal.Z);

            //Console.WriteLine("UA_PITCH: " + pitch);

            // Compute angular reference frame used for roll, by pitching our original frame forward.
            Matrix srRef2 = Matrix.Multiply(Matrix.CreateRotationX((float)Math.PI / 2 - pitch), srRef);
            Matrix srRef2Inv = Matrix.Invert(srRef2);
            // Compute roll by transforming into this frame, and projecting onto X-Y plane.
            Vector3 elocal2 = Vector3.Transform(UA, srRef2Inv);
            float roll = (float)(Math.Atan2(elocal2.X, elocal2.Y));

            //elocal2.Normalize();
            //lines.Add(new LabelledVector(Vector3.Zero, elocal2 * 3, Color.Gold, "e_local"));

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

                debugReferenceFrame(ankleRoll.ToString(), fRef2, 4, getLoc(skeleton.Joints[extremity == "rl" ? JointID.AnkleLeft : JointID.AnkleRight]));
                debugReferenceFrame(anklePitch.ToString(), eRef, 4, getLoc(skeleton.Joints[extremity == "rl" ? JointID.KneeLeft : JointID.KneeRight]));
            }

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

            switch (extremity)
            {
                case "ra":
                    {
                        //if (hand < 1.4 && rhand) nao.SetRHand(rhand = false);
                        //if (hand > 1.7 && !rhand) nao.SetRHand(rhand = true);
                        //  debugReferenceFrame("wr = " + hand.ToString(), wRef, 3, getLoc(skeleton.Joints[JointID.WristRight]));
                        naoSim.UpdateAngle("RShoulderPitch", pitch);
                        naoSim.UpdateAngle("RShoulderRoll", roll - (float)Math.PI);
                        naoSim.UpdateAngle("RElbowYaw", eyaw + (float)(Math.PI / 2));
                        naoSim.UpdateAngle("RElbowRoll", eroll + (float)Math.PI);
                        
                        break;
                    }
                case "la":
                    {
                        //if (hand < 1.4 && lhand) nao.SetLHand(lhand = false);
                        //if (hand > 1.7 && !lhand) nao.SetLHand(lhand = true);
                        naoSim.UpdateAngle("LShoulderPitch", pitch);
                        naoSim.UpdateAngle("LShoulderRoll", -(roll - (float)Math.PI));
                        naoSim.UpdateAngle("LElbowYaw", -(eyaw + (float)(Math.PI / 2)));
                        naoSim.UpdateAngle("LElbowRoll", -(eroll + (float)Math.PI));

                        //Console.WriteLine("elbowyaw: " + (-(eyaw + (float)(Math.PI / 2))));
                        //Console.WriteLine("elbowroll: " + (-(eroll + (float)Math.PI)));
                     
                        break;
                    }
                case "rl":
                    {
                        roll = roll - (float)Math.PI;
                        if (roll < -(float)Math.PI) roll += 2 * (float)Math.PI;
                        naoSim.UpdateAngle("RHipRoll", roll);
                        naoSim.UpdateAngle("RHipPitch", pitch - (float)Math.PI / 2);
                        naoSim.UpdateAngle("RKneePitch", knee);
                        
                        
                        /*
                        if (skeleton.Joints[JointID.FootRight].TrackingState == JointTrackingState.Tracked)
                        {
                            
                            naoSim.UpdateAngle("RAnklePitch", anklePitch, 0.5f);
                            naoSim.UpdateAngle("RAnkleRoll", ankleRoll,0.5f);
                        }
                        */
                        break;
                    }
                case "ll":
                    {
                        roll = roll - (float)Math.PI;
                        if (roll < -(float)Math.PI) roll += 2 * (float)Math.PI;

                        naoSim.UpdateAngle("LHipRoll", roll);
                        naoSim.UpdateAngle("LHipPitch", pitch - (float)Math.PI / 2);
                        naoSim.UpdateAngle("LKneePitch", knee);
                       
                        
                        
                        //naoSim.RKUpdatePitch(knee / 2);

                        /*
                        if (skeleton.Joints[JointID.FootLeft].TrackingState == JointTrackingState.Tracked) {
                            
                            naoSim.UpdateAngle("LAnklePitch", anklePitch, 0.5f);
                            naoSim.UpdateAngle("LAnkleRoll", ankleRoll, 0.5f);
                         
                       
                        }
                        */
                        break;
                    }
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
