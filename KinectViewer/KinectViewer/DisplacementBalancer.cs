using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using Microsoft.Xna.Framework;

namespace KinectViewer
{
    class DisplacementBalancer
    {
        
        NaoSimulator naoSim;
        float feetWidth;
        float hipWidth;
        float skew;
        float uLength;
        float lLength;
        float length;
        double[] initialAngles;
        public float lforward, rforward, lleftward, rleftward;
        string[] legJoints;

        // How much of a squat we're in - from 0 to 1.
        public float curVT;
        // side-to-side. 0.5 == center
        public float curHT;

        public DisplacementBalancer(NaoSimulator naoSim)
        {
            this.naoSim = naoSim;
            legJoints = new string[] { "LHipRoll", "RHipRoll", "LHipPitch", "RHipPitch", "LKneePitch", "RKneePitch" };
        }

        public void InitializeTwoLegStance(Vector3 initCenter)
        {
            Vector3 lhip = naoSim.GetPosition("LHipRoll"),  rhip = naoSim.GetPosition("RHipRoll");
            Vector3 lknee = naoSim.GetPosition("LKneePitch");
            Vector3 lfoot = naoSim.GetPosition("LAnkleRoll"), rfoot = naoSim.GetPosition("RAnkleRoll");
            this.hipWidth  = Vector3.Distance(lhip,  rhip);
            this.feetWidth = 3f; //Vector3.Distance(lfoot, rfoot);
            this.uLength   = Vector3.Distance(lhip,  lknee);
            this.lLength   = Vector3.Distance(lknee, lfoot);
            this.length = uLength + lLength;
            this.skew = (feetWidth - hipWidth) / 2;
            this.initialAngles = LegIK(Matrix.Identity, new Vector3(0, length - 0.01f, 0), Vector3.Zero);
        }

        public void AnkleBalance(FootState state)
        {
            Vector3 com = naoSim.GetCOM();
            if (state == FootState.LEFT)
            {
                string prefix = "L";
                Vector3 delta = Vector3.Subtract(com, naoSim.GetPosition(prefix + "AnkleRoll"));
                TweakedBalance(prefix, delta);
            }
            else if (state == FootState.RIGHT)
            {
                string prefix = "R";
                Vector3 delta = Vector3.Subtract(com, naoSim.GetPosition(prefix + "AnkleRoll"));
                TweakedBalance(prefix, delta);
            } else if (state == FootState.BOTH)
            {
                Vector3 delta = Vector3.Subtract(com, BetweenFeet());
                delta.X = 0;
                TweakedBalance("L", delta);
                TweakedBalance("R", delta);
            }
        }

        public void TweakedBalance(String prefix, Vector3 target)
        {
            var targetFoot = prefix == "R" ? naoSim.GetRightFoot() : naoSim.GetLeftFoot();
            float forwardBias = MathUtils.Average(targetFoot.ffl - targetFoot.frl, targetFoot.ffr - targetFoot.frr) * 0.01f;
            float leftwardBias = MathUtils.Average(targetFoot.ffl - targetFoot.ffr, targetFoot.frl - targetFoot.frr) * 0.01f;
            if (prefix == "R")
            {
                // TODO: PID!
                rforward += forwardBias;
                rforward *= 0.5f;
                rleftward += leftwardBias;
                rleftward *= 0.5f;
                Viewer.debugOrigin = new Vector3(-7f, 2f, 0f);
                Viewer.DebugVector("", new Vector3(leftwardBias * 100, forwardBias * 100, 0), Color.OrangeRed);
                Viewer.DebugVector("rb", new Vector3(rleftward * 100, rforward * 100, 00), Color.MediumVioletRed);
                SetFootNormal(prefix, target, rforward - 0.08f, rleftward);
            }
            else
            {
                // TODO: PID!
                lforward += forwardBias;
                lforward *= 0.5f;
                lleftward += leftwardBias;
                lleftward *= 0.5f;
                Viewer.debugOrigin = new Vector3(-5f, 2f, 0f);
                Viewer.DebugVector("", new Vector3(leftwardBias * 100, forwardBias * 100, 0), Color.OrangeRed);
                Viewer.DebugVector("lb", new Vector3(lleftward * 100, lforward * 100, 0), Color.MediumVioletRed);
                SetFootNormal(prefix, target, lforward - 0.08f, lleftward);
            }
        }

        public void SetFootNormal(String prefix, Vector3 target)
        {
            SetFootNormal(prefix, target, 0.0f, 0.0f);
        }

        public void SetFootNormal(String prefix, Vector3 target,  float pitchTweak, float rollTweak) {
            // Transform into lower leg local space.
            Matrix mat = MathUtils.ExtractRotation(Matrix.Invert(naoSim.GetTransform(prefix + "KneePitch")));
            Vector3 local = Vector3.Transform(target, mat);

            // Take the angle of the vector to be the angle we need to rotate
            // the ground plane in order to achieve balance.
            float pitch = (float)Math.Atan2(local.Z, local.Y);
            float roll = (float)Math.Atan2(local.X, local.Y);

            naoSim.AUpdate(prefix, pitch + pitchTweak, -roll + rollTweak); //(prefix == "R" ? -0.05f : 0.05f));
        }

        public Vector3 BetweenFeet()
        {
            Vector3 result;
            Vector3 lfoot = naoSim.GetPosition("LAnkleRoll"), rfoot = naoSim.GetPosition("RAnkleRoll");
            Vector3.Lerp(ref lfoot, ref rfoot, 0.5f, out result);
            return result;
        }

        public void Balance(FootState state, Vector3 position, float width)
        {
            if (state == FootState.BOTH)
            {
                TwoLegs(position, width);
            }
            else if (state == FootState.LEFT)
            {
                curHT = 0.5f;
            }
            naoSim.UpdatePositions();
            AnkleBalance(state);
        }
        
        //for when both feet are on the ground
        public void TwoLegs(Vector3 displacement, float width)
        {

            if (width < 2.3f) width = 2.3f;

            displacement.Z = 0.0f;
            Viewer.DebugVector("d1", new Vector3(-8f, -2f, 0f), displacement, Color.Plum);

            float currentFeetWidth = MathUtils.Lerp(curVT, width, 1.5f);
            Console.WriteLine(displacement.Y.ToString());
            displacement.X *= (1.2f - curVT);
            // Compute hip / knee

            if (displacement.Y < length * -0.85f)
            {
                displacement.Y = length * -0.85f;
                displacement.X = 0f;
                //currentFeetWidth = hipWidth;
            }

            Vector3 hipL = new Vector3(displacement.X - hipWidth / 2, length * 0.9f + displacement.Y, 0);
            Vector3 targetL = new Vector3(currentFeetWidth / -2f, 0, 0);
            Vector3 deltaL = Vector3.Subtract(targetL, hipL);

            float ldist = deltaL.Length();

            if (ldist > length) {
                Vector3 correction = Vector3.Subtract(deltaL, Vector3.Multiply(deltaL, length / ldist - 0.02f));
                Viewer.DebugVector("cl", Vector3.Add(new Vector3(-8f, -2f, 0f), displacement), correction, Color.Snow);
                hipL = Vector3.Add(correction, hipL);
                displacement = Vector3.Add(correction, displacement);
            }

            Vector3 hipR = new Vector3(displacement.X + hipWidth / 2, length * 0.9f + displacement.Y, 0);
            Vector3 targetR = new Vector3(currentFeetWidth / 2f, 0, 0);
            Vector3 deltaR = Vector3.Subtract(targetR, hipR);
            float rdist = deltaR.Length();

            if (rdist > length) {
                Vector3 correction = Vector3.Subtract(deltaR, Vector3.Multiply(deltaR, length / rdist - 0.02f));
                Viewer.DebugVector("cr", Vector3.Add(new Vector3(-8f, -2f, 0f), displacement), correction, Color.Snow);
                hipR = Vector3.Add(correction, hipR);
                hipL = Vector3.Add(correction, hipL);
                displacement = Vector3.Add(correction, displacement);
            }

            curVT = MathUtils.Clamp((float)Math.Pow(-displacement.Y / (length * 0.9f), 1f), 0, 1);
            curHT = (float)(0.5f + displacement.X / feetWidth);
            Viewer.DebugVector(curHT.ToString(), new Vector3(-8f, -3f, 0f), new Vector3(0, curHT * -5f, 0), Color.MediumSpringGreen);

            double[] anglesr, anglesl;
            anglesr = LegIK(Matrix.Identity, hipR, targetR);
            anglesl = LegIK(Matrix.Identity, hipL, targetL);

            float rcHipPitch  = (float)(anglesr[0] - initialAngles[0]);
            float rcHipRoll   = (float)(anglesr[1] - initialAngles[1]);
            float rcKneePitch = (float)(anglesr[2] - initialAngles[2]);

            float lcHipPitch = (float)(anglesl[0] - initialAngles[0]);
            float lcHipRoll  = (float)(anglesl[1] - initialAngles[1]);
            float lcKneePitch = (float)(anglesl[2] - initialAngles[2]);

            Viewer.debugOrigin = BetweenFeet();
            //Viewer.DebugVector("", hipL, Color.Red);
            //Viewer.DebugVector("", hipR, Color.Red);
            Viewer.DebugVector(lcKneePitch.ToString(), new Vector3(width, 0, 0), Color.MintCream);
            /*
            Viewer.debugOrigin = new Vector3(-3f, 0, 0);
            Viewer.DebugReferenceFrameAtOrigin(lcHipPitch.ToString(), Matrix.CreateRotationX(lcHipPitch));
            Viewer.debugOrigin = new Vector3(-6f, 1f, 0);
            Viewer.DebugReferenceFrameAtOrigin(lcHipRoll.ToString(), Matrix.CreateRotationZ(lcHipRoll));
            Viewer.debugOrigin = new Vector3(-9f, 2f, 0);
            Viewer.DebugReferenceFrameAtOrigin(lcKneePitch.ToString(), Matrix.CreateRotationX(lcKneePitch));
            */

            //naoSim.SenseJoints(legJoints);
            //naoSim.UpdatePositions();
            //AnkleBalance(FootState.BOTH);

            // Apply some leg safety.
            /*
            float hipDelta = rcHipRoll - lcHipRoll;
            if (hipDelta > 5) {
                rcHipRoll -= hipDelta / 2;
                lcHipRoll += hipDelta / 2;
            }*/

            float smooth = 0.2f;
            naoSim.UpdateAngle("RHipPitch",  -rcHipPitch, smooth);
            naoSim.UpdateAngle("RHipRoll", rcHipRoll, smooth);
            //naoSim.UpdateAngle("RAnkleRoll", rcHipRoll, smooth);
            naoSim.UpdateAngle("RKneePitch", rcKneePitch, smooth);

            naoSim.UpdateAngle("LHipPitch", -lcHipPitch, smooth);
            naoSim.UpdateAngle("LHipRoll", lcHipRoll, smooth);
            //naoSim.UpdateAngle("LAnkleRoll", lcHipRoll, smooth);
            naoSim.UpdateAngle("LKneePitch", lcKneePitch, smooth);
        }

        private double[] LegIK(Matrix BodyTxform, Vector3 hip, Vector3 foot)
        {
            // (1) find hip roll 
            // from 0 (X-axis) to -pi (negative X-axis) where X-axis is model's right-to-left vector
            // get hip to foot vector in world space
            Vector3 hip_to_foot = foot - hip;
            // txform to torso space
            Vector3 hip_to_foot_tx = Vector3.Transform(hip_to_foot, BodyTxform);

            // project onto XY plane in torso space
            double hiproll = -Math.Atan2(hip_to_foot_tx.Y, hip_to_foot_tx.X) - Math.PI;

            // now do other two angles, will need distance from hip to foot for this
            float hip_to_foot_len;
            Vector3.Distance(ref hip, ref foot, out hip_to_foot_len);

            // (2) find knee pitch (easy, use law of cosines)
            double a2 = uLength;
            double b2 = lLength;
            double c2 = hip_to_foot_len;
            double kneepitch;
            if (c2 > a2 + b2) kneepitch = 0;
            else kneepitch = Math.Acos((a2 * a2 + b2 * b2 - c2 * c2) / (-2 * a2 * b2));

            // (3) find hip pitch (there are two parts to this) 
            // from 0 (Z-axis) to -pi (negative Z-axis) where Z-axis is vector out of model's torso

            // part 1: use law of cosines
            double a1 = hip_to_foot_len;
            double b1 = uLength;
            double c1 = lLength;
            double p2 = Math.Acos((a1 * a1 + b1 * b1 - c1 * c1) / (2 * a1 * b1));

            // part 2: rotate hip-to-foot vector into YZ plane, and then project it onto YZ plane in torso space
            // rotate
            float theta = (float)(-hiproll - Math.PI / 2);
            Matrix YZfix = Matrix.CreateRotationZ(-theta);
            var hip_to_foot_yz = Vector3.Transform(hip_to_foot_tx, YZfix);

            // project
            double p1 = Math.Atan2(hip_to_foot_yz.Y, hip_to_foot_yz.Z);

            // now combine part 1 and 2
            double hippitch = p2 + p1;

            //double anklepitch = 

            // return five angles in a double[]
            double[] angles = new double[5];
            angles[0] = hippitch; angles[1] = hiproll; angles[2] = kneepitch;
            //angles[3] = anklepitch; angles[4] = ankleroll;

            return angles;
        }
    }
}