﻿using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using Microsoft.Xna.Framework;

namespace KinectViewer
{
    class DisplacementBalancer
    {
        
        NaoSimulator naoSim;
        Vector3 initCenter;
        float maxHeight;
        float feetWidth;
        float hipWidth;
        float skew;
        float uLength;
        float lLength;
        float length;
        double[] initialAngles;

        public DisplacementBalancer(NaoSimulator naoSim)
        {
            this.naoSim = naoSim;
        }

        public void InitializeTwoLegStance(Vector3 initCenter)
        {
            Vector3 lhip = naoSim.GetPosition("LHipRoll"),  rhip = naoSim.GetPosition("RHipRoll");
            Vector3 lknee = naoSim.GetPosition("LKneePitch");
            Vector3 lfoot = naoSim.GetPosition("LAnkleRoll"), rfoot = naoSim.GetPosition("RAnkleRoll");
            this.hipWidth  = Vector3.Distance(lhip,  rhip);
            this.feetWidth = 2.38f; //Vector3.Distance(lfoot, rfoot);
            this.uLength   = Vector3.Distance(lhip,  lknee);
            this.lLength   = Vector3.Distance(lknee, lfoot);
            this.length = uLength + lLength;
            this.skew = (feetWidth - hipWidth) / 2;
            this.maxHeight = (float)Math.Sqrt(Math.Pow(length, 2) - Math.Pow(skew, 2));
            this.initCenter = initCenter;
            this.initialAngles = LegIK(Matrix.Identity, new Vector3(0, length - 0.01f, 0), Vector3.Zero);
        }

        //for when both feet are on the ground
        public void AdjustFeet(Vector3 position)
        {
            Vector3 displacement = Vector3.Multiply(Vector3.Subtract(position, initCenter), 0.6f);

            Viewer.debugOrigin = new Vector3(-8f, -2f, 0f);

            displacement.Z = 0.0f;
            Viewer.DebugVector("d1", displacement, Color.Plum);

            // Compute hip / knee

            Vector3 hipL = new Vector3(displacement.X - hipWidth / 2, maxHeight * 0.9f + displacement.Y, 0);
            Vector3 targetL = new Vector3(feetWidth / -2f, 0, 0);
            Vector3 deltaL = Vector3.Subtract(targetL, hipL);

            float ldist = deltaL.Length();

            if (ldist > length) {
                Vector3 correction = Vector3.Subtract(deltaL, Vector3.Multiply(deltaL, length / ldist));
                hipL = Vector3.Add(correction, hipL);
                displacement = Vector3.Add(correction, displacement);
            }

            Vector3 hipR = new Vector3(displacement.X + hipWidth / 2, maxHeight, 0);
            Vector3 targetR = new Vector3(feetWidth /  2f, 0, 0);
            Vector3 deltaR = Vector3.Subtract(targetR, hipR);
            float rdist = deltaR.Length();

            if (rdist > length) {
                Vector3 correction = Vector3.Subtract(deltaR, Vector3.Multiply(deltaR, length / rdist));
                hipR = Vector3.Add(correction, hipR);
                hipL = Vector3.Add(correction, hipL);
                displacement = Vector3.Add(correction, displacement);
            }

            double[] anglesr, anglesl;
            anglesr = LegIK(Matrix.Identity, hipR, targetR);
            anglesl = LegIK(Matrix.Identity, hipL, targetL);

            float rcHipPitch  = (float)(anglesr[0] - initialAngles[0]);
            float rcHipRoll   = (float)(anglesr[1] - initialAngles[1]);
            float rcKneePitch = (float)(anglesr[2] - initialAngles[2]);

            float lcHipPitch = (float)(anglesl[0] - initialAngles[0]);
            float lcHipRoll  = (float)(anglesl[1] - initialAngles[1]);
            float lcKneePitch = (float)(anglesl[2] - initialAngles[2]);

            Viewer.debugOrigin = new Vector3(-6f, -2f, 0f);
            Viewer.DebugVector(feetWidth.ToString(), displacement, Color.Pink);

            Viewer.debugOrigin = new Vector3(-3f, 0, 0);
            Viewer.DebugReferenceFrameAtOrigin(lcHipPitch.ToString(), Matrix.CreateRotationX(lcHipPitch));
            Viewer.debugOrigin = new Vector3(-6f, 1f, 0);
            Viewer.DebugReferenceFrameAtOrigin(lcHipRoll.ToString(), Matrix.CreateRotationZ(lcHipRoll));
            Viewer.debugOrigin = new Vector3(-9f, 2f, 0);
            Viewer.DebugReferenceFrameAtOrigin(lcKneePitch.ToString(), Matrix.CreateRotationX(lcKneePitch));

            if (float.IsNaN(rcHipPitch)) rcHipPitch = 0;
            if (float.IsNaN(lcHipPitch)) lcHipPitch = 0;

            naoSim.UpdateAngle("RHipPitch", -rcHipPitch);
            naoSim.UpdateAngle("RHipRoll",   rcHipRoll);
            naoSim.UpdateAngle("RKneePitch", rcKneePitch);

            naoSim.UpdateAngle("LHipPitch", -lcHipPitch);
            naoSim.UpdateAngle("LHipRoll",  -lcHipRoll);
            naoSim.UpdateAngle("LKneePitch", lcKneePitch);
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