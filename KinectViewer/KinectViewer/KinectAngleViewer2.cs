using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using Microsoft.Research.Kinect.Nui;
using Microsoft.Xna.Framework;
using Microsoft.Xna.Framework.Graphics;

namespace KinectViewer
{
    class KinectAngleViewer2 : KinectViewer2
    {

        protected override void updateSkeleton(SkeletonData skeleton)
        {


            // Get and return the angle between two NOT JOINT limbs of the skeleton, e.g. angle between (hip to hip vector) ^ ( upper arm )

            float RshoulderRoll = getAngleBetweenLimbs(skeleton, skeleton.Joints[JointID.HipRight], skeleton.Joints[JointID.HipLeft], skeleton.Joints[JointID.ShoulderRight], skeleton.Joints[JointID.ElbowRight]);
            float RshoulderPitch = getLimbAngle(skeleton, skeleton.Joints[JointID.HipRight], skeleton.Joints[JointID.ShoulderRight], skeleton.Joints[JointID.ElbowRight]);
            float RelbowRoll = getLimbAngle(skeleton, skeleton.Joints[JointID.ShoulderRight], skeleton.Joints[JointID.ElbowRight], skeleton.Joints[JointID.HandRight]);
            float RelbowYaw = getAngleBetweenLimbs(skeleton, skeleton.Joints[JointID.ShoulderRight], skeleton.Joints[JointID.HipRight], skeleton.Joints[JointID.ElbowRight], skeleton.Joints[JointID.HandRight]);
            double shoulderPitch2 = RshoulderPitch * Math.PI / 180.0 - 1.6;
            Console.WriteLine(shoulderPitch2);
            if (shoulderPitch2 >= -2.0 && shoulderPitch2 <= 2.0)
            {
                nao.RSUpdatePitch((double)shoulderPitch2);
            }
            double shoulderRoll2 = 1.63 - RshoulderRoll * Math.PI / 180.0;
            if (-0.01 >= shoulderRoll2 && shoulderRoll2 >= -1.63)
            {
                nao.RSUpdateRoll((double)shoulderRoll2);
            }
            double elbowRoll2 = RelbowRoll * Math.PI / 180.0;
            if (1.55 >= elbowRoll2 && elbowRoll2 >= 0.01)
            {
                nao.REUpdateRoll((double)elbowRoll2);
            }
            double elbowYaw2 = RelbowYaw * Math.PI / 180.0 - 1.57;
            if (2.0 >= elbowYaw2 && elbowYaw2 >= -2.0)
            {
                nao.REUpdateYaw((double)elbowYaw2);
            }

            float LelbowRoll = getLimbAngle(skeleton, skeleton.Joints[JointID.ShoulderLeft], skeleton.Joints[JointID.ElbowLeft], skeleton.Joints[JointID.HandLeft]);
            double LelbowRoll2 = -1 * LelbowRoll * Math.PI / 180.0;
            if (LelbowRoll2 >= -1.55 && LelbowRoll2 <= -0.01)
            {
                nao.LEUpdateRoll(LelbowRoll2);
            }

            float LelbowYaw = getAngleBetweenLimbs(skeleton, skeleton.Joints[JointID.ShoulderLeft], skeleton.Joints[JointID.HipLeft], skeleton.Joints[JointID.ElbowLeft], skeleton.Joints[JointID.HandLeft]);
            double LelbowYaw2 = 1.57 - LelbowYaw * Math.PI / 180.0; ;
            if (2.0 >= LelbowYaw2 && LelbowYaw2 >= -2.0)
            {
                nao.LEUpdateYaw(LelbowYaw2);
            }

            float LshoulderRoll = getAngleBetweenLimbs(skeleton, skeleton.Joints[JointID.HipLeft], skeleton.Joints[JointID.HipRight], skeleton.Joints[JointID.ShoulderLeft], skeleton.Joints[JointID.ElbowLeft]);
            double LshoulderRoll2 = LshoulderRoll * Math.PI / 180.0 - 1.57;
            if (1.63 >= LshoulderRoll2 && LshoulderRoll2 >= 0.01)
            {
                nao.LSUpdateRoll(LshoulderRoll2);
            }

            float LshoulderPitch = getLimbAngle(skeleton, skeleton.Joints[JointID.HipLeft], skeleton.Joints[JointID.ShoulderLeft], skeleton.Joints[JointID.ElbowLeft]);
            double LshoulderPitch2 = LshoulderPitch * Math.PI / 180.0 - 1.6;
            if (LshoulderPitch2 >= -2.0 && LshoulderPitch2 <= 2.0)
            {
                nao.LSUpdatePitch(LshoulderPitch2);
            }


            //    // Head Angle Yaw
            //double headYaw = getAngleBetweenLimbs(skeleton, skeleton.Joints[JointID.HipRight], skeleton.Joints[JointID.HipLeft], skeleton.Joints[JointID.ElbowRight], skeleton.Joints[JointID.HandRight])*Math.PI/180.0;
            //headYaw = headYaw - 1.57;
            //if (2.0 >= headYaw && headYaw >= -2.0)
            //{
            //    nao.HeadUpdateYaw(headYaw);
            //}

            //// Head Angle Pitch
            //double headPitch = getAngleBetweenLimbs(skeleton, skeleton.Joints[JointID.ShoulderRight], skeleton.Joints[JointID.HipRight], skeleton.Joints[JointID.ElbowRight], skeleton.Joints[JointID.HandRight]) * Math.PI / 180.0;
            //headPitch = 0.5 - headPitch;
            //if (0.5 >= headPitch && headPitch >= -0.66)
            //{
            //    nao.HeadUpdatePitch(headPitch);
            //}



        }

        private float getAngleBetweenLimbs(SkeletonData skeleton, Joint joint1, Joint joint2, Joint joint3, Joint joint4)
        {

            // This function is different than getLimbAngle, in the sense that, it finds the angle of two disconnected
            // limbs (each limb considered as a vector).  

            // In order to find the shoulder roll angles of Nao, we need to find the angle between
            // [Right Hip --> Left Hip] vector and [Right Shoulder --> Right Elbow] vector

            if (skeleton == null)
            {
                Console.WriteLine("not tracked!");
                return (float)-999.99;
                //throw exception
            }
            else
            {
                // Make sure that all joints are found with enough confidence...

                Vector[] pt = new Vector[4]; // Create four points
                pt[0] = joint1.Position; // Vector 0, Point 0
                pt[1] = joint2.Position; // Vector 0, Point 1
                pt[2] = joint3.Position; // Vector 1, Point 0
                pt[3] = joint4.Position; // Vector 1, Point 1

                Vector3[] v = new Vector3[2]; // Create two vectors

                // First vector
                // V_{RHIP,LHIP} = V_{LHIP} - V_{RHIP}
                v[0].X = pt[2].X - pt[3].X;
                v[0].Y = pt[2].Y - pt[3].Y;
                v[0].Z = pt[2].Z - pt[3].Z;

                // Second vector
                // V_{S,E} = V_{E} - V_{S}
                v[1].X = pt[0].X - pt[1].X;
                v[1].Y = pt[0].Y - pt[1].Y;
                v[1].Z = pt[0].Z - pt[1].Z;

                // Calculate the magnitude of the vectors (limbs)

                float v0_magnitude = (float)Math.Sqrt(v[0].X * v[0].X + v[0].Y * v[0].Y + v[0].Z * v[0].Z);
                float v1_magnitude = (float)Math.Sqrt(v[1].X * v[1].X + v[1].Y * v[1].Y + v[1].Z * v[1].Z);

                //printf("V0 MAG: %f, V1 MAG: %f\n",v0_magnitude, v1_magnitude);

                // If neither of vectors != 0.0 then find the angle between them
                if (v0_magnitude != 0.0 && v1_magnitude != 0.0)
                {
                    v[0].X = v[0].X * (float)(1.0 / v0_magnitude);
                    v[0].Y = v[0].Y * (float)(1.0 / v0_magnitude);
                    v[0].Z = v[0].Z * (float)(1.0 / v0_magnitude);

                    v[1].X = v[1].X * (float)(1.0 / v1_magnitude);
                    v[1].Y = v[1].Y * (float)(1.0 / v1_magnitude);
                    v[1].Z = v[1].Z * (float)(1.0 / v1_magnitude);


                    // Find and convert the angle between upper and lower arms
                    float theta = (float)Math.Acos(v[0].X * v[1].X + v[0].Y * v[1].Y + v[0].Z * v[1].Z);
                    float angle_in_degrees = theta * 180 / (float)Math.PI;
                    return angle_in_degrees;
                }
                return (float)-99.99;

            }
        }



        private float getLimbAngle(SkeletonData skeleton, Joint joint1, Joint joint2, Joint joint3)
        {


            Vector[] pt = new Vector[3]; // Create 3 points
            pt[0] = joint1.Position; // E.g. Shoulder
            pt[1] = joint2.Position; // E.g. Elbow
            pt[2] = joint3.Position; // E.g. Hand

            // or another e.g. 
            // pt_0: Hip
            // pt_1: Knee
            // pt_2: Foot

            // The first vector is, Shoulder --> Elbow or Hip --> Knee
            // The second vector is,  Elbow --> Hand or Knee --> Foot 

            Vector3[] v = new Vector3[2]; // Create two vectors

            // S: Shoulder, E: Elbow, H: Hand
            // S  *
            //    |
            // E  *---*
            //        H

            // Calculate the first vector
            // V_{H,E} = V_{E} - V_{H}
            v[0].X = pt[1].X - pt[2].X;
            v[0].Y = pt[1].Y - pt[2].Y;
            v[0].Z = pt[1].Z - pt[2].Z;

            // Calculate the second vector
            // V_{E,S} = V_{S} - V_{E}
            v[1].X = pt[0].X - pt[1].X;
            v[1].Y = pt[0].Y - pt[1].Y;
            v[1].Z = pt[0].Z - pt[1].Z;

            // Calculate the magnitudes of the vectors
            float v0_magnitude = (float)Math.Sqrt(v[0].X * v[0].X + v[0].Y * v[0].Y + v[0].Z * v[0].Z);
            float v1_magnitude = (float)Math.Sqrt(v[1].X * v[1].X + v[1].Y * v[1].Y + v[1].Z * v[1].Z);

            //printf("V0 MAG: %f, V1 MAG: %f\n",v0_magnitude, v1_magnitude);

            if (v0_magnitude != 0.0 && v1_magnitude != 0.0)
            {
                v[0].X = v[0].X * (float)(1.0 / v0_magnitude);
                v[0].Y = v[0].Y * (float)(1.0 / v0_magnitude);
                v[0].Z = v[0].Z * (float)(1.0 / v0_magnitude);

                v[1].X = v[1].X * (float)(1.0 / v1_magnitude);
                v[1].Y = v[1].Y * (float)(1.0 / v1_magnitude);
                v[1].Z = v[1].Z * (float)(1.0 / v1_magnitude);


                // Find and convert the angle between JOINT limbs
                float theta = (float)Math.Acos(v[0].X * v[1].X + v[0].Y * v[1].Y + v[0].Z * v[1].Z);
                float angle_in_degrees = theta * 180 / (float)Math.PI;

                return angle_in_degrees;
            }
            else return (float)-997.0; // Error code (though, it's not used yet) 
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

