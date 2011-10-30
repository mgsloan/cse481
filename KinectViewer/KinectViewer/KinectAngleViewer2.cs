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
            double rspAngle = AngleBetween(skeleton, skeleton.Joints[JointID.HipRight], skeleton.Joints[JointID.ShoulderRight], skeleton.Joints[JointID.ShoulderRight], skeleton.Joints[JointID.ElbowRight]) * Math.PI / 180.0 - 1.6;
            UpdateLimb(rspAngle, "RShoulderPitch", -2.0, 2.0);

            double rsrAngle = 1.63 - AngleBetween(skeleton, skeleton.Joints[JointID.HipRight], skeleton.Joints[JointID.HipLeft], skeleton.Joints[JointID.ShoulderRight], skeleton.Joints[JointID.ElbowRight]) * Math.PI / 180.0;
            UpdateLimb(rsrAngle, "RShoulderRoll", -1.63, -0.01);
            
            double rerAngle = AngleBetween(skeleton, skeleton.Joints[JointID.ShoulderRight], skeleton.Joints[JointID.ElbowRight], skeleton.Joints[JointID.ElbowRight], skeleton.Joints[JointID.HandRight]) * Math.PI / 180.0;
            UpdateLimb(rerAngle, "RElbowRoll", 0.01, 1.55);

            RightElbowYaw(skeleton);

            double lerAngle = AngleBetween(skeleton, skeleton.Joints[JointID.ShoulderLeft], skeleton.Joints[JointID.ElbowLeft], skeleton.Joints[JointID.ElbowLeft],  skeleton.Joints[JointID.HandLeft]) * -1 * Math.PI / 180.0;
            UpdateLimb(lerAngle, "LElbowRoll", -1.55, -0.01);

            LeftElbowYaw(skeleton);
            
            double lsrAngle = AngleBetween(skeleton, skeleton.Joints[JointID.HipLeft], skeleton.Joints[JointID.HipRight], skeleton.Joints[JointID.ShoulderLeft], skeleton.Joints[JointID.ElbowLeft]) * Math.PI / 180.0 - 1.57;
            UpdateLimb(lsrAngle, "LShoulderRoll", 0.01, 1.63);
            
            double lspAngle = AngleBetween(skeleton, skeleton.Joints[JointID.HipLeft], skeleton.Joints[JointID.ShoulderLeft], skeleton.Joints[JointID.ShoulderLeft], skeleton.Joints[JointID.ElbowLeft]) * Math.PI / 180.0 - 1.6;
            UpdateLimb(lspAngle, "LShoulderPitch", -2.0, 2.0);

        }

        private void RightElbowYaw(SkeletonData skeleton)
        {
            Vector3 cshoulder_chip = makeVector(skeleton.Joints[JointID.HipCenter], skeleton.Joints[JointID.ShoulderCenter]);
            Vector3 rshoulder_relbow = makeVector(skeleton.Joints[JointID.ElbowRight], skeleton.Joints[JointID.ShoulderRight]);
            rshoulder_relbow.Normalize();
            Vector3 relbow_rhand = makeVector(skeleton.Joints[JointID.HandRight], skeleton.Joints[JointID.ElbowRight]);
            Vector3 elbowRef = Vector3.Cross(cshoulder_chip, rshoulder_relbow);
            elbowRef.Normalize();

            Vector3 relbow_rshoulder = makeVector(skeleton.Joints[JointID.ShoulderRight], skeleton.Joints[JointID.ElbowRight]);
            relbow_rshoulder.Normalize();
            Vector3 eSide = Vector3.Cross(relbow_rhand, relbow_rshoulder);
            eSide.Normalize();
            Vector3 ePerp = Vector3.Cross(eSide, rshoulder_relbow);
            ePerp.Normalize();

            Vector3 origin = new Vector3(0, 0, 0);
            //lines.Clear();
            //lines.Add(new LabelledVector(origin, ePerp * 2, Color.Red, "E_Perp"));
            //lines.Add(new LabelledVector(origin, eSide * 2, Color.Black, "E_SIDE"));
            //lines.Add(new LabelledVector(origin, rshoulder_relbow * 2, Color.Green, "U_ARM"));
            //lines.Add(new LabelledVector(origin, elbowRef * 2, Color.Gold, "E_REF"));

            float reYawAngle = getAngleBetween(elbowRef, eSide);
            float reYawAngleScaled = (float) Math.PI / 2 - reYawAngle;
            //Console.WriteLine("reYawAngle: " + reYawAngle * 180 / Math.PI);
            //Console.WriteLine("reYawScaled: " + reYawAngleScaled * 180 / Math.PI);
            UpdateLimb(reYawAngleScaled, "RElbowYaw", -2.0, 2.0);
        }

        private void LeftElbowYaw(SkeletonData skeleton)
        {
            Vector3 cshoulder_chip = makeVector(skeleton.Joints[JointID.HipCenter], skeleton.Joints[JointID.ShoulderCenter]);
            Vector3 lshoulder_lelbow = makeVector(skeleton.Joints[JointID.ElbowLeft], skeleton.Joints[JointID.ShoulderLeft]);
            lshoulder_lelbow.Normalize();
            Vector3 lelbow_lhand = makeVector(skeleton.Joints[JointID.HandLeft], skeleton.Joints[JointID.ElbowLeft]);
            Vector3 elbowRef = Vector3.Cross(cshoulder_chip, lshoulder_lelbow);
            elbowRef.Normalize();

            Vector3 lelbow_lshoulder = makeVector(skeleton.Joints[JointID.ShoulderLeft], skeleton.Joints[JointID.ElbowLeft]);
            Vector3 eSide = Vector3.Cross(lelbow_lhand, lelbow_lshoulder);
            Vector3 ePerp = Vector3.Cross(eSide, lshoulder_lelbow);

            float leYawAngle = getAngleBetween(elbowRef, eSide);
            float leYawAngleScaled = leYawAngle - (float) Math.PI/2;
            //Console.WriteLine("leYawAngle: " + leYawAngle * 180 / Math.PI);
            //Console.WriteLine("leYawScaled: " + leYawAngleScaled * 180 / Math.PI);
            UpdateLimb(leYawAngleScaled, "LElbowYaw", -2.0, 2.0);
        }

        private void UpdateLimb(double angle, string jointName, double min, double max) {
            if (angle >= min && angle <= max)
            {
                nao.UpdateLimb(angle, jointName);
            }


        }

        private double AngleBetween(SkeletonData skeleton, Joint joint1, Joint joint2, Joint joint3, Joint joint4)
        {

            if (skeleton == null)
            {
                Console.WriteLine("not tracked!");
                return -999.99;
                //throw exception
            }
            else
            {
                Vector3[] points = new Vector3[4]; 
                
                points[0] = new Vector3(joint1.Position.X, joint1.Position.Y, joint1.Position.Z);
                points[1] = new Vector3(joint2.Position.X, joint2.Position.Y, joint2.Position.Z);
                points[2] = new Vector3(joint3.Position.X, joint3.Position.Y, joint3.Position.Z);
                points[3] = new Vector3(joint4.Position.X, joint4.Position.Y, joint4.Position.Z); 

                Vector3[] vectors = new Vector3[2]; 

                vectors[0] = Vector3.Subtract(points[2], points[3]);
                vectors[1] = Vector3.Subtract(points[0], points[1]);
                
                double vector1Magnitude = Math.Sqrt(Math.Pow(vectors[0].X,2) + Math.Pow(vectors[0].Y,2) + Math.Pow(vectors[0].Z,2));
                double vector2Magnitude = Math.Sqrt(Math.Pow(vectors[1].X,2) + Math.Pow(vectors[1].Y,2) + Math.Pow(vectors[1].Z,2));

                if (vector1Magnitude != 0.0 && vector2Magnitude != 0.0)
                {
                    double theta = Math.Acos(Vector3.Dot(vectors[0],vectors[1])/(vector1Magnitude*vector2Magnitude));
                    double angle_in_degrees = theta * 180 / Math.PI;
                    return angle_in_degrees;
                }
                return -99.990;

            }
        }

        // j1 is endpoint
        private Vector3 makeVector(Joint j1, Joint j2)
        {
            return Vector3.Subtract(getLoc(j1), getLoc(j2));
        }

        private float getAngleBetween(Vector3 v1, Vector3 v2)
        {
            return (float) Math.Acos(Vector3.Dot(v1, v2) / (Magnitude(v1) * Magnitude(v2)));
        }

        private double Magnitude(Vector3 v)
        {
            return Math.Sqrt(Vector3.Dot(v, v));
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

