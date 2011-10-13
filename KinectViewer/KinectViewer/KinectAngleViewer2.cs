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


            double reyAngle = AngleBetween(skeleton, skeleton.Joints[JointID.ShoulderRight], skeleton.Joints[JointID.HipRight], skeleton.Joints[JointID.ElbowRight], skeleton.Joints[JointID.HandRight]) * Math.PI / 180.0 - 1.57;
            UpdateLimb(reyAngle, "RElbowAngle", -2.0, 2.0);

            double lerAngle = AngleBetween(skeleton, skeleton.Joints[JointID.ShoulderLeft], skeleton.Joints[JointID.ElbowLeft], skeleton.Joints[JointID.ElbowLeft],  skeleton.Joints[JointID.HandLeft]) * -1 * Math.PI / 180.0;
            UpdateLimb(lerAngle, "LElbowRoll", -1.55, -0.01);

            double leyAngle = 1.57 -  AngleBetween(skeleton, skeleton.Joints[JointID.ShoulderLeft], skeleton.Joints[JointID.HipLeft], skeleton.Joints[JointID.ElbowLeft], skeleton.Joints[JointID.HandLeft]) * Math.PI / 180.0;
            UpdateLimb(leyAngle, "LElbowYaw", -2.0, 2.0);
            

            double lsrAngle = AngleBetween(skeleton, skeleton.Joints[JointID.HipLeft], skeleton.Joints[JointID.HipRight], skeleton.Joints[JointID.ShoulderLeft], skeleton.Joints[JointID.ElbowLeft]) * Math.PI / 180.0 - 1.57;
            UpdateLimb(lsrAngle, "LShoulderRoll", 0.01, 1.63);
            

            double lspAngle = AngleBetween(skeleton, skeleton.Joints[JointID.HipLeft], skeleton.Joints[JointID.ShoulderLeft], skeleton.Joints[JointID.ShoulderLeft], skeleton.Joints[JointID.ElbowLeft]) * Math.PI / 180.0 - 1.6;
            UpdateLimb(lspAngle, "LShoulderPitch", -2.0, 2.0);


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

