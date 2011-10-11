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
        void updateSkeleton(SkeletonData skeleton)
        {
            Joint elbowRight = skeleton.Joints[JointID.ElbowRight],
                    handRight = skeleton.Joints[JointID.HandRight],
                    shoulderRight = skeleton.Joints[JointID.ShoulderRight],
                    shoulderLeft = skeleton.Joints[JointID.ShoulderLeft],
                    shoulderCenter = skeleton.Joints[JointID.ShoulderCenter],
                    spine = skeleton.Joints[JointID.Spine],
                    elbowLeft = skeleton.Joints[JointID.ElbowLeft];

            lines.Clear();
            
            Vector3 dx = Vector3.Subtract(getLoc(shoulderRight), getLoc(shoulderLeft));
            Vector3 dy = Vector3.Subtract(getLoc(spine), getLoc(shoulderCenter));
            Vector3 dz = Vector3.Cross(dx, dy);
           // Matrix.CreateWorld

           // Matrix bodyRef = makeReferenceFrame(getLoc(shoulderCenter), getLoc(shoulderLeft), getLoc(spine));

            //Vector3D s_e = between(shoulderRight, elbowRight),
            //         e_h = between(elbowRight, handRight); 

            //double angle = Vector3D.AngleBetween(s_e, e_h);
            //nao.REUpdateRoll(angle);

            //Matrix3D bodyRef = makeReferenceFrame(getLoc(shoulderCenter), getLoc(shoulderLeft), getLoc(spine));
            /*
            Vector3D dx = between(shoulderLeft, shoulderRight);
            Vector3D dy = between(spine, shoulderCenter);
            dx.Normalize(); dy.Normalize();
            Vector3D dz = Vector3D.CrossProduct(dx, dy);
            Console.WriteLine(dz.ToString());

            Matrix3D shoulderFrame = makeMatrix(dx, dy, dz, getLoc(shoulderRight));
            shoulderFrame.Invert();

            Vector3D elbowRightLocal = shoulderFrame.Transform(getLoc(elbowRight));
            Console.WriteLine(elbowRightLocal.ToString());

            //Console.WriteLine(s_e.ToString());
            //Console.WriteLine(s_e_local.ToString());
            //Canvas.SetLeft(ellipse4, );
            double a1 = Math.Atan2(elbowRightLocal.Y, elbowRightLocal.Z);
            double a2 = Math.Atan2(elbowRightLocal.X, elbowRightLocal.Y);
            (*/
            /*
            double a1 = Vector3D.AngleBetween(new Vector3D(0, elbowRightLocal.Y, elbowRightLocal.Z), new Vector3D(0, 0, -1)),
                    a2 = Vector3D.AngleBetween(new Vector3D(elbowRightLocal.X, elbowRightLocal.Y, 0), new Vector3D(0, -1, 0));
                */
            //setAngle(line_ang1, a1, 20);
            //setAngle(line_ang2, a2, 20);
            /*
            Console.WriteLine("LeftShoulderPitch angle: " + a1.ToString());
            Console.WriteLine("LeftShoulderRoll angle: " + a2.ToString());
            nao.LSUpdatePitch(a1);
            nao.LSUpdateRoll(a2);
            */
        }

        private Matrix makeReferenceFrame(Vector3 c, Vector3 x, Vector3 y)
        {
            Vector3 dx = Vector3.Subtract(x, c), dy = Vector3.Subtract(y, c);
            dx.Normalize(); dy.Normalize();
            Vector3 dz = Vector3.Cross(dx, dy);
            return Matrix.CreateWorld(c, dz, dy);
        }
        
        /*
        // TODO: is this in homogenous coords?

        private Vector3D between(Joint j1, Joint j2)
        {
            return Vector3D.Subtract(getLoc(j2), getLoc(j1));
        }



        
         */

    }
}
