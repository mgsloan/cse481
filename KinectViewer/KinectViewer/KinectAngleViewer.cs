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
                    shoulderLeft   = getLoc(skeleton.Joints[JointID.ShoulderLeft]),
                    shoulderCenter = getLoc(skeleton.Joints[JointID.ShoulderCenter]),
                    spine          = getLoc(skeleton.Joints[JointID.Spine]),
                    elbowLeft      = getLoc(skeleton.Joints[JointID.ElbowLeft]);

            lines.Clear();
            
            Vector3 dx = Vector3.Subtract(shoulderLeft, shoulderRight);
            Vector3 dy = Vector3.Subtract(shoulderCenter, spine);
            Vector3 dz = Vector3.Cross(dx, dy);
            Matrix srRef = Matrix.CreateWorld(shoulderRight, dz, dy);
            debugReferenceFrame("sr", srRef, 3);

            Matrix srRefInv = Matrix.Invert(srRef);
            Vector3 elocal = Vector3.Transform(elbowRight, srRefInv);
            Vector3 offset = Vector3.Add(spine, new Vector3(5, 0, 0));
            Vector3 pitchv = new Vector3(0, elocal.Y, elocal.Z);
            Vector3 rollv = new Vector3(elocal.X, elocal.Y, 0);
            double pitch = Math.Atan2(pitchv.Y, pitchv.Z);
            double roll = Math.Atan2(rollv.X, rollv.Y);
            lines.Add(new LabelledVector(offset, Vector3.Add(offset, Vector3.Multiply(pitchv, 5)), Color.Black, "pitch = " + pitch.ToString()));
            lines.Add(new LabelledVector(offset, Vector3.Add(offset, Vector3.Multiply(rollv, 5)), Color.Black, "roll = " + roll.ToString()));

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

        private void debugReferenceFrame(String str, Matrix m, float sz)
        {
            lines.Add(new LabelledVector(m.Translation, m.Translation + m.Right * sz, Color.Red, str));
            lines.Add(new LabelledVector(m.Translation, m.Translation + m.Up * sz, Color.Green, str));
            lines.Add(new LabelledVector(m.Translation, m.Translation + m.Forward * sz, Color.Blue, str));
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
