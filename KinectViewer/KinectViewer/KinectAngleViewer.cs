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
            dx.Normalize(); dy.Normalize();
            Vector3 dz = Vector3.Cross(dx, dy);
            Matrix srRef = Matrix.CreateWorld(Vector3.Zero, dz, dy);
            //debugReferenceFrame("sr", srRef, 3, shoulderRight);

            Vector3 s_e = Vector3.Subtract(elbowRight, shoulderRight);

            Matrix srRefInv = Matrix.Invert(srRef);
            Vector3 elocal = Vector3.Transform(s_e, srRefInv);
            float pitch = (float)(Math.Abs(elocal.Z) < 0.1 ? 0 : (Math.Atan2(elocal.Z, elocal.Y) + Math.PI));

            Matrix srRef2 = Matrix.Multiply(Matrix.CreateRotationX(pitch), srRef);
            //debugReferenceFrame("sr2", srRef2, 3);
            Matrix srRef2Inv = Matrix.Invert(srRef);
            Vector3 elocal2 = Vector3.Transform(s_e, srRef2Inv);
            float roll = (float)(Math.Atan2(elocal2.X, elocal2.Y));
            
            Vector3 offset = Vector3.Add(spine, new Vector3(5, 0, 0));
            //lines.Add(new LabelledVector(offset, Vector3.Add(offset, Vector3.Multiply(new Vector3(0, elocal.Y, elocal.Z), 5)), Color.Black, "pitch = " + pitch.ToString()));
            //lines.Add(new LabelledVector(offset, Vector3.Add(offset, Vector3.Multiply(new Vector3(elocal2.X, elocal2.Y, 0), 5)), Color.Black, "roll = " + roll.ToString()));

            Matrix eRef = Matrix.Multiply(Matrix.CreateRotationZ((float)Math.PI - roll), srRef2);
            debugReferenceFrame("er", eRef, 3, elbowRight);

            Vector3 e_h = Vector3.Subtract(handRight, elbowRight);
            Vector3 hlocal = Vector3.Transform(e_h, eRef);
            float epitch = (float)(Math.Atan2(hlocal.X, hlocal.Z));
            Matrix eRef2 = Matrix.Multiply(Matrix.CreateRotationY((float)Math.PI + epitch), eRef);
            //debugReferenceFrame(epitch.ToString(), eRef2, 3, elbowRight);

            // Debug pitch / roll calculation
            /*
            Vector3 pitchv = new Vector3(0, elocal.Y, elocal.Z);
            Vector3 rollv = new Vector3(elocal.X, elocal.Y, 0);
            Vector3 offset = Vector3.Add(spine, new Vector3(5, 0, 0));
            lines.Add(new LabelledVector(offset, Vector3.Add(offset, Vector3.Multiply(pitchv, 5)), Color.Black, "pitch = " + pitch.ToString()));
             */
           // lines.Add(new LabelledVector(offset, Vector3.Add(offset, Vector3.Multiply(rollv, 5)), Color.Black, "roll = " + roll.ToString()));

            nao.RSUpdatePitch(pitch);
           // nao.RSUpdateRoll(roll);

            Vector3 dz2 = Vector3.Subtract(elbowRight, shoulderRight);
            dz2.Normalize();
            //Matrix erRef = Matrix.CreateWorld(elbowRight, dz2, );

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
            debugReferenceFrame(str, m, sz, m.Translation);
        }

        private void debugReferenceFrame(String str, Matrix m, float sz, Vector3 origin)
        {
            lines.Add(new LabelledVector(origin, origin + m.Right * sz, Color.Red, str));
            lines.Add(new LabelledVector(origin, origin + m.Up * sz, Color.Green, ""));
            lines.Add(new LabelledVector(origin, origin + m.Forward * sz, Color.Blue, ""));
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
