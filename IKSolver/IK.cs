using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using Microsoft.Xna.Framework;

namespace IKSolver
{
    class IK
    {
        static void Main(string[] args)
        {
            Vector3 left = new Vector3(-1, 0, 0); // should be -1
            Vector3 up = new Vector3(0, 1, 0);
            Vector3 forward = Vector3.Cross(left, up);
            Matrix refFrame = Matrix.CreateWorld(new Vector3(0, 0, 0), forward, up);

            Vector3 hip = new Vector3(-1, 0, 0);
            Vector3 foot = new Vector3(-1, -1, 1);
            double[] angles = LegIK(refFrame, hip, foot, 1, 1);
            Console.WriteLine("" + angles[0] + ", " + angles[1] + ", " + angles[2]);

            foot.Z -= .2f;
            angles = LegIK(refFrame, hip, foot, 1, 1);
            Console.WriteLine("" + angles[0] + ", " + angles[1] + ", " + angles[2]);

            foot.Z -= .2f;
            angles = LegIK(refFrame, hip, foot, 1, 1);
            Console.WriteLine("" + angles[0] + ", " + angles[1] + ", " + angles[2]);

            foot.Z -= .2f;
            angles = LegIK(refFrame, hip, foot, 1, 1);
            Console.WriteLine("" + angles[0] + ", " + angles[1] + ", " + angles[2]);

            foot.Z -= .2f;
            angles = LegIK(refFrame, hip, foot, 1, 1);
            Console.WriteLine("" + angles[0] + ", " + angles[1] + ", " + angles[2]);

            foot.Z -= .2f;
            angles = LegIK(refFrame, hip, foot, 1, 1);
            Console.WriteLine("" + angles[0] + ", " + angles[1] + ", " + angles[2]);

            foot.Z -= .2f;
            angles = LegIK(refFrame, hip, foot, 1, 1);
            Console.WriteLine("" + angles[0] + ", " + angles[1] + ", " + angles[2]);

            foot.Z -= .2f;
            angles = LegIK(refFrame, hip, foot, 1, 1);
            Console.WriteLine("" + angles[0] + ", " + angles[1] + ", " + angles[2]);

            foot.Z -= .2f;
            angles = LegIK(refFrame, hip, foot, 1, 1);
            Console.WriteLine("" + angles[0] + ", " + angles[1] + ", " + angles[2]);

            foot.Z -= .2f;
            angles = LegIK(refFrame, hip, foot, 1, 1);
            Console.WriteLine("" + angles[0] + ", " + angles[1] + ", " + angles[2]);

            foot.Z -= .2f;
            angles = LegIK(refFrame, hip, foot, 1, 1);
            Console.WriteLine("" + angles[0] + ", " + angles[1] + ", " + angles[2]);

            Vector3 foot2 = new Vector3(-1, -1, -1);
            LegIK(refFrame, hip, foot2, 1, 1);

            Vector3 foot3 = new Vector3(-1, 0, -1);
            LegIK(refFrame, hip, foot3, 1, 1);

            left = new Vector3(-1, 0, 0); // should be -1
            up = new Vector3(0, .5773f, .4226f);
            forward = Vector3.Cross(left, up);
            refFrame = Matrix.CreateWorld(new Vector3(0, 0, 0), forward, up);

            createPosition(refFrame, 1, 1, 0, 0, Math.PI / 2);

            //createPosition(1, 1, Math.PI / 2, 0, Math.PI / 2);

            //createPosition(1, 1, Math.PI / 4, 0, Math.PI / 4);

            createPosition(refFrame, 1, 1, Math.PI / 6, 0, Math.PI / 6);

            createPosition(refFrame, 1, 1, Math.PI / 2, Math.PI / 4, Math.PI / 2);

            createPosition(refFrame, 1, 1, Math.PI / 4, Math.PI / 4, Math.PI / 4);

            createPosition(refFrame, 1, 1, Math.PI / 6, Math.PI / 6, Math.PI / 6);

            createPosition(refFrame, 1, 1, Math.PI / 4, Math.PI / 6, Math.PI / 4);
        }

        private static void createPosition(Matrix refFrame, double UL_len, double LL_len, double hippitch, double hiproll, double kneepitch)
        {
            Vector3 UL = new Vector3(0, 0, (float)UL_len);
            Vector3 LL = new Vector3(0, 0, (float)LL_len);

            Matrix m_hippitch = Matrix.CreateRotationX((float)hippitch);
            Matrix m_hiproll = Matrix.CreateRotationZ((float)hiproll);
            Matrix tx = Matrix.Multiply(m_hippitch, m_hiproll);

            Vector3 UL_tx = Vector3.Transform(UL, tx);

            Matrix m_kneepitch = Matrix.CreateRotationX((float)kneepitch);
            Matrix tx2 = Matrix.Multiply(m_kneepitch, tx);
            Vector3 LL_tx = Vector3.Transform(LL, tx2);
            LL_tx += UL_tx;

            LL_tx = Vector3.Transform(LL_tx, Matrix.Invert(refFrame));

            // note that the inverse of the refframe is used here
            double[] result = LegIK(refFrame, new Vector3(0, 0, 0), LL_tx, UL_len, LL_len);

            Console.WriteLine("" + degrees(hippitch) + ", " + degrees(hiproll) + ", " + degrees(kneepitch));
            Console.WriteLine("" + Math.Round(degrees(-result[0]), 3) + ", " + Math.Round(-degrees(result[1] + Math.PI / 2), 3) + ", " + Math.Round(degrees(result[2]), 3));
            Console.WriteLine();
        }

        private static double degrees(double rads) { return rads * 180.0 / Math.PI; }

        private static double[] LegIK(Matrix BodyTxform, Vector3 hip, Vector3 foot, double UL_len, double LL_len)
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
            double a2 = UL_len;
            double b2 = LL_len;
            double c2 = hip_to_foot_len;
            double kneepitch;
            if (c2 > a2 + b2) kneepitch = Math.PI;
            else kneepitch = Math.Acos((a2 * a2 + b2 * b2 - c2 * c2) / (2 * a2 * b2));

            // (3) find hip pitch (there are two parts to this) 
            // from 0 (Z-axis) to -pi (negative Z-axis) where Z-axis is vector out of model's torso

            // part 1: use law of cosines
            double a1 = hip_to_foot_len;
            double b1 = UL_len;
            double c1 = LL_len;
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

            // return three angles in a double[]
            double[] angles = new double[3];
            angles[0] = hippitch; angles[1] = hiproll; angles[2] = kneepitch;

            return angles;
        }
    }
}
