using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using Microsoft.Xna.Framework;
using Microsoft.Research.Kinect.Nui;
namespace KinectViewer
{
    class MathUtils
    {
        public static float FromRad(float rad) { return rad / (float)Math.PI * 180f; }
        public static float ToRad(float rad) { return rad * (float)Math.PI / 180f; }

        public static float Clamp(float v, float f, float t)
        {
            return Math.Max(f, Math.Min(t, v));
        }

        public static float Lerp(float v, float f, float t)
        {
            return (1 - v) * t + v * f;
        }

        public static float UnLerp(float v, float f, float t)
        {
            return (v - f) / (t - f);
        }


        private static float ScaleToRange(double val, float min, float max)
        {
            float returnVal = (float)((val / 180.0) * (max - min)) + min;
            return returnVal;
        }
        
        public static Vector3 VectorFromList(List<float> fs)
        {
            return new Vector3(fs[0], fs[1], fs[2]);
        }

        public static float Average(params float[] xs)
        {
            float sum = 0;
            foreach (float x in xs)
            {
                sum += x;
            }
            return sum / xs.Length;
        }

        public static Matrix ExtractRotation(Matrix mat)
        {
            Matrix result = new Matrix();
            result.Up = mat.Up;
            result.Right = mat.Right;
            result.Forward = mat.Forward;
            result.Translation = Vector3.Zero;
            result.M44 = 1;
            return result;
        }

        // Multiplies the first matrix by the second, as if the first matrix had zero translation.
        public static Matrix RotateBy(Matrix a, Matrix b)
        {
            Matrix result = ExtractRotation(a);
            Matrix.Multiply(ref result, ref b, out result);
            result.Translation = a.Translation;
            return result;
        }

        public static float AngleBetween(Vector3 v1, Vector3 v2)
        {
            return (float)Math.Acos(Vector3.Dot(v1, v2) / (v1.Length() * v2.Length()));
        }

        public static Vector3 FromKinectSpace(Vector position)
        {
            var returnVector = new Vector3();
            returnVector.X = position.X * 10;
            returnVector.Y = position.Y * 10;
            returnVector.Z = position.Z * 10;
            return returnVector;
        }
    }
}
