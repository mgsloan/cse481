using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using Microsoft.Xna.Framework;

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
    }
}
