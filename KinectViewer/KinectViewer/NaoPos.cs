using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using Microsoft.Xna.Framework;

namespace KinectViewer
{
    // Utility to convert from / to a Nao-space position / rotation.
    class NaoPos
    {
        public Matrix transform;

        public Vector3 position {
            get {
                return transform.Translation;
            }
            set {
                transform.Translation = value;
            }
        }

        public NaoPos(List<float> fs, Matrix trans)
        {
            transform = ConvertRotation(fs[3], fs[4], fs[5]);
            position = Convert(new Vector3(fs[0], fs[1], fs[2]));
            transform = Matrix.Multiply(transform, trans);
        }

        public LabelledVector DebugLine(float size, Color c, String txt) {
            return DebugLine(size, c, txt, Vector3.UnitY);
        }

        public LabelledVector DebugLine(float size, Color c, String txt, Vector3 dir) {
            return new LabelledVector(position, Vector3.Transform(Vector3.Multiply(dir, size), transform), c, txt);
        }

        public static Vector3 Convert(Vector3 v) { return Convert(v, 15f); }

        public static Vector3 Convert(Vector3 v, float scale)
        {
            return new Vector3(v.Y * scale, v.Z * scale, v.X * scale);
        }

        public static Matrix ConvertRotation(float ax, float ay, float az) {
            return Matrix.CreateFromYawPitchRoll(az, ay, ax);
        }
    }
}
