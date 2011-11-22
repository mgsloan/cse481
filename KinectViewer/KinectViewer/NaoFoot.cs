using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using Microsoft.Xna.Framework;

namespace KinectViewer
{
    class NaoFoot
    {
        public string name;
        public float ffr, frr, ffl, frl;
        public NaoPos pfr, prr, pfl, prl;
        public float outerEdge, innerEdge, width;

        public NaoFoot(string side)
        {
            this.name = side;
        }

        public Vector3 GetDirection() {
            return VectorAverage(Vector3.Subtract(pfr.position, prr.position),
                                 Vector3.Subtract(pfl.position, prl.position));
        }

        public Vector3 GetCenter() {
            return VectorAverage(pfr.position, prr.position, pfl.position, prl.position);
        }


        public void FootLines(List<LabelledVector> ls) {
            ls.Add(pfr.DebugLine(ffr * 4f, Color.Black, "", Vector3.Up));
            ls.Add(prr.DebugLine(frr * 4f, Color.Black, "", Vector3.Up));
            ls.Add(pfl.DebugLine(ffl * 4f, Color.Black, "", Vector3.Up));
            ls.Add(prl.DebugLine(frl * 4f, Color.Black, "", Vector3.Up));
            Vector3 center = GetCenter();
            ls.Add(new LabelledVector(center, Vector3.Add(center, GetDirection()), Color.Red, ""));
        }

        public static Vector3 VectorAverage(params Vector3[] vs)
        {
            Vector3 result = new Vector3();
            foreach (Vector3 v in vs) {
                result = Vector3.Add(v, result);
            }
            return Vector3.Divide(result, vs.Length);
        }

        public float GetOffset()
        {
            if (outerEdge < width)
            {
                return 0;
            }
            else if (outerEdge < innerEdge)
            {
                return 0;
            }
            else
            {
                return innerEdge;
            }
        }
    }
}
