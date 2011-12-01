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

        public NaoFoot(NaoFoot other)
        {
            if (other != null)
            {
                name = other.name;
                ffr = other.ffr; frr = other.frr; ffl = other.ffl; frl = other.frl;
                pfr = other.pfr; prr = other.prr; pfl = other.pfl; prl = other.prl;
                outerEdge = other.outerEdge; innerEdge = other.innerEdge; width = other.width;
            }
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
            //Console.WriteLine(ffr.ToString() + " " + frr.ToString() + " " + ffl.ToString() + " " + frl.ToString());
        }

        public static Vector3 VectorAverage(params Vector3[] vs)
        {
            Vector3 result = new Vector3();
            foreach (Vector3 v in vs) {
                result = Vector3.Add(v, result);
            }
            return Vector3.Divide(result, vs.Length);
        }

        public void updateFoot(Vector3 COM)
        {
            Vector3 fl = pfl.position,
                    fr = pfr.position,
                    rl = prl.position,
                    rr = prr.position;

            Vector3 leftSide = Vector3.Subtract(fl, rl);
            Vector3 rightSide = Vector3.Subtract(fr, rr);

            Plane footPlane = new Plane(fr, fl, rr);
            Vector3 planeNormal = footPlane.Normal;

            // BEST PROJECTION FOUND
            float COMoffset = Vector3.Dot((fl - COM), planeNormal) / (Vector3.Dot(planeNormal, planeNormal));
            Vector3 COMPROJ = COMoffset * planeNormal + COM;
            // TEST PROJECTION
            float zero = Vector3.Dot((COMPROJ - fl), planeNormal);

            // ORIGINAL PROJECTION
            //A || B = B x (A x B) / |B|^2 
            //Vector3 COMproj = Vector3.Cross(planeNormal, (Vector3.Cross(COM, planeNormal))) / (planeNormal.LengthSquared());
            // test projection
            //float shouldbezero = Vector3.Dot((COMproj - fr), planeNormal);

            //(AB x AC)/|AB|
            // AB = leftSide, rightSide
            // A = rl, rr
            Vector3 tempL = Vector3.Subtract(COMPROJ, rl);
            Vector3 tempR = Vector3.Subtract(COMPROJ, rr);

            // use sine to get the distance from COMproj to the foot edges
            double distance1 = Math.Sin(Math.Acos((double)(Vector3.Dot(leftSide, tempL) / (leftSide.Length() * tempL.Length())))) * tempL.Length();
            double distance2 = Math.Sin(Math.Acos((double)(Vector3.Dot(rightSide, tempR) / (rightSide.Length() * tempR.Length())))) * tempR.Length();

            float width = Vector3.Distance(fr, fl);
            // width front = 0.053

            if (name == "R")
            {
                innerEdge = (float)distance1;
                outerEdge = (float)distance2;
            }
            else
            {
                innerEdge = (float)distance2;
                outerEdge = (float)distance1;
            }
            this.width = width;
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
