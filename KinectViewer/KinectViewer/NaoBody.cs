using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using Aldebaran.Proxies;
using System.Collections;
using Microsoft.Xna.Framework;

namespace KinectViewer
{
    class NaoBody
    {
        MemoryProxy _memory = null;
        MotionProxy _motion = null;

        public bool connected
        {
            get
            {
                return _motion != null;
            }
        }

        private float speed = 0.2f;

        public ArrayList joints, values, limits, parts;

        public NaoFoot leftFoot, rightFoot;
        public Matrix gyrot;

        public void Connect(string ip)
        {
            leftFoot = new NaoFoot("L");
            rightFoot = new NaoFoot("R");

            values = new ArrayList();
            limits = new ArrayList();

            joints = new ArrayList(new String[] {
                "RShoulderPitch",
                "RShoulderRoll",
                "RElbowRoll",
                "RElbowYaw",
                "LShoulderPitch",
                "LShoulderRoll",
                "LElbowRoll",
                "LElbowYaw",
                "RHipRoll",
                "RHipPitch",
                "RKneePitch",
                "RAnklePitch",
                "RAnkleRoll",
                "LHipRoll",
                "LHipPitch",
                "LKneePitch",
                "LAnklePitch",
                "LAnkleRoll" });

            parts = new ArrayList(new String[] {
                "Torso",
                "RShoulderPitch",
                "LShoulderPitch",
                "RWristYaw",
                "LWristYaw",
                "HeadYaw",
                "LKneePitch",
                "RKneePitch",
                "LAnklePitch",
                "RAnklePitch",
                "LHipPitch",
                "RHipPitch",
                "LAnklePitch" });


            for (int i = 0; i < joints.Count; i++) {
                values.Add(0.0f);
            }

            try
            {
                _memory = new MemoryProxy(ip, 9559);
                _motion = new MotionProxy(ip, 9559);

                for (int i = 0; i < joints.Count; i++)
                {
                    limits.Add(((ArrayList)_motion.getLimits((string)joints[i]))[0]);
                }
                // give the joints some stiffness
                _motion.setStiffnesses("Body", 1.0f);

            }
            catch (Exception e)
            {
                Console.Out.WriteLine("Elbow.Connect exception: " + e);
            }
        }

        public void Relax() { Relax("Body"); }
        public void Relax(String part)
        {
            _motion.setStiffnesses("Body", 0.0f);
        }

        public void Stiffen() { Stiffen("Body"); }
        public void Stiffen(String part)
        {
            _motion.setStiffnesses(part, 1.0f);
        }

        public void RSSend()
        {
            if (_motion == null) return;
            ArrayList joints2 = (ArrayList)joints.Clone(), values2 = (ArrayList)values.Clone();
            //joints2.RemoveRange(8 + 5, 5);
            //values2.RemoveRange(8 + 5, 5);
            _motion.setAngles(joints2, values2, speed);
        }

        public void RSSendBlocking()
        {
            if (_motion == null) return;
            _motion.setAngles(joints, values, 0.2f);
            //_motion.wait(id, 10000);
            System.Threading.Thread.Sleep(3000);
        }

        public void SetJoint(int ix, float val) { SetJoint(ix, val, 0); }

        public void SetJoint(int ix, float val, float smooth)
        {
            float prior = (float)values[ix];
            if (float.IsNaN(prior)) prior = 0;
            values[ix] = limits.Count <= ix
                       ? val
                       : ClampToRange(val, (float)((ArrayList)limits[ix])[0], (float)((ArrayList)limits[ix])[1]);
            if (smooth != 0)
            {
                values[ix] = prior * smooth + (float)values[ix] * (1 - smooth);
                Console.WriteLine("smooth: " + prior.ToString() + " " + values[ix].ToString());
            }
        }

        public void RSUpdatePitch(float val) { SetJoint(0, val);  }
        public void RSUpdateRoll (float val) { SetJoint(1, val);  }
        public void REUpdateYaw  (float val) { SetJoint(3, val);  }
        public void REUpdateRoll (float val) { SetJoint(2, val);  }
        public void LSUpdatePitch(float val) { SetJoint(4, val);  }
        public void LSUpdateRoll (float val) { SetJoint(5, val);  }
        public void LEUpdateYaw  (float val) { SetJoint(7, val);  }
        public void LEUpdateRoll (float val) { SetJoint(6, val);  }

        public void RHUpdateRoll (float val) { SetJoint(8, val);  }
        public void RHUpdatePitch(float val) { SetJoint(9, val);  }
        public void RKUpdatePitch(float val) { SetJoint(10, val); }
        public void RAUpdatePitch(float val) { SetJoint(11, val, 0.5f); }
        public void RAUpdateRoll (float val) { SetJoint(12, val, 0.5f); }

        public void LHUpdateRoll (float val) { SetJoint(13, val); }
        public void LHUpdatePitch(float val) { SetJoint(14, val); }
        public void LKUpdatePitch(float val) { SetJoint(15, val); }
        public void LAUpdatePitch(float val) { SetJoint(16, val, 0.5f); }
        public void LAUpdateRoll (float val) { SetJoint(17, val, 0.5f); }

        public static float Clamp(float f, float t, float v) {
            return Math.Max(f, Math.Min(t, v));
        }

        public static float Lerp(float f, float t, float v) {
            return (1 - v) * t + v * f;
        }

        public static float UnLerp(float f, float t, float v) {
            return (v - f) / (t - f);
        }

        public static float InterpClamp(float x1, float t1, float f1, float x2, float t2, float f2, float x, float y)
        {
            float t = UnLerp(x1, x2, x);
            float l = Lerp(f1, f2, t), h = Lerp(t1, t2, t);
            //Console.WriteLine("Clamp: " + l.ToString() + " " + h.ToString());
            return Clamp(l, h, y);
        }

        public static float FromRad(float rad) { return rad / (float)Math.PI * 180f; }
        public static float ToRad(float rad) { return rad * (float)Math.PI / 180f; }


        // Clamps roll for the left foot diagram.
        public static float NearestFeasibleRoll(float pitch, float roll) {
            float pitchDeg = FromRad(pitch);
            float rollDeg = FromRad(roll);
            if (pitchDeg < -48.12) {
                return ToRad(InterpClamp(-68.15f, 2.86f, -4.29f,
                                         -48.12f, 10.31f, -9.74f,
                                         pitchDeg, rollDeg));
            } else if (pitchDeg < -40.10) {
                return ToRad(InterpClamp(-48.12f, 10.31f, -9.74f,
                                         -40.10f, 22.79f, -12.60f,
                                         pitchDeg, rollDeg));
            } else if (pitchDeg < -25.78) {
                return ToRad(InterpClamp(-40.10f, 22.79f, -12.60f,
                                         -25.78f, 22.79f, -44.06f,
                                         pitchDeg, rollDeg));
            } else if (pitchDeg < 5.72) {
                return ToRad(InterpClamp(-25.78f, 22.79f, -44.06f,
                                         5.72f, 22.79f, -44.06f,
                                         pitchDeg, rollDeg));
            } else if (pitchDeg < 20.05) {
                return ToRad(InterpClamp(5.72f, 22.79f, -44.06f,
                                         20.05f, 22.79f, -31.54f,
                                         pitchDeg, rollDeg));
            } else {
                return ToRad(InterpClamp(20.05f, 22.79f, -31.54f,
                                         52.86f, 0f, -2.86f,
                                         pitchDeg, rollDeg));
            }
        }

        public void RAUpdate(float pitch, float roll) {
            float pitch2 = Clamp(-1.189516f, 0.922747f, pitch);
            RAUpdatePitch(pitch2); RAUpdateRoll(-NearestFeasibleRoll(pitch2, -roll));
        }

        public void LAUpdate(float pitch, float roll) {
            float pitch2 = Clamp(-1.189516f, 0.922747f, pitch);
            float roll2 = NearestFeasibleRoll(pitch2, roll);
            Console.WriteLine("pitch = " + pitch2.ToString());
            Console.WriteLine("roll = " + roll2.ToString());
            LAUpdatePitch(pitch2); LAUpdateRoll(roll2);
        }

        public void SetLHand(bool close)
        {
            if (_motion != null)
            {
                try
                {
                    if (close)
                    {
                        _motion.closeHand("LHand");
                    }
                    else
                    {
                        _motion.openHand("LHand");
                    }
                }
                catch (Exception e)
                {
                    Console.Out.WriteLine("SetLHand exception: " + e);
                }
            }
        }
        
        public void SetRHand(bool close)
        {
            if (_motion != null)
            {
                try
                {
                    if (close)
                    {
                        _motion.closeHand("RHand");
                    }
                    else
                    {
                        _motion.openHand("RHand");
                    }
                }
                catch (Exception e)
                {
                    Console.Out.WriteLine("SetRHand exception: " + e);
                }
            }
        }

        private static float ClampToRange(float val, float min, float max)
        {
            if (val < min) return min;
            if (val > max) return max;
            return val;
        }

        private static float ScaleToRange(double val, float min, float max)
        {
            float returnVal = (float)((val / 180.0) * (max - min)) + min;
            return returnVal;
        }

        public void walk(string direction)
        {
            if (_motion != null)
            {
                try
                {
                    //var postion = _motion.getRobotPosition(true);
                    //var listofPositions = postion.ToList();
                    switch (direction)
                    {
                        case "left":
                            _motion.walkTo(0.0f, 1.0f, 0f);
                            break;
                        case "right":
                            _motion.walkTo(0.0f, -1.0f, 0f);
                            break;
                        case "forward":
                            _motion.walkTo(1f, 0.0f, 0f);
                            break;
                        case "back":
                            _motion.walkTo(-1f, 0.0f, 0f);
                            break;
                    }
                }
                catch (Exception e)
                {
                    Console.Out.WriteLine("Walking exception: " + e);
                }
            }
        }

        public void readFSR()
        {
            List<string> list = _memory.getEventList();
            Console.WriteLine(_memory.getData("Device/SubDeviceList/LFoot/FSR/FrontLeft/Sensor/Value"));

            Console.WriteLine(_memory.getData("Device/SubDeviceList/InertialSensor/AccX/Sensor/Value"));
            Console.WriteLine(_memory.getData("Device/SubDeviceList/InertialSensor/AccY/Sensor/Value"));
            Console.WriteLine(_memory.getData("Device/SubDeviceList/InertialSensor/AccZ/Sensor/Value"));

            Console.WriteLine(_memory.getData("Device/SubDeviceList/InertialSensor/AngleX/Sensor/Value"));
        }



        public static Vector3 VectorFromList(List<float> fs)
        {
            return new Vector3(fs[0], fs[1], fs[2]);
        }

        public Vector3 GetCOM()
        {
            return Vector3.Transform(NaoPos.Convert(VectorFromList(_motion.getCOM("Body", 0, false))), Matrix.Identity);
        }

        public NaoPos GetPosition(string part)
        {
            return new NaoPos(_motion.getPosition(part, 0, false), Matrix.Identity);
        }

        public void PollSensors() {
            PollFootSensors(rightFoot, "R");
            PollFootSensors(leftFoot, "L");
            float gx = (float) _memory.getData("Device/SubDeviceList/InertialSensor/AngleX/Sensor/Value");
            float gy = (float) _memory.getData("Device/SubDeviceList/InertialSensor/AngleY/Sensor/Value");
            gyrot = NaoPos.ConvertRotation(gx, gy, 0);
        }

        public void PollFootSensors(NaoFoot foot, string prefix) {
            foot.pfr = GetPosition(prefix + "FsrFR");
            foot.prr = GetPosition(prefix + "FsrRR");
            foot.pfl = GetPosition(prefix + "FsrFL");
            foot.prl = GetPosition(prefix + "FsrRL");
            foot.ffr = (float)_memory.getData("Device/SubDeviceList/" + prefix + "Foot/FSR/FrontRight/Sensor/Value");
            foot.frr = (float)_memory.getData("Device/SubDeviceList/" + prefix + "Foot/FSR/RearRight/Sensor/Value");
            foot.ffl = (float)_memory.getData("Device/SubDeviceList/" + prefix + "Foot/FSR/FrontLeft/Sensor/Value");
            foot.frl = (float)_memory.getData("Device/SubDeviceList/" + prefix + "Foot/FSR/RearLeft/Sensor/Value");
        }

        public float Average(params float[] xs) {
            float sum = 0;
            foreach (float x in xs)
            {
                sum += x;
            }
            return sum / xs.Length;
        }

        public void Balance(int feet, List<LabelledVector> ls)
        {
            NaoFoot targetFoot = feet == 2 ? rightFoot : leftFoot;

            // Center of mass, and center of target, both in torso space.
            Vector3 com = GetCOM();
            Vector3 target = targetFoot.GetCenter();

            // Balance vector.  We need it to be vertical.
            Vector3 delta = Vector3.Subtract(com, target);

            // Transform into lower leg local space.
            Matrix mat = Matrix.Invert(GetPosition((feet == 2 ? "R" : "L") + "KneePitch").transform);
            Vector3 local = Vector3.Transform(delta, mat);

            // Take the angle of the vector to be the angle we need to rotate
            // the ground plane in order to achieve balance.
            float roll  = (float) Math.Atan2(local.X, local.Y);
            float pitch = (float) Math.Atan2(local.Z, local.Y);

            // Use Force sensors to tweak result.
            float forwardBias = Average(targetFoot.ffl - targetFoot.frl, targetFoot.ffr - targetFoot.frr) * 0.01f;
            float leftwardBias = Average(targetFoot.ffl - targetFoot.ffr, targetFoot.frl - targetFoot.frr) * 0.01f;
            Console.WriteLine("Biases: " + forwardBias.ToString() + " " + leftwardBias.ToString());

            Vector3 offset = new Vector3(0, 0, 3f);
            ls.Add(new LabelledVector(offset, Vector3.Add(offset, local), Color.Black, ""));
            ls.Add(new LabelledVector(offset, new Vector3(leftwardBias, 1f, 3f + forwardBias), Color.Green, ""));
            
            // Foot commands with experimental fudge factors
            if (feet == 2)
            {
                pitch += forwardBias;
                roll += leftwardBias;
                RAUpdate(pitch + 0.05f, -roll);
            }
            else
            {
                pitch += forwardBias;
                roll += leftwardBias;
                LAUpdate(pitch - 0.05f, 0.05f - roll);
            }
        }

        public void BalanceOld(int feet, List<LabelledVector> ls)
        {
            /*
            Matrix mat = Matrix.Identity;
            NaoFoot targetFoot = feet == 2 ? rightFoot : leftFoot;
            Vector3 target = Vector3.Transform(targetFoot.GetCenter(), mat);
            Vector3 delt = Vector3.Subtract(Vector3.Transform(GetCOM(), mat), target);
             */
            Matrix inv = Matrix.Invert(gyrot);
            Matrix mat = Matrix.Invert(GetPosition("LKneePitch").transform);
            //Vector3 kneeVec = Vector3.Transform(Vector3.UnitY, mat);

            //Matrix mat = GetPosition("LKneePitch").transform;

            NaoFoot targetFoot = feet == 2 ? rightFoot : leftFoot;

            float forwardBias = Average(targetFoot.ffl - targetFoot.frl, targetFoot.ffr - targetFoot.frr);
            float leftwardBias = Average(targetFoot.ffl - targetFoot.ffr, targetFoot.frl - targetFoot.frr);
            Console.WriteLine(forwardBias.ToString() + " " + leftwardBias.ToString());

            Vector3 com = Vector3.Transform(GetCOM(), mat);
            Vector3 target = targetFoot.GetCenter();
            Vector3 orient = targetFoot.GetDirection();
            // TODO: make orient direction independent of current foot position.
            orient.Y = 0;
            Vector3 perp = new Vector3(-orient.Z, 0, orient.X);

            ls.Add(new LabelledVector(target, Vector3.Add(target, perp), Color.Green, ""));
            ls.Add(new LabelledVector(target, com, Color.Blue, ""));
            ls.Add(new LabelledVector(target, Vector3.Add(target, orient), Color.Red, ""));

            /*
            Matrix mat = Matrix.Invert(GetPosition("LKneePitch").transform);
            //Vector3 comT    = Vector3.Transform(com,    mat);
            Vector3 targetT = Vector3.Transform(target, mat);
            Vector3 orientT = Vector3.Transform(orient, mat);
            Vector3 perpT   = Vector3.Transform(perp,   mat);
            */

            Vector3 delt = Vector3.Subtract(com, target);
            
            orient.Normalize();
            perp.Normalize();
            delt.Normalize();
            //kneeVec.Normalize();

            //delt = Vector3.Subtract(delt, kneeVec);
            //delt.Normalize();

            /*
            delt.Normalize();
            Vector3 orient = Vector3.Transform(targetFoot.GetDirection(), mat);
            orient.Z = 0;
            orient.Normalize();
            Vector3 perp = new Vector3(-orient.Y, orient.X, 0);
            */

            Vector3 offset = new Vector3(0, 0, 3f);

            ls.Add(new LabelledVector(offset, Vector3.Add(offset, perp),  Color.Green, ""));
            ls.Add(new LabelledVector(offset, Vector3.Add(offset, delt),  Color.Blue,  ""));
            ls.Add(new LabelledVector(offset, Vector3.Add(offset, orient), Color.Red, ""));
           // ls.Add(new LabelledVector(offset, Vector3.Add(offset, kneeVec), Color.Purple, ""));

            float roll  = (float)(Math.PI / 2 - Math.Acos((double)Vector3.Dot(perp, delt)));
            float pitch = (float)(Math.PI / 2 - Math.Acos((double)Vector3.Dot(orient, delt)));
            //Console.WriteLine("pitch = " + pitch.ToString());
            //Console.WriteLine("roll = " + roll.ToString());
            /* if (feet == 3)
            {
                RAUpdate((float)(Math.PI / 2) - pitch, 0);
                LAUpdate((float)(Math.PI / 2) - pitch, 0);
            }
            else */
            //if (forwardBias > 0)
            pitch -= 0.04f * forwardBias;
            roll  -= 0.04f * leftwardBias;
            if (feet == 2)
            {
                RAUpdate(-pitch, -roll);
            }
            else if (feet == 1)
            {
                LAUpdate(pitch - 0.15f, roll + 0.1f);
            }
        }

        public void updateFoot(NaoFoot foot)
        {
            Vector3 fr = GetPosition(foot.name + "FsrFR").position,
                    rr = GetPosition(foot.name + "FsrRR").position,
                    fl = GetPosition(foot.name + "FsrFL").position,
                    rl = GetPosition(foot.name + "FsrRL").position;
 
            Vector3 leftSide = Vector3.Subtract(fl, rl);
            Vector3 rightSide = Vector3.Subtract(fr, rr);
            
            Vector3 COM =  GetCOM();
            //A || B = B Ã— (A Ã— B) / |B|Â² 

            Plane footPlane = new Plane(fr, fl, rr);
            Vector3 planeNormal = footPlane.Normal;

            Vector3 COMproj = Vector3.Cross(planeNormal, (Vector3.Cross(COM, planeNormal))) / (planeNormal.LengthSquared());

            //(AB x AC)/|AB|
            // AB = leftSide, rightSide
            // A = rl, rr
            Vector3 tempL = Vector3.Subtract(COMproj, rl);
            Vector3 tempR = Vector3.Subtract(COMproj, rr);


            double distance1 = Math.Sin(Math.Acos((double)(Vector3.Dot(leftSide, tempL) / (leftSide.Length() * tempL.Length())))) * tempL.Length();
            double distance2 = Math.Sin(Math.Acos((double)(Vector3.Dot(rightSide, tempR) / (rightSide.Length() * tempR.Length())))) * tempR.Length();

            float d1 = Vector3.Distance(leftSide, COMproj);
            float d2 = Vector3.Distance(rightSide, COMproj);

            double rise = (fl.Y - fr.Y);
            double run = (fl.X - fr.X);
            double width2D = Math.Sqrt(rise*rise + run*run);
            float width = Vector3.Distance(fr, fl);
            // width front = 0.053
            

            //if (distance1 < distance2)
            if (foot.name == "R")
            {
                foot.innerEdge = (float) distance1;
                foot.outerEdge = (float) distance2;
            }
            else
            {
                foot.innerEdge = (float) distance2;
                foot.outerEdge = (float) distance1;
            }
            foot.width = width;
        }

        public float computeOffsetParam()
        {
            updateFoot(leftFoot);
            updateFoot(rightFoot);
            float offsetL = leftFoot.GetOffset();
            float offsetR = rightFoot.GetOffset();
            return OffsetParameter(offsetL, offsetR);
        }

        public float OffsetParameter(float offsetL, float offsetR)
        {
            if (offsetL < offsetR)
            {
                return (offsetL / offsetR) / 2;
            }
            else if (offsetR < offsetL)
            {
                return 1 - ((offsetR / offsetL) / 2);
            }
            else
            {
                return 0.5f;
            }
        }
    }
}