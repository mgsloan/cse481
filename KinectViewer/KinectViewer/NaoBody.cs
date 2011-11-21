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

        private float speed = 1.0f;

        public ArrayList joints;
        public ArrayList values;
        public ArrayList limits;
        public ArrayList parts;

        public void Connect(string ip)
        {
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

        public void Relax()
        {
            _motion.setStiffnesses("Body", 0.0f);
        }

        public void RSSend()
        {
            if (_motion == null) return;
            _motion.setAngles(joints, values, speed);
        }

        public void RSSendBlocking()
        {
            if (_motion == null) return;
            _motion.setAngles(joints, values, 0.2f);
            //_motion.wait(id, 10000);
            System.Threading.Thread.Sleep(3000);
        }

        public void SetJoint(int ix, float val)
        {
            values[ix] = limits.Count <= ix
                       ? val
                       : ClampToRange(val, (float)((ArrayList)limits[ix])[0], (float)((ArrayList)limits[ix])[1]);
        }

        public static Vector3 VectorFromList(List<float> fs)
        {
            return new Vector3(fs[0], fs[1], fs[2]);
        }

        public Vector3 getGyro()
        {
            float x = (float) _memory.getData("Device/SubDeviceList/InertialSensor/AngleX/Sensor/Value");
            float y = (float) _memory.getData("Device/SubDeviceList/InertialSensor/AngleY/Sensor/Value");
            Vector3 v = new Vector3();
            v.X = x;
            v.Y = y;
            v.Z = 0;

            return v; 
        }

        public Vector3 getCOM()
        {
            return VectorFromList(_motion.getCOM("Body", 0, false));
        }

        public Vector3 getPosition(string part)
        {
            return VectorFromList(_motion.getPosition(part, 0, false));
        }

        //TODO: return the support polygon
        public Tuple<Vector3, Vector3> getFootInfo(string prefix)
        {
            Vector3 fr = getPosition(prefix + "FsrFR"),
                    rr = getPosition(prefix + "FsrRR"),
                    fl = getPosition(prefix + "FsrFL"),
                    rl = getPosition(prefix + "FsrFL");
            return new Tuple<Vector3, Vector3>(
                vectorAverage(fr, rr, fl, rl),
                vectorAverage(Vector3.Subtract(fr, rr), Vector3.Subtract(fl, rl)));
        }

        protected static Vector3 vectorAverage(params Vector3[] vs)
        {
            Vector3 result = new Vector3();
            foreach (Vector3 v in vs) {
                result = Vector3.Add(v, result);
            }
            return Vector3.Divide(result, vs.Length);
        }

        public void supportedBalance(int feet, List<LabelledVector> ls)
        {
            if (feet == 3)
            {
                Tuple<Vector3, Vector3> infr = getFootInfo("R"), infl = getFootInfo("L");
                
                balance(new Tuple<Vector3, Vector3>(
                    vectorAverage(infr.Item1, infl.Item1),
                    vectorAverage(infr.Item2, infl.Item2)), feet, ls);
            }
            else if (feet == 2)
            {
                balance(getFootInfo("R"), feet, ls);
            }
            else if (feet == 1)
            {
                balance(getFootInfo("L"), feet, ls);
            }
        }

        public void balance(Tuple<Vector3, Vector3> target, int feet, List<LabelledVector> ls)
        {
            Vector3 com = getCOM();
            Vector3 delt = Vector3.Subtract(com, target.Item1);
            delt.Normalize();
            Vector3 orient = target.Item2;
            orient.Z = 0;
            orient.Normalize();
            Vector3 perp = new Vector3(-orient.Y, orient.X, 0);
            ls.Clear();
            //ls.Add(new LabelledVector(Vector3.Zero, Vector3.Multiply(orient, 5f), Color.Red,   "o"));
            //ls.Add(new LabelledVector(Vector3.Zero, Vector3.Multiply(perp, 5f), Color.Green, "p"));
            //ls.Add(new LabelledVector(Vector3.Zero, Vector3.Multiply(delt, 5f), Color.Blue, "d"));
            ls.Add(new LabelledVector(Vector3.Zero, Vector3.Multiply(com, 5f), Color.Blue, "c"));
            ls.Add(new LabelledVector(Vector3.Zero, Vector3.Multiply(target.Item1, 5f), Color.Red, "d"));
            float pitch = (float)Math.Acos((double)Vector3.Dot(orient, delt));
            float roll  = (float)Math.Acos((double)Vector3.Dot(perp,   delt));
            Console.WriteLine("pitch = " + pitch.ToString());
            Console.WriteLine("roll = " + roll.ToString());
            if (feet == 3)
            {
                RAUpdate((float)(Math.PI / 2) - pitch, 0);
                LAUpdate((float)(Math.PI / 2) - pitch, 0);
            }
            else if (feet == 2)
            {
                RAUpdate((float)(Math.PI / 2) - pitch, (float)(Math.PI / 2) - roll);
            }
            else if (feet == 1)
            {
                LAUpdate((float)(Math.PI / 2) - pitch, (float)(Math.PI / 2) - roll);
            }
        }

        public void forceBalance() {
            Console.WriteLine(_memory.getData("Device/SubDeviceList/LFoot/FSR/FrontLeft/Sensor/Value"));
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
        public void RAUpdatePitch(float val) { SetJoint(11, val); }
        public void RAUpdateRoll (float val) { SetJoint(12, val); }

        public void LHUpdateRoll (float val) { SetJoint(13, val); }
        public void LHUpdatePitch(float val) { SetJoint(14, val); }
        public void LKUpdatePitch(float val) { SetJoint(15, val); }
        public void LAUpdatePitch(float val) { SetJoint(16, val); }
        public void LAUpdateRoll (float val) { SetJoint(17, val); }

        public static float Clamp(float f, float t, float v) {
            return Math.Max(f, Math.Min(t, v));
        }

        public static float Lerp(float f, float t, float v) {
            return (1 - v) * t + v * f;
        }

        public static float UnLerp(float f, float t, float v) {
            return (v - f) / (t - v);
        }

        public static float InterpClamp(float x1, float t1, float f1, float x2, float t2, float f2, float x, float y)
        {
            float t = UnLerp(x1, x2, x);
            return Clamp(Lerp(f1, f2, t), Lerp(t1, t2, t), y);
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
                                         rollDeg, pitchDeg));
            } else if (pitchDeg < -40.10) {
                return ToRad(InterpClamp(-48.12f, 10.31f, -9.74f,
                                         -40.10f, 22.79f, -12.60f,
                                         rollDeg, pitchDeg));
            } else if (pitchDeg < -25.78) {
                return ToRad(InterpClamp(-40.10f, 22.79f, -12.60f,
                                         -25.78f, 22.79f, -44.06f,
                                         rollDeg, pitchDeg));
            } else if (pitchDeg < 5.72) {
                return ToRad(InterpClamp(-25.78f, 22.79f, -44.06f,
                                         5.72f, 22.79f, -44.06f,
                                         rollDeg, pitchDeg));
            } else if (pitchDeg < 20.05) {
                return ToRad(InterpClamp(5.72f, 22.79f, -44.06f,
                                         20.05f, 22.79f, -31.54f,
                                         rollDeg, pitchDeg));
            } else {
                return ToRad(InterpClamp(20.05f, 22.79f, -31.54f,
                                         52.86f, 0f, -2.86f,
                                         rollDeg, pitchDeg));
            }
        }

        public void RAUpdate(float pitch, float roll) {
            float pitch2 = Clamp(-1.189516f, 0.922747f, pitch);
            RAUpdatePitch(pitch2); RAUpdateRoll(-NearestFeasibleRoll(pitch2, -roll));
        }

        public void LAUpdate(float pitch, float roll) {
            float pitch2 = Clamp(-1.189516f, 0.922747f, pitch);
            LAUpdatePitch(pitch2); LAUpdateRoll(NearestFeasibleRoll(pitch2, roll));
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
    }
}
