using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using Aldebaran.Proxies;
using System.Collections;
using Microsoft.Xna.Framework;
using System.Threading;

namespace KinectViewer
{
    class NaoBody
    {
        public bool connected
        {
            get
            {
                return proxy != null;
            }
        }

        private float speed = 0.2f;

        public ArrayList parts;
        public Dictionary<string, float> jointToAngle;
        public Dictionary<string, ArrayList> limits;
        private NaoProxy proxy;

        public void Connect(string ip)
        {
            jointToAngle = new Dictionary<string, float>();
            limits = new Dictionary<string, ArrayList>();

            jointToAngle.Add("RShoulderPitch", 0);
            jointToAngle.Add("RShoulderRoll", 0);
            jointToAngle.Add("RElbowRoll", 0);
            jointToAngle.Add("RElbowYaw", 0);
            jointToAngle.Add("LShoulderPitch",0);
            jointToAngle.Add("LShoulderRoll", 0);
            jointToAngle.Add("LElbowRoll", 0);
            jointToAngle.Add("LElbowYaw", 0);
            jointToAngle.Add("RHipRoll", 0);
            jointToAngle.Add("RHipPitch", 0);
            jointToAngle.Add("RKneePitch", 0);
            jointToAngle.Add("RAnklePitch", 0);
            jointToAngle.Add("RAnkleRoll", 0);
            jointToAngle.Add("LHipRoll", 0);
            jointToAngle.Add("LHipPitch", 0);
            jointToAngle.Add("LKneePitch", 0);
            jointToAngle.Add("LAnklePitch", 0);
            jointToAngle.Add("LAnkleRoll", 0);


           
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


            

            try
            {
                MemoryProxy memory = new MemoryProxy(ip, 9559);
                MotionProxy motion = new MotionProxy(ip, 9559);

                foreach (string joint in jointToAngle.Keys)
                {
                   
                    limits.Add(joint, (ArrayList)motion.getLimits(joint));
                }
                // give the joints some stiffness
                motion.setStiffnesses("Body", 1.0f);

                proxy = new NaoProxy(memory, motion, parts, 100);
                Thread thread = new Thread(new ThreadStart(proxy.PollLoop));
                thread.Start();
            }
            catch (Exception e)
            {
                Console.Out.WriteLine("Elbow.Connect exception: " + e);
            }
        }

        public void Relax() { proxy.Relax("Body"); }


        public void Stiffen() { proxy.Stiffen("Body"); }


        public NaoFoot GetRightFoot()
        {
            return proxy.getRightFoot();
        }

        public NaoFoot GetLeftFoot()
        {
            return proxy.getRightFoot();
        }

        public Matrix GetGyrot()
        {
            return proxy.GetGyRot();
        }

        public void RSSend()
        {
            if (proxy == null) return;
            ArrayList joints = new ArrayList();
            ArrayList values = new ArrayList();
            foreach (string joint in jointToAngle.Keys)
            {
                joints.Add(joint);
                values.Add(jointToAngle[joint]);
            }
            proxy.SetAngles(joints, values, speed);
        }

        public void NaoSimUpdate(NaoSimulator sim)
        {
            
            sim.UpdatePositions(jointToAngle);

        }


        public void RSSendBlocking()
        {
            if (proxy == null) return;
            ArrayList joints = new ArrayList();
            ArrayList values = new ArrayList();
            foreach (string joint in jointToAngle.Keys)
            {
                joints.Add(joint);
                values.Add(jointToAngle[joint]);
            }
            proxy.SetAngles(joints, values, .2f);
            //_motion.wait(id, 10000);
            System.Threading.Thread.Sleep(3000);
        }

        //public void SetJoint(string jointName, float val) { SetJoint(jointName, val, 0); }

        public void SetJoint(string jointName, float val, float smooth)
        {
            float prior = (float)jointToAngle[jointName];
            
            if (float.IsNaN(prior)) prior = 0;
                ArrayList limit = (ArrayList) limits[jointName][0];
                jointToAngle[jointName] = ClampToRange(val, (float) limit[0], (float) limit[1]);
               
            if (smooth != 0)
            {
                jointToAngle[jointName] = prior * smooth + (float)jointToAngle[jointName] * (1 - smooth);
                //Console.WriteLine("smooth: " + prior.ToString() + " " + values[ix].ToString());
            }
        }

        public void UpdateAngle(string jointName, float val, float smooth) 
        {
            SetJoint(jointName, val, smooth);
        }

        public void UpdateAngle(string jointName, float val)
        {
           UpdateAngle(jointName, val, 0);
        }
        

        //public void RSUpdatePitch(float val) { SetJoint(0, val);  }
        //public void RSUpdateRoll (float val) { SetJoint(1, val);  }
        //public void REUpdateYaw  (float val) { SetJoint(3, val);  }
        //public void REUpdateRoll (float val) { SetJoint(2, val);  }
        //public void LSUpdatePitch(float val) { SetJoint(4, val);  }
        //public void LSUpdateRoll (float val) { SetJoint(5, val);  }
        //public void LEUpdateYaw  (float val) { SetJoint(7, val);  }
        //public void LEUpdateRoll (float val) { SetJoint(6, val);  }

        //public void RHUpdateRoll (float val) { SetJoint(8, val);  }
        //public void RHUpdatePitch(float val) { SetJoint(9, val);  }
        //public void RKUpdatePitch(float val) { SetJoint(10, val); }
        //public void RAUpdatePitch(float val) { SetJoint(11, val, 0.5f); }
        //public void RAUpdateRoll (float val) { SetJoint(12, val, 0.5f); }

        //public void LHUpdateRoll (float val) { SetJoint(13, val); }
        //public void LHUpdatePitch(float val) { SetJoint(14, val); }
        //public void LKUpdatePitch(float val) { SetJoint(15, val); }
        //public void LAUpdatePitch(float val) { SetJoint(16, val, 0.5f); }
        //public void LAUpdateRoll (float val) { SetJoint(17, val, 0.5f); }

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
            //RAUpdatePitch(pitch2); RAUpdateRoll(-NearestFeasibleRoll(pitch2, -roll));
            UpdateAngle("RAnklePitch", pitch2);
            UpdateAngle("RAnkleRoll", -NearestFeasibleRoll(pitch2, -roll));
        }

        public void LAUpdate(float pitch, float roll) {
            float pitch2 = Clamp(-1.189516f, 0.922747f, pitch);
            float roll2 = NearestFeasibleRoll(pitch2, roll);
            //Console.WriteLine("pitch = " + pitch2.ToString());
            //Console.WriteLine("roll = " + roll2.ToString());
            UpdateAngle("LAnklePitch", pitch2);
            UpdateAngle("LAnkleRoll", roll2);
            //LAUpdatePitch(pitch2); LAUpdateRoll(roll2);
        }

        /*
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
        */

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
            if (proxy != null)
            {
                proxy.walk(direction);
            }
        }

        public static Vector3 VectorFromList(List<float> fs)
        {
            return new Vector3(fs[0], fs[1], fs[2]);
        }

        public Vector3 GetCOM()
        {
            return Vector3.Transform(NaoPos.Convert(proxy.GetCOM()), Matrix.Identity);
        }

        public NaoPos GetPosition(string part)
        {
            return proxy.GetPosition(part);
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
            NaoFoot targetFoot = feet == 2 ? proxy.getRightFoot() : proxy.getLeftFoot();

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
            //Console.WriteLine("Biases: " + forwardBias.ToString() + " " + leftwardBias.ToString());

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

        public float computeOffsetParam()
        {
            NaoFoot leftFoot = proxy.getLeftFoot();
            NaoFoot rightFoot = proxy.getRightFoot();
            leftFoot.updateFoot(GetCOM());
            rightFoot.updateFoot(GetCOM());
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