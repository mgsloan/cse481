using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using Aldebaran.Proxies;
using System.Collections;

namespace KinectViewer
{
    class NaoUpperBody
    {
        MemoryProxy _memory = null;
        MotionProxy _motion = null;

        private float speed = 0.5f;

        private ArrayList names;
        public ArrayList values;
        private ArrayList limits;

        public void Connect(string ip)
        {
            names = new ArrayList();
            values = new ArrayList();
            limits = new ArrayList();

            names.Add("RShoulderPitch");
            names.Add("RShoulderRoll");
            names.Add("RElbowRoll");
            names.Add("RElbowYaw");
            names.Add("LShoulderPitch");
            names.Add("LShoulderRoll");
            names.Add("LElbowRoll");
            names.Add("LElbowYaw");
            names.Add("RHipRoll");
            names.Add("RHipPitch");
            names.Add("RKneePitch");
            names.Add("LHipRoll");
            names.Add("LHipPitch");
            names.Add("LKneePitch");

            for (int i = 0; i < names.Count; i++) {
                values.Add(0.0f);
            }

            try
            {
                _memory = new MemoryProxy(ip, 9559);
                _motion = new MotionProxy(ip, 9559);

                for (int i = 0; i < names.Count; i++)
                {
                    limits.Add(((ArrayList)_motion.getLimits((string)names[i]))[0]);
                }

                // give the joints some stiffness
                _motion.stiffnessInterpolation("RArm", 1.0f, 1.0f);
                _motion.stiffnessInterpolation("LArm", 1.0f, 1.0f);
                _motion.stiffnessInterpolation("LLeg", 1.0f, 1.0f);
                _motion.stiffnessInterpolation("RLeg", 1.0f, 1.0f);

            }
            catch (Exception e)
            {
                Console.Out.WriteLine("Elbow.Connect exception: " + e);
            }
        }

        public void RSSend()
        {
            if (_motion == null) return;
            _motion.setAngles(names, values, speed);
        }

        public void RSSendBlocking()
        {
            if (_motion == null) return;
            _motion.setAngles(names, values, 0.2f);
            //_motion.wait(id, 10000);
            System.Threading.Thread.Sleep(3000);
        }

        public void SetJoint(int ix, float val)
        {
            values[ix] = limits.Count <= ix
                       ? val
                       : ClampToRange(val, (float)((ArrayList)limits[ix])[0], (float)((ArrayList)limits[ix])[1]);
        }
        
        public void RecordAngles(System.IO.StreamWriter writer)
        {

        }

        public void RSUpdatePitch(float val) { SetJoint(0, val); }
        public void RSUpdateRoll(float val)  { SetJoint(1, val); }
        public void REUpdateYaw(float val)   { SetJoint(3, val); }
        public void REUpdateRoll(float val)  { SetJoint(2, val); }
        public void LSUpdatePitch(float val) { SetJoint(4, val); }
        public void LSUpdateRoll(float val)  { SetJoint(5, val); }
        public void LEUpdateYaw(float val)   { SetJoint(7, val); }
        public void LEUpdateRoll(float val)  { SetJoint(6, val); }

        public void RHUpdateRoll(float val)  { SetJoint(8, val); }
        public void RHUpdatePitch(float val) { SetJoint(9, val); }
        public void RKUpdatePitch(float val) { SetJoint(10, val); }

        public void LHUpdateRoll(float val) { SetJoint(11, val); }
        public void LHUpdatePitch(float val) { SetJoint(12, val); }
        public void LKUpdatePitch(float val) { SetJoint(13, val); }

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
