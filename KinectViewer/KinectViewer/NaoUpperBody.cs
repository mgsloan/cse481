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
        MotionProxy _motion = null;

        private float _RSminRoll = 0.0f;
        private float _RSmaxRoll = 0.0f;
        private float _RSminPitch = 0.0f;
        private float _RSmaxPitch = 0.0f;
        private float _REminYaw = 0.0f;
        private float _REmaxYaw = 0.0f;
        private float _REminRoll = 0.0f;
        private float _REmaxRoll = 0.0f;

        private float _LSminRoll = 0.0f;
        private float _LSmaxRoll = 0.0f;
        private float _LSminPitch = 0.0f;
        private float _LSmaxPitch = 0.0f;
        private float _LEminYaw = 0.0f;
        private float _LEmaxYaw = 0.0f;
        private float _LEminRoll = 0.0f;
        private float _LEmaxRoll = 0.0f;

        private float speed = 0.5f;

        private ArrayList names;
        private ArrayList values;
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


            for (int i = 0; i < names.Count; i++) {
                values.Add(0);
            }

            try
            {
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
            _motion.setAngles(names, values, speed);
        }

        public void SetJoint(int ix, float val)
        {
            if (_motion == null || limits.Count <= ix) return;
            values[ix] = ClampToRange(val, (float)((ArrayList)limits[ix])[0], (float)((ArrayList)limits[ix])[1]);
        }

        public void RecordAngles(System.IO.StreamWriter writer)
        {
            StringBuilder builder = new StringBuilder();
                    
            for (int i = 0; i < values.Count; i++)
            { 
                builder.AppendFormat("{0}", values[i]);
                if (i != values.Count) builder.Append(", ");
            }
            writer.WriteLine(builder.ToString());
        }

        public void RSUpdatePitch(float val) { SetJoint(0, val); }
        public void RSUpdateRoll(float val)  { SetJoint(1, val); }
        public void REUpdateYaw(float val)   { SetJoint(3, val); }
        public void REUpdateRoll(float val)  { SetJoint(2, val); }
        public void LSUpdatePitch(float val) { SetJoint(4, val); }
        public void LSUpdateRoll(float val)  { SetJoint(5, val); }
        public void LEUpdateYaw(float val)   { SetJoint(7, val); }
        public void LEUpdateRoll(float val)  { SetJoint(6, val); }

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
    }
}
