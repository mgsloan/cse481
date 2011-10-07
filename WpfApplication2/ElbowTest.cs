using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using Aldebaran.Proxies;
using System.Collections;

namespace WpfApplication2
{
    class ElbowTest
    {
        MotionProxy _motion = null;

        private float _minYaw = 0.0f;
        private float _maxYaw = 0.0f;
        private float _minRoll = 0.0f;
        private float _maxRoll = 0.0f;
        private float _minShoulderRoll = 0.0f;
        private float _maxShoulderRoll = 0.0f;
        private float _minShoulderPitch = 0.0f;
        private float _maxShoulderPitch = 0.0f;

        public void Connect(string ip)
        {
            try
            {
                _motion = new MotionProxy(ip, 9559);
                // --------------- prepare limits --------------------------
                // three floats as an ArrayList for each joint in the chain
                // min,max,maxNoLoadSpeedPerCycle
                ArrayList limits = (ArrayList)_motion.getLimits("RArm");
                ArrayList yawLimits = (ArrayList)limits[2];
                ArrayList rollLimits = (ArrayList)limits[3];
                _minYaw = (float)yawLimits[0];
                _maxYaw = (float)yawLimits[1];
                _minRoll = (float)rollLimits[0];
                _maxRoll = (float)rollLimits[1];

                ArrayList leftArmLimits = (ArrayList)_motion.getLimits("LArm");
                ArrayList shoulderPitchLimits = (ArrayList)leftArmLimits[0];
                _minShoulderPitch = (float)shoulderPitchLimits[0];
                _maxShoulderPitch = (float)shoulderPitchLimits[1];

                ArrayList shoulderRollLimits = (ArrayList)leftArmLimits[1];
                _minShoulderRoll = (float)shoulderRollLimits[0];
                _maxShoulderRoll = (float)shoulderRollLimits[1];

                // give the joints some stiffness
                _motion.stiffnessInterpolation("RArm", 1.0f, 1.0f);

            }
            catch (Exception e)
            {
                Console.Out.WriteLine("Elbow.Connect exception: " + e);
            }
        }

        public void UpdateYaw(double val)
        {
            if (_motion != null)
            {
                try
                {
                    _motion.setAngles("RElbowYaw", ScaleToRange(val, _minYaw, _maxYaw), 0.1f);
                }
                catch (Exception e)
                {
                    Console.Out.WriteLine("Elbow.UpdateYaw exception: " + e);
                }
            }
        }

        public void UpdateRoll(double val)
        {
            if (_motion != null)
            {
                try
                {
                    _motion.setAngles("RElbowRoll", ScaleToRange(val, _minRoll, _maxRoll), 0.1f);
                }
                catch (Exception e)
                {
                    Console.Out.WriteLine("Elbow.UpdateRoll exception: " + e);
                }
            }
        }

        public void updateLeftShoulderRoll(double val)
        {
            if (_motion != null)
            {
                try
                {
                    int num = Convert.ToInt32(180-val);
                    Console.WriteLine("updateLeftShoulder angle: " + num);
                    _motion.setAngles("LShoulderRoll", ScaleToRange(num, _minShoulderRoll, _maxShoulderRoll), 0.1f);
                }
                catch (Exception e)
                {
                    Console.Out.WriteLine("LShoulder.UpdateRoll exception: " + e);
                }
            }
        }

        public void updateLeftShoulderPitch(double val)
        {
            if (_motion != null)
            {
                try
                {
                    int num = Convert.ToInt32(180-val);
                    Console.WriteLine("updateLeftShoulder angle: " + num);
                    _motion.setAngles("LShoulderPitch", ScaleToRange(num, _minShoulderPitch, _maxShoulderPitch), 0.1f);
                }
                catch (Exception e)
                {
                    Console.Out.WriteLine("LShoulder.UpdatePitch exception: " + e);
                }
            }
        }


        private static float ScaleToRange(double val, float min, float max)
        {
            float returnVal = (float) ((val / 180.0) * (max - min)) + min;
            return returnVal;
        }
    }
}
