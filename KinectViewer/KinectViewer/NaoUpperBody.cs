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

        public void Connect(string ip)
        {
            try
            {
                _motion = new MotionProxy(ip, 9559);
                // --------------- prepare limits --------------------------
                // three floats as an ArrayList for each joint in the chain
                // min,max,maxNoLoadSpeedPerCycle
                ArrayList rightArmLimits = (ArrayList)_motion.getLimits("RArm");
                ArrayList RSPitchLimits = (ArrayList)rightArmLimits[0];
                _RSminPitch = (float)RSPitchLimits[0];
                _RSmaxPitch = (float)RSPitchLimits[1];

                ArrayList RSRollLimits = (ArrayList)rightArmLimits[1];
                _RSminRoll = (float)RSRollLimits[0];
                _RSmaxRoll = (float)RSRollLimits[1];

                ArrayList REYawLimits = (ArrayList)rightArmLimits[2];
                _REminYaw = (float)REYawLimits[0];
                _REmaxYaw = (float)REYawLimits[1];

                ArrayList RERollLimits = (ArrayList)rightArmLimits[3];
                _REminRoll = (float)RERollLimits[0];
                _REmaxRoll = (float)RERollLimits[1];


                ArrayList leftArmLimits = (ArrayList)_motion.getLimits("LArm");
                ArrayList LSPitchLimits = (ArrayList)leftArmLimits[0];
                _LSminPitch = (float)LSPitchLimits[0];
                _LSmaxPitch = (float)LSPitchLimits[1];

                ArrayList LSRollLimits = (ArrayList)leftArmLimits[1];
                _LSminRoll = (float)LSRollLimits[0];
                _LSmaxRoll = (float)LSRollLimits[1];

                ArrayList LEYawLimits = (ArrayList)leftArmLimits[2];
                _LEminYaw = (float)LEYawLimits[0];
                _LEmaxYaw = (float)LEYawLimits[1];

                ArrayList LERollLimits = (ArrayList)leftArmLimits[3];
                _LEminRoll = (float)LERollLimits[0];
                _LEmaxRoll = (float)LERollLimits[1];

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

        public void RSUpdateRoll(float val)
        {
            if (_motion != null)
            {
                try
                {
                    _motion.setAngles("RShoulderRoll", ClampToRange(val, _RSminRoll, _RSmaxRoll), speed);
                }
                catch (Exception e)
                {
                    Console.Out.WriteLine("RSUpdateRoll exception: " + e);
                }
            }
        }

        public void RSUpdatePitch(float val)
        {
            if (_motion != null)
            {
                try
                {
                    _motion.setAngles("RShoulderPitch", ClampToRange(val, _RSminPitch, _RSmaxPitch), speed);
                }
                catch (Exception e)
                {
                    Console.Out.WriteLine("RSUpdatePitch exception: " + e);
                }
            }
        }

        public void REUpdateYaw(float val)
        {
            if (_motion != null)
            {
                try
                {
                    _motion.setAngles("RElbowYaw", ClampToRange(val, _REminYaw, _REmaxYaw), speed);
                }
                catch (Exception e)
                {
                    Console.Out.WriteLine("REUpdateYaw exception: " + e);
                }
            }
        }

        public void REUpdateRoll(float val)
        {
            if (_motion != null)
            {
                try
                {
                    _motion.setAngles("RElbowRoll", ClampToRange(val, _REminRoll, _REmaxRoll), speed);
                }
                catch (Exception e)
                {
                    Console.Out.WriteLine("REUpdateRoll exception: " + e);
                }
            }
        }

        public void LSUpdateRoll(float val)
        {
            if (_motion != null)
            {
                try
                {
                    _motion.setAngles("LShoulderRoll", ClampToRange(val, _LSminRoll, _LSmaxRoll), speed);
                }
                catch (Exception e)
                {
                    Console.Out.WriteLine("LShoulder.UpdateRoll exception: " + e);
                }
            }
        }

        public void LSUpdatePitch(float val)
        {
            if (_motion != null)
            {
                try
                {
                    _motion.setAngles("LShoulderPitch", ClampToRange(val, _LSminPitch, _LSmaxPitch), speed);
                }
                catch (Exception e)
                {
                    Console.Out.WriteLine("LShoulder.UpdatePitch exception: " + e);
                }
            }
        }

        public void LEUpdateYaw(float val)
        {
            if (_motion != null)
            {
                try
                {
                    _motion.setAngles("LElbowYaw", ClampToRange(val, _LEminYaw, _LEmaxYaw), speed);
                }
                catch (Exception e)
                {
                    Console.Out.WriteLine("LEUpdateYaw exception: " + e);
                }
            }
        }

        public void LEUpdateRoll(float val)
        {
            if (_motion != null)
            {
                try
                {
                    _motion.setAngles("LElbowRoll", ClampToRange(val, _LEminRoll, _LEmaxRoll), speed);
                }
                catch (Exception e)
                {
                    Console.Out.WriteLine("LEUpdateRoll exception: " + e);
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
