using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading;
using Aldebaran.Proxies;
using Microsoft.Xna.Framework;
using System.Collections;

namespace KinectViewer
{
    class NaoProxy
    {
        // fields for storing polled items
        private float gx;
        private float gy;
        private Matrix gyrot;
        private Vector3 COM;

        private NaoFoot leftFoot;
        private NaoFoot rightFoot;

        private MemoryProxy _memory = null;
        private MotionProxy _motion = null;

        private Dictionary<String, NaoPos> positions = new Dictionary<string, NaoPos>();

        private ArrayList parts;
        private int msBetweenPolls;

        //locks
        Object objLock = new Object();

        //parts should include every part you'd be interested in asking about at any time in the future
        public NaoProxy(MemoryProxy memory, MotionProxy motion, ArrayList parts, int msBetweenPolls)
        {
            _memory = memory;
            _motion = motion;
            this.parts = parts;
            leftFoot = new NaoFoot("L");
            rightFoot = new NaoFoot("R");
            this.msBetweenPolls = msBetweenPolls;
        }

        // method that runs in a loop updating the fields

        public void PollLoop()
        {
            while (true)
            {
                Poll();

                Thread.Sleep(this.msBetweenPolls);
            }
        }

        private void Poll()
        {
            lock (objLock)
            {
                gx = (float)_memory.getData("Device/SubDeviceList/InertialSensor/AngleX/Sensor/Value");
                gy = (float)_memory.getData("Device/SubDeviceList/InertialSensor/AngleY/Sensor/Value");
                gyrot = NaoPos.ConvertRotation(gx, gy, 0);
            }

            lock (objLock)
            {
                COM = VectorFromList(_motion.getCOM("Body", 0, false));
            }

            lock (objLock)
            {
                PollFoot(rightFoot, "R");
                PollFoot(leftFoot, "L");
                rightFoot.updateFoot(COM);
                leftFoot.updateFoot(COM);
            }

            lock (objLock)
            {
                foreach (String part in this.parts) positions[part] = PollPosition(part);
            }
        }

        // not thread safe - lock before calling this
        private void PollFoot(NaoFoot foot, string prefix)
        {
            foot.pfr = PollPosition(prefix + "FsrFR");
            foot.prr = PollPosition(prefix + "FsrRR");
            foot.pfl = PollPosition(prefix + "FsrFL");
            foot.prl = PollPosition(prefix + "FsrRL");
            foot.ffr = (float)_memory.getData("Device/SubDeviceList/" + prefix + "Foot/FSR/FrontRight/Sensor/Value");
            foot.frr = (float)_memory.getData("Device/SubDeviceList/" + prefix + "Foot/FSR/RearRight/Sensor/Value");
            foot.ffl = (float)_memory.getData("Device/SubDeviceList/" + prefix + "Foot/FSR/FrontLeft/Sensor/Value");
            foot.frl = (float)_memory.getData("Device/SubDeviceList/" + prefix + "Foot/FSR/RearLeft/Sensor/Value");
        }

        // not thread safe - lock before calling this
        private NaoPos PollPosition(string part)
        {
            return new NaoPos(_motion.getPosition(part, 0, false), Matrix.Identity);
        }

        // setter methods for motion

        public void Relax(String part)
        {
            lock (objLock)
            {
                _motion.setStiffnesses("Body", 0.0f);
            }
        }

        public void Stiffen(String part)
        {
            lock (objLock)
            {
                _motion.setStiffnesses(part, 1.0f);
            }
        }

        public void SetAngles(ArrayList joints, ArrayList values, float speed)
        {
            lock (objLock)
            {
                _motion.setAngles(joints, values, speed);
            }
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

        // getter methods for getting data

        public NaoPos GetPosition(string part)
        {
            lock (objLock)
            {
                return positions[part];
            }
        }

        public NaoFoot getLeftFoot()
        {
            NaoFoot copyLeft;
            lock (objLock) { copyLeft = new NaoFoot(leftFoot); }
            return copyLeft;
        }

        public NaoFoot getRightFoot()
        {
            NaoFoot copyRight;
            lock (objLock) { copyRight = new NaoFoot(rightFoot); }
            return copyRight;
        }

        public Vector3 GetCOM()
        {
            Vector3 comCopy = new Vector3();
            lock (objLock) 
            {
                comCopy.X = COM.X;
                comCopy.Y = COM.Y;
                comCopy.Z = COM.Z;
            }
            return comCopy;
        }

        public Matrix GetGyRot()
        {
            Matrix copy = new Matrix();
            lock (objLock)
            {
                copy.Up = gyrot.Up;
                copy.Left = gyrot.Left;
                copy.Forward = gyrot.Forward;
            }
            return copy;
        }

        public static Vector3 VectorFromList(List<float> fs)
        {
            return new Vector3(fs[0], fs[1], fs[2]);
        }
    }
}
