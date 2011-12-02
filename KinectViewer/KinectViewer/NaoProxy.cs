﻿using System;
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
        //private float grx, gry; // raw gyro
        private Vector3 COM;
        private Dictionary<string, float> angles = new Dictionary<string, float>();
        private Dictionary<string, float> masses = new Dictionary<string,float>();
        private Dictionary<string, Vector3> com = new Dictionary<string, Vector3>();
        private List<float> vel;

        private NaoFoot leftFoot;
        private NaoFoot rightFoot;

        public  MemoryProxy _memory = null;
        public  MotionProxy _motion = null;

        private Dictionary<String, NaoPos> positions = new Dictionary<string, NaoPos>();

        private ArrayList parts;
        private int msBetweenPolls;

        public int sleepTime { get; set; }

        //locks
        Object objLock = new Object();

        List<Action> listeners;

        //parts should include every part you'd be interested in asking about at any time in the future
        public NaoProxy(MemoryProxy memory, MotionProxy motion, ArrayList parts, int msBetweenPolls)
        {
            _memory = memory;
            _motion = motion;
            this.parts = parts;
            leftFoot = new NaoFoot("L");
            rightFoot = new NaoFoot("R");
            this.msBetweenPolls = msBetweenPolls;
            listeners = new List<Action>();
        }

        // method that runs in a loop updating the fields

        public void PollLoop()
        {
            while (true)
            {
                DateTime from = DateTime.Now;
                Poll();
                sleepTime = this.msBetweenPolls - DateTime.Now.Subtract(from).Milliseconds;
                if (sleepTime > 0) Thread.Sleep(sleepTime);
            }
        }

        public void InitialPoll()
        {
            foreach (String part in this.parts)
            {
                if (part != "Torso") angles.Add(part, _motion.getAngles(part, false)[0]);
                com.Add(part, NaoPos.Convert(VectorFromList(_motion.getCOM(part, 0, false))));
                positions.Add(part, PollPosition(part));
                masses.Add(part, _motion.getMass(part));
            }
        }

        private void Poll()
        {
            lock (objLock)
            {
                gx = (float)_memory.getData("Device/SubDeviceList/InertialSensor/AngleX/Sensor/Value");
                gy = (float)_memory.getData("Device/SubDeviceList/InertialSensor/AngleY/Sensor/Value");
                gyrot = NaoPos.ConvertRotation(gx, gy, 0);
 
                /* grx = (float)_memory.getData("Device/SubDeviceList/InertialSensor/GyrX/Sensor/Value");
                gry = (float)_memory.getData("Device/SubDeviceList/InertialSensor/GyrY/Sensor/Value");
                Console.WriteLine("grx: " + grx.ToString() + " gry: " + gry.ToString());
                */

                COM = VectorFromList(_motion.getCOM("Body", 0, false));
            
                PollFoot(rightFoot, "R");
                PollFoot(leftFoot, "L");
                rightFoot.updateFoot(COM);
                leftFoot.updateFoot(COM);

                foreach (String part in this.parts)
                {
                    if (part != "Torso") angles[part] = _motion.getAngles(part, false)[0];
                }
            
                foreach (String part in this.parts) com[part] = NaoPos.Convert(VectorFromList(_motion.getCOM(part, 0, false)));
            
                foreach (String part in this.parts) positions[part] = PollPosition(part);
           
                foreach (String part in this.parts) masses[part] = _motion.getMass(part);
            
                vel = _motion.getRobotVelocity();
            }

            foreach (Action a in this.listeners)
                a.Invoke();
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
            return new NaoPos(_motion.getPosition(part, 0, false));
        }


        public float GetAngles(string part) 
        {
            return angles[part];
        
        }

        public float GetMass(string part)
        {
            return masses[part];
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

        public Vector3 GetCOM(string part)
        {
            return com[part];
        }

        public NaoFoot GetLeftFoot()
        {
            NaoFoot copyLeft;
            lock (objLock) { copyLeft = new NaoFoot(leftFoot); }
            return copyLeft;
        }

        public NaoFoot GetRightFoot()
        {
            NaoFoot copyRight;
            lock (objLock) { copyRight = new NaoFoot(rightFoot); }
            return copyRight;
        }

        //returns the overal com for the body
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

        public Vector3 GetVelocity()
        {
            Vector3 result;
            lock (objLock)
            {
                result.X = vel[0];
                result.Y = vel[1];
                result.Z = vel[2];
            }
            return result;
        }

        public static Vector3 VectorFromList(List<float> fs)
        {
            return new Vector3(fs[0], fs[1], fs[2]);
        }

        public float GetData(string data)
        {
            lock (objLock)
            {
                return (float)_memory.getData("Device/SubDeviceList/" + data + "/Position/Sensor/Value");
            }
        }

        public Vector3 GetPos(string part)
        {
            lock (objLock)
            {
                return VectorFromList(_motion.getPosition(part, 0, false));
            }
        }

        public void positionInterpolation(string effector, int space, object path, int axisMask, object time, bool isAblosute)
        {
            lock (objLock)
            {
                _motion.positionInterpolation(effector, space, path, axisMask, time, isAblosute);
            }
        }
    }
}
