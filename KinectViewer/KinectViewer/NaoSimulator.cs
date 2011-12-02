﻿using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using Microsoft.Xna.Framework;
using Aldebaran.Proxies;
using System.Collections;

namespace KinectViewer
{
    class NaoSimulator
    {
        private Dictionary<string, JointNode> jointToNode;

        /*setting up the joint nodes for the robot. */
        JointNode LShoulderP, RShoulderP, RShoulderR, LShoulderR, LElbowY, RElbowY, LElbowR, RElbowR, LWristY,
                RWristY, LHipPitch, RHipPitch, LHipRoll, RHipRoll, LKneePitch, RKneePitch, LAnklePitch, RAnklePitch,
                LAnkleRoll, RAnkleRoll, Torso, HeadYaw, HeadPitch;

        NaoFoot rightF, leftF;
        Vector3[] rightFLocal;
        Vector3[] leftFLocal;

        double UL_len, LL_len; //upper/lower leg length

        LinkedList<JointNode> Robot;
        /*this constructs a NAO object. Will initial the NAO parts 
        /to their corresponding position vector. 
        */
        public NaoSimulator(string ip, NaoBody nao)
            /********create the kinematic chains*****/
        {
            LShoulderP = new JointNode("LShoulderPitch", Vector3.Left);
            RShoulderP = new JointNode("RShoulderPitch", Vector3.Left);
            RShoulderR = new JointNode("RShoulderRoll", Vector3.Forward);
            LShoulderR = new JointNode("LShoulderRoll", Vector3.Forward);
            LElbowY = new JointNode("LElbowRoll", Vector3.Left);
            RElbowY = new JointNode("RElbowRoll", Vector3.Right);
            LElbowR = new JointNode("LElbowYaw", Vector3.Up);
            RElbowR = new JointNode("RElbowYaw", Vector3.Up);
            LWristY = new JointNode("LWristYaw", Vector3.Up);
            RWristY = new JointNode("RWristYaw", Vector3.Up);
            LHipPitch = new JointNode("LHipPitch", Vector3.Left);
            RHipPitch = new JointNode("RHipPitch", Vector3.Left);
            LHipRoll = new JointNode("LHipRoll", Vector3.Backward);
            RHipRoll = new JointNode("RHipRoll", Vector3.Forward);
            LKneePitch = new JointNode("LKneePitch", Vector3.Left);
            RKneePitch = new JointNode("RKneePitch", Vector3.Left);
            LAnklePitch = new JointNode("LAnklePitch", Vector3.Up);
            RAnklePitch = new JointNode("RAnklePitch", Vector3.Up);
            LAnkleRoll = new JointNode("LAnkleRoll", Vector3.Up);
            RAnkleRoll = new JointNode("RAnkleRoll", Vector3.Up);
            Torso = new JointNode("Torso", Vector3.Up);
            HeadYaw = new JointNode("HeadYaw", Vector3.Up);
            HeadPitch = new JointNode("HeadPitch", Vector3.Up);

            jointToNode = new Dictionary<string, JointNode>();

            jointToNode.Add("RShoulderPitch", RShoulderP);
            jointToNode.Add("RShoulderRoll", RShoulderR);
            jointToNode.Add("RElbowRoll", RElbowR);
            jointToNode.Add("RElbowYaw", RElbowY);
            jointToNode.Add("LShoulderPitch", LShoulderP);
            jointToNode.Add("LShoulderRoll", LShoulderR);
            jointToNode.Add("LElbowRoll", LElbowR);
            jointToNode.Add("LElbowYaw", LElbowY);
            jointToNode.Add("RHipRoll", RHipRoll);
            jointToNode.Add("RHipPitch", RHipPitch);
            jointToNode.Add("RKneePitch", RKneePitch);
            jointToNode.Add("RAnklePitch", RAnklePitch);
            jointToNode.Add("RAnkleRoll", RAnkleRoll);
            jointToNode.Add("LHipRoll", LHipRoll);
            jointToNode.Add("LHipPitch", LHipPitch);
            jointToNode.Add("LKneePitch", LKneePitch);
            jointToNode.Add("LAnklePitch", LAnklePitch);
            jointToNode.Add("LAnkleRoll", LAnkleRoll);
            jointToNode.Add("LWristYaw", LWristY);
            jointToNode.Add("RWristYaw", RWristY);
            jointToNode.Add("Torso", Torso);
            jointToNode.Add("HeadYaw", HeadYaw);
            jointToNode.Add("HeadPitch", HeadPitch);

            leftF = new NaoFoot("L");
            rightF = new NaoFoot("R");
            
            JointNode LeftArm = CreateChain(new JointNode[] { LShoulderP, LShoulderR, LElbowY, LElbowR, LWristY });
            JointNode RightArm = CreateChain(new JointNode[] { RShoulderP, RShoulderR, RElbowY, RElbowR, RWristY });
            JointNode LeftLeg = CreateChain(new JointNode[] { LHipRoll, LHipPitch,  LKneePitch, LAnklePitch, LAnkleRoll });
            JointNode RightLeg = CreateChain(new JointNode[] { RHipRoll, RHipPitch, RKneePitch, RAnklePitch, RAnkleRoll });
            JointNode Head =    CreateChain(new JointNode[] { HeadYaw, HeadPitch });
            JointNode Body = CreateChain(new JointNode[] { Torso });
            //MotionProxy proxy = new MotionProxy(ip, 9559);


            Robot = new LinkedList<JointNode>(new JointNode[] { LeftArm, RightArm, LeftLeg, RightLeg, Head, Body });

           
            //initialize the positions of the robot.
            InitializePositions(Robot, nao);

                      
        }

        private JointNode CreateChain(JointNode[] chain) { 
            JointNode head = new JointNode();
            var chainlst = chain.ToList();
            for (int i = 0; i < chain.Length - 1; i++) 
            {
                chainlst[i].next = chainlst[i + 1]; 
            }
            head.next = chainlst[0];
            return head;
        }

        

        private void InitializePositions(LinkedList<JointNode> chains, NaoBody nao)
        {
            Matrix prev = Matrix.Identity;
            foreach (JointNode chain in chains) {
                JointNode cur = chain.next;
                while (cur != null) {
                    cur.localPosition = nao.GetPosition(cur.name).transform;
                    cur.torsoSpacePosition = cur.localPosition;
                    var temp = cur.localPosition;
                    cur.localPosition = Matrix.Multiply(cur.torsoSpacePosition, Matrix.Invert(prev));
                    
                    Matrix toLocal = Matrix.Invert(cur.torsoSpacePosition);

                    cur.mass = nao.GetMass(cur.name);
                    if (cur.name != "Torso") cur.initialAngle = nao.GetAngles(cur.name);
                    Vector3 torsoCom = nao.GetCOM(cur.name);
                    cur.com = Vector3.Transform(torsoCom, toLocal);
                    //Console.WriteLine(torsoCom.ToString() + ", transformed: " + cur.com.ToString()); 
                    
                    //set the foot sensors in ankleroll reference frame
                    if (cur.name == "RAnkleRoll")
                    {
                        //set the rankleroll angles 
                        var rightFoot = nao.GetRightFoot();
                        rightFLocal = new Vector3[4];
                        rightFLocal[0] = Vector3.Transform(rightFoot.pfl.position, toLocal);
                        rightFLocal[1] = Vector3.Transform(rightFoot.pfr.position, toLocal);
                        rightFLocal[2] = Vector3.Transform(rightFoot.prl.position, toLocal);
                        rightFLocal[3] = Vector3.Transform(rightFoot.prr.position, toLocal);
                    }
                    else if (cur.name == "LAnkleRoll")
                    {
                        var leftFoot = nao.GetLeftFoot();
                        leftFLocal = new Vector3[4];
                        leftFLocal[0] = Vector3.Transform(leftFoot.pfl.position, toLocal);
                        leftFLocal[1] = Vector3.Transform(leftFoot.pfr.position, toLocal);
                        leftFLocal[2] = Vector3.Transform(leftFoot.prl.position, toLocal);
                        leftFLocal[3] = Vector3.Transform(leftFoot.prr.position, toLocal);
                    }

                        

                    cur = cur.next;
                    prev = temp;
                }
                prev = Matrix.Identity;
      
            }

            UL_len = (getPosition("RKneePitch") - getPosition("RHipPitch")).Length();
            LL_len = (getPosition("RAnklePitch") - getPosition("RKneePitch")).Length();
        }   
       
        //TODO: limits?
        public void UpdatePositions(Dictionary<string, float> angles)
        {
            //iterate through the chains 
            Matrix prev = Matrix.Identity;
            foreach (JointNode chain in Robot)
            {
                JointNode cur = chain.next;
                while (cur != null)
                {

                    cur.torsoSpacePosition = Matrix.Multiply(cur.localPosition, prev);

                    Vector3 trans = cur.torsoSpacePosition.Translation;
                    cur.torsoSpacePosition.Translation = Vector3.Zero;
                    if (angles.ContainsKey(cur.name))
                    {
                        float angle = angles[cur.name];
                        if (cur.name == "RShoulderPitch" || cur.name == "RShoulderRoll")
                            Console.WriteLine(cur.name + angle);
                        cur.torsoSpacePosition = Matrix.Multiply(cur.torsoSpacePosition, cur.MakeRotation(angle));
                    }
                    cur.torsoSpacePosition.Translation = trans;

                    prev = cur.torsoSpacePosition;
                    cur = cur.next;
                }
                prev = Matrix.Identity;
            }
        }

        public Vector3 getPosition(string part)
        {
            return jointToNode[part].torsoSpacePosition.Translation;
        }

        // Gets axis angle, given an axis of rotation
        // UGLY
        public float GetAxisAngle(Vector3 v, Vector3 axis)
        {
            if (axis.X ==  1f) return (float)Math.Atan2(axis.Y, axis.Z);
            if (axis.X == -1f) return (float)Math.Atan2(axis.Z, axis.Y);
            if (axis.Y ==  1f) return (float)Math.Atan2(axis.X, axis.Z);
            if (axis.Y == -1f) return (float)Math.Atan2(axis.Z, axis.X);
            if (axis.Z ==  1f) return (float)Math.Atan2(axis.X, axis.Y);
            if (axis.Z == -1f) return (float)Math.Atan2(axis.Y, axis.X);
            throw new Exception("Cannot get angle for non axial rotation.");
        }

        public Tuple<float, float> GetAngleRequired(string a, string b, Vector3 vec)
        {
            JointNode ja = jointToNode[a], jb = jointToNode[b];
            Vector3 local = Vector3.Transform(vec, Matrix.Invert(ja.torsoSpacePosition));
            float angle = GetAxisAngle(local, ja.orientation);
            Matrix trans = Matrix.Multiply(Matrix.Multiply(ja.torsoSpacePosition, ja.MakeRotation(angle)), jb.localPosition);
            Vector3 local2 = Vector3.Transform(vec, Matrix.Invert(trans));
            return new Tuple<float, float>(angle, GetAxisAngle(local2, jb.orientation));
        }

        public NaoFoot GetRightFoot()
        {
            var ankleRef = RAnkleRoll.torsoSpacePosition;
            rightF.pfl.position = Vector3.Transform(rightFLocal[0], ankleRef);
            rightF.pfr.position = Vector3.Transform(rightFLocal[1], ankleRef);
            rightF.prl.position = Vector3.Transform(rightFLocal[2], ankleRef);
            rightF.prr.position = Vector3.Transform(rightFLocal[3], ankleRef);

            return rightF;
        }

        public NaoFoot GetLeftFoot()
        {
            var ankleRef = LAnkleRoll.torsoSpacePosition;
            leftF.pfl.position = Vector3.Transform(leftFLocal[0], ankleRef);
            leftF.pfr.position = Vector3.Transform(leftFLocal[1], ankleRef);
            leftF.prl.position = Vector3.Transform(leftFLocal[2], ankleRef);
            leftF.prr.position = Vector3.Transform(leftFLocal[3], ankleRef);

            return leftF;
        }

        //returns the current COM based off the current positions of the parts and their masses
        public Vector3 GetCOM()
        {
            //multiply each mass w/ position and divde by total mass
            double totalMass = 0;
            Vector3 numerator = Vector3.Zero;
            
            foreach (JointNode chain in Robot)
            {
                JointNode cur = chain.next;
                while (cur != null)
                {
                    double mass = cur.mass;
                    totalMass += mass;
                    Vector3 position = Vector3.Transform(cur.com, cur.torsoSpacePosition);
                    numerator += Vector3.Multiply(position, (float) mass);
                    cur = cur.next;
                }   

            }

            Vector3 com = Vector3.Multiply(numerator, (float) 1/ (float) totalMass);
            
            Console.WriteLine(com);
            return com;
        
        }

        public static Vector3 VectorFromList(List<float> fs)
        {
            return new Vector3(fs[0], fs[1], fs[2]);
        }



        public LinkedList<JointNode> getRobot()
        {
            return Robot;
        }   


        public Vector3 getFootTarget(Matrix BodyTxform)
        {
            Vector3 COM = GetCOM();

            Vector3 RFoot = getPosition("RAnklePitch");

            //use COM as origin 
            Vector3 RFoot_tr = RFoot - COM;

            //tx to world space
            Vector3 RFoot_tx = Vector3.Transform(RFoot_tr, Matrix.Invert(BodyTxform));

            //project onto y=foot_tx.Y plane in world space
            Vector3 RTarget_tx = new Vector3(0, RFoot_tx.Y * .9f, 0);

            //transform target point from world to torso space
            Vector3 RTarget_torso = Vector3.Transform(RTarget_tx, BodyTxform);

            //reset the origin
            RTarget_torso += COM;

            return RTarget_torso;     
        }

        //limb lengths: check
        public double[] readjustLegs(Matrix BodyTxform)
        {
            Vector3 RTarget_torso = getFootTarget(BodyTxform);

            double[] newRAngles = LegIK(Matrix.Identity, getPosition("RHipPitch"), RTarget_torso, UL_len, LL_len);

            Console.WriteLine("hr: " + newRAngles[1] + ", hp: " + newRAngles[0] + ", kp: " + newRAngles[2]); 
        
            return newRAngles;
        }

        private double[] LegIK(Matrix BodyTxform, Vector3 hip, Vector3 foot, double UL_len, double LL_len)
        {
            // (1) find hip roll 
            // from 0 (X-axis) to -pi (negative X-axis) where X-axis is model's right-to-left vector
            // get hip to foot vector in world space
            Vector3 hip_to_foot = foot - hip;
            // txform to torso space
            Vector3 hip_to_foot_tx = Vector3.Transform(hip_to_foot, BodyTxform);

            // project onto XY plane in torso space
            double hiproll = -Math.Atan2(hip_to_foot_tx.Y, hip_to_foot_tx.X) - Math.PI;

            // now do other two angles, will need distance from hip to foot for this
            float hip_to_foot_len;
            Vector3.Distance(ref hip, ref foot, out hip_to_foot_len);

            // (2) find knee pitch (easy, use law of cosines)
            double a2 = UL_len;
            double b2 = LL_len;
            double c2 = hip_to_foot_len;
            double kneepitch;
            if (c2 > a2 + b2) kneepitch = Math.PI;
            else kneepitch = Math.Acos((a2 * a2 + b2 * b2 - c2 * c2) / (2 * a2 * b2));

            // (3) find hip pitch (there are two parts to this) 
            // from 0 (Z-axis) to -pi (negative Z-axis) where Z-axis is vector out of model's torso

            // part 1: use law of cosines
            double a1 = hip_to_foot_len;
            double b1 = UL_len;
            double c1 = LL_len;
            double p2 = Math.Acos((a1 * a1 + b1 * b1 - c1 * c1) / (2 * a1 * b1));

            // part 2: rotate hip-to-foot vector into YZ plane, and then project it onto YZ plane in torso space
            // rotate
            float theta = (float)(-hiproll - Math.PI / 2);
            Matrix YZfix = Matrix.CreateRotationZ(-theta);
            var hip_to_foot_yz = Vector3.Transform(hip_to_foot_tx, YZfix);

            // project
            double p1 = Math.Atan2(hip_to_foot_yz.Y, hip_to_foot_yz.Z);

            // now combine part 1 and 2
            double hippitch = p2 + p1;

            // return three angles in a double[]
            double[] angles = new double[3];
            angles[0] = hippitch; angles[1] = hiproll; angles[2] = kneepitch;

            return angles;
        }
    }
}