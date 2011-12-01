using System;
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
        JointNode LShoulderP; 
        JointNode RShoulderP;
        JointNode RShoulderR;
        JointNode LShoulderR;
        JointNode LElbowY;
        JointNode RElbowY;
        JointNode LElbowR;
        JointNode RElbowR;
        JointNode LWristY;
        JointNode RWristY;
        JointNode LHipPitch;
        JointNode RHipPitch;
        JointNode LHipRoll;
        JointNode RHipRoll;
        JointNode LKneePitch;
        JointNode RKneePitch;
        JointNode LAnklePitch;
        JointNode RAnklePitch;
        JointNode LAnkleRoll;
        JointNode RAnkleRoll;
        JointNode Torso;
        JointNode HeadYaw;
        JointNode HeadPitch;


        LinkedList<JointNode> Robot;
        /*this constructs a NAO object. Will initial the NAO parts 
        /to their corresponding position vector. 
        */
        public NaoSimulator(string ip)
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


            
            JointNode LeftArm = CreateChain(new JointNode[] { LShoulderP, LShoulderR, LElbowY, LElbowR, LWristY });
            JointNode RightArm = CreateChain(new JointNode[] { RShoulderP, RShoulderR, RElbowY, RElbowR, RWristY });
            JointNode LeftLeg = CreateChain(new JointNode[] { LHipRoll, LHipPitch,  LKneePitch, LAnklePitch, LAnkleRoll });
            JointNode RightLeg = CreateChain(new JointNode[] { RHipRoll, RHipPitch, RKneePitch, RAnklePitch, RAnkleRoll });
            JointNode Head =    CreateChain(new JointNode[] { HeadYaw, HeadPitch });
            JointNode Body = CreateChain(new JointNode[] { Torso });
            MotionProxy proxy = new MotionProxy(ip, 9559);


            Robot = new LinkedList<JointNode>(new JointNode[] { LeftArm, RightArm, LeftLeg, RightLeg, Head, Body });

           
            //initialize the positions of the robot.
            InitializePositions(Robot, proxy);

                      
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

        

        private void InitializePositions(LinkedList<JointNode> chains, MotionProxy proxy)
        {
            Matrix prev = Matrix.Identity;
            foreach (JointNode chain in chains) {
                JointNode cur = chain.next;
                while (cur != null) {
                    cur.localPosition = new NaoPos(proxy.getPosition(cur.name, 0, false)).transform;
                    cur.torsoSpacePosition = cur.localPosition;
                    var temp = cur.localPosition;
                    cur.localPosition = Matrix.Multiply(cur.torsoSpacePosition, Matrix.Invert(prev));
                    
                    cur.mass = proxy.getMass(cur.name);
                    if (cur.name != "Torso") cur.initialAngle = proxy.getAngles(cur.name, false)[0];
                    Vector3 torsoCom = NaoPos.Convert(VectorFromList(proxy.getCOM(cur.name, 0, false)));
                    cur.com = Vector3.Transform(torsoCom, Matrix.Invert(cur.torsoSpacePosition));
                    Console.WriteLine(torsoCom.ToString() + ", transformed: " + cur.com.ToString()); 
                    cur = cur.next;
                    prev = temp;
                }
                prev = Matrix.Identity;
      
            }
        }   
       

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
                        if (cur.name == "RShoulderPitch" || cur.name == "RShoulderRoll")
                            Console.WriteLine(cur.name + angles[cur.name]);
                        cur.torsoSpacePosition = Matrix.Multiply(cur.torsoSpacePosition, Matrix.CreateFromAxisAngle(cur.orientation, cur.initialAngle - angles[cur.name]));
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
            
            

        //returns the current COM based off the current positions of the parts and their masses
        public Vector3 getCOM()
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




    
    }
}
