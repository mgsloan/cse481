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
        

        /*setting up the joint nodes for the robot. */
        JointNode LShoulderP, RShoulderP, RShoulderR, LShoulderR, LElbowY, RElbowY, LElbowR, RElbowR, LWristY,
                RWristY, LHipPitch, RHipPitch, LHipRoll, RHipRoll, LKneePitch, RKneePitch, LAnklePitch, RAnklePitch,
                LAnkleRoll, RAnkleRoll, Torso, HeadYaw, HeadPitch;

        NaoFoot rightF, leftF;
        Vector3[] rightFLocal;
        Vector3[] leftFLocal;



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




    
    }
}
