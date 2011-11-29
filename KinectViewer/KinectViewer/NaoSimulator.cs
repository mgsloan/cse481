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
            RShoulderP = new JointNode("RShoulderPitch", Vector3.Right);
            RShoulderR = new JointNode("RShoulderRoll", Vector3.Forward);
            LShoulderR = new JointNode("LShoulderRoll", Vector3.Forward);
            LElbowY = new JointNode("LElbowRoll", Vector3.Up);
            RElbowY = new JointNode("RElbowRoll", Vector3.Up);
            LElbowR = new JointNode("LElbowYaw", Vector3.Up);
            RElbowR = new JointNode("RElbowYaw", Vector3.Up);
            LWristY = new JointNode("LWristYaw", Vector3.Up);
            RWristY = new JointNode("RWristYaw", Vector3.Up);
            LHipPitch = new JointNode("LHipPitch", Vector3.Up);
            RHipPitch = new JointNode("RHipPitch", Vector3.Up);
            LHipRoll = new JointNode("LHipRoll", Vector3.Up);
            RHipRoll = new JointNode("RHipRoll", Vector3.Up);
            LKneePitch = new JointNode("LKneePitch", Vector3.Up);
            RKneePitch = new JointNode("RKneePitch", Vector3.Up);
            LAnklePitch = new JointNode("LAnklePitch", Vector3.Up);
            RAnklePitch = new JointNode("RAnklePitch", Vector3.Up);
            LAnkleRoll = new JointNode("LAnkleRoll", Vector3.Up);
            RAnkleRoll = new JointNode("RAnkleRoll", Vector3.Up);
            Torso = new JointNode("Torso", Vector3.Up);
            HeadYaw = new JointNode("HeadYaw", Vector3.Up);
            HeadPitch = new JointNode("HeadPitch", Vector3.Up);


            
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
            ComputeLocalMatrix(Robot);
                      
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
            foreach (JointNode chain in chains) {
                JointNode cur = chain.next;
                while (cur != null) {
                    cur.localPosition = new NaoPos(proxy.getPosition(cur.name, 0, false)).transform;
                    cur.torsoSpacePosition = cur.localPosition;
                    cur.mass = proxy.getMass(cur.name);
                    if (cur.name != "Torso") cur.initialAngle = proxy.getAngles(cur.name, false)[0];
                    cur.com = VectorFromList(proxy.getCOM(cur.name, 0, false));
                    cur = cur.next;
                }
      
            }
        }   
        
        private void ComputeLocalMatrix(LinkedList<JointNode> chains) 
        {
            Matrix prev = Matrix.Identity;
            foreach (JointNode chain in chains)
            {
                JointNode cur = chain.next;
                while (cur != null)
                {
                    var temp = cur.localPosition;
                    cur.localPosition = Matrix.Multiply(cur.torsoSpacePosition, Matrix.Invert(prev));
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
            foreach (JointNode chain in Robot) {
                JointNode cur = chain.next;
                while (cur != null) {
                    cur.torsoSpacePosition = !angles.ContainsKey(cur.name) ? prev :
                        Matrix.Multiply(Matrix.Multiply(prev, cur.localPosition), Matrix.CreateFromAxisAngle(cur.orientation, cur.initialAngle - angles[cur.name]));
                    prev = cur.torsoSpacePosition;
                    cur = cur.next;
                }
                prev = Matrix.Identity;
            }
            
            
            
            
            
            //rightarm



        ////    //length of arm.
        //    Vector3 upperArm_Length = new Vector3(0f, .09f, 0f);
           
           

        //    //Rotation Matrix
        //    Matrix m_rshoulderpitch = Matrix.CreateRotationX(angles["RShoulderPitch"]);
        //    Matrix m_rshoulderroll = Matrix.CreateRotationZ(angles["RShoulderRoll"]);

        //    //multiply the rotation matrixes together
        //    Matrix rshoulder_rotation = Matrix.Multiply(m_rshoulderpitch, m_rshoulderroll);

        //    //get the current orientation of the shoulder
        //    Vector3 rshoulder_ref = Vector3.Transform(RShoulderR.position, rshoulder_rotation);

        //    //translate the shoulder by the upper arm length
        //    Matrix elbow_translation = Matrix.CreateTranslation(upperArm_Length);
            
        //    //position of the elbow
        //    Vector3 relbow = Vector3.Transform(rshoulder_ref, elbow_translation);


        //    Console.WriteLine("calculated elbow position: " + relbow);
        //    RElbowR.position = relbow;
        //    RElbowY.position = relbow;

        //    Vector3 lowerArm_Length = new Vector3(0f, .05055f, 0f);
        //    Matrix rwrist_translation = Matrix.CreateTranslation(lowerArm_Length);
            
        //    Matrix m_relbowroll = Matrix.CreateRotationX(angles["RElbowRoll"]);
        //    Matrix m_relbowyaw = Matrix.CreateRotationZ(angles["RElbowYaw"]);
           
        //    Matrix elbow_rotation = Matrix.Multiply(m_relbowroll, m_relbowyaw);
        //    Vector3 relbow_ref = Vector3.Transform(relbow, m_relbowroll);
        //    Vector3 rwrist = Vector3.Transform(relbow_ref, rwrist_translation);
        //    Console.WriteLine("calculated wrist position: " + rwrist);
        //    RWristY.position = rwrist;

        //    //leftarm 

        //    //Vector3 upperArm_Length = new Vector3(0f, .09f, 0f);
        //    //Matrix elbow_translation = Matrix.CreateTranslation(upperArm_Length);
        //    //Vector3 base_transformation = LShoulderR.position;

        //    ////Console.WriteLine("initial shoulder position: " + shoulder_position);
        //    ////Console.WriteLine("initial elbow position: " + LElbowR.position);
        //    //////Vector3 upperArm_Length = Vector3.Subtract(LElbowR.position, LShoulderP.position);
        //    //Matrix m_lshoulderpitch = Matrix.CreateRotationX(angles["LShoulderPitch"]);
        //    //Matrix m_lshoulderroll = Matrix.CreateRotationZ(angles["LShoulderRoll"]);
            
        //    //Matrix shoulder_rotation = Matrix.Multiply(m_lshoulderpitch, m_lshoulderroll);
        //    //Vector3 shoulder_ref = Vector3.Transform(base_transformation, shoulder_rotation);
        //    //Vector3 elbow = Vector3.Transform(shoulder_ref, elbow_translation);

         
        //    //Console.WriteLine("calculated position elbow: " + elbow);
        //    //LElbowR.position = elbow;
        //    //LElbowY.position = elbow;

        //    //Vector3 lowerArm_Length = new Vector3(0f, 0f, .05055f);
        //    //Matrix wrist_translation = Matrix.CreateTranslation(lowerArm_Length);
        //    //Matrix m_lelbowroll = Matrix.CreateRotationX(-angles["LElbowRoll"]);
        //    //Console.WriteLine("elbow roll: " + (-angles["LElbowRoll"]));
        //    ////Matrix m_lelbowyaw = Matrix.CreateRotationZ(angles["LElbowYaw"] + (float) Math.PI / 2);
        //    ////Console.WriteLine("elbow yaw: " + (angles["LElbowYaw"] +  (float) Math.PI / 2));

        //    ////Matrix elbow_rotation = Matrix.Multiply(m_lelbowroll, m_lelbowyaw);
        //    //Vector3 elbow_ref = Vector3.Transform(elbow, m_lelbowroll);
        //    //Vector3 wrist = Vector3.Transform(elbow_ref, wrist_translation);
        //    //Console.WriteLine("calculated position wrist: " + wrist);
        //    //LWristY.position = wrist;

        //    //right leg 
        //    //Vector3 UL = new Vector3(0f, 0f, .1f);
        //    //Vector3 LL = new Vector3(0f, 0f, .102f);
        //    //Vector3 rhip_reference =RHipPitch.position;
        //    //Matrix knee_translation = Matrix.CreateTranslation(UL);

        //    //Matrix m_hippitch = Matrix.CreateRotationX(angles["RHipPitch"]);
        //    //Matrix m_hiproll = Matrix.CreateRotationZ(angles["RHipRoll"]);
          
        //    //Matrix rhip_rotation = Matrix.Multiply(m_hippitch, m_hiproll);
        //    //Vector3 cur_hip_ref = Vector3.Transform(rhip_reference, rhip_rotation);
            
        //    //Vector3 UL_tx = Vector3.Transform(cur_hip_ref, knee_translation);
        //    //RKneePitch.position = UL_tx;
                
        //    //Matrix m_kneepitch = Matrix.CreateRotationX(angles["RKneePitch"]);
        //    //// notice here that the orientation matrix from before is now augmented with the rotation for the next
        //    //// link in the chain
        //    //Vector3 cur_knee_ref = Vector3.Transform(UL_tx, m_hippitch);
        //    //Matrix ankle_translation = Matrix.CreateTranslation(LL);
        //    //Vector3 LL_tx = Vector3.Transform(cur_knee_ref, ankle_translation);
        //    // you can add the re-oriented limb vectors at any point to get the position of a particular link in the chain
        //    // here the upper and lower leg vectors are added together to get the position of the foot (i'm pretty sure that in
        //    // general you can work out the orientation of each limb vector without using the position information, as was 
        //    // done here)
          
        //    //RAnklePitch.position = LL_tx;
           
                  
        }




        //returns the current COM based of the current positions of the parts and their masses
        //public Vector3 getCOM()
        //{
        //    //multiply each mass w/ position and divde by total mass
        //    float totalMass = 0;
        //    Vector3 numerator = new Vector3();
        //    foreach (parts part in positions.Keys)
        //    {
        //        float mass = (float)masses[part];
        //        totalMass += mass;
        //        NaoPos position = positions[part];
        //        numerator += Vector3.Multiply(position.position, mass); 
                

        //    }

        //    Vector3 com = Vector3.Multiply(numerator, (float) 1/ totalMass);
            
        //    Console.WriteLine(com);
        //    return com;
        
        //}

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
