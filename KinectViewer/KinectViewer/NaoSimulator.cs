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
        JointNode LShoulderP = new JointNode("LShoulderPitch", 69.96);
        JointNode RShoulderP = new JointNode("RShoulderPitch", 69.96);
        JointNode RShoulderR = new JointNode("RShoulderRoll", 123.09);
        JointNode LShoulderR = new JointNode("LShoulderRoll", 123.09);
        JointNode LElbowY = new JointNode("LElbowRoll", 59.71);
        JointNode RElbowY = new JointNode("RElbowRoll", 59.71);
        JointNode LElbowR = new JointNode("LElbowYaw", 77.24);
        JointNode RElbowR = new JointNode("RElbowYaw", 77.24);
        JointNode LWristY = new JointNode("LWristYaw", 166.53);
        JointNode RWristY = new JointNode("RWristYaw", 166.53);
        JointNode LHipPitch = new JointNode("LHipPitch", 394.21);
        JointNode RHipPitch = new JointNode("RHipPitch", 394.21);
        JointNode LHipRoll = new JointNode("LHipRoll", 206.47);
        JointNode RHipRoll = new JointNode("RHipRoll", 206.47);
        JointNode LKneePitch = new JointNode("LKneePitch", 291.59);
        JointNode RKneePitch = new JointNode("RKneePitch", 291.59);
        JointNode LAnklePitch = new JointNode("LAnklePitch", 138.92);
        JointNode RAnklePitch = new JointNode("RAnklePitch", 138.92);
        JointNode LAnkleRoll = new JointNode("LAnkleRoll", 161.75);
        JointNode RAnkleRoll = new JointNode("RAnkleRoll", 161.75);
        JointNode Torso = new JointNode("Torso", 1039.48);
        JointNode HeadYaw = new JointNode("HeadYaw", 59.30);
        JointNode HeadPitch = new JointNode("HeadPitch", 520.65);


        LinkedList<JointNode> Robot;
        /*this constructs a NAO object. Will initial the NAO parts 
        /to their corresponding position vector. 
        */
        public NaoSimulator(string ip)
            /********create the kinematic chains*****/
        {
            MotionProxy proxy = new MotionProxy(ip, 9559);


            Robot = new LinkedList<JointNode>(new JointNode[] { LShoulderP,LShoulderR,RShoulderP,RShoulderR,LElbowR,LElbowY,RElbowR, RElbowY, LWristY, RWristY,
                                                                LHipPitch,RHipPitch,LHipRoll,RHipRoll,LKneePitch,RKneePitch,LAnklePitch,RAnklePitch, LAnkleRoll,RAnkleRoll,
                                                                Torso, HeadYaw, HeadPitch });

           
            //initialize the positions of the robot.
            InitializePositions(Robot, proxy);
            
            

                      
        }

        

        private void InitializePositions(LinkedList<JointNode> joints, MotionProxy proxy)
        {
            var jointslst = joints.ToList();
            for (int j = 0; j < joints.Count; j++)
            {
                jointslst[j].position = new NaoPos(proxy.getPosition(jointslst[j].name, 0, false), Matrix.Identity).position;
                
            }
        }

        public void UpdatePositions(Dictionary<string, float> angles)
        {
            //rightarm

            //length of arm.
            Vector3 upperArm_Length = new Vector3(0f, .09f, 0f);
           
           

            //Rotation Matrix
            Matrix m_rshoulderpitch = Matrix.CreateRotationX(angles["RShoulderPitch"]);
            Matrix m_rshoulderroll = Matrix.CreateRotationZ(angles["RShoulderRoll"]);

            //multiply the rotation matrixes together
            Matrix rshoulder_rotation = Matrix.Multiply(m_rshoulderpitch, m_rshoulderroll);

            //get the current orientation of the shoulder
            Vector3 rshoulder_ref = Vector3.Transform(RShoulderR.position, rshoulder_rotation);

            //translate the shoulder by the upper arm length
            Matrix elbow_translation = Matrix.CreateTranslation(upperArm_Length);
            
            //position of the elbow
            Vector3 relbow = Vector3.Transform(rshoulder_ref, elbow_translation);


            Console.WriteLine("calculated elbow position: " + relbow);
            RElbowR.position = relbow;
            RElbowY.position = relbow;

            Vector3 lowerArm_Length = new Vector3(0f, .05055f, 0f);
            Matrix rwrist_translation = Matrix.CreateTranslation(lowerArm_Length);
            
            Matrix m_relbowroll = Matrix.CreateRotationX(angles["RElbowRoll"]);
            Matrix m_relbowyaw = Matrix.CreateRotationZ(angles["RElbowYaw"]);
           
            Matrix elbow_rotation = Matrix.Multiply(m_relbowroll, m_relbowyaw);
            Vector3 relbow_ref = Vector3.Transform(relbow, m_relbowroll);
            Vector3 rwrist = Vector3.Transform(relbow_ref, rwrist_translation);
            Console.WriteLine("calculated wrist position: " + rwrist);
            RWristY.position = rwrist;

            //leftarm 

            //Vector3 upperArm_Length = new Vector3(0f, .09f, 0f);
            //Matrix elbow_translation = Matrix.CreateTranslation(upperArm_Length);
            //Vector3 base_transformation = LShoulderR.position;

            ////Console.WriteLine("initial shoulder position: " + shoulder_position);
            ////Console.WriteLine("initial elbow position: " + LElbowR.position);
            //////Vector3 upperArm_Length = Vector3.Subtract(LElbowR.position, LShoulderP.position);
            //Matrix m_lshoulderpitch = Matrix.CreateRotationX(angles["LShoulderPitch"]);
            //Matrix m_lshoulderroll = Matrix.CreateRotationZ(angles["LShoulderRoll"]);
            
            //Matrix shoulder_rotation = Matrix.Multiply(m_lshoulderpitch, m_lshoulderroll);
            //Vector3 shoulder_ref = Vector3.Transform(base_transformation, shoulder_rotation);
            //Vector3 elbow = Vector3.Transform(shoulder_ref, elbow_translation);

         
            //Console.WriteLine("calculated position elbow: " + elbow);
            //LElbowR.position = elbow;
            //LElbowY.position = elbow;

            //Vector3 lowerArm_Length = new Vector3(0f, 0f, .05055f);
            //Matrix wrist_translation = Matrix.CreateTranslation(lowerArm_Length);
            //Matrix m_lelbowroll = Matrix.CreateRotationX(-angles["LElbowRoll"]);
            //Console.WriteLine("elbow roll: " + (-angles["LElbowRoll"]));
            ////Matrix m_lelbowyaw = Matrix.CreateRotationZ(angles["LElbowYaw"] + (float) Math.PI / 2);
            ////Console.WriteLine("elbow yaw: " + (angles["LElbowYaw"] +  (float) Math.PI / 2));

            ////Matrix elbow_rotation = Matrix.Multiply(m_lelbowroll, m_lelbowyaw);
            //Vector3 elbow_ref = Vector3.Transform(elbow, m_lelbowroll);
            //Vector3 wrist = Vector3.Transform(elbow_ref, wrist_translation);
            //Console.WriteLine("calculated position wrist: " + wrist);
            //LWristY.position = wrist;

            //right leg 
            Vector3 UL = new Vector3(0f, 0f, .1f);
            Vector3 LL = new Vector3(0f, 0f, .102f);
            Vector3 rhip_reference =RHipPitch.position;
            Matrix knee_translation = Matrix.CreateTranslation(UL);

            Matrix m_hippitch = Matrix.CreateRotationX(angles["RHipPitch"]);
            Matrix m_hiproll = Matrix.CreateRotationZ(angles["RHipRoll"]);
          
            Matrix rhip_rotation = Matrix.Multiply(m_hippitch, m_hiproll);
            Vector3 cur_hip_ref = Vector3.Transform(rhip_reference, rhip_rotation);
            
            Vector3 UL_tx = Vector3.Transform(cur_hip_ref, knee_translation);
            RKneePitch.position = UL_tx;

            Matrix m_kneepitch = Matrix.CreateRotationX(angles["RKneePitch"]);
            // notice here that the orientation matrix from before is now augmented with the rotation for the next
            // link in the chain
            Vector3 cur_knee_ref = Vector3.Transform(UL_tx, m_hippitch);
            Matrix ankle_translation = Matrix.CreateTranslation(LL);
            Vector3 LL_tx = Vector3.Transform(cur_knee_ref, ankle_translation);
            // you can add the re-oriented limb vectors at any point to get the position of a particular link in the chain
            // here the upper and lower leg vectors are added together to get the position of the foot (i'm pretty sure that in
            // general you can work out the orientation of each limb vector without using the position information, as was 
            // done here)
          
            RAnklePitch.position = LL_tx;
           
                  
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

        public LinkedList<JointNode> getRobot()
        {
            return Robot;
        }   




    
    }
}
