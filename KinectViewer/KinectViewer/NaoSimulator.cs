using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using Microsoft.Xna.Framework;
using Aldebaran.Proxies;
using System.Collections;
using System.Threading;

namespace KinectViewer
{
    class NaoSimulator
    {
        public Dictionary<string, JointNode> jointToNode { get; set; }
        private float speed = 0.1f;
        public NaoProxy proxy { get; set; }
        Dictionary<string, ArrayList> limits = new Dictionary<string, ArrayList>();

        /*setting up the joint nodes for the robot. */
        JointNode LShoulderPitch, RShoulderPitch, RShoulderRoll, LShoulderRoll, LElbowYaw, RElbowYaw, LElbowRoll, RElbowRoll, LWristYaw,
                RWristYaw, LHipYawPitch, RHipYawPitch, LHipPitch, RHipPitch, LHipRoll, RHipRoll, LKneePitch, RKneePitch, LAnklePitch,
                RAnklePitch, LAnkleRoll, RAnkleRoll, Torso, HeadYaw, HeadPitch;

        NaoFoot rightF, leftF;
        Vector3[] rightFLocal;
        Vector3[] leftFLocal;

        double UL_len, LL_len; //upper/lower leg length
        Vector3 initHr, initHl, initFr, initFl, initCenter;
        public bool twoLegs { get; set; }

        LinkedList<JointNode> Robot;
        /*this constructs a NAO object. Will initial the NAO parts 
        /to their corresponding position vector. 
        */
        public bool connected
        {
            get
            {
                return proxy != null;
            }
        }

        public NaoSimulator(string ip)
        {
            LShoulderPitch = new JointNode("LShoulderPitch", Vector3.Left);
            RShoulderPitch = new JointNode("RShoulderPitch", Vector3.Left);
            RShoulderRoll = new JointNode("RShoulderRoll", Vector3.Forward);
            LShoulderRoll = new JointNode("LShoulderRoll", Vector3.Forward);
            LElbowYaw = new JointNode("LElbowRoll", Vector3.Left);
            RElbowYaw = new JointNode("RElbowRoll", Vector3.Left);
            LElbowRoll = new JointNode("LElbowYaw", Vector3.Up);
            RElbowRoll = new JointNode("RElbowYaw", Vector3.Up);
            LWristYaw = new JointNode("LWristYaw", Vector3.Up);
            RWristYaw = new JointNode("RWristYaw", Vector3.Up);
            float sq2 = (float)Math.Sqrt(2.0) / 2.0f;
            LHipYawPitch = new JointNode("LHipYawPitch", new Vector3(-sq2, sq2, 0f));
            RHipYawPitch = new JointNode("RHipYawPitch", new Vector3(-sq2, -sq2, 0f));
            RHipPitch = new JointNode("RHipPitch", Vector3.Left);
            LHipPitch = new JointNode("LHipPitch", Vector3.Left);
            RHipPitch = new JointNode("RHipPitch", Vector3.Left);
            LHipRoll = new JointNode("LHipRoll", Vector3.Forward);
            RHipRoll = new JointNode("RHipRoll", Vector3.Forward);
            LKneePitch = new JointNode("LKneePitch", Vector3.Left);
            RKneePitch = new JointNode("RKneePitch", Vector3.Left);
            LAnklePitch = new JointNode("LAnklePitch", Vector3.Left);
            RAnklePitch = new JointNode("RAnklePitch", Vector3.Left);
            LAnkleRoll = new JointNode("LAnkleRoll", Vector3.Backward);
            RAnkleRoll = new JointNode("RAnkleRoll", Vector3.Backward);
            Torso = new JointNode("Torso", Vector3.Up);
            HeadYaw = new JointNode("HeadYaw", Vector3.Up);
            HeadPitch = new JointNode("HeadPitch", Vector3.Left);

            jointToNode = new Dictionary<string, JointNode>()
            {
                {"RShoulderPitch", RShoulderPitch}, {"RShoulderRoll", RShoulderRoll},  {"RElbowRoll", RElbowRoll}, {"RElbowYaw", RElbowYaw}, {"RWristYaw", RWristYaw},
                {"LShoulderPitch", LShoulderPitch} , {"LShoulderRoll", LShoulderRoll}, {"LElbowRoll", LElbowRoll} , {"LElbowYaw", LElbowYaw}, {"LWristYaw", LWristYaw},
                {"RHipYawPitch", RHipYawPitch}, {"RHipRoll", RHipRoll}, {"RHipPitch", RHipPitch}, {"RKneePitch", RKneePitch}, {"RAnklePitch", RAnklePitch}, {"RAnkleRoll", RAnkleRoll},
                {"LHipYawPitch", LHipYawPitch}, {"LHipRoll", LHipRoll}, {"LHipPitch", LHipPitch}, {"LKneePitch", LKneePitch}, {"LAnklePitch", LAnklePitch}, {"LAnkleRoll", LAnkleRoll},
                {"Torso", Torso}, {"HeadYaw", HeadYaw}, {"HeadPitch", HeadPitch} 
            };

            //jointToNode.Keys.ToList(),
            proxy = new NaoProxy(ip, jointToNode.Keys.ToList(), 100);
            proxy.InitialPoll();
            Thread thread = new Thread(new ThreadStart(proxy.PollLoop));
            thread.Start();

            leftF = new NaoFoot("L");
            rightF = new NaoFoot("R");

            /********create the kinematic chains*****/
            JointNode LeftArm  = CreateChain(new JointNode[] { LShoulderPitch, LShoulderRoll, LElbowYaw, LElbowRoll, LWristYaw });
            JointNode RightArm = CreateChain(new JointNode[] { RShoulderPitch, RShoulderRoll, RElbowYaw, RElbowRoll, RWristYaw });
            JointNode LeftLeg  = CreateChain(new JointNode[] { LHipYawPitch, LHipRoll, LHipPitch, LKneePitch, LAnklePitch, LAnkleRoll });
            JointNode RightLeg = CreateChain(new JointNode[] { RHipYawPitch, RHipRoll, RHipPitch, RKneePitch, RAnklePitch, RAnkleRoll });
            JointNode Head     = CreateChain(new JointNode[] { HeadYaw, HeadPitch });
            JointNode Body     = CreateChain(new JointNode[] { Torso });

            Robot = new LinkedList<JointNode>(new JointNode[] { LeftArm, RightArm, LeftLeg, RightLeg, Head, Body });

            //initialize the positions of the robot.   

            InitializePositions(Robot);
            InitializeLimits();
        }


        // Functions for initializing and computing NAO kinematics.
        // ====================================================================

        private void InitializeLimits()
        {
            limits = proxy.GetLimits();          
        }

        private JointNode CreateChain(JointNode[] chain)
        {
            JointNode head = new JointNode();
            var chainlst = chain.ToList();
            for (int i = 0; i < chain.Length - 1; i++)
            {
                chainlst[i].next = chainlst[i + 1];
            }
            head.next = chainlst[0];
            return head;
        }

        private void InitializePositions(LinkedList<JointNode> chains)
        {
            Matrix prev = Matrix.Identity;
            foreach (JointNode chain in chains)
            {
                JointNode cur = chain.next;
                while (cur != null)
                {
                    cur.localPosition = proxy.GetPosition(cur.name).transform;
                    cur.torsoSpacePosition = cur.localPosition;
                    var temp = cur.localPosition;
                    cur.localPosition = Matrix.Multiply(cur.torsoSpacePosition, Matrix.Invert(prev));

                    Matrix toLocal = Matrix.Invert(cur.torsoSpacePosition);

                    cur.mass = proxy.GetMass(cur.name);
                    if (cur.name != "Torso")
                    {
                        cur.initialAngle = proxy.GetAngles(cur.name);
                        cur.updatedAngle = cur.initialAngle;
                    }
                    Vector3 torsoCom = proxy.GetCOM(cur.name);
                    cur.com = Vector3.Transform(torsoCom, toLocal);

                    //Console.WriteLine(torsoCom.ToString() + ", transformed: " + cur.com.ToString()); 

                 
                    
                    cur = cur.next;
                    prev = temp;
                }
                prev = Matrix.Identity;

            }

            UL_len = (GetPosition("RKneePitch") - GetPosition("RHipPitch")).Length();
            LL_len = (GetPosition("RAnklePitch") - GetPosition("RKneePitch")).Length();
        }

        private void UpdateChain(JointNode chain)
        {
            Matrix prev = chain.torsoSpacePosition;
            JointNode cur = chain.next;
            while (cur != null)
            {
                cur.torsoSpacePosition = MathUtils.RotateBy(
                    Matrix.Multiply(cur.localPosition, prev),
                    cur.MakeRotation(cur.updatedAngle));
                
                prev = cur.torsoSpacePosition;
                cur = cur.next;
            }
        }

        //TODO: limits?
        public void UpdatePositions()
        {
            //iterate through the chains 
            Matrix prev = Matrix.Identity;
            foreach (JointNode chain in Robot)
            {
                UpdateChain(chain);
            }
        }

        // Given a joint name, sets the angle to the one reported by the robot.
        public void SenseJoint(string part)
        {
            JointNode node = jointToNode[part];
            UpdateAngle(part, proxy.GetAngles(part));
        }


        // Communication with the Nao - propogate the current angles of the
        // simulator.
        // ====================================================================

        public void RSSendBlocking()
        {
            if (proxy == null) return;
            ArrayList joints = new ArrayList();
            ArrayList values = new ArrayList();
            foreach (string joint in jointToNode.Keys)
            {
                if (joint != "Torso")
                {
                    joints.Add(joint);
                    values.Add(jointToNode[joint].updatedAngle);
                }
            }
            proxy.SetAngles(joints, values, .2f);
            //_motion.wait(id, 10000);
            System.Threading.Thread.Sleep(3000);
        }

        public void RSSend()
        {
            if (proxy == null) return;
            ArrayList joints = new ArrayList();
            ArrayList values = new ArrayList();
            foreach (string joint in jointToNode.Keys)
            {
                if (joint != "Torso" && joint != "RAnklePitch" && joint != "LAnklePitch")
                {
                    joints.Add(joint);
                    values.Add(jointToNode[joint].updatedAngle);
                }
            }

            proxy.SetAngles(new ArrayList(new string[] { "RAnklePitch", "LAnklePitch" }),
                        new ArrayList(new float[] { jointToNode["RAnklePitch"].updatedAngle, jointToNode["LAnklePitch"].updatedAngle }), .1f);
            proxy.SetAngles(joints, values, speed);
        }


        // Mutators
        // ====================================================================

        public void UpdateAngle(string jointName, float val, float smooth) { if (!float.IsNaN(val)) { SetJoint(jointName, val, smooth); } }
        public void UpdateAngle(string jointName, float val) { if (!float.IsNaN(val)) { SetJoint(jointName, val, 0); } }

        public void UpdateAngleAndPos(string jointName, float val, float smooth) { UpdateChain(SetJoint(jointName, val, smooth)); }
        public void UpdateAngleAndPos(string jointName, float val)               { UpdateChain(SetJoint(jointName, val, 0.7f)); }

        private JointNode SetJoint(string jointName, float val, float smooth)
        {
            
            bool isPelvis = (jointName == "LHipYawPitch" || jointName == "RHipYawPitch");
            JointNode node = jointToNode[isPelvis ? "LHipYawPitch" : jointName];
            float prior = node.updatedAngle;

            if (float.IsNaN(prior)) prior = 0;
            ArrayList limit = (ArrayList)limits[jointName][0];
            node.updatedAngle = MathUtils.Clamp(val, (float)limit[0], (float)limit[1]);

            node.updatedAngle = prior * smooth + node.updatedAngle * (1 - smooth);
            if (isPelvis) jointToNode["RHipYawPitch"].updatedAngle = node.updatedAngle;
            //Console.WriteLine("smooth: " + prior.ToString() + " " + values[ix].ToString());
            return node;
            
        }

        // Setting ankle angles, limited to the feasible ranges.
        public void LAUpdate(float pitch, float roll)
        {
            float pitch2 = MathUtils.Clamp(pitch, -1.189516f, 0.922747f);
            UpdateAngle("LAnklePitch", pitch2);
            UpdateAngle("LAnkleRoll", NearestFeasibleRoll(pitch2, roll));
        }
        public void RAUpdate(float pitch, float roll)
        {
            float pitch2 = MathUtils.Clamp(pitch, -1.189516f, 0.922747f);
            UpdateAngle("RAnklePitch", pitch2);
            UpdateAngle("RAnkleRoll", -NearestFeasibleRoll(pitch2, -roll));
        }
        
        // Clamps roll for the left foot diagram.
        private static float NearestFeasibleRoll(float pitch, float roll)
        {
            float pitchDeg = MathUtils.FromRad(pitch);
            float rollDeg = MathUtils.FromRad(roll);
            if (pitchDeg < -48.12) {
                return MathUtils.ToRad(InterpClamp(
                    -68.15f, 2.86f, -4.29f,
                    -48.12f, 10.31f, -9.74f,
                    pitchDeg, rollDeg));
            } else if (pitchDeg < -40.10) {
                return MathUtils.ToRad(InterpClamp(
                    -48.12f, 10.31f, -9.74f,
                    -40.10f, 22.79f, -12.60f,
                    pitchDeg, rollDeg));
            } else if (pitchDeg < -25.78) {
                return MathUtils.ToRad(InterpClamp(
                    -40.10f, 22.79f, -12.60f,
                    -25.78f, 22.79f, -44.06f,
                    pitchDeg, rollDeg));
            } else if (pitchDeg < 5.72) {
                return MathUtils.ToRad(InterpClamp(
                    -25.78f, 22.79f, -44.06f,
                    5.72f, 22.79f, -44.06f,
                    pitchDeg, rollDeg));
            } else if (pitchDeg < 20.05) {
                return MathUtils.ToRad(InterpClamp(
                    5.72f, 22.79f, -44.06f,
                    20.05f, 22.79f, -31.54f,
                    pitchDeg, rollDeg));
            } else {
                return MathUtils.ToRad(InterpClamp(
                    20.05f, 22.79f, -31.54f,
                    52.86f, 0f, -2.86f,
                    pitchDeg, rollDeg));
            }
        }

        // Interpolate clamping values for y, dependent on x.
        private static float InterpClamp(float x1, float t1, float f1, float x2, float t2, float f2, float x, float y)
        {
            float t = MathUtils.UnLerp(x, x1, x2);
            float l = MathUtils.Lerp(t, f1, f2), h = MathUtils.Lerp(t, t1, t2);
            return MathUtils.Clamp(y, l, h);
        }
        
        // Accessors
        // ====================================================================

        public Vector3 GetPosition(string part)
        {
            return jointToNode[part].torsoSpacePosition.Translation;
        }

        public Matrix GetTransform(string part)
        {
            return jointToNode[part].torsoSpacePosition;
        }

        // Gets axis angle, given an axis of rotation
        // UGLY
        public float GetAxisAngle(Vector3 v, Vector3 axis)
        {
            if (axis.X == 1f)  return (float)Math.Atan2(v.Y, v.Z);
            if (axis.X == -1f) return (float)Math.Atan2(v.Z, v.Y);
            if (axis.Y == 1f)  return (float)Math.Atan2(v.X, v.Z);
            if (axis.Y == -1f) return (float)Math.Atan2(v.Z, v.X);
            if (axis.Z == 1f)  return (float)Math.Atan2(v.X, v.Y);
            if (axis.Z == -1f) return (float)Math.Atan2(v.Y, v.X);
            throw new Exception("Cannot get angle for non axial rotation.");
        }
        
        /*
        public Tuple<float, float> GetAngleRequired(string a, string b, Vector3 vec)
        {
            JointNode ja = jointToNode[a], jb = jointToNode[b];
            Vector3 local = Vector3.Transform(vec, Matrix.Invert(ja.torsoSpacePosition));
            float angle = GetAxisAngle(local, ja.orientation);
            Matrix trans = Matrix.Multiply(Matrix.Multiply(ja.torsoSpacePosition, ja.MakeRotation(angle)), jb.localPosition);
            Vector3 local2 = Vector3.Transform(vec, Matrix.Invert(trans));
            return new Tuple<float, float>(angle, GetAxisAngle(local2, jb.orientation));
        }
         */
        /*
        private Tuple<float, float> GetAnglesInternal(JointNode ja, JointNode jb, JointNode jc, Vector3 vec)
        {
            // TODO: this reduplicates the logic expressed in the UpdatePositions method, but without actually mutating
            // the simulator.  It's quite possible that this is only used in the "SetAngleRequired" context, in which
            // case the mutators which update the subsequent position matrices for the chain would be more appropriate.
            
            // Local space of the first joint, with zero rotation.
            Matrix trans = MathUtils.RotateBy(
                Matrix.Multiply(jb.localPosition, ja.torsoSpacePosition),
                jb.MakeRotation(0.0f));

            Vector3 local1 = Vector3.Transform(vec, MathUtils.ExtractRotation(Matrix.Invert(trans)));
            float angle1 = GetAxisAngle(local1, jb.orientation);

            // Transformation appropriately rotated by the determined angle.
            trans = MathUtils.RotateByRev(Matrix.CreateFromAxisAngle(jb.orientation, -angle1), trans);

            Viewer.debugOrigin = new Vector3(3f, 0, 0f);
            Viewer.DebugReferenceFrame("t1", trans);

            // Local space of the second joint, with zero rotation.
            trans = MathUtils.RotateBy(Matrix.Multiply(jc.localPosition, trans), jc.MakeRotation(0.0f));
            Vector3 local2 = Vector3.Transform(vec, MathUtils.ExtractRotation(Matrix.Invert(trans)));
            float angle2 = GetAxisAngle(local2, jc.orientation);
            Console.WriteLine(angle2.ToString());


            trans = MathUtils.RotateByRev(Matrix.CreateFromAxisAngle(jc.orientation, -angle2), trans);
            Viewer.DebugReferenceFrame("t2", trans);
            Viewer.debugOrigin = new Vector3(4.0f, 0, 0);


            return new Tuple<float, float>(angle1, angle2);
        }

        // Takes the name of the joint prior to the two joints for which this will compute rotation angles.  
        public Tuple<float, float> GetAnglesRequired(string a, Vector3 vec)
        {
            JointNode ja = jointToNode[a], jb = ja.next, jc = jb.next;
            if (ja == null || jb == null || jc == null) throw new Exception("Misuse of GetAngleRequired");
            return GetAnglesInternal(ja, jb, jc, vec);
        }

        public void SetAnglesRequired(string a, Vector3 vec, float smooth)
        {
            JointNode ja = jointToNode[a], jb = ja.next, jc = jb.next;
            if (ja == null || jb == null || jc == null) throw new Exception("Misuse of SetAngleRequired");

            Tuple<float, float> angles = GetAnglesInternal(ja, jb, jc, vec);
            SetJoint(jb.name, angles.Item1, smooth);
            SetJoint(jc.name, angles.Item2, smooth);
        }
         */



        public NaoFoot GetRightFoot()
        {
            //var ankleRef = RAnkleRoll.torsoSpacePosition;
            //rightF.pfl.position = Vector3.Transform(rightFLocal[0], ankleRef);
            //rightF.pfr.position = Vector3.Transform(rightFLocal[1], ankleRef);
            //rightF.prl.position = Vector3.Transform(rightFLocal[2], ankleRef);
            //rightF.prr.position = Vector3.Transform(rightFLocal[3], ankleRef);

            return rightF;
        }
        public NaoFoot GetLeftFoot()
        {
            
            //var ankleRef = LAnkleRoll.torsoSpacePosition;
            //leftF.pfl.position = Vector3.Transform(leftFLocal[0], ankleRef);
            //leftF.pfr.position = Vector3.Transform(leftFLocal[1], ankleRef);
            //leftF.prl.position = Vector3.Transform(leftFLocal[2], ankleRef);
            //leftF.prr.position = Vector3.Transform(leftFLocal[3], ankleRef);
            return leftF;
            
        }

        //assumes both feet are flat on the ground
        public Vector3 GetTwoFootCenter()
        {
            var LAnkleRef = LAnkleRoll.torsoSpacePosition;
            var RAnkleRef = RAnkleRoll.torsoSpacePosition;

            return NaoFoot.VectorAverage(new Vector3[] {LAnkleRef.Translation, RAnkleRef.Translation} );
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
                    numerator += Vector3.Multiply(position, (float)mass);
                    cur = cur.next;
                }

            }

            Vector3 com = Vector3.Multiply(numerator, (float)1 / (float)totalMass);

            //Console.WriteLine(com);
            return com;
        }
        
        public LinkedList<JointNode> GetRobot()
        {
            return Robot;
        }

        public float GetInitialAngle(string jointName)
        {
            return jointToNode[jointName].initialAngle;
        }

        public float GetCurrentAngle(string jointName)
        {
            return jointToNode[jointName].updatedAngle;
        }
        

        // Code involved in stance computations.
        // ====================================================================

        //for single legged balancing, produce a target for a foot (currently the 
        //right foot, although this could be general)
        public Vector3 GetFootTarget(Matrix BodyTxform)
        {
            Vector3 COM = GetCOM();

            Vector3 RFoot = GetPosition("RAnklePitch");

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
        public double[] ReadjustLegs(Matrix BodyTxform)
        {
            Vector3 RTarget_torso = GetFootTarget(BodyTxform);

            double[] newRAngles = LegIK(Matrix.Identity, GetPosition("RHipPitch"), RTarget_torso, UL_len, LL_len);

            //Console.WriteLine("hr: " + newRAngles[1] + ", hp: " + newRAngles[0] + ", kp: " + newRAngles[2]); 

            return newRAngles;
        }


        public void InitializeTwoLegStance(Vector3 initCenter)
        {
            twoLegs = true;
            initHl = GetPosition("RHipPitch");
            initHr = GetPosition("LHipPitch");
            initFl = GetPosition("RAnklePitch");
            initFr = GetPosition("LAnklePitch");
            this.initCenter = initCenter;
        }

        //for when both feet are on the ground
        public double[] AdjustFeet(Vector3 position)
        {
            
            Vector3 displacement = position - initCenter;
            Viewer.debugOrigin = new Vector3(3f, 0f, 0f);
            Viewer.DebugVector("displacement", displacement, Color.Yellow);
            

            Vector3 finalFr = initFr - displacement;
            Vector3 finalFl = initFl - displacement;

            // do IK from each hip to the corresponding foot
            double[] lAngles = IKSolver.IK.LegIK(Matrix.Identity, initHr, finalFr, UL_len, LL_len);
            double[] rAngles = IKSolver.IK.LegIK(Matrix.Identity, initHl, finalFl, UL_len, LL_len);

            return lAngles;
            //Console.WriteLine("Left: " + lAngles[0] + ", " + lAngles[1] + ", " + lAngles[2]);
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