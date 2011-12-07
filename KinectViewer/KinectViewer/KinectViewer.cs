using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using Microsoft.Research.Kinect.Audio;
using Microsoft.Research.Kinect.Nui;
using Microsoft.Xna.Framework;
using Microsoft.Xna.Framework.Graphics;
using Microsoft.Xna.Framework.Input;
using Microsoft.Xna.Framework.Content;
using System.IO;

namespace KinectViewer
{
    public enum FootState { LEFT, RIGHT, BOTH };

    class KinectViewer : Viewer
    {
        public static KinectViewer instance;

        public NaoSimulator naoSim;
        //protected Balancer balancer;
        //private TwoFootBalancer twoFootBalancer;
        public DisplacementBalancer displacementBalancer;
        public Vector3 prevDisplacement;

        Runtime nui = new Runtime();
        protected SkeletonData cur_skeleton;
       // "128.208.4.14";  //
        const string IP = true ? "128.208.4.14" : "127.0.0.1";
        SpherePrimitive sphere;
        SpherePrimitive COMsphere;
        SpherePrimitive BodySphere;
        SpherePrimitive RobotSimSphere;
        Vector3 leftFootInitial;
        Vector3 rightFootInitial;

        //manipulated by subclass
        protected Vector3 initHr, initHl, initFr, initFl; //hip right, hip left, foot right, foot left
        protected Vector3 curHr, curHl, curFr, curFl;
        protected bool TwoLegStand = false;
        protected FootState footState = FootState.LEFT;
        protected int twoLegStartFrame = -1;
        protected Vector3 twoLegInitial;

        //subclasses implement, this class decides when to call
        protected virtual void SetTwoLegStance() { }
        protected virtual void SetOneLegStance() { }

        // Kinect-derived angles (manipulated in subclasses)
        protected Dictionary<String, float> kinectAngles = new Dictionary<string, float>();

        // Torso reference (manipulated in subclasses)
        protected Matrix srRef { get; set; }
        
        public Vector3 getLoc(JointID j) { return getLoc(cur_skeleton.Joints[j]); }
        public Vector3 getLoc(Joint j) { return getLoc(j.Position); }
        public Vector3 getLoc(Vector v) { return Vector3.Multiply(new Vector3(v.X, v.Y, v.Z), 10); }


        protected virtual void UpdateSkeleton(SkeletonData skeleton)
        {
            if (cur_skeleton == null)
            {
                //set the initial position of the feet
                //var set = TryInitializeFeetPosition(skeleton);
            }
            frame++;
            cur_skeleton = skeleton;
            determineFootElevation(skeleton);
        }


        // This method is here so that it's easy to switch back and forth between using render requests  to
        // update the robot and using kinect updates to trigger robot update.
        public void UpdateRobot()
        {
            if (naoSim == null || cur_skeleton == null) return;
            lines.Clear();

            if (cur_skeleton != null)
            {
                //lock (kinectAngles)
                //{
                    //foreach (KeyValuePair<String, float> entry in kinectAngles) naoSim.UpdateAngle(entry.Key, entry.Value, 0f);
                //}

                string[] kinectParts = new string[] { "RShoulderPitch", "RShoulderRoll", "RElbowRoll", "RElbowYaw", "RWristYaw",
                    "LShoulderPitch", "LShoulderRoll", "LElbowRoll", "LElbowYaw", "LWristYaw", "", "", "" };


                if (footState == FootState.LEFT)
                {
                    kinectParts[kinectParts.Length - 1] = "RHipRoll";
                    kinectParts[kinectParts.Length - 2] = "RHipPitch";
                    kinectParts[kinectParts.Length - 3] = "RKneePitch";
                }
                else if (footState == FootState.RIGHT)
                {
                    kinectParts[kinectParts.Length - 1] = "LHipRoll";
                    kinectParts[kinectParts.Length - 2] = "LHipPitch";
                    kinectParts[kinectParts.Length - 3] = "LKneePitch";
                }

                Vector3 lFoot = getLoc(JointID.AnkleLeft), rFoot = getLoc(JointID.AnkleRight);
                Vector3 hCenter = getLoc(JointID.HipCenter);
                Vector3 lDelta = Vector3.Subtract(hCenter, lFoot);
                Vector3 rDelta = Vector3.Subtract(hCenter, rFoot);
                lDelta.Normalize();
                rDelta.Normalize();
                float ang = (float)Math.Acos(Vector3.Dot(lDelta, rDelta));
                DebugVector(ang.ToString(), new Vector3(-4f, 4f, 0f), Vector3.Zero, Color.MistyRose);

                foreach (string part in kinectParts) {
                    if (kinectAngles.ContainsKey(part)) {
                        float angle = kinectAngles[part];
                        float smooth = 0.3f;
                        if (footState == FootState.LEFT && part == "LHipRoll") { angle = ang / 2f; smooth = 0.8f; }
                        if (footState == FootState.LEFT && part == "RHipRoll") { angle = ang / -2f; smooth = 0.8f; }
                        if (footState == FootState.RIGHT && part == "LHipRoll") { angle = ang / 2f; smooth = 0.8f; }
                        if (footState == FootState.RIGHT && part == "RHipRoll") { angle = ang / -2f; smooth = 0.8f; }
                        naoSim.UpdateAngle(part, angle, smooth);
                    }
                }

                if (frame < twoLegStartFrame + 30)
                {
                    twoLegInitial = Vector3.Lerp(twoLegInitial, getLoc(JointID.Spine), 0.5f);
                }
                naoSim.UpdateAngle("RHipYawPitch", 0f);

                float toNaoSpace = 0.6f;

                Vector3 target = getLoc(JointID.Spine);
                Vector3 initial = twoLegInitial;
                initial.X = (2 * initial.X + lFoot.X + rFoot.X) / 4f;
                Vector3 displacement = Vector3.Multiply(Vector3.Subtract(target, initial), toNaoSpace);
                float dist = Vector3.Distance(displacement, prevDisplacement);
                if (dist > 0.1) displacement = Vector3.Add(Vector3.Multiply(Vector3.Subtract(displacement, prevDisplacement), 0.04f / dist),
                    prevDisplacement);
                prevDisplacement = displacement;
                displacementBalancer.Balance(footState, displacement, Vector3.Distance(lFoot, rFoot) * toNaoSpace);
            }
            else
            {
                    displacementBalancer.Balance(footState, Vector3.Zero, 3.0f);

            }
            naoSim.RSSend();

        }

        protected override void DrawStuff()
        {
            if (naoSim.connected)
            {
                //displacementBalancer.InitializeTwoLegStance(Vector3.Zero);
                DrawRobot();
            }

            if (cur_skeleton != null)
            {
                if (cur_skeleton.TrackingState == SkeletonTrackingState.Tracked)
                {
                    foreach (Joint joint in cur_skeleton.Joints)
                    {
                        var position = Vector3.Add(Vector3.Subtract(FromKinectSpace(joint.Position), getLoc(JointID.Spine)), new Vector3(6f, 0, 0));
                        position.X = -position.X;
                        position.Z = -position.Z;
                        drawPrimitive(sphere, Vector3.Multiply(position, 0.4f), Color.Red);
                    }
                }
            }
        }

        private void DrawRobot()
        {
            var robot = naoSim.GetRobot();

            var rightF = naoSim.GetRightFoot();
            var leftF = naoSim.GetLeftFoot();

            foreach (NaoFoot foot in new[] { rightF, leftF })
            {
                NaoPos[] poss = new[] { foot.pfl, foot.pfr, foot.prl, foot.prr };
                float[] forces = new[] { foot.ffl, foot.ffr, foot.frl, foot.frr };
                for (int i = 0; i < 4; i++)
                {
                    NaoPos pos = poss[i];
                    float force = forces[i];
                    if (force < 1f) force = 0.2f;
                    RobotSimSphere.Draw(Matrix.Multiply(Matrix.CreateScale(0.2f, force / 3f, 0.2f), Matrix.CreateTranslation(pos.position)),
                                            viewMatrix, projection, Color.Orange);
                    
                }
            }

            drawPrimitive(COMsphere, naoSim.GetTwoFootCenter(), Color.Green);
            Vector3 COM = naoSim.GetCOM();
            drawPrimitive(COMsphere, COM, Color.Green);
            //drawPrimitive(COMsphere, naoSim.proxy.GetCOM(), Color.Red);
            Vector3 Rdisplace = Vector3.Transform((naoSim.GetFootTarget(srRef) - COM), Matrix.Invert(srRef));

            double[] legRAngles = naoSim.ReadjustLegs(srRef);
            /*
            var legRUpdate = new Dictionary<string, float>();
            legRUpdate.Add("RHipRoll", (float)(legRAngles[1]));
            legRUpdate.Add("RHipPitch", (float)(legRAngles[0] - Math.PI / 2));
            legRUpdate.Add("RKneePitch", (float)(legRAngles[2]));
            sim.UpdatePositions(legRUpdate); 
            */

            foreach (JointNode chain in robot)
            {
                JointNode cur = chain.next;
                while (cur != null)
                {
                    if (cur.name == "RKneePitch" || cur.name == "LKneePitch")
                        Viewer.DebugReferenceFrame("", cur.torsoSpacePosition);
                    Matrix w1 = Matrix.Multiply(Matrix.CreateScale(0.3f, 0.3f, 0.3f), Matrix.CreateTranslation(cur.torsoSpacePosition.Translation));
                    //if (srRef != null) w1 = Matrix.Multiply(w1, Matrix.Invert(srRef));
                    RobotSimSphere.Draw(w1, viewMatrix, projection, Color.Red);

                    float sc = (float)cur.mass * 2;
                    Matrix w2 = Matrix.Multiply(Matrix.CreateScale(sc, sc * 2, sc), Matrix.Multiply(Matrix.CreateTranslation(cur.com), cur.torsoSpacePosition));
                    //if (srRef != null) w2 = Matrix.Multiply(w2, Matrix.Invert(srRef));
                    RobotSimSphere.Draw(w2, viewMatrix, projection, Color.White);
                    cur = cur.next;
                }
            }

            //drawPrimitive(COMsphere, nao.getGyro(), Color.Red);
            //float offset = nao.computeOffsetParam();
            //Console.WriteLine("offset: " + offset);
        }

        protected override void LoadContent()
        {
            instance = this;
            base.LoadContent();
            sphere = new SpherePrimitive(GraphicsDevice, 0.5f, 8);
            COMsphere = new SpherePrimitive(GraphicsDevice, 0.5f, 8);
            BodySphere = new SpherePrimitive(GraphicsDevice, 0.2f, 8);
            RobotSimSphere = new SpherePrimitive(GraphicsDevice, 0.6f, 8);
            leftFootInitial = new Vector3();
            rightFootInitial = new Vector3();

            try
            {
                nui.Initialize(RuntimeOptions.UseSkeletalTracking);

                nui.SkeletonFrameReady += new EventHandler<SkeletonFrameReadyEventArgs>(nui_SkeletonFrameReady);
                // Must set to true and set after call to Initialize
                nui.NuiCamera.ElevationAngle = -15;

                nui.SkeletonEngine.TransformSmooth = true;
                // Use to transform and reduce jitter   
                var parameters = new TransformSmoothParameters
                {
                    Smoothing = 0.5f,
                    Correction = 0.5f,
                    Prediction = 0.0f,
                    JitterRadius = 0.05f,
                    MaxDeviationRadius = 0.05f
                };
                nui.SkeletonEngine.SmoothParameters = parameters;
            }
            catch
            {
                clearColor = Color.Turquoise;
            }

            naoSim = new NaoSimulator(IP);
            //balancer = new Balancer(naoSim);
            //twoFootBalancer = new TwoFootBalancer(naoSim);
            displacementBalancer = new DisplacementBalancer(naoSim);
        }

        void nui_SkeletonFrameReady(object sender, SkeletonFrameReadyEventArgs e)
        {
            SkeletonFrame allSkeletons = e.SkeletonFrame;

            SkeletonData skeleton = null;
            float dist = 10000f;
            foreach (SkeletonData s in allSkeletons.Skeletons)
            {

                float sdist = getLoc(s.Position).Length();
                if (s.TrackingState != SkeletonTrackingState.NotTracked && sdist < dist)
                {
                    dist = sdist;
                    skeleton = s;
                }
            }

            if (skeleton != null)
            {
                UpdateSkeleton(skeleton);
                //float offset = nao.computeOffsetParam();
                //Console.WriteLine("offset: " + offset);
            }
            else
            {
                cur_skeleton = null;
            }
        }

        //once the intial position has been determined, this method determines which foot is 
        //lifted
        private void determineFootElevation(SkeletonData skeleton)
        {
            var leftjoint = skeleton.Joints[JointID.AnkleLeft];
            var rightjoint = skeleton.Joints[JointID.AnkleRight];

            var cur_left = FromKinectSpace(leftjoint.Position);
            var cur_right = FromKinectSpace(rightjoint.Position);

            //will only determine which foot is lifted if both feet are being tracked
            //sets the threshold at .4
            if (leftjoint.TrackingState == JointTrackingState.Tracked && rightjoint.TrackingState == JointTrackingState.Tracked)
            {
                //Console.WriteLine("init_left: " + leftFootInitial.Y);
                //Console.WriteLine("init_right: " + rightFootInitial.Y);
                //Console.WriteLine("cur_left: " + cur_left.Y);
                //Console.WriteLine("cur_right: " + cur_right.Y);
                bool isLow = displacementBalancer.curVT > 0.7;
                if (!isLow && cur_left.Y - cur_right.Y > .85)
                {
                    if (!footState.Equals(FootState.LEFT)) Console.WriteLine("your left foot is up");
                    footState = FootState.RIGHT;
                    //SetOneLegStance();
                }
                else if (!isLow && cur_left.Y - cur_right.Y < -.85)
                {
                    if (!footState.Equals(FootState.RIGHT)) Console.WriteLine("your right foot is up");
                    footState = FootState.LEFT;
                    //SetOneLegStance();
                }
                else
                {
                    if (!footState.Equals(FootState.BOTH))
                    {
                        Console.WriteLine("both of your feet are on the ground");
                        displacementBalancer.InitializeTwoLegStance(getLoc(JointID.Spine));
                        twoLegStartFrame = frame;
                        twoLegInitial = getLoc(JointID.Spine);
                    }
                    footState = FootState.BOTH;
                    //SetTwoLegStance();
                }
            }
            else
            {
                //Console.WriteLine("Feet are not tracked");
            }

        }

        //tries to initialize the feet position. if it succeeds then returns true, otherwise false.
        private bool TryInitializeFeetPosition(SkeletonData skeleton)
        {
            //get ankle joints
            var leftjoint = skeleton.Joints[JointID.AnkleLeft];
            var rightjoint = skeleton.Joints[JointID.AnkleRight];

            var left = FromKinectSpace(leftjoint.Position);
            var right = FromKinectSpace(rightjoint.Position);

            //the feet must be tracked and the y coordinates cannot differ more than .3 to initialize the position.
            if (Math.Abs(left.Y - right.Y) < .3 && leftjoint.TrackingState == JointTrackingState.Tracked
                && rightjoint.TrackingState == JointTrackingState.Tracked)
            {
                leftFootInitial = left;
                rightFootInitial = right;
                return true;
            }
            return false;

        }


        public Vector3 FromKinectSpace(Vector position)
        {
            var returnVector = new Vector3();
            returnVector.X = position.X * 10;
            returnVector.Y = position.Y * 10;
            returnVector.Z = position.Z * 10;
            return returnVector;
        }
    }
}