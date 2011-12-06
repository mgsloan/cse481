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
    class KinectViewer : Viewer
    {
        private enum FootState { LEFT, RIGHT, BOTH }

        protected NaoSimulator naoSim;
        protected Balancer balancer;
        private TwoFootBalancer fixedBalancer;

        Runtime nui = new Runtime();
        protected SkeletonData cur_skeleton;

        const string IP = "127.0.0.1"; // "128.208.4.225";

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
        private FootState footState = FootState.BOTH;

        //subclasses implement, this class decides when to call
        protected virtual void SetTwoLegStance() { }
        protected virtual void SetOneLegStance() { }

        // Kinect-derived angles (manipulated in subclasses)
        protected Dictionary<String, float> kinectAngles = new Dictionary<string, float>();

        // Torso reference (manipulated in subclasses)
        protected Matrix srRef { get; set; }

        protected virtual void UpdateSkeleton(SkeletonData skeleton)
        {
            if (cur_skeleton == null)
            {
                //set the initial position of the feet
                //var set = TryInitializeFeetPosition(skeleton);
            }
            cur_skeleton = skeleton;
            determineFootElevation(skeleton);
            gridOrigin = getLoc(cur_skeleton.Joints[JointID.Spine]);
            //sc.sendRotationSpeeds(nao.values);

            fixedBalancer.balance(lines, srRef.Forward, cur_skeleton);
            naoSim.UpdatePositions();
            naoSim.RSSend();

            //BALANCE METHOD 2
            //fixedBalancer.balance(lines, srRef.Forward); //should do this before calling UpdatePositions

            //END BALANCE METHOD 2

        }

        // This method is here so that it's easy to switch back and forth between using render requests to
        // update the robot and using kinect updates to trigger robot update.
        protected void UpdateRobot()
        {
            float ang = 0.2f;
            //naoSim.UpdateAngle("LKneePitch", ang);
            //naoSim.UpdateAngle("LHipPitch", -ang);
            //naoSim.UpdateAngle("RHipRoll", -.73f);
            //naoSim.UpdateAngle("RHipPitch", -.5f);
            //naoSim.UpdateAngle("RHipYawPitch", 0f);
            //naoSim.UpdateAngle("LHipYawPitch", 0f);

            //naoSim.SenseJoint("RHipYawPitch");
            //naoSim.SenseJoint("RAnkleRoll");
            //naoSim.SenseJoint("RAnklePitch");
           // balancer.Balance(1, lines, srRef.Up, frame);

            naoSim.RSSend();
        }

        protected override void DrawStuff()
        {
            if (naoSim.connected)
            {
                UpdateRobot();
                naoSim.UpdatePositions();
                frame++;
                DrawRobot();
            }

            if (cur_skeleton != null)
            {
                if (cur_skeleton.TrackingState == SkeletonTrackingState.Tracked)
                {
                    foreach (Joint joint in cur_skeleton.Joints)
                    {
                        var position = FromKinectSpace(joint.Position);
                        drawPrimitive(sphere, position, Color.Red);
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
                foreach (NaoPos pos in new[] { foot.pfl, foot.pfr, foot.prl, foot.prr })
                {
                    RobotSimSphere.Draw(Matrix.Multiply(Matrix.CreateScale(0.2f, 0.2f, 0.2f), Matrix.CreateTranslation(pos.position)),
                                            viewMatrix, projection, Color.Black);
                }
            }

            drawPrimitive(COMsphere, naoSim.GetTwoFootCenter(), Color.Green);
            Vector3 COM = naoSim.GetCOM();
            drawPrimitive(COMsphere, COM, Color.Green);
            Vector3 Rdisplace = Vector3.Transform((naoSim.GetFootTarget(srRef) - COM), Matrix.Invert(srRef));
            lines.Add(new LabelledVector(COM, COM + Rdisplace, Color.Black, "T"));

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
                    //if (cur.name == "RAnklePitch" || cur.name == "RAnkleRoll")
                    //    Viewer.DebugReferenceFrame("", cur.torsoSpacePosition);
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
            base.LoadContent();

            sphere = new SpherePrimitive(GraphicsDevice, 0.5f, 8);
            COMsphere = new SpherePrimitive(GraphicsDevice, 0.5f, 8);
            BodySphere = new SpherePrimitive(GraphicsDevice, 0.4f, 8);
            RobotSimSphere = new SpherePrimitive(GraphicsDevice, 0.6f, 8);
            leftFootInitial = new Vector3();
            rightFootInitial = new Vector3();

            try
            {
                nui.Initialize(RuntimeOptions.UseSkeletalTracking);

                nui.SkeletonFrameReady += new EventHandler<SkeletonFrameReadyEventArgs>(nui_SkeletonFrameReady);
                // Must set to true and set after call to Initialize
                nui.NuiCamera.ElevationAngle = 1;

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
            balancer = new Balancer(naoSim);
            fixedBalancer = new TwoFootBalancer(naoSim);
        }

        void nui_SkeletonFrameReady(object sender, SkeletonFrameReadyEventArgs e)
        {
            SkeletonFrame allSkeletons = e.SkeletonFrame;

            // Get the first tracked skeleton
            SkeletonData skeleton = (from s in allSkeletons.Skeletons
                                     where s.TrackingState == SkeletonTrackingState.Tracked
                                     select s).FirstOrDefault();

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
                if (cur_left.Y - cur_right.Y > .3)
                {
                    if (!footState.Equals(FootState.LEFT)) Console.WriteLine("your left foot is up");
                    footState = FootState.LEFT;
                    SetOneLegStance();
                }
                else if (cur_left.Y - cur_right.Y < -.3)
                {
                    if (!footState.Equals(FootState.RIGHT)) Console.WriteLine("your right foot is up");
                    footState = FootState.RIGHT;
                    SetOneLegStance();
                }
                else
                {
                    if (!footState.Equals(FootState.BOTH)) Console.WriteLine("both of your feet are on the ground");
                    footState = FootState.BOTH;
                    SetTwoLegStance();
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