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
using HMMTest;

namespace KinectViewer
{
    class KinectViewer : Microsoft.Xna.Framework.Game
    {
        protected NaoBody nao = new NaoBody();
        protected NaoSpeech naoSpeech = new NaoSpeech();
        Runtime nui = new Runtime();
        SkeletonData cur_skeleton;
        protected SpeechRecognition sr = new SpeechRecognition();

        protected SoundController sc = new SoundController();

        bool trap_mouse = true;
        KeyboardState prior_keys;
        
        GraphicsDeviceManager graphics;
        SpriteBatch spriteBatch;
        SpriteFont spriteFont;
        SpherePrimitive sphere;
        SpherePrimitive COMsphere;
        SpherePrimitive BodySphere;
        SampleGrid grid;
        SampleGrid grid2;
        Vector3 leftFootInitial;
        Vector3 rightFootInitial;


        protected List<LabelledVector> lines = new List<LabelledVector>();
        Matrix projection;
        int frame = 0;

        public KinectViewer()
        {
            Content.RootDirectory = "Content";
            graphics = new GraphicsDeviceManager(this);
        }

        protected override void LoadContent()
        {
            spriteBatch = new SpriteBatch(GraphicsDevice);

            spriteFont = Content.Load<SpriteFont>(@"SpriteFont1");

            sphere = new SpherePrimitive(GraphicsDevice, 0.5f, 8);

            COMsphere = new SpherePrimitive(GraphicsDevice, 0.5f, 8);
            BodySphere = new SpherePrimitive(GraphicsDevice, 0.4f, 8);
            leftFootInitial = new Vector3();
            rightFootInitial = new Vector3();


            grid = new SampleGrid();
            grid.GridSize = 16;
            grid.GridScale = 1.0f;
            grid.LoadGraphicsContent(GraphicsDevice);
            grid2 = new SampleGrid();
            grid2.GridSize = 16;
            grid2.GridScale = 1.0f;
            grid2.GridColor = Color.Black;
            grid2.LoadGraphicsContent(GraphicsDevice);

            LabelledVector.Load(GraphicsDevice);

            nui.Initialize(//RuntimeOptions.UseColor | RuntimeOptions.UseDepthAndPlayerIndex |
                RuntimeOptions.UseSkeletalTracking);

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
            
            //naoSpeech.Connect("128.208.4.225");
            nao.Connect("127.0.0.1");
            //nao.Connect("128.208.4.225");
        }

        protected virtual void updateSkeleton(SkeletonData skeleton)
        {
            //sc.sendRotationSpeeds(nao.values);
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

                //first time to see a skeleton.
                if (cur_skeleton == null)
                {
                    //set the initial position of the feet
                    //var set = TryInitializeFeetPosition(skeleton);
                    //if (set) //if it succesfully sets the feet
                    //{
                    //    cur_skeleton = skeleton;
                    //}
                    //else
                    //{
                    //    cur_skeleton = null;
                    //}
                }

                //if the initial positions have been set
                //if (cur_skeleton != null)
                //{
                    determineFootElevation(skeleton);
                //}

                //updateSkeleton(skeleton);
                float offset = nao.computeOffsetParam();
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
                Console.WriteLine("init_left: " + leftFootInitial.Y);
                Console.WriteLine("init_right: " + rightFootInitial.Y);
                Console.WriteLine("cur_left: " + cur_left.Y);
                Console.WriteLine("cur_right: " + cur_right.Y);
                if (cur_left.Y - cur_right.Y > .3)
                {
                    Console.WriteLine("your left foot is up");
                }
                else if (cur_left.Y- cur_right.Y < -.3)
                {
                    Console.WriteLine("your right foot is up");
                }
                else
                {
                    Console.WriteLine("both of your feet are on the ground");
                }
            }
            else
            {
                Console.WriteLine("Feet are not tracked");
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


        protected override void Draw(GameTime gameTime)
        {
            
            GraphicsDevice.Clear(Color.CornflowerBlue);

            GraphicsDevice.RasterizerState = RasterizerState.CullCounterClockwise;

            //view = Matrix.CreateLookAt(new Vector3(0, 0, -20), new Vector3(0, 0, 100), Vector3.Up);
            projection = Matrix.CreatePerspectiveFieldOfView(MathHelper.PiOver2,
                                                        GraphicsDevice.Viewport.AspectRatio,
                                                        1.0f,
                                                        100);

            grid.ViewMatrix = viewMatrix;
            grid.ProjectionMatrix = projection;
            grid2.ViewMatrix = viewMatrix;
            grid2.ProjectionMatrix = projection;
            if (cur_skeleton != null)
            {
                Vector3 gpos = getLoc(cur_skeleton.Joints[JointID.Spine]);
                grid.WorldMatrix = Matrix.CreateTranslation(gpos);
                grid2.WorldMatrix = Matrix.Multiply(Matrix.CreateRotationX((float)Math.PI / 2), grid.WorldMatrix);
            }
            grid.Draw();
            grid2.Draw();


            if (nao.connected)
            {
                frame++;
                //display COM (indicated by a green ball.
                nao.PollSensors();
                int foot = 1; 
                nao.Balance(foot, lines);
                if (foot == 2)
                {
                    nao.RKUpdatePitch(0.5f);
                    nao.RHUpdatePitch(-0.5f);
                    nao.LHUpdateRoll(0.73f);
                    nao.LHUpdatePitch(-0.5f);
                    nao.rightFoot.FootLines(lines);
                }
                else
                {
                    float ang = 0.2f;
                    //float ang = (float)Math.Sin(frame / 10) * 0.2f + 0.2f;
                    nao.LKUpdatePitch(ang);
                    nao.LHUpdatePitch(-ang);
                    nao.RHUpdateRoll(-0.73f);
                    nao.RHUpdatePitch(-0.5f);
                    nao.leftFoot.FootLines(lines);
                }
                nao.RSSend();
                //nao.rightFoot.FootLines(lines);
                debugReferenceFrame("", nao.gyrot, 3.0f);
                drawRobot();
                //nao.readFSR();
            }

            spriteBatch.Begin();
            foreach(LabelledVector l in lines) {
                l.Draw(GraphicsDevice, viewMatrix, projection, spriteBatch, spriteFont);
            }
            lines.Clear();

            spriteBatch.End();

            try
            {
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
            catch
            {
            }

            // Reset the fill mode renderstate.
            GraphicsDevice.RasterizerState = RasterizerState.CullCounterClockwise;

            base.Draw(gameTime);
        }

        private void drawRobot()
        {
            drawPrimitive(COMsphere, nao.GetCOM(), Color.Green);
            //drawPrimitive(COMsphere, nao.getGyro(), Color.Red);
            foreach (String part in nao.parts) {
                NaoPos p = nao.GetPosition(part);
                //lines.Add(p.DebugLine(3.0f, Color.Black, ""));
                if (part == "LKneePitch")
                    debugReferenceFrame("", p.transform, 3.0f);
                drawPrimitive(BodySphere, p.position, Color.Blue);
            }
        }

        public void drawPrimitive(GeometricPrimitive p, Vector3 pos, Color c)
        {
            p.Draw(Matrix.CreateTranslation(pos), viewMatrix, projection, c);
        }

        public void debugReferenceFrame(String str, Matrix m, float sz)
        {
            debugReferenceFrame(str, m, sz, m.Translation);
        }
        
        public void debugReferenceFrame(String str, Matrix m, float sz, Vector3 origin)
        {
            lines.Add(new LabelledVector(origin, origin + m.Right * sz, Color.Red, str));
            lines.Add(new LabelledVector(origin, origin + m.Up * sz, Color.Green, ""));
            lines.Add(new LabelledVector(origin, origin + m.Forward * sz, Color.Blue, ""));
        }

        public Vector3 FromKinectSpace(Vector position)
        {
            var returnVector = new Vector3();
            returnVector.X = position.X * 10;
            returnVector.Y = position.Y * 10;
            returnVector.Z = position.Z * 10;
            return returnVector;
        }

        Matrix viewMatrix;

        Vector3 cameraPosition = new Vector3(0,1,-2);
        float leftrightRot = 0;
        float updownRot = -MathHelper.Pi / 10.0f;
        const float rotationSpeed = 0.3f;
        const float moveSpeed = 10.0f;

        protected override void Update(GameTime gameTime)
        {
            if (Keyboard.GetState().IsKeyDown(Keys.Escape))
                this.Exit();


            float timeDifference = (float)gameTime.ElapsedGameTime.TotalMilliseconds / 1000.0f;
            ProcessInput(timeDifference);

            base.Update(gameTime);
        }

        private bool KeyFreshPress(KeyboardState ks, Keys k)
        {
            return ks.IsKeyDown(k) && (prior_keys == null || prior_keys.IsKeyUp(k));
        }

        private void ProcessInput(float amount)
        {
            Vector3 moveVector = new Vector3(0, 0, 0);
            KeyboardState keyState = Keyboard.GetState();
            MouseState currentMouseState = Mouse.GetState();
            if (trap_mouse)
            {
                float xDifference = currentMouseState.X - GraphicsDevice.Viewport.Width / 2;
                float yDifference = currentMouseState.Y - GraphicsDevice.Viewport.Height / 2;
                leftrightRot -= rotationSpeed * xDifference * amount;
                updownRot += rotationSpeed * yDifference * amount;
                Mouse.SetPosition(GraphicsDevice.Viewport.Width / 2, GraphicsDevice.Viewport.Height / 2);
                UpdateViewMatrix();

                if (keyState.IsKeyDown(Keys.Up) || keyState.IsKeyDown(Keys.W) || currentMouseState.LeftButton.HasFlag(ButtonState.Pressed))
                    moveVector -= new Vector3(0, 0, -1);
                if (keyState.IsKeyDown(Keys.Down) || keyState.IsKeyDown(Keys.S) || currentMouseState.RightButton.HasFlag(ButtonState.Pressed))
                    moveVector -= new Vector3(0, 0, 1);
                if (keyState.IsKeyDown(Keys.Right) || keyState.IsKeyDown(Keys.D))
                    moveVector -= new Vector3(1, 0, 0);
                if (keyState.IsKeyDown(Keys.Left) || keyState.IsKeyDown(Keys.A))
                    moveVector -= new Vector3(-1, 0, 0);
                if (keyState.IsKeyDown(Keys.Q))
                    moveVector += new Vector3(0, 1, 0);
                if (keyState.IsKeyDown(Keys.Z))
                    moveVector += new Vector3(0, -1, 0);
                AddToCameraPosition(moveVector * amount);
            }

            if (KeyFreshPress(keyState, Keys.K)) trap_mouse = !trap_mouse;

            if (KeyFreshPress(keyState, Keys.F))
            {
                if (graphics.IsFullScreen)
                {
                    graphics.PreferredBackBufferWidth = 800;
                    graphics.PreferredBackBufferHeight = 600;
                    graphics.IsFullScreen = false;
                }
                else
                {
                    graphics.PreferredBackBufferWidth = 1280;
                    graphics.PreferredBackBufferHeight = 1024;
                    graphics.IsFullScreen = true;
                }
                graphics.ApplyChanges();
            }

            // if (keyState.IsKeyDown(Keys.Escape)) 

            prior_keys = keyState;


        }

        private void AddToCameraPosition(Vector3 vectorToAdd)
        {
            Matrix cameraRotation = Matrix.CreateRotationX(updownRot) * Matrix.CreateRotationY(leftrightRot);
            Vector3 rotatedVector = Vector3.Transform(vectorToAdd, cameraRotation);
            cameraPosition += moveSpeed * rotatedVector;
            UpdateViewMatrix();
        }

        private void UpdateViewMatrix()
        {
            Matrix cameraRotation = Matrix.CreateRotationX(updownRot) * Matrix.CreateRotationY(leftrightRot);

            Vector3 cameraOriginalTarget = new Vector3(0, 0, 100);
            Vector3 cameraOriginalUpVector = new Vector3(0, 1, 0);

            Vector3 cameraRotatedTarget = Vector3.Transform(cameraOriginalTarget, cameraRotation);
            Vector3 cameraFinalTarget = cameraPosition + cameraRotatedTarget;

            Vector3 cameraRotatedUpVector = Vector3.Transform(cameraOriginalUpVector, cameraRotation);

            viewMatrix = Matrix.CreateLookAt(cameraPosition, cameraFinalTarget, cameraOriginalUpVector);
        }

        public static Vector3 getLoc(Joint j) { return getLoc(j.Position); }
        public static Vector3 getLoc(Vector v) { return Vector3.Multiply(new Vector3(v.X, v.Y, v.Z), 10); }
    }
}