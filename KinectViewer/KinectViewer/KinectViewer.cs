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
        protected NaoUpperBody nao = new NaoUpperBody();
        protected NaoSpeech naoSpeech = new NaoSpeech();
        Runtime nui = new Runtime();
        SkeletonData cur_skeleton;
        const int WINDOW_SIZE = 200;
        LinkedList<double[]> motion_window = new LinkedList<double[]>();
        Vector3 skeletonStartPos;
        HMMClassifier[] classifiers;
        double[] classifier_probs;
        String recordFile;
        protected SpeechRecognition sr = new SpeechRecognition();

        protected double[][] naoPerformMotionData = null;
        protected bool naoPerformMotion = false;
        protected int naoPerformLineNum = 0;

        bool trap_mouse = true;
        bool record_ang = true;
        KeyboardState prior_keys;
        System.IO.StreamWriter recording = null;
        
        GraphicsDeviceManager graphics;
        SpriteBatch spriteBatch;
        SpriteFont spriteFont;
        SpherePrimitive sphere;
        SampleGrid grid;
        SampleGrid grid2;
        protected List<LabelledVector> lines = new List<LabelledVector>();

        public KinectViewer()
        {
            Content.RootDirectory = "Content";
            graphics = new GraphicsDeviceManager(this);
            InitializeClassifiers();
            //graphics.PreferredBackBufferWidth = 1280;
            //graphics.PreferredBackBufferHeight = 1024;
            //graphics.IsFullScreen = true;
        }

        protected override void LoadContent()
        {
            spriteBatch = new SpriteBatch(GraphicsDevice);

            spriteFont = Content.Load<SpriteFont>(@"SpriteFont1");

            sphere = new SpherePrimitive(GraphicsDevice, 0.5f, 8);

            grid = new SampleGrid();
            grid.GridSize = 16;
            grid.GridScale = 1.0f;
            grid.LoadGraphicsContent(GraphicsDevice);
            grid2 = new SampleGrid();
            grid2.GridSize = 16;
            grid2.GridScale = 1.0f;
            grid2.GridColor = Color.BlueViolet;
            grid2.LoadGraphicsContent(GraphicsDevice);

            LabelledVector.Load(GraphicsDevice);

            nui.Initialize(RuntimeOptions.UseColor | RuntimeOptions.UseDepthAndPlayerIndex
                | RuntimeOptions.UseSkeletalTracking);

            nui.SkeletonFrameReady += new EventHandler<SkeletonFrameReadyEventArgs>(nui_SkeletonFrameReady);
            // Must set to true and set after call to Initialize

            nui.SkeletonEngine.TransformSmooth = true;
            // Use to transform and reduce jitter
            var parameters = new TransformSmoothParameters
            {
                Smoothing = 0.75f,
                Correction = 0.0f,
                Prediction = 0.0f,
                JitterRadius = 0.05f,
                MaxDeviationRadius = 0.04f
            };
            nui.SkeletonEngine.SmoothParameters = parameters;
            
            //nao.Connect("128.208.7.48");
            //nao.Connect("128.208.4.10");
            nao.Connect("127.0.0.1");
            //naoSpeech.Connect("127.0.0.1");
            sr.InitalizeKinect(nao, naoSpeech, this);
        }

        protected virtual void updateSkeleton(SkeletonData skeleton)
        {
            if (recording != null)
            {
                if (record_ang)
                {
                    nao.RecordAngles(recording);
                }
                else
                {
                    Vector3 pos = getLoc(cur_skeleton.Joints[JointID.Spine]);
                    recording.WriteLine(DateTime.Now.ToFileTime().ToString() + ", " + (skeletonStartPos.X - pos.X)
                                                                             + ", " + (skeletonStartPos.Y - pos.Y)
                                                                             + ", " + (skeletonStartPos.Z - pos.Z));
                }
            }
            else
            {
                double[] sample = new double[nao.values.Count];
                Type typ = nao.values[0].GetType();
                if (typ == typeof(float))
                {
                    for (int i = 0; i < sample.Count(); i++)
                    {
                        sample[i] = (double)(float)nao.values[i];
                    }
                    motion_window.AddLast(sample);
                    if (motion_window.Count > WINDOW_SIZE) motion_window.RemoveFirst();
                    if (classifier_probs == null) classifier_probs = new double[classifiers.Length];
                    if (motion_window.Count >= WINDOW_SIZE)
                    {
                        for (int i = 0; i < classifiers.Length; i++)
                        {
                            // TODO: this is sorta inefficient..
                            classifier_probs[i] = classifiers[i].evaluate(motion_window.ToArray());
                        }
                    }
                }
                else
                {
                    Console.WriteLine("hrmph");
                }
            }
        }

        protected virtual void naoPerformAction()
        {
        }

        void nui_SkeletonFrameReady(object sender, SkeletonFrameReadyEventArgs e)
        {
            SkeletonFrame allSkeletons = e.SkeletonFrame;

            // Get the first tracked skeleton
            SkeletonData skeleton = (from s in allSkeletons.Skeletons
                                     where s.TrackingState == SkeletonTrackingState.Tracked
                                     select s).FirstOrDefault();

            if (naoPerformMotion)
            {
                naoPerformAction();
            }
            else if (skeleton != null)
            {
                cur_skeleton = skeleton;
                updateSkeleton(skeleton);
            }
        }

        protected override void Draw(GameTime gameTime)
        {
            GraphicsDevice.Clear(Color.CornflowerBlue);

            GraphicsDevice.RasterizerState = RasterizerState.CullCounterClockwise;

            Matrix view = Matrix.CreateLookAt(new Vector3(0, 0, -20), new Vector3(0, 0, 100), Vector3.Up);
            Matrix projection = Matrix.CreatePerspectiveFieldOfView(MathHelper.PiOver2,
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

            spriteBatch.Begin();
            foreach(LabelledVector l in lines) {
                l.Draw(GraphicsDevice, viewMatrix, projection, spriteBatch, spriteFont);
            }
            if (classifier_probs != null)
            {
                for (int i = 0; i < classifier_probs.Length; i++)
                {
                    double prob = classifier_probs[i];
                    spriteBatch.DrawString(spriteFont, classifiers[i].getName() + " " + prob.ToString(),
                        new Vector2((float)(prob * 640.0f), (float)(i * 20.0f + 20)), Color.Black);
                }
            }
            spriteBatch.End();

            try
            {
                if (cur_skeleton != null)
                {
                    if (cur_skeleton.TrackingState == SkeletonTrackingState.Tracked)
                    {
                        foreach (Joint joint in cur_skeleton.Joints)
                        {
                            var position = ConvertRealWorldPoint(joint.Position);
                            sphere.Draw(Matrix.CreateTranslation(position), viewMatrix, projection, Color.Red);
                        }
                    }
                }
            }
            catch
            {
                // lol
            }

            // Reset the fill mode renderstate.
            GraphicsDevice.RasterizerState = RasterizerState.CullCounterClockwise;

            /*
            // Draw overlay text.
            string text = "A or tap top of screen = Change primitive\n" +
                          "B or tap bottom left of screen = Change color\n" +
                          "Y or tap bottom right of screen = Toggle wireframe";

            spriteBatch.Begin();
            spriteBatch.DrawString(spriteFont, text, new Vector2(48, 48), Color.White);
            spriteBatch.End();
            */
            base.Draw(gameTime);
        }

        private Vector3 ConvertRealWorldPoint(Vector position)
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
            MouseState currentMouseState = Mouse.GetState();
            if (trap_mouse)
            {
                float xDifference = currentMouseState.X - GraphicsDevice.Viewport.Width / 2;
                float yDifference = currentMouseState.Y - GraphicsDevice.Viewport.Height / 2;
                leftrightRot -= rotationSpeed * xDifference * amount;
                updownRot += rotationSpeed * yDifference * amount;
                Mouse.SetPosition(GraphicsDevice.Viewport.Width / 2, GraphicsDevice.Viewport.Height / 2);
                UpdateViewMatrix();
            }

            Vector3 moveVector = new Vector3(0, 0, 0);
            KeyboardState keyState = Keyboard.GetState();

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
            }

            bool rPressed = KeyFreshPress(keyState, Keys.R);
            if (KeyFreshPress(keyState, Keys.T) || rPressed)
            {
                record_ang = rPressed;
                skeletonStartPos = getLoc(cur_skeleton.Joints[JointID.Spine]);
                if (recording == null)
                {
                    System.IO.Directory.CreateDirectory("saved");
                    //recording = System.IO.File.Create("saved/" + DateTime.Now.ToString() + ".rec");
                    recordFile = System.IO.Path.Combine(System.IO.Directory.GetCurrentDirectory(), "saved/" + DateTime.Now.ToFileTime().ToString() + ".rec");
                    recording = new System.IO.StreamWriter(recordFile);            
                }
                else
                {
                    recording.Close();
                    double[][] motion = HMMClassifier.getMotion(recordFile);
                    ClassifyMotion(motion);
                    recording = null;
                }
            }

            prior_keys = keyState;

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

        public Vector3 getLoc(Joint j) { return getLoc(j.Position); }
        public Vector3 getLoc(Vector v) { return Vector3.Multiply(new Vector3(v.X, v.Y, v.Z), 10); }

        private void InitializeClassifiers()
        {
            String directory = Directory.GetCurrentDirectory();
            String motion_dir = directory + "\\motion_data";
            if (!Directory.Exists(motion_dir)) Directory.CreateDirectory(motion_dir);
            String[] directories = Directory.GetDirectories(motion_dir);
            this.classifiers = new HMMClassifier[directories.Length];
            for (int i = 0; i < directories.Length; i++)
            {
                classifiers[i] = new HMMClassifier();
                classifiers[i].Initialize(directories[i]);
            }
            Console.WriteLine("Initialized " + classifiers.Length + " classifiers.");
        }

        private void ClassifyMotion(double[][] motion)
        {
            Console.WriteLine("Classifying...");
            for (int i = 0; i < classifiers.Length; i++)
            {
                if (classifiers[i].isMember(motion)) 
                    Console.WriteLine("Recognized: " + classifiers[i].getName());
            }
            Console.WriteLine("Done");
        }

        public void performAction(string action)
        {
            double[][] performMotion = null;
            for (int i = 0; i < classifiers.Length; i++)
            {
                if (classifiers[i].getName() == action)
                    performMotion = classifiers[i].getPerformMotion();
            }
            if (performMotion != null)
            {
                naoPerformMotionData = performMotion;
                naoPerformMotion = true;
            }
        }
    }
}