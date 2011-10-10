using Microsoft.Xna.Framework;
using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using Microsoft.Research.Kinect.Audio;
using Microsoft.Research.Kinect.Nui;
using Microsoft.Xna.Framework.Graphics;
using Microsoft.Xna.Framework.Input;
using Microsoft.Xna.Framework.Content;

namespace KinectViewer
{
    class KinectViewer : Microsoft.Xna.Framework.Game
    {
        NaoUpperBody nao = new NaoUpperBody();
        Runtime nui = new Runtime();
        SkeletonData cur_skeleton;

        /*
        bool key = false;
        bool keydownseen = false;
        */

        GraphicsDeviceManager graphics;
        SpriteBatch spriteBatch;
        SpriteFont spriteFont;
        SpherePrimitive sphere;
        SampleGrid grid;
        SampleGrid grid2;
        List<LabelledVector> lines = new List<LabelledVector>();

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

            /*nui.VideoFrameReady += new EventHandler<ImageFrameReadyEventArgs>(nui_VideoFrameReady);
            nui.VideoStream.Open(ImageStreamType.Video, 2,
                ImageResolution.Resolution640x480, ImageType.Color);

            nui.DepthFrameReady += new EventHandler<ImageFrameReadyEventArgs>(nui_DepthFrameReady);
            nui.DepthStream.Open(ImageStreamType.Depth, 2,
                ImageResolution.Resolution320x240, ImageType.DepthAndPlayerIndex);
            */

            nui.SkeletonFrameReady += new EventHandler<SkeletonFrameReadyEventArgs>(nui_SkeletonFrameReady);
            //Must set to true and set after call to Initialize

            nui.SkeletonEngine.TransformSmooth = true;
            //Use to transform and reduce jitter
            var parameters = new TransformSmoothParameters
            {
                Smoothing = 0.75f,
                Correction = 0.0f,
                Prediction = 0.0f,
                JitterRadius = 0.05f,
                MaxDeviationRadius = 0.04f
            };
            nui.SkeletonEngine.SmoothParameters = parameters;

            nao.Connect("127.0.0.1");
        }

        void nui_SkeletonFrameReady(object sender, SkeletonFrameReadyEventArgs e)
        {
            SkeletonFrame allSkeletons = e.SkeletonFrame;

            //get the first tracked skeleton
            SkeletonData skeleton = (from s in allSkeletons.Skeletons
                                     where s.TrackingState == SkeletonTrackingState.Tracked
                                     select s).FirstOrDefault();

            if (skeleton != null)
            {
                cur_skeleton = skeleton;
                // Get joint data

                Joint elbowRight = skeleton.Joints[JointID.ElbowRight],
                      handRight = skeleton.Joints[JointID.HandRight],
                      shoulderRight = skeleton.Joints[JointID.ShoulderRight],
                      shoulderLeft = skeleton.Joints[JointID.ShoulderLeft],
                      shoulderCenter = skeleton.Joints[JointID.ShoulderCenter],
                      spine = skeleton.Joints[JointID.Spine],
                      elbowLeft = skeleton.Joints[JointID.ElbowLeft];

                lines.Clear();

                //Vector3D s_e = between(shoulderRight, elbowRight),
                //         e_h = between(elbowRight, handRight); 

                //double angle = Vector3D.AngleBetween(s_e, e_h);
                //nao.REUpdateRoll(angle);

                //Matrix3D bodyRef = makeReferenceFrame(getLoc(shoulderCenter), getLoc(shoulderLeft), getLoc(spine));
                /*
                Vector3D dx = between(shoulderLeft, shoulderRight);
                Vector3D dy = between(spine, shoulderCenter);
                dx.Normalize(); dy.Normalize();
                Vector3D dz = Vector3D.CrossProduct(dx, dy);
                Console.WriteLine(dz.ToString());

                Matrix3D shoulderFrame = makeMatrix(dx, dy, dz, getLoc(shoulderRight));
                shoulderFrame.Invert();

                Vector3D elbowRightLocal = shoulderFrame.Transform(getLoc(elbowRight));
                Console.WriteLine(elbowRightLocal.ToString());

                //Console.WriteLine(s_e.ToString());
                //Console.WriteLine(s_e_local.ToString());
                //Canvas.SetLeft(ellipse4, );
                double a1 = Math.Atan2(elbowRightLocal.Y, elbowRightLocal.Z);
                double a2 = Math.Atan2(elbowRightLocal.X, elbowRightLocal.Y);
                (*/
                /*
                double a1 = Vector3D.AngleBetween(new Vector3D(0, elbowRightLocal.Y, elbowRightLocal.Z), new Vector3D(0, 0, -1)),
                       a2 = Vector3D.AngleBetween(new Vector3D(elbowRightLocal.X, elbowRightLocal.Y, 0), new Vector3D(0, -1, 0));
                 */
                //setAngle(line_ang1, a1, 20);
                //setAngle(line_ang2, a2, 20);
                /*
                Console.WriteLine("LeftShoulderPitch angle: " + a1.ToString());
                Console.WriteLine("LeftShoulderRoll angle: " + a2.ToString());
                nao.LSUpdatePitch(a1);
                nao.LSUpdateRoll(a2);
                */
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

        /*
        void nui_VideoFrameReady(object sender, ImageFrameReadyEventArgs e)
        {
            PlanarImage image = e.ImageFrame.Image;
            image1.Source = BitmapSource.Create(image.Width, image.Height,
                96, 96, PixelFormats.Bgr32, null, image.Bits, image.Width * image.BytesPerPixel);
        }

        void nui_DepthFrameReady(object sender, ImageFrameReadyEventArgs e)
        {
            //Convert depth information for a pixel into color information
            byte[] ColoredBytes = GenerateColoredBytes(e.ImageFrame);

            //create an image based on returned colors

            PlanarImage image = e.ImageFrame.Image;
            image2.Source = BitmapSource.Create(image.Width, image.Height, 96, 96, PixelFormats.Bgr32, null,
                ColoredBytes, image.Width * PixelFormats.Bgr32.BitsPerPixel / 8);
        }
        */

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

        private void ProcessInput(float amount)
        {
            MouseState currentMouseState = Mouse.GetState();
            float xDifference = currentMouseState.X - GraphicsDevice.Viewport.Width / 2;
            float yDifference = currentMouseState.Y - GraphicsDevice.Viewport.Height / 2;
            leftrightRot -= rotationSpeed * xDifference * amount;
            updownRot += rotationSpeed * yDifference * amount;
            Mouse.SetPosition(GraphicsDevice.Viewport.Width / 2, GraphicsDevice.Viewport.Height / 2);
            UpdateViewMatrix();

            Vector3 moveVector = new Vector3(0, 0, 0);
            KeyboardState keyState = Keyboard.GetState();
            if (keyState.IsKeyDown(Keys.Up) || keyState.IsKeyDown(Keys.W))
                moveVector -= new Vector3(0, 0, -1);
            if (keyState.IsKeyDown(Keys.Down) || keyState.IsKeyDown(Keys.S))
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

        private Vector3 getLoc(Joint j) { return getLoc(j.Position); }
        private Vector3 getLoc(Vector v) { return Vector3.Multiply(new Vector3(v.X, v.Y, v.Z), 10); }

        /*
        private Vector3 cvtVec(Vector3D v) { return new Vector3((float)v.X * 10, (float)v.Y * 10, (float)v.Z * 10); }


        // TODO: is this in homogenous coords?

        private Vector3D between(Joint j1, Joint j2)
        {
            return Vector3D.Subtract(getLoc(j2), getLoc(j1));
        }

        private Matrix3D makeReferenceFrame(Vector3D c, Vector3D x, Vector3D y)
        {
            Vector3D dx = Vector3D.Subtract(x, c), dy = Vector3D.Subtract(y, c);
            dx.Normalize(); dy.Normalize();
            Vector3D dz = Vector3D.CrossProduct(dx, dy);
            return makeMatrix(dx, dy, dz, c);
        }

        private Matrix3D makeMatrix(Vector3D dx, Vector3D dy, Vector3D dz, Vector3D c)
        {
            return new Matrix3D(dx.X, dx.Y, dx.Z, 0,
                                dy.X, dy.Y, dy.Z, 0,
                                dz.X, dz.Y, dz.Z, 0,
                                 c.X, c.Y, c.Z, 1);
        }
         */
    }
}