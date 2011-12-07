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
    class Viewer : Microsoft.Xna.Framework.Game
    {
        bool trap_mouse = true;
        KeyboardState prior_keys;

        protected GraphicsDeviceManager graphics;
        protected Color clearColor = Color.Black;
        protected SpriteBatch spriteBatch;
        protected SpriteFont spriteFont;
        protected SampleGrid grid;
        protected SampleGrid grid2;
        protected Vector3 gridOrigin;

        protected List<LabelledVector> lines = new List<LabelledVector>();
        protected Dictionary<Keys, Action> keyHandlers = new Dictionary<Keys, Action>();

        protected int frame = 0;

        protected Matrix projection;
        protected Matrix viewMatrix;

        Vector3 cameraPosition = new Vector3(-5f, -2f, -6f);
        float leftrightRot = 0;
        float updownRot = 0;
        const float rotationSpeed = 0.3f;
        const float moveSpeed = 10.0f;

        static Viewer instance;

        // Constructor / Initializer
        // ====================================================================

        public Viewer()
        {
            Content.RootDirectory = "Content";
            graphics = new GraphicsDeviceManager(this);
            instance = this;
            debugOrigin = Vector3.Zero;
        }

        protected override void LoadContent()
        {
            spriteBatch = new SpriteBatch(GraphicsDevice);
            spriteFont = Content.Load<SpriteFont>(@"SpriteFont1");

            grid = new SampleGrid();
            grid.GridSize = 16;
            grid.GridScale = 1.0f;
            grid.GridColor = Color.DarkGreen;
            grid.LoadGraphicsContent(GraphicsDevice);
            grid2 = new SampleGrid();
            grid2.GridSize = 16;
            grid2.GridScale = 1.0f;
            grid2.GridColor = Color.Green;
            grid2.LoadGraphicsContent(GraphicsDevice);

            LabelledVector.Load(GraphicsDevice);
        }


        // Convenient debugging utilities
        // ====================================================================

        public void drawPrimitive(GeometricPrimitive p, Vector3 pos, Color c)
        {
            p.Draw(Matrix.CreateTranslation(pos), viewMatrix, projection, c);
        }

        public static Vector3 debugOrigin { get; set; }

        public static void DebugReferenceFrameAtOrigin(String str, Matrix m) {
            DebugReferenceFrame(str, m, 3.0f, debugOrigin);
        }

        public static void DebugReferenceFrame(String str, Matrix m)
        {
            DebugReferenceFrame(str, m, 3.0f, m.Translation);
        }

        public static void DebugReferenceFrame(String str, Matrix m, float sz, Vector3 origin)
        {
            instance.lines.Add(new LabelledVector(origin, origin + m.Right * sz, Color.Red, str));
            instance.lines.Add(new LabelledVector(origin, origin + m.Up * sz, Color.Green, ""));
            instance.lines.Add(new LabelledVector(origin, origin + m.Forward * sz, Color.Blue, ""));
        }

        public static void DebugVector(String str, Vector3 vec, Color c)
        {
            DebugVector(str, debugOrigin, vec, c);
        }

        public static void DebugVector(String str, Vector3 orig, Vector3 vec, Color c)
        {
            instance.lines.Add(new LabelledVector(orig, Vector3.Add(orig, vec), c, str));
        }

        // Method subclasses overload to actually draw interesting things.
        protected virtual void DrawStuff()
        {

        }

        protected override void Draw(GameTime gameTime)
        {
            GraphicsDevice.Clear(clearColor);

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
            grid.WorldMatrix = Matrix.CreateTranslation(gridOrigin);
            grid2.WorldMatrix = Matrix.Multiply(Matrix.CreateRotationX((float)Math.PI / 2), grid.WorldMatrix);
            grid.Draw();
            grid2.Draw();
            
            spriteBatch.Begin();
            LabelledVector[] lines2 = new LabelledVector[lines.Count];
            lines.CopyTo(lines2);
            foreach (LabelledVector l in lines2)
            {
                l.Draw(GraphicsDevice, viewMatrix, projection, spriteBatch, spriteFont);
            }

            spriteBatch.End();

            DrawStuff();

            // Reset the fill mode renderstate.
            GraphicsDevice.RasterizerState = RasterizerState.CullCounterClockwise;

            base.Draw(gameTime);
        }
        
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
                foreach (KeyValuePair<Keys, Action> entry in keyHandlers)
                {
                    if (keyState.IsKeyDown(entry.Key)) entry.Value.Invoke();
                }

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

        protected void AddKeyHandler(Keys k, Action handler)
        {
            keyHandlers.Add(k, handler);
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

    }
}