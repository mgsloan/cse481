using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Windows.Controls;
using System.Windows.Data;
using System.Windows.Documents;
using System.Windows.Input;
using System.Windows.Media;
using System.Windows.Media.Imaging;
using System.Windows.Navigation;
using System.Windows.Shapes;
using Microsoft.Research.Kinect.Audio;
using Microsoft.Research.Kinect.Nui;
using System.Windows.Media.Media3D;

namespace WpfApplication2
{
    /// <summary>
    /// Interaction logic for MainWindow.xaml
    /// </summary>
    public partial class MainWindow : System.Windows.Window
    {
        ElbowTest elbow = new ElbowTest();

        public MainWindow()
        {
            try {
                InitializeComponent();
            } catch (Exception e) {
                Console.Out.WriteLine("Window exception: " + e);
            }
        }

        Runtime nui = new Runtime();
        private void Window_Loaded(object sender, System.Windows.RoutedEventArgs e)
        {
            nui.Initialize(RuntimeOptions.UseColor | RuntimeOptions.UseDepthAndPlayerIndex 
                | RuntimeOptions.UseSkeletalTracking);

            nui.VideoFrameReady += new EventHandler<ImageFrameReadyEventArgs>(nui_VideoFrameReady);
            nui.VideoStream.Open(ImageStreamType.Video, 2,
                ImageResolution.Resolution640x480, ImageType.Color);

            nui.DepthFrameReady += new EventHandler<ImageFrameReadyEventArgs>(nui_DepthFrameReady);
            nui.DepthStream.Open(ImageStreamType.Depth, 2,
                ImageResolution.Resolution320x240, ImageType.DepthAndPlayerIndex);

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

            elbow.Connect("127.0.0.1");
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
                // Get joint data

                Joint elbowRight = skeleton.Joints[JointID.ElbowRight],
                      handRight = skeleton.Joints[JointID.HandRight],
                      shoulderRight = skeleton.Joints[JointID.ShoulderRight],
                      shoulderLeft = skeleton.Joints[JointID.ShoulderLeft],
                      shoulderCenter = skeleton.Joints[JointID.ShoulderCenter],
                      spine = skeleton.Joints[JointID.Spine],
                      elbowLeft = skeleton.Joints[JointID.ElbowLeft];

                SetEllipsePosition(ellipse1, shoulderRight);
                SetEllipsePosition(ellipse2, elbowRight);
                SetEllipsePosition(ellipse3, handRight);

                Vector3D se = between(shoulderRight, elbowRight),
                         eh = between(elbowRight, handRight); 

                double angle = Vector3D.AngleBetween(se, eh);
                elbow.UpdateYaw(angle);
                
                //Matrix3D bodyRef = makeReferenceFrame(getLoc(shoulderCenter), getLoc(shoulderLeft), getLoc(spine));
                Vector3D dx = between(shoulderLeft, shoulderRight);
                Vector3D dy = between(spine, shoulderCenter);
                dx.Normalize(); dy.Normalize();
                Vector3D dz = Vector3D.CrossProduct(dx, dy);

                Matrix3D bodyRef = makeMatrix(dx, dy, dz, new Vector3D(0, 0, 0));
                Matrix3D bodyRef2 = makeMatrix(dx, dy, dz, new Vector3D(0, 0, 0));
                bodyRef.Invert();

                Vector3D sel = bodyRef.Transform(se);
                
                double a1 = Vector3D.AngleBetween(new Vector3D(0, sel.Y, sel.Z), new Vector3D(0,-1,0)),
                       a2 = Vector3D.AngleBetween(new Vector3D(sel.X, sel.Y, 0), new Vector3D(0,-1,0));

                elbow.updateLeftShoulderPitch(a1);
                elbow.updateLeftShoulderRoll(a2);


                Vector3D other = new Vector3D(sel.X, 0, sel.Z);
                Matrix3D upperRightRef = makeReferenceFrame(bodyRef.Transform(between(spine, shoulderRight)),
                                                            other,
                                                            sel);

                Matrix3D upperRightInv = Matrix3D.Multiply(bodyRef2, upperRightRef);
                upperRightInv.Invert();

                Vector3D handLoc = upperRightInv.Transform(getLoc(handRight));
                handLoc.Y = 0;
                
                double a3 = Vector3D.AngleBetween(handLoc, new Vector3D(1, 0, 0));
                Console.WriteLine(a3.ToString());

                /*
                Vector3D bodyVec = getVec(spine, shoulderCenter),
                         leftBicep = getVec(elbowLeft, shoulderLeft),
                         shoulderVec = getVec(shoulderRight, shoulderLeft);
                
                // plane = (p3 - p1) x (p2 - p1) -- NOTE: not using same p1...
                Vector3D bodyPlane = Vector3D.CrossProduct(shoulderVec, bodyVec);

                //A || B = B × (A × B) / |B|²
                Vector3D bicepProjection = Vector3D.CrossProduct(bodyPlane, Vector3D.CrossProduct(bodyPlane, Leftbicep)) / Vector3D.DotProduct(bodyPlane, bodyPlane);
                double leftShoulderAngle = Vector3D.AngleBetween(bicepProjection, bodyVec);

                //elbow.updateLeftShoulderRoll(leftShoulderAngle);


                //A || B = B × (A × B) / |B|²
                // B = shoulderVec, A = Leftbicep
                Vector3D bicepProjection2 = Vector3D.CrossProduct(shoulderVec, Vector3D.CrossProduct(shoulderVec, Leftbicep)) / Vector3D.DotProduct(shoulderVec, shoulderVec);
                double leftShoulderAngle2 = Vector3D.AngleBetween(bicepProjection2, bodyVec);

                elbow.updateLeftShoulderPitch(leftShoulderAngle2);
                */
            }
        }

        private void SetEllipsePosition(System.Windows.FrameworkElement ellipse, Joint joint)
        {
            float x = joint.Position.X;
            float scaledX = (float) (x + .5) * 320;
            float y = joint.Position.Y;
            float scaledY = 240 - (float) (y + .5) * 240;

            Canvas.SetLeft(ellipse, scaledX);
            Canvas.SetTop(ellipse, scaledY);
        }

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

        private byte[] GenerateColoredBytes(ImageFrame imageFrame)
        {
            int height = imageFrame.Image.Height;
            int width = imageFrame.Image.Width;

            //Depth data for each pixel
            Byte[] depthData = imageFrame.Image.Bits;

            //colorFrame contains color information for all pixels in image
            //Height x Width x 4 (Red, Green, Blue, empty byte)
            Byte[] colorFrame = new byte[imageFrame.Image.Height * imageFrame.Image.Width * 4];

            //Bgr32  - Blue, Green, Red, empty byte
            //Bgra32 - Blue, Green, Red, transparency 
            //You must set transparency for Bgra as .NET defaults a byte to 0 = fully transparent

            //hardcoded locations to Blue, Green, Red (BGR) index positions       
            const int BlueIndex = 0;
            const int GreenIndex = 1;
            const int RedIndex = 2;

            var depthIndex = 0;
            for (var y = 0; y < height; y++)
            {

                var heightOffset = y * width;

                for (var x = 0; x < width; x++)
                {

                    var index = ((width - x - 1) + heightOffset) * 4;
                    var distance = GetDistanceWithPlayerIndex(depthData[depthIndex], depthData[depthIndex + 1]);

                    //equal coloring for monochromatic histogram
                    var intensity = CalculateIntensityFromDepth(distance);
                    colorFrame[index + BlueIndex] = intensity;
                    colorFrame[index + GreenIndex] = intensity;
                    colorFrame[index + RedIndex] = intensity;

                    //Color a player
                    if (GetPlayerIndex(depthData[depthIndex]) > 0)
                    {
                        //we are the farthest
                        colorFrame[index + BlueIndex] = 0;
                        colorFrame[index + GreenIndex] = 255;
                        colorFrame[index + RedIndex] = 0;
                    }

                    //jump two bytes at a time
                    depthIndex += 2;
                }
            }

            return colorFrame;
        }

        private static int GetPlayerIndex(byte firstFrame)
        {
            //returns 0 = no player, 1 = 1st player, 2 = 2nd player...
            //bitwise & on firstFrame
            return (int) firstFrame & 7;
        }

        private int GetDistanceWithPlayerIndex(byte firstFrame, byte secondFrame)
        {
            //offset by 3 in first byte to get value after player index 
            int distance = (int)(firstFrame >> 3 | secondFrame << 5);
            return distance;
        }

        const float MaxDepthDistance = 4000; // max value returned
        const float MinDepthDistance = 850; // min value returned
        const float MaxDepthDistanceOffset = MaxDepthDistance - MinDepthDistance;

        public static byte CalculateIntensityFromDepth(int distance)
        {
            //formula for calculating monochrome intensity for histogram
            return (byte)(255 - (255 * Math.Max(distance - MinDepthDistance, 0)
                / (MaxDepthDistanceOffset)));
        }

        private void Window_Closed(object sender, EventArgs e)
        {
            nui.Uninitialize();
        }

        private Vector3D getLoc(Joint j) { return getLoc(j.Position); }

        // TODO: is this in homogenous coords?
        private Vector3D getLoc(Vector v) { return new Vector3D(v.X, v.Y, v.Z); }

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
                                 c.X,  c.Y,  c.Z, 1);
        }
    }
}
