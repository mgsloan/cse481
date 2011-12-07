using System;

namespace KinectViewer
{
#if WINDOWS || XBOX
    static class Program
    {
        /// <summary>
        /// The main entry point for the application.
        /// </summary>
        static void Main(string[] args)
        {
            //comment out the version you do not want to use. 
            //KinectAngleViewer game = new KinectAngleViewer();
            //game.Run();

            KinectAngleViewer game2 = new KinectAngleViewer();
            Console.WriteLine("using KV1");
                game2.Run();
        }
    }
#endif
}