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
            using (KinectViewer game = new KinectViewer())
            {
                game.Run();
            }
        }
    }
#endif
}

