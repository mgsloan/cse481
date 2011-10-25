using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using Aldebaran.Proxies;

namespace KinectViewer
{
    class NaoSpeech
    {
        TextToSpeechProxy _speak = null;

        public void Connect(string ip)
        {
            try
            {
                _speak = new TextToSpeechProxy(ip, 9559);
            }
            catch (Exception e)
            {
                Console.Out.WriteLine("Speech.Connect exception: " + e);
            }
        }

        public void Say(string sentance)
        {
            if (_speak != null)
            {
                _speak.say(sentance);
            }
        }

        public void UnrecognizedAction(string sentance = "I don't know what you're doing.")
        {
            if (_speak != null)
            {
                _speak.say(sentance);
            }
        }

    }
}
