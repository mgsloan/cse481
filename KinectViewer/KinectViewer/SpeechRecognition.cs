using Microsoft.Speech.Recognition;
using Microsoft.Speech.AudioFormat;
using Microsoft.Research.Kinect.Audio;
using Microsoft.Research.Kinect.Nui;
using System.IO;
using System;
using System.Linq;
using System.Collections.Generic;

namespace KinectViewer
{
    class SpeechRecognition 
    {
        NaoUpperBody nao;
        NaoSpeech naoSpeech;
        KinectAudioSource kinectSource;
        SpeechRecognitionEngine naoListener;
        Stream stream;
        string RecognizerId = "SR_MS_en-US_Kinect_10.0";


        public void InitalizeKinect(NaoUpperBody naoBody, NaoSpeech naoSpeaker)
        {
            nao = naoBody;
            naoSpeech = naoSpeaker;
            kinectSource = new KinectAudioSource();
            kinectSource.FeatureMode = true;
            kinectSource.AutomaticGainControl = false;
            kinectSource.SystemMode = SystemMode.OptibeamArrayOnly;

            var rec = (from r in SpeechRecognitionEngine.InstalledRecognizers() where r.Id == RecognizerId select r).FirstOrDefault();

            naoListener = new SpeechRecognitionEngine(rec.Id);

            /*
            SemanticResultValue forward = new SemanticResultValue("Walk forward");
            SemanticResultValue back = new SemanticResultValue("Walk back");
            SemanticResultValue left = new SemanticResultValue("Walk left");
            SemanticResultValue right = new SemanticResultValue("Walk right");
            SemanticResultValue goodbye = new SemanticResultValue("Wave Goodbye");
            SemanticResultValue dance = new SemanticResultValue("Do the Robot");
            SemanticResultValue superman = new SemanticResultValue("Superman");

            Choices actions = new Choices();
            actions.Add(new Choices(new GrammarBuilder[] { forward, back, left, right, goodbye, superman, dance }));
            */
            Choices actions = new Choices(new string[] {",walk forward", ",walk back", ",walk left", ",walk right", ",wave goodbye", ",superman", ",do the robot"});
            GrammarBuilder gb = new GrammarBuilder("Nao");
            //gb.Append(new SemanticResultKey("actions", actions));
            gb.Append(actions, 0, 5);

            gb.Culture = rec.Culture;

            GrammarBuilder dictation = new GrammarBuilder();
            dictation.AppendDictation("new actions");




            var g = new Grammar(gb);
            naoListener.LoadGrammar(g);

          
            naoListener.SpeechRecognized += new EventHandler<SpeechRecognizedEventArgs>(SreSpeechRecognized);
           

            stream = kinectSource.Start();
            naoListener.SetInputToAudioStream(stream,
                          new SpeechAudioFormatInfo(
                              EncodingFormat.Pcm, 16000, 16, 1,
                              32000, 2, null));

  
            naoListener.RecognizeAsync(RecognizeMode.Multiple);
        }


        void SreSpeechRecognized(object sender, SpeechRecognizedEventArgs e)
        {
            string[] phrases = e.Result.Text.Split(',');
            foreach (string word in phrases)
            {
                Console.WriteLine(word);
            }
            if (phrases.GetLength(0) > 2)
            {
                // new sequence
            }
            Console.WriteLine("Action: " + e.Result.Text);
            Console.WriteLine("Confidence Level: " + e.Result.Confidence);
            if (e.Result.Confidence > .95)
            {
                switch (phrases.ToList()[1])
                {
                    case "walk forward":
                        Console.WriteLine("Yes, I can move forward");
                        nao.walk("forward");
                        break;
                    case "walk left":
                        nao.walk("left");
                        Console.WriteLine("Yes, I can move left");
                        break;
                    case "walk right":
                        nao.walk("right");
                        Console.WriteLine("Yes, I can move right");
                        break;
                    case "walk back":
                        nao.walk("back");
                        Console.WriteLine("Yes, I can move back");
                        break;
                    case "superman":
                        Console.WriteLine("Yes, I can fly");
                        break;
                    case "do the robot":
                        Console.WriteLine("Yes, I can dance!");
                        break;
                    case "wave goodbye":
                        Console.WriteLine("Goodbye");
                        break;
                    default:
                        Console.WriteLine("I don’t know how to do that");
                        break;
                }
            }
          

        }
      
    }


}