using System;
using System.Collections;
using System.Net;
using Bespoke.Common.Osc;

namespace KinectViewer
{
    public class SoundController
    {
        IPEndPoint _sound_source = null;
        IPEndPoint _sound_dest = null;
        ArrayList prior = null;
        float[] speeds = null;

        public SoundController(int dport = 10337, int sport = 10338)
        {
            _sound_source = new IPEndPoint(IPAddress.Loopback, sport);
            _sound_dest = new IPEndPoint(IPAddress.Loopback, dport);

            // Barf
            OscPacket.LittleEndianByteOrder = false;
        }

        public void sendRotationSpeeds(ArrayList values)
        {
            if (prior != null)
            {
                for (int i = 0; i < values.Count; i++)
                {
                    speeds[i] = (speeds[i] + (float)values[i] - (float)prior[i]) / 2;
                    if (float.IsNaN(speeds[i]) || float.IsInfinity(speeds[i])) speeds[i] = 0f;
                    prior[i] = (float)values[i];
                    float val = (Math.Abs((float)speeds[i]) * 10000f);
                    OscMessage msg = new OscMessage(_sound_source, "/motor" + i.ToString(), val);
                    msg.Send(_sound_dest);
                }
            }
            else
            {
                speeds = new float[values.Count];
            }
            prior = (ArrayList) values.Clone();
        }

        public void triggerDing(float freq)
        {
            OscMessage msg = new OscMessage(_sound_source, "/ding", freq);
            msg.Send(_sound_dest);
        }
    }
}