using System;
using System.Net;
using Bespoke.Common.Osc;

namespace KinectViewer
{
    public class SoundController
    {
        IPEndPoint _sound_source = null;
        IPEndPoint _sound_dest = null;
        double[] prior = null;
        double[] speeds = null;

        public SoundController(int dport = 10337, int sport = 10338)
        {
            _sound_source = new IPEndPoint(IPAddress.Loopback, sport);
            _sound_dest = new IPEndPoint(IPAddress.Loopback, dport);

            // Barf
            OscPacket.LittleEndianByteOrder = false;
        }

        public void sendRotationSpeeds(double[] values)
        {
            if (prior != null)
            {
                for (int i = 0; i < values.Length; i++)
                {
                    speeds[i] = ((float)speeds[i] + (float)values[i] - (float)prior[i]) / 2;
                    if (float.IsNaN((float)speeds[i]) || float.IsInfinity((float)speeds[i])) speeds[i] = 0f;
                    prior[i] = values[i];
                    float val = (Math.Abs((float)speeds[i]) * 10000f);
                    OscMessage msg = new OscMessage(_sound_source, "/motor" + i.ToString(), val);
                    msg.Send(_sound_dest);
                }
            }
            else
            {
                speeds = new double[values.Length];
            }
            prior = (double[]) values.Clone();
        }

        public void triggerDing()
        {
            OscMessage msg = new OscMessage(_sound_source, "/ding", 1);
            msg.Send(_sound_dest);
        }
    }
}