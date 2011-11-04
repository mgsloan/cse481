using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.IO;
using Microsoft.Xna.Framework;

namespace KinectViewer
{
    
class MotionRecord
{
    public LinkedList<Tuple<long, double[]>> data = new LinkedList<Tuple<long, double[]>>();

    public void TakeAngleSample(NaoUpperBody nao, int limit = -1)
    {
        Type typ = nao.values[0].GetType();
        if (typ == typeof(float))
        {
            double[] sample = new double[nao.values.Count];
            for (int i = 0; i < sample.Count(); i++)
            {
                sample[i] = (double)(float)nao.values[i];
            }
            AddSample(sample, limit);
        }
    }

    public void TakePosSample(Vector3 pos, int limit = -1)
    {
        double[] sample = new double[3];
        sample[0] = pos.X; sample[1] = pos.Y; sample[2] = pos.Z;
        AddSample(sample, limit);
    }

    public void AddSample(double[] sample, int limit = -1)
    {
        data.AddLast(new Tuple<long, double[]>(DateTime.Now.ToFileTime(), sample));
        if (limit > 0 && data.Count > limit) data.RemoveFirst();
    }

    public double[][] GetArray()
    {
        double[][] result = new double[data.Count][];
        int i = 0;
        foreach (Tuple<long, double[]> tup in data) {
            result[i] = tup.Item2;
            i += 1;
        }
        return result;
    }

    public void SaveRecording()
    {
        System.IO.Directory.CreateDirectory("saved");
        string path = Path.Combine(System.IO.Directory.GetCurrentDirectory(), "saved/" + DateTime.Now.ToFileTime().ToString() + ".rec");
        SaveRecording(path);
    }

    public void SaveRecording(string name)
    {
        StreamWriter writer = new StreamWriter(name);
        StringBuilder builder = new StringBuilder();
        builder.AppendFormat("{0}", DateTime.Now.ToFileTime().ToString());
        foreach (Tuple<long, double[]> tup in data)
        {
            builder.Append(tup.Item1);
            double[] vals = tup.Item2;
            for (int i = 0; i < vals.Length; i++)
            {
                if (i != vals.Length) builder.Append(", ");
                builder.AppendFormat("{0}", vals[i]);
            }
            writer.WriteLine(builder.ToString());
            builder.Clear();
        }
    }

    public int TrimMotionless(bool fromBack, int minRem, double sqThresh)
    {
        double[] prior = null;
        int remCount = 0;
        for (int i = fromBack ? data.Count-1 : 0;
            fromBack ? i >= 0 : i < data.Count;
            i += fromBack ? -1 : 1)
        {
            double[] vals = data.ElementAt(i).Item2;
            if (prior != null)
            {
                double dist = 0;
                for (int j = 0; j < vals.Length; j++) {
                    dist += Math.Pow(vals[j] - prior[j], 2);
                }
                if (vals[0] != 0.0)
                    Console.WriteLine("dist = " + dist.ToString());
                if (dist < sqThresh || double.IsNaN(dist)) remCount++; else break;
            }
            prior = (double[])vals.Clone();
        }
        if (remCount > minRem)
        {
            if (fromBack) {
                for (int i = 0; i < remCount; i++) data.RemoveLast();
            } else {
                for (int i = 0; i < remCount; i++) data.RemoveFirst();
            }
            return remCount;
        }
        return 0;
    }
}

}