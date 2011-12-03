using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;

namespace KinectViewer
{
    
    //given new hip, knee, and ankle pitch, output new
    //hip, knee and pitch
    //expects to be initialized with a stable standing position
    class FixedBalancer
    {
        double initHip; //initial pitch for both hips
        double initKnee; //initial pitch for both knees
        double initAnkle; //inital pitch for both ankles

        public FixedBalancer(double initHip, double initKnee, double initAnkle)
        {
            this.initHip = initHip; //current best = .183259
            this.initKnee = initKnee; //current best = -.076658
            this.initAnkle = initAnkle; //current best = .085945
        }

        //http://users.aldebaran-robotics.com/docs/site_en/reddoc/hardware/joints-names_3.3.html

        //if knee angle increases by T, hip pitch increases by T, and 
        //ankle pitch increases by T
        public Tuple<double, double> updateHipAnkle(double k)
        {
            double delta_k = k - initKnee;

            return new Tuple<double, double>(initHip + delta_k, initAnkle + delta_k);
        }

        //if knee angle increases by T, ankle pitch increases by T
        //if hip angle increases by T, ankle pitch increases by T
        public double updateAnkle(double h, double k)
        {
            double delta_h = h - initHip;
            double delta_k = k - initKnee;

            return (initAnkle - .5 * delta_h - 1 * delta_k);
        }
    }
}
