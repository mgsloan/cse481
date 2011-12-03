using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using Microsoft.Xna.Framework;

namespace KinectViewer
{
    
    //given new hip, knee, and ankle pitch, output new
    //hip, knee and pitch
    //expects to be initialized with a stable standing position
    //assumes feet are fixed
    class TwoFootBalancer
    {
        private double initHip; //initial pitch for both hips
        private double initKnee; //initial pitch for both knees
        private double initAnkle; //inital pitch for both ankles
        NaoSimulator naoSim;

        public TwoFootBalancer(NaoSimulator naoSim)
        {
            this.naoSim = naoSim;
            this.initHip = naoSim.GetInitialAngle("RHipPitch"); //current best = .183259
            this.initKnee = naoSim.GetInitialAngle("RKneePitch"); //current best = -.076658
            this.initAnkle = naoSim.GetInitialAngle("RAnklePitch"); //current best = .085945
        }

        //http://users.aldebaran-robotics.com/docs/site_en/reddoc/hardware/joints-names_3.3.html

        //if knee angle increases by T, hip pitch increases by T, and 
        //ankle pitch increases by T (currently not used)
        private Tuple<double, double> updateHipAnkle(double k)
        {
            double delta_k = k - initKnee;

            return new Tuple<double, double>(initHip + delta_k, initAnkle + delta_k);
        }

        //expects stable initial position
        //if knee angle increases by T, ankle pitch increases by T
        //if hip angle increases by T, ankle pitch increases by T
        public void balance(List<LabelledVector> ls, Vector3 torsoForward)
        {
            double delta_h = naoSim.GetCurrentAngle("RHipPitch") - initHip;
            double delta_k = naoSim.GetCurrentAngle("RKneePitch") - initKnee;

            naoSim.UpdateAngle("RAnklePitch", (float) (initAnkle - .5 * delta_h - 1 * delta_k), .05f);
            naoSim.UpdateAngle("LKneePitch", naoSim.GetCurrentAngle("RKneePitch"));
            naoSim.UpdateAngle("LHipPitch", naoSim.GetCurrentAngle("RHipPitch"));
            naoSim.UpdateAngle("LAnklePitch", naoSim.GetCurrentAngle("RAnklePitch"));

            //SmartBalance(ls, torsoForward);
        }

        //for now assume that the feet are parallel WRT torso space
        public void SmartBalance(List<LabelledVector> ls, Vector3 torsoForward)
        {
            //in a loop:
                // get COM

                // get Foot Positions

                // if COM is "beyond" foot positions, then pick one of ankle, knee, or hip
                // and move it by D

            Vector3 com = naoSim.GetCOM();
            Vector3 supportcenter = naoSim.GetTwoFootCenter();

            Vector3 displacement = com - supportcenter;

            Boolean isForward = (Vector3.Dot(displacement, torsoForward) > 0);

            //hipangles += delta
        }
    }
}
