using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using Microsoft.Xna.Framework;

namespace KinectViewer
{
    class TransitionBalancer
    {
        protected Vector3 initHr, initHl, initFr, initFl; //hip right, hip left, foot right, foot left
        protected double initRKnee, initRHip, initRAnkle, initLKnee, initLHip, initLAnkle;
        protected NaoSimulator naoSim;

        protected double UL_len, LL_len;

        public TransitionBalancer(NaoSimulator naoSim)
        {
            this.initHr = naoSim.GetPosition("RHipRoll");
            this.initHl = naoSim.GetPosition("LHipRoll");
            this.initFr = naoSim.GetPosition("RAnkleRoll");
            this.initFl = naoSim.GetPosition("LAnkleRoll");

            Vector3 rKnee = naoSim.GetPosition("RKneePitch");
            UL_len = (rKnee - initHr).Length();
            LL_len = (initFr - rKnee).Length();

            this.initRKnee = naoSim.GetCurrentAngle("RKneePitch");
            this.initRHip = naoSim.GetCurrentAngle("RHipPitch");
            this.initRAnkle = naoSim.GetCurrentAngle("RAnklePitch");

            this.initLKnee = naoSim.GetCurrentAngle("LKneePitch");
            this.initLHip = naoSim.GetCurrentAngle("LHipPitch");
            this.initLAnkle = naoSim.GetCurrentAngle("LAnklePitch");
            this.naoSim = naoSim;
        }

        public void balance()
        {
            calculatePitches();
            calculateRolls();
        }

        private void calculate()
        {
            Vector3 curHr = naoSim.GetPosition("RHipRoll");
            Vector3 curFr = naoSim.GetPosition("RAnkleRoll");

            Vector3 hipToFoot = curFr - curHr;
            double hipToFoot_len = hipToFoot.Length();

            // find knee pitch (easy, use law of cosines)
            double a2 = UL_len;
            double b2 = LL_len;
            double c2 = hipToFoot_len;
            double kneepitch;
            if (c2 > a2 + b2) kneepitch = Math.PI;
            else kneepitch = Math.Acos((a2 * a2 + b2 * b2 - c2 * c2) / (2 * a2 * b2));

            naoSim.UpdateAngle("RKneePitch", (float) kneepitch);

            //create a lever by fixing pitches in the leg
            naoSim.UpdateAngle("LAnklePitch", (float)initLAnkle);
            naoSim.UpdateAngle("LKneePitch", (float)initLKnee);
            naoSim.UpdateAngle("LHipPitch", (float)initLHip);
        }

        //if knee angle increases by T, hip pitch increases by T, and 
        //ankle pitch increases by T (currently not used)
        private void calculatePitches()
        {
            double delta_k = naoSim.GetCurrentAngle("RKneePitch") - initRKnee;

            naoSim.UpdateAngle("RHipPitch", (float) (initRHip - 0.8 * delta_k), .05f);

            naoSim.UpdateAngle("RAnklePitch", (float) (initRAnkle - delta_k), .05f);

            //create a lever by fixing pitches in the leg
            naoSim.UpdateAngle("LAnklePitch", (float)initLAnkle);
            naoSim.UpdateAngle("LKneePitch", (float)initLKnee);
            naoSim.UpdateAngle("LHipPitch", (float)initLHip);
        }

        private void calculateRolls()
        {
            Vector3 curHr = naoSim.GetPosition("RHipRoll");
            Vector3 curFr = naoSim.GetPosition("RAnkleRoll");

            //curHr_curFr to curHr_initHl
            Vector3 rLeg = curFr - curHr;
            Vector3 rlHip = initHl - curHr;
            double rHipRoll = MathUtils.AngleBetween(rLeg, rlHip);

            //curHr_initHl to initHl_initHl
            Vector3 lrHip = curHr - initHl;
            Vector3 lLeg = initFl - initHl;
            double lHipRoll = MathUtils.AngleBetween(lLeg, lrHip);

            //initial left and right hip roll (when the two legged stance was started)
            double initRHRoll = MathUtils.AngleBetween((initFr - initHr), (initHl - initHr));
            double initLHRoll = MathUtils.AngleBetween((initFl - initHl), (initHr - initHl));

            //change in left & right hip roll
            double deltaRH = rHipRoll - initRHRoll;
            double deltaLH = lHipRoll - initLHRoll;

            //initial left & right ankle roll (take angle between feet and from foot to hip)
            double initLARoll = MathUtils.AngleBetween((initHl - initFl), (initFr - initFl));
            double initRARoll = MathUtils.AngleBetween((initHr - initFr), (initFl - initFr));

            //ankles counterrotate with respect to hip
            double curLARoll = initLARoll - deltaLH;
            double curRARoll = initRARoll - deltaRH;

            naoSim.UpdateAngle("RHipYawPitch", 0f);
            naoSim.UpdateAngle("RHipRoll", (float) rHipRoll);
            naoSim.UpdateAngle("RAnkleRoll", (float) curRARoll);

            naoSim.UpdateAngle("LHipRoll", (float)lHipRoll);
            naoSim.UpdateAngle("LAnkleRoll", (float)curLARoll);
        }
    }
}
