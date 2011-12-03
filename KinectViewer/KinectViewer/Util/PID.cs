using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;

namespace KinectViewer
{
    // http://www.codeproject.com/KB/recipes/IndustrialDotNetPID.aspx
    public class PID
    {
        #region Fields

        //Gains
        private double kp;
        private double ki;
        private double kd;

        //Running Values
        private DateTime lastUpdate;
        private double lastPV;
        private double errSum;
        
        //Max/Min Calculation
        private double pvMax;
        private double pvMin;
        private double outMax;
        private double outMin;
        
        #endregion

        #region Properties

        public double PGain
        {
            get { return kp; }
            set { kp = value; }
        }

        public double IGain
        {
            get { return ki; }
            set { ki = value; }
        }

        public double DGain
        {
            get { return kd; }
            set { kd = value; }
        }

        public double PVMin
        {
            get { return pvMin; }
            set { pvMin = value; }
        }

        public double PVMax
        {
            get { return pvMax; }
            set { pvMax = value; }
        }

        public double OutMin
        {
            get { return outMin; }
            set { outMin = value; }
        }

        public double OutMax
        {
            get { return outMax; }
            set { outMax = value; }
        }

        #endregion

        #region Construction / Deconstruction

        public PID(double pG, double iG, double dG,
            double pMax, double pMin, double oMax, double oMin)
        {
            kp = pG;
            ki = iG;
            kd = dG;
            pvMax = pMax;
            pvMin = pMin;
            outMax = oMax;
            outMin = oMin;
        }

        ~PID()
        {
        }

        #endregion

        #region Public Methods

        public void Reset()
        {
            errSum = 0.0f;
            lastUpdate = DateTime.Now;
        }

        #endregion

        #region Private Methods

        private double ScaleValue(double value, double valuemin,
                double valuemax, double scalemin, double scalemax)
        {
            double vPerc = (value - valuemin) / (valuemax - valuemin);
            double bigSpan = vPerc * (scalemax - scalemin);

            double retVal = scalemin + bigSpan;

            return retVal;
        }

        private double Clamp(double value, double min, double max)
        {
            if (value > max)
                return max;
            if (value < min)
                return min;
            return value;
        }

        public double Compute(double pv, double sp)
        {
            //We need to scale the pv to +/- 100%, but first clamp it
            pv = Clamp(pv, pvMin, pvMax);
            pv = ScaleValue(pv, pvMin, pvMax, -1.0f, 1.0f);

            //We also need to scale the setpoint
            sp = Clamp(sp, pvMin, pvMax);
            sp = ScaleValue(sp, pvMin, pvMax, -1.0f, 1.0f);

            //Now the error is in percent...
            double err = sp - pv;

            double pTerm = err * kp;
            double iTerm = 0.0f;
            double dTerm = 0.0f;

            double partialSum = 0.0f;
            DateTime nowTime = DateTime.Now;

            if (lastUpdate != null)
            {
                double dT = (nowTime - lastUpdate).TotalSeconds;

                //Compute the integral if we have to...
                if (pv >= pvMin && pv <= pvMax)
                {
                    partialSum = errSum + dT * err;
                    iTerm = ki * partialSum;
                }

                if (dT != 0.0f)
                    dTerm = kd * (pv - lastPV) / dT;
            }

            lastUpdate = nowTime;
            errSum = partialSum;
            lastPV = pv;

            //Now we have to scale the output value to match the requested scale
            double outReal = pTerm + iTerm + dTerm;

            outReal = Clamp(outReal, -1.0f, 1.0f);
            outReal = ScaleValue(outReal, -1.0f, 1.0f, outMin, outMax);

            return outReal;
        }

        #endregion

    }
}
