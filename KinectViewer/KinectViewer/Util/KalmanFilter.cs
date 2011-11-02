using DotNetMatrix;

namespace KinectViewer
{
    public sealed class KalmanFilter
    {
        //System matrices
        public GeneralMatrix X0, P0;

        public GeneralMatrix F { get; private set; }
        public GeneralMatrix B { get; private set; }
        public GeneralMatrix U { get; private set; }
        public GeneralMatrix Q { get; private set; }
        public GeneralMatrix H { get; private set; }
        public GeneralMatrix R { get; private set; }

        public GeneralMatrix State { get; private set; } 
        public GeneralMatrix Covariance { get; private set; }  

        public KalmanFilter(GeneralMatrix f, GeneralMatrix b, GeneralMatrix u, GeneralMatrix q, GeneralMatrix h,
                            GeneralMatrix r)
        {
            F = f;
            B = b;
            U = u;
            Q = q;
            H = h;
            R = r;
        }

        public void Predict()
        {
            X0 = F*State + (B*U);
            P0 = F*Covariance*F.Transpose() + Q;
        }

        public void Correct(GeneralMatrix z)
        {
            GeneralMatrix s = H*P0*H.Transpose() + R;
            GeneralMatrix k = P0*H.Transpose()*s.Inverse();
            State = X0 + (k*(z - (H*X0)));
            GeneralMatrix I = GeneralMatrix.Identity(P0.RowDimension, P0.ColumnDimension);
            Covariance = (I - k*H)*P0;
        }
    }
}