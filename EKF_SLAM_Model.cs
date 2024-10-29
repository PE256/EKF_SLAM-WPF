using OpenCvSharp;

namespace OpenCV_WPF_try3
{
    public class EKF_SLAM_Model : ILocalizable
    {
        int NumberOfStateVariables { get; set; }
        int NumberOfMeasureVariables { get; set; }
        ExtendedKalmanFilter KalmanFilter { get; set; }
        double[,] CameraStateVector { get; set; }
        double[,] TransitionMatrix { get; set; }
        double[,] CovarianceMatrixP { get; set; }
        double[,] CovarianceMatrixQ { get; set; }
        double[,] CovarianceMatrixR { get; set; }
        double XAccelerationVariance { get; set; }
        double ZAccelerationVariance { get; set; }
        double AngleVariance { get; set; }
        bool IsTimerStarted { get; set; }
        Mat? Measurement { get; set; }
        double[] MeasurementArray { get; set; }

        double dT;
        private double ticks;
        private double precTick;

        List<Point2d> Map { get; set; }
        double CameraViewAngle { get; set; }
        Queue<double[]> MeasurementQueue { get; set; }
        double Camera_X { get; set; }
        double Camera_Z { get; set; }
        double Camera_Theta { get; set; }

        double[,] array;

        public EKF_SLAM_Model()
        {
            CameraViewAngle = Math.PI;

            precTick = 0;
            ticks = 1;
            dT = ticks - precTick;
            IsTimerStarted = false;

            InitSpecialEKF();

            Map = new List<Point2d>();
            MeasurementQueue = new Queue<double[]>();
            MeasurementArray = new double[KalmanFilter.NumberOfMeasureVariables];
        }
        void InitSpecialEKF()
        {
            NumberOfStateVariables = 7;
            NumberOfMeasureVariables = 2;
            KalmanFilter = new ExtendedKalmanFilter(NumberOfStateVariables, NumberOfMeasureVariables, 0, MatType.CV_64F);

            CameraStateVector = new double[NumberOfStateVariables, 1];
            CameraStateVector[0, 0] = Math.Pow(0.01, 2);
            CameraStateVector[1, 0] = Math.Pow(0, 2);
            CameraStateVector[2, 0] = Math.Pow(0, 2);
            CameraStateVector[3, 0] = Math.Pow(0.01, 2);
            CameraStateVector[4, 0] = Math.Pow(0, 2);
            CameraStateVector[5, 0] = Math.Pow(0, 2);
            CameraStateVector[6, 0] = Math.Pow(Math.PI / 180, 2);
            KalmanFilter.StatePost = new Mat(NumberOfStateVariables, 1, MatType.CV_64FC1, CameraStateVector); //x(k)

            TransitionMatrix = new double[NumberOfStateVariables, NumberOfStateVariables];
            //eye matrix
            for (int i = 0; i < TransitionMatrix.GetLength(0); i++)
            {
                for (int j = 0; j < TransitionMatrix.GetLength(1); j++)
                {
                    if (i == j)
                    {
                        TransitionMatrix[i, j] = 1;
                    }
                }
            }
            //add dt to matrix
            TransitionMatrix[0, 1] = dT;
            TransitionMatrix[1, 2] = dT;
            TransitionMatrix[3, 4] = dT;
            TransitionMatrix[4, 5] = dT;
            //add dt^2/2 to matrix
            TransitionMatrix[0, 2] = Math.Pow(dT, 2) / 2;
            TransitionMatrix[3, 5] = Math.Pow(dT, 2) / 2;
            KalmanFilter.TransitionMatrix = new Mat(NumberOfStateVariables, NumberOfStateVariables, MatType.CV_64F, TransitionMatrix); //F(k)

            CovarianceMatrixP = new double[NumberOfStateVariables, NumberOfStateVariables];
            CovarianceMatrixP[0, 0] = Math.Pow(1, 2);
            CovarianceMatrixP[1, 1] = Math.Pow(1, 2);
            CovarianceMatrixP[2, 2] = Math.Pow(1, 2);
            CovarianceMatrixP[3, 3] = Math.Pow(1, 2);
            CovarianceMatrixP[4, 4] = Math.Pow(1, 2);
            CovarianceMatrixP[5, 5] = Math.Pow(1, 2);
            CovarianceMatrixP[6, 6] = Math.Pow(Math.PI / 18, 2);
            KalmanFilter.ErrorCovPost = new Mat(NumberOfStateVariables, NumberOfStateVariables, MatType.CV_64F, CovarianceMatrixP); //P

            CovarianceMatrixR = new double[NumberOfMeasureVariables, NumberOfMeasureVariables];
            CovarianceMatrixR[0, 0] = Math.Pow(20, 2);
            CovarianceMatrixR[1, 1] = Math.Pow(Math.PI / 60, 2);
            KalmanFilter.MeasurementNoiseCov = new Mat(NumberOfMeasureVariables, NumberOfMeasureVariables, MatType.CV_64F, CovarianceMatrixR); //R

            CovarianceMatrixQ = new double[NumberOfStateVariables, NumberOfStateVariables];
            XAccelerationVariance = 0.01;
            CovarianceMatrixQ[0, 0] = Math.Pow(dT, 4) / 4 * Math.Pow(XAccelerationVariance, 2);
            CovarianceMatrixQ[0, 1] = Math.Pow(dT, 3) / 2 * Math.Pow(XAccelerationVariance, 2);
            CovarianceMatrixQ[0, 2] = Math.Pow(dT, 2) / 2 * Math.Pow(XAccelerationVariance, 2);
            CovarianceMatrixQ[1, 0] = CovarianceMatrixQ[0, 1];
            CovarianceMatrixQ[1, 1] = Math.Pow(dT, 2) * Math.Pow(XAccelerationVariance, 2);
            CovarianceMatrixQ[1, 2] = dT * Math.Pow(XAccelerationVariance, 2);
            CovarianceMatrixQ[2, 0] = CovarianceMatrixQ[0, 2];
            CovarianceMatrixQ[2, 1] = CovarianceMatrixQ[1, 2];
            CovarianceMatrixQ[2, 2] = Math.Pow(XAccelerationVariance, 2);
            ZAccelerationVariance = 0.01;
            CovarianceMatrixQ[3, 3] = Math.Pow(dT, 4) / 4 * Math.Pow(ZAccelerationVariance, 2);
            CovarianceMatrixQ[3, 4] = Math.Pow(dT, 3) / 2 * Math.Pow(ZAccelerationVariance, 2);
            CovarianceMatrixQ[3, 5] = Math.Pow(dT, 2) / 2 * Math.Pow(ZAccelerationVariance, 2);
            CovarianceMatrixQ[4, 3] = CovarianceMatrixQ[3, 4];
            CovarianceMatrixQ[4, 4] = Math.Pow(dT, 2) * Math.Pow(ZAccelerationVariance, 2);
            CovarianceMatrixQ[4, 5] = dT * Math.Pow(ZAccelerationVariance, 2);
            CovarianceMatrixQ[5, 3] = CovarianceMatrixQ[3, 5];
            CovarianceMatrixQ[5, 4] = CovarianceMatrixQ[4, 5];
            CovarianceMatrixQ[5, 5] = Math.Pow(ZAccelerationVariance, 2);
            AngleVariance = Math.PI / 180;
            CovarianceMatrixQ[6, 6] = Math.Pow(AngleVariance, 2);
            KalmanFilter.ProcessNoiseCov = new Mat(NumberOfStateVariables, NumberOfStateVariables, MatType.CV_64F, CovarianceMatrixQ); //Q
        }
        public (double[,], List<Point2d>) Localize(List<Point3d> ListOfMeasurements)
        {
            //update dT (time between measurements)
            UpdateTime();
            //update matrices A & Q
            UpdateA();
            UpdateQ();

            //prediction
            double velocity = KalmanFilter.StatePost.At<double>(4, 0);
            KalmanFilter.Predict();

            bool isNew = false;
            Camera_X = KalmanFilter.StatePre.At<double>(0, 0);
            Camera_Z = KalmanFilter.StatePre.At<double>(3, 0);
            Camera_Theta = KalmanFilter.StatePre.At<double>(6, 0);
            for (int i = 0; i < ListOfMeasurements.Count; i++)
            {
                if (ListOfMeasurements[i].X == 0) continue;

                isNew = true;
                for (int j = 0; j < Map.Count; j++)
                {
                    double MapFeature_dX = Map[j].X - Camera_X;
                    double MapFeature_dZ = Map[j].Y - Camera_Z;
                    double MapFeature_l = Math.Sqrt(Math.Pow(MapFeature_dX, 2) + Math.Pow(MapFeature_dZ, 2));
                    double MapFeature_phi = Math.Atan2(MapFeature_dX, MapFeature_dZ) - Camera_Theta;
                    if (MapFeature_phi < Camera_Theta + CameraViewAngle / 2 && MapFeature_phi > Camera_Theta - CameraViewAngle / 2)
                    {
                        if (Math.Abs(ListOfMeasurements[i].Y - MapFeature_phi) < Math.PI / 18)
                        {
                            //there is not a new feature
                            isNew = false;
                            if ((ListOfMeasurements[i].X - MapFeature_l) < 400)
                            {
                                //Add to Queue
                                MeasurementQueue.Enqueue(new double[4] { Map[j].X,//x
                                                                         Map[j].Y,//z
                                                                         ListOfMeasurements[i].X,   //l
                                                                         ListOfMeasurements[i].Y}); //phi
                                double ex = KalmanFilter.ErrorCovPost.At<double>(0, 0);
                                Point2d feature = new Point2d(Map[j].X + (Map[j].X - Camera_X + ListOfMeasurements[i].X * Math.Sin(ListOfMeasurements[i].Y + Camera_Theta)) / Map[j].X * 0.1 / (KalmanFilter.StatePre.At<double>(1, 0) + KalmanFilter.ErrorCovPost.At<double>(1, 1) + 0.0001),// ~1/скорость
                                                              Map[j].Y + (Map[j].Y - Camera_Z + ListOfMeasurements[i].X * Math.Cos(ListOfMeasurements[i].Y + Camera_Theta)) / Map[j].Y * 0.1 / (KalmanFilter.StatePre.At<double>(4, 0) + KalmanFilter.ErrorCovPost.At<double>(4, 4) + 0.0001));
                                Map[j] = feature;
                                break;
                            }
                            else 
                            { 
                                //there is overlap
                                break; 
                            }
                        }
                    }
                }
                if (isNew)
                {
                    //Add to Map
                    Map.Add(new Point2d(Camera_X + ListOfMeasurements[i].X * Math.Sin(ListOfMeasurements[i].Y + Camera_Theta),
                                        Camera_Z + ListOfMeasurements[i].X * Math.Cos(ListOfMeasurements[i].Y + Camera_Theta)));
                }
            }
            //Measurement with queue
            if (MeasurementQueue.Count == 0)
            {
                KalmanFilter.StatePost = KalmanFilter.StatePre;
            }
            while (MeasurementQueue.Count > 0)
            {
                if (MeasurementQueue.Count == 1)
                {
                    EstimateLast(MeasurementQueue.Dequeue());
                }
                else
                {
                    AccumulateEstimation(MeasurementQueue.Dequeue());
                }
            }
            KalmanFilter.StatePost.GetRectangularArray(out array);
            var systemState = (array, Map);
            return systemState;
        }
        private void UpdateTime()
        {
            if (!IsTimerStarted)
            {
                IsTimerStarted = true;
                precTick = 0;
                ticks = 1;
                dT = ticks - precTick;
                ticks = (double)Cv2.GetTickCount();
            }
            else
            {
                precTick = ticks;
                ticks = (double)Cv2.GetTickCount();
                dT = (ticks - precTick) / Cv2.GetTickFrequency(); //seconds
            }
        }
        private void UpdateA()
        {
            KalmanFilter.TransitionMatrix.At<double>(0, 1) = dT;
            KalmanFilter.TransitionMatrix.At<double>(0, 2) = Math.Pow(dT, 2) / 2;
            KalmanFilter.TransitionMatrix.At<double>(1, 2) = dT;
            KalmanFilter.TransitionMatrix.At<double>(3, 4) = dT;
            KalmanFilter.TransitionMatrix.At<double>(3, 5) = Math.Pow(dT, 2) / 2;
            KalmanFilter.TransitionMatrix.At<double>(4, 5) = dT;
        }
        private void UpdateQ()
        {
            XAccelerationVariance = KalmanFilter.TransitionMatrix.At<double>(2, 2);
            KalmanFilter.ProcessNoiseCov.At<double>(0, 0) = Math.Pow(dT, 4) / 4 * Math.Pow(XAccelerationVariance, 2);
            KalmanFilter.ProcessNoiseCov.At<double>(0, 1) = Math.Pow(dT, 3) / 2 * Math.Pow(XAccelerationVariance, 2);
            KalmanFilter.ProcessNoiseCov.At<double>(0, 2) = Math.Pow(dT, 2) / 2 * Math.Pow(XAccelerationVariance, 2);
            KalmanFilter.ProcessNoiseCov.At<double>(1, 0) = CovarianceMatrixQ[0, 1];
            KalmanFilter.ProcessNoiseCov.At<double>(1, 1) = Math.Pow(dT, 2) * Math.Pow(XAccelerationVariance, 2);
            KalmanFilter.ProcessNoiseCov.At<double>(1, 2) = dT * Math.Pow(XAccelerationVariance, 2);
            KalmanFilter.ProcessNoiseCov.At<double>(2, 0) = CovarianceMatrixQ[0, 2];
            KalmanFilter.ProcessNoiseCov.At<double>(2, 1) = CovarianceMatrixQ[1, 2];
            KalmanFilter.ProcessNoiseCov.At<double>(2, 2) = Math.Pow(XAccelerationVariance, 2);
            ZAccelerationVariance = KalmanFilter.TransitionMatrix.At<double>(5, 5);
            KalmanFilter.ProcessNoiseCov.At<double>(3, 3) = Math.Pow(dT, 4) / 4 * Math.Pow(ZAccelerationVariance, 2);
            KalmanFilter.ProcessNoiseCov.At<double>(3, 4) = Math.Pow(dT, 3) / 2 * Math.Pow(ZAccelerationVariance, 2);
            KalmanFilter.ProcessNoiseCov.At<double>(3, 5) = Math.Pow(dT, 2) / 2 * Math.Pow(ZAccelerationVariance, 2);
            KalmanFilter.ProcessNoiseCov.At<double>(4, 3) = CovarianceMatrixQ[3, 4];
            KalmanFilter.ProcessNoiseCov.At<double>(4, 4) = Math.Pow(dT, 2) * Math.Pow(ZAccelerationVariance, 2);
            KalmanFilter.ProcessNoiseCov.At<double>(4, 5) = dT * Math.Pow(ZAccelerationVariance, 2);
            KalmanFilter.ProcessNoiseCov.At<double>(5, 3) = CovarianceMatrixQ[3, 5];
            KalmanFilter.ProcessNoiseCov.At<double>(5, 4) = CovarianceMatrixQ[4, 5];
            KalmanFilter.ProcessNoiseCov.At<double>(5, 5) = Math.Pow(ZAccelerationVariance, 2);
            AngleVariance = KalmanFilter.TransitionMatrix.At<double>(6, 6);
            KalmanFilter.ProcessNoiseCov.At<double>(6, 6) = Math.Pow(AngleVariance, 2);
        }
        private void AccumulateEstimation(double[] feature)
        {
            MeasurementArray[0] = feature[2];
            MeasurementArray[1] = feature[3];
            Measurement = new Mat(NumberOfMeasureVariables, 1, MatType.CV_64F, MeasurementArray);
            KalmanFilter.RefinePrediction(Measurement, feature[0], feature[1]);
        }
        private void EstimateLast(double[] feature)
        {
            MeasurementArray[0] = feature[2];
            MeasurementArray[1] = feature[3];
            Measurement = new Mat(NumberOfMeasureVariables, 1, MatType.CV_64F, MeasurementArray);
            KalmanFilter.Estimate(Measurement, feature[0], feature[1]);
        }
    }

    public class ExtendedKalmanFilter
    {
        public Mat StatePost { get; set; }          //x
        public Mat StatePre { get; set; }           //x_
        public Mat TransitionMatrix { get; set; }   //F
        public Mat ErrorCovPost { get; set; }       //P
        public Mat MeasurementNoiseCov { get; set; }//R
        public Mat ProcessNoiseCov { get; set; }    //Q
        public Mat MeasurementMatrix { get; set; }  //H
        public Mat Residual { get; set; }           //y
        public Mat Gain { get; set; }               //K
        public int NumberOfStateVariables { get; set; }
        public int NumberOfMeasureVariables { get; set; }
        public ExtendedKalmanFilter(int numberOfStateVariables, int numberOfMeasureVariables, int controlParams = 0, int type = MatType.CV_64F)
        {
            NumberOfStateVariables = numberOfStateVariables;
            NumberOfMeasureVariables = numberOfMeasureVariables;
            StatePost = new Mat(numberOfStateVariables, 1, type, new double[numberOfStateVariables, 1]);
            StatePre = new Mat(numberOfStateVariables, 1, type, new double[numberOfStateVariables, 1]);
            TransitionMatrix = new Mat(numberOfStateVariables, numberOfStateVariables, type, new double[numberOfStateVariables, numberOfStateVariables]);
            ErrorCovPost = new Mat(numberOfStateVariables, numberOfStateVariables, type, new double[numberOfStateVariables, numberOfStateVariables]);
            MeasurementNoiseCov = new Mat(numberOfMeasureVariables, numberOfMeasureVariables, type, new double[numberOfMeasureVariables, numberOfMeasureVariables]);
            ProcessNoiseCov = new Mat(numberOfStateVariables, numberOfStateVariables, type, new double[numberOfStateVariables, numberOfStateVariables]);
            MeasurementMatrix = new Mat(numberOfMeasureVariables, numberOfStateVariables, type, new double[numberOfMeasureVariables, numberOfStateVariables]);
            Residual = new Mat(numberOfMeasureVariables, 1, type, new double[numberOfMeasureVariables, 1]);
            Gain = new Mat(numberOfStateVariables, numberOfMeasureVariables, type, new double[numberOfStateVariables, numberOfMeasureVariables]);
        }
        public void Predict()
        {
            StatePre = TransitionMatrix * StatePost;
            ErrorCovPost = TransitionMatrix * ErrorCovPost * TransitionMatrix.Transpose() + ProcessNoiseCov;
        }
        public void RefinePrediction(Mat MeasurementArray, double Xp, double Zp)
        {
            //y = z - h(x)
            Residual = MeasurementArray - LinearizeMeasurement(StatePost, StatePre, Xp, Zp);
            //H = Jacobian (dh(x) / dx)
            MeasurementMatrix = ComputeJacobianH(StatePost, StatePre, Xp, Zp);
            //K = P * H_T * (H * P * H_T + R)^-1
            Gain = ErrorCovPost * MeasurementMatrix.Transpose() * (MeasurementMatrix * ErrorCovPost * MeasurementMatrix.Transpose() + MeasurementNoiseCov).Inv();
            //x = x + Ky
            StatePre = StatePre + Gain * Residual;
            //P = (I - K * H) * P
            ErrorCovPost = (Mat.Eye(new Size(Gain.Rows, MeasurementMatrix.Cols), Gain.Type()) - Gain * MeasurementMatrix) * ErrorCovPost;
        }
        public void Estimate(Mat MeasurementArray, double Xp, double Zp)
        {
            //y = z - h(x)
            Residual = MeasurementArray - LinearizeMeasurement(StatePost, StatePre, Xp, Zp);
            //H = Jacobian (dh(x) / dx)
            MeasurementMatrix = ComputeJacobianH(StatePost, StatePre, Xp, Zp);
            //K = P * H_T * (H * P * H_T + R)^-1
            Gain = ErrorCovPost * MeasurementMatrix.Transpose() * (MeasurementMatrix * ErrorCovPost * MeasurementMatrix.Transpose() + MeasurementNoiseCov).Inv();
            //x = x + Ky
            StatePost = StatePre + Gain * Residual;
            //P = (I - K * H) * P
            ErrorCovPost = (Mat.Eye(new Size(Gain.Rows, MeasurementMatrix.Cols), Gain.Type()) - Gain * MeasurementMatrix) * ErrorCovPost;
        }
        private Mat ComputeJacobianH(Mat state, Mat predictedState, double Xp, double Zp)
        {
            double X = state.At<double>(0, 0);
            double Z = state.At<double>(3, 0);
            double Theta = state.At<double>(6, 0);

            double dXp = Xp - X;
            double dZp = Zp - Z;
            double devider_p = Math.Pow((dXp), 2) + Math.Pow((dZp), 2);
            double sqrt_p = Math.Sqrt(devider_p);
            double l_p = sqrt_p;
            double phi_p = Math.Atan2(dXp, dZp) - Theta;

            if (X == 0) X = 1 / 100000.0;
            if (Z == 0) Z = 1 / 100000.0;
            if (Theta == 0) Theta = 1 / 1000000.0;

            double[,] JacobianH = new double[2, 7] { { (-dXp / sqrt_p),      0, 0, (-dZp / sqrt_p),   0, 0, 0 },
                                                     { (-dZp / devider_p), 0, 0, (dXp / devider_p), 0, 0, -1 }};
            return new Mat(2, 7, MatType.CV_64FC1, JacobianH);
        }
        private Mat LinearizeMeasurement(Mat state, Mat predictedState, double Xp, double Zp)
        {
            double X = state.At<double>(0, 0);
            double Z = state.At<double>(3, 0);
            double Theta = state.At<double>(6, 0);

            double predicted_X = predictedState.At<double>(0, 0);
            double predicted_Z = predictedState.At<double>(3, 0);
            double predicted_Theta = predictedState.At<double>(6, 0);
            double predicted_gainX = predicted_X - X;
            double predicted_gainZ = predicted_Z - Z;
            double predicted_gainTheta = predicted_Theta - Theta;

            double dXp = Xp - X;
            double dZp = Zp - Z;
            double devider_p = Math.Pow((dXp), 2) + Math.Pow((dZp), 2);
            double sqrt_p = Math.Sqrt(devider_p);
            double l_p = sqrt_p;
            double phi_p = Math.Atan2(dXp, dZp) - Theta;
            if (X == 0) X = 1 / 100000.0;
            if (Z == 0) Z = 1 / 100000.0;
            if (Theta == 0) Theta = 1 / 1000000.0;
            if (predicted_X == 0) predicted_X = 1 / 100000.0;
            if (predicted_Z == 0) predicted_Z = 1 / 100000.0;
            if (predicted_Theta == 0) predicted_Theta = 1 / 1000000.0;
            //h(x)
            double[,] LinearisedMeasurement = new double[2, 1] { { l_p   - dXp * predicted_gainX / sqrt_p    - dZp * predicted_gainZ / sqrt_p },
                                                                 { phi_p - dZp * predicted_gainX / devider_p + dXp * predicted_gainZ / devider_p - predicted_gainTheta }};
            return new Mat(2, 1, MatType.CV_64FC1, LinearisedMeasurement);
        }
    }

    public interface ILocalizable
    {
        public (double[,], List<Point2d>) Localize(List<Point3d> ListOfMeasurements);
    }
}
