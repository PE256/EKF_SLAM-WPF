using OpenCvSharp.WpfExtensions;
using OpenCvSharp;
using System.Windows.Media.Imaging;
using System.Windows.Media;

namespace OpenCV_WPF_try3
{
    public class ViewModel
    {
        private ILocalizable ModelSLAM { get; set; }
        public ViewBindingSource ViewSource { get; set; }
        private bool IsRunning { get; set; }

        public ImageSource? InitialImage { get; set; }
        public ImageSource? UndistortedImage { get; set; }
        public ImageSource? HSVImage { get; set; }
        public ImageSource? ImageThresholdedByHSV { get; set; }
        public ImageSource? ImageErodedAndDelated { get; set; }
        public ImageSource? MapImage { get; set; }
        public DrawingImage MapDrawingImage { get; set; }
        private Mat InitialFrame { get; set; }
        private Mat UndistortedFrame { get; set; }
        private Mat FrameHSV { get; set; }
        private Mat FrameThresholdedByHSV { get; set; }
        private Mat FrameErodedAndDelated { get; set; }
        private List<Point3d> FeatureCoordinatesOnFrame { get; set; } //parameters: distance, alpha, beta
        private double[,] CameraPosition { get; set; } 
        private List<Point2d> Map { get; set; } //parameters: x, y
        private Rect bounding_rect;
        private Point[][] contours; // Vector for storing contour
        private HierarchyIndex[] hierarchy;


        private double[,] DefaultCameraMatrix { get; set; }
        private double[] DefaultDistCoeffs { get; set; }
        private Mat CameraMatrixInputArray { get; set; }
        private Mat DistCoeffsInputArray { get; set; }

        
        //Parameters of the features and camera
        private readonly double featureWidth = 45f;
        private readonly double focalLength = 3.8f;
        private readonly double pixelToMilemetersPrescaler = 0.00584210526f;

        public ViewModel(ViewBindingSource sourcesForView)
        {
            ModelSLAM = new EKF_SLAM_Model();
            ViewSource = sourcesForView;

            IsRunning = false;

            FeatureCoordinatesOnFrame = new List<Point3d>();

            InitialFrame = new Mat();
            UndistortedFrame = new Mat();
            FrameHSV = new Mat();
            FrameThresholdedByHSV = new Mat();
            FrameErodedAndDelated = new Mat();

            DefaultCameraMatrix = new double[3, 3] { {453.08463027168779, 0,                  299.31819451279819},
                                                     { 0,                 451.6553142872001,  244.114196390533},
                                                     { 0,                 0,                  1} };
            CameraMatrixInputArray = new Mat(3, 3, MatType.CV_64FC1, DefaultCameraMatrix);
            DefaultDistCoeffs = new double[5] { -0.13420040967885613, 0.079967892070645419, 0.0029490255986522693, 0.0081203566527775339, -0.061543489987792022 };
            DistCoeffsInputArray = new Mat(1, 8, MatType.CV_64FC1, DefaultDistCoeffs);
        }
        public void CameraCalibration()
        {
            using (VideoCapture capture = new VideoCapture(0))
            {
                // Check if the webcam is opened correctly
                if (!capture.IsOpened())
                {
                    System.Console.WriteLine("Error: Couldn't open the webcam.");
                    return;
                }
                //https://docs.opencv.org/4.x/checkerboard_radon.png
                Size chessboardSize = new Size(14, 9);
                Point2f[] corners;
                //IEnumerable<Point3f> objectPoints = new Point3f[4];
                Point3f[] objectPoints = new Point3f[chessboardSize.Width * chessboardSize.Height];

                int k = 0;
                for (int i = 0; i < chessboardSize.Height; i++)
                {
                    for (int j = 0; j < chessboardSize.Width; j++)
                    {
                        //(0,0) (0,1) (0,2)
                        //(1,0) (1,1) (1,2)
                        //k0 = (0,0),..., k6 = (1,2)
                        objectPoints[k].Y = i;
                        objectPoints[k].X = j;
                        k++;
                    }
                }
                IEnumerable<IEnumerable<Point3f>> objectPointsArray = new List<IEnumerable<Point3f>>();
                IEnumerable<IEnumerable<Point2f>> imagePoints = new List<IEnumerable<Point2f>>();
                double[,] cameraMatrix = new double[3, 3];
                double[] distCoeffs = new double[5];
                Vec3d[] rvecs;
                Vec3d[] tvecs;
                //Rect validPixROI;
                capture.Read(InitialFrame);
                Size imageSize = InitialFrame.Size();
                Mat grayscaleFrame = new Mat();

                int numberOfFoundedContours = 0;
                while (numberOfFoundedContours < 10)
                {
                    capture.Read(InitialFrame);

                    Cv2.CvtColor(InitialFrame, grayscaleFrame, ColorConversionCodes.BGR2GRAY);
                    Cv2.ImShow("GrayscaleImg", grayscaleFrame);
                    Cv2.WaitKey(10);
                    bool isCornersFound = Cv2.FindChessboardCornersSB(grayscaleFrame, chessboardSize, out corners);
                    if (isCornersFound)
                    {
                        Cv2.DrawChessboardCorners(grayscaleFrame, chessboardSize, corners, isCornersFound);
                        Cv2.ImShow("GrayscaleImg", grayscaleFrame);
                        Cv2.WaitKey(50);
                        ((List<IEnumerable<Point3f>>)objectPointsArray).Add(objectPoints);
                        ((List<IEnumerable<Point2f>>)imagePoints).Add(corners);
                        Cv2.CalibrateCamera(objectPointsArray, imagePoints, imageSize, cameraMatrix, distCoeffs, out rvecs, out tvecs, CalibrationFlags.None, null);

                        numberOfFoundedContours++;
                    }
                }
                capture.Read(InitialFrame);
                Cv2.CvtColor(InitialFrame, grayscaleFrame, ColorConversionCodes.BGR2GRAY);
                Cv2.ImShow("Img", grayscaleFrame);
                Cv2.WaitKey(10);

                //Cv2.GetOptimalNewCameraMatrix(cameraMatrix, distCoeffs, imageSize, 1, imageSize, out validPixROI);
                CameraMatrixInputArray = new Mat(3, 3, MatType.CV_64FC1, cameraMatrix);
                DistCoeffsInputArray = new Mat(1, 8, MatType.CV_64FC1, distCoeffs);

                //Cv2.InitUndistortRectifyMap(cameraMatrixInputArray, distCoeffsInputArray, cameraMatrixInputArray, cameraMatrixInputArray, imageSize)
                Cv2.Undistort(grayscaleFrame, UndistortedFrame, CameraMatrixInputArray, DistCoeffsInputArray, CameraMatrixInputArray);
                Cv2.ImShow("UndistortedImg", UndistortedFrame);
                Cv2.WaitKey(10);


                // cv2.getOptimalNewCameraMatrix(mtx, dist, (w, h), 1, (w, h))
                //undistortedImg = cv2.undistort(img, mtx, dist, None, newCameraMtx)
            }
        }
        public void StartOrStopCamera()
        {
            if (IsRunning)
            {
                IsRunning = false;
            }
            else
            {
                IsRunning = true;

                Thread thread = new Thread(new ThreadStart(VideoCapturing));
                thread.IsBackground = true;
                thread.Name = "VideoCapturingThreadFromPathFinder";
                thread.Start();
                ViewSource.AppTextSource = "Thread is started";
            }
        }
        public void VideoCapturing()
        {

            using VideoCapture capture = new VideoCapture(0);
            
            // Check if the webcam is opened correctly
            if (!capture.IsOpened())
            {
                ViewSource.AppTextSource = "Error: Couldn't open the webcam.";
                return;
            }

            //Starts camera, waits for brightness balanced capture
            capture.Read(InitialFrame);
            Cv2.WaitKey(2000);

            // Loop to continuously get frames from the webcam
            while (IsRunning)
            {
                // Capture a frame
                capture.Read(InitialFrame);

                // Check if the frame is empty
                if (InitialFrame.Empty())
                {
                    System.Console.WriteLine("Error: Couldn't read a frame from the webcam.");
                    break;
                }
                HandleInitFrame();

                var message = ComputeCoordinatesOnFrame();
                ViewSource.AppTextSource = message;

                (CameraPosition, Map) = ModelSLAM.Localize(FeatureCoordinatesOnFrame);
                            
                //Thread.Sleep(1000);

                //Convert frames that need to be converted;
                ConvertFrames();

                //delay 10 ms
                Cv2.WaitKey(10);
            }

            // Release the capture
            capture.Release();
            ViewSource.AppTextSource = "Thread is stopped";
            IsRunning = false;
        }
        /// <summary>
        /// Handles initial frame, provides steps to select features on it
        /// </summary>
        private void HandleInitFrame()
        {
            // Undistort that frame
            Cv2.Undistort(InitialFrame, UndistortedFrame, CameraMatrixInputArray, DistCoeffsInputArray);

            // Convert the frame to HSV
            Cv2.CvtColor(UndistortedFrame, FrameHSV, ColorConversionCodes.BGR2HSV_FULL);

            //Checks if array elements lie between the elements of two other arrays, that represents a thresholds of HSV parameters
            Cv2.InRange(FrameHSV,
                        new Scalar(ViewSource.ThresholdHSV_HLow * 255 / 355,
                                    ViewSource.ThresholdHSV_SLow * 255 / 100,
                                    ViewSource.ThresholdHSV_VLow * 255 / 100),
                        new Scalar(ViewSource.ThresholdHSV_HHigh * 255 / 355,
                                    ViewSource.ThresholdHSV_SHigh * 255 / 100,
                                    ViewSource.ThresholdHSV_VHigh * 255 / 100),
                        FrameThresholdedByHSV);

            //Invert frame
            //Cv2.BitwiseNot(FrameThresholdedByHSV, FrameThresholdedByHSV);

            //morphological opening (remove small objects from the foreground)
            InputArray? elementForRemoveShapes = Cv2.GetStructuringElement(MorphShapes.Ellipse,
                                                                           new OpenCvSharp.Size(ViewSource.SizeOfShapeToRemove,
                                                                                                ViewSource.SizeOfShapeToRemove));

            Cv2.Erode(FrameThresholdedByHSV, FrameErodedAndDelated, elementForRemoveShapes);
            Cv2.Dilate(FrameErodedAndDelated, FrameErodedAndDelated, elementForRemoveShapes);

            InputArray? elementForFillHoles = Cv2.GetStructuringElement(MorphShapes.Ellipse,
                                                                        new OpenCvSharp.Size(ViewSource.SizeOfHoleToFill,
                                                                                             ViewSource.SizeOfHoleToFill));
            //morphological closing (fill small holes in the foreground)
            Cv2.Dilate(FrameErodedAndDelated, FrameErodedAndDelated, elementForFillHoles);
            Cv2.Erode(FrameErodedAndDelated, FrameErodedAndDelated, elementForFillHoles);
        }
        /// <summary>
        /// Computes coordinates on frame for each feature contour on the it
        /// </summary>
        /// <returns>Message about results</returns>
        /// <exception cref="Exception"></exception>
        private string ComputeCoordinatesOnFrame()
        {
            Cv2.FindContours(FrameErodedAndDelated, out contours, out hierarchy, OpenCvSharp.RetrievalModes.CComp, ContourApproximationModes.ApproxSimple);
            if (contours.Length > 0)
            {
                //Adding structures to list if quantity of features more than list.Count
                while (FeatureCoordinatesOnFrame.Count < contours.Length) FeatureCoordinatesOnFrame.Add(new Point3d(0, 0, 0));

                //Computing coordinates of each feature, rest structures in list = 0
                for (int i = 0; i < FeatureCoordinatesOnFrame.Count; i++)
                {
                    if (i < contours.Length)
                    {
                        bounding_rect = Cv2.BoundingRect(contours[i]);

                        //filter features that touches borders of frame
                        if (bounding_rect.X == 0 || bounding_rect.X + bounding_rect.Width == FrameErodedAndDelated.Width ||
                            bounding_rect.Y == 0 || bounding_rect.Y + bounding_rect.Height == FrameErodedAndDelated.Height)
                        {
                            //Feature coordinates = 0
                            FeatureCoordinatesOnFrame[i] *= 0;
                            continue;
                        }

                        //Draw feature on frame
                        Cv2.Rectangle(UndistortedFrame, bounding_rect, new Scalar(0, 0, 255), 2, LineTypes.Link8, 0);

                        double l_0 = bounding_rect.Width * pixelToMilemetersPrescaler;
                        double x_0 = (bounding_rect.X + bounding_rect.Width / 2.0 - FrameErodedAndDelated.Width / 2) * pixelToMilemetersPrescaler;
                        //Computing feature coordinates on frame
                        Point3d feature = FeatureCoordinatesOnFrame[i];
                        //distance
                        if (Math.Tan((Math.Atan2(x_0, focalLength) - Math.Atan2(x_0 - l_0, focalLength)) / 2) == 0) feature.X = int.MaxValue;
                        feature.X = 0.5 * featureWidth / Math.Tan((Math.Atan2(x_0, focalLength) - Math.Atan2(x_0 - l_0, focalLength)) / 2);
                        //alpfa
                        feature.Y = 0.5 * (Math.Atan2(x_0 - l_0, focalLength) + Math.Atan2(x_0, focalLength));
                        //beta
                        feature.Z = Math.Atan2((FrameErodedAndDelated.Height / 2 - (bounding_rect.Y + bounding_rect.Height / 2.0)) * pixelToMilemetersPrescaler, focalLength);
                        FeatureCoordinatesOnFrame[i] = feature;
                    }
                    else
                    {
                        //Feature coordinates = 0
                        FeatureCoordinatesOnFrame[i] *= 0;
                    }
                }

                return "Camera is running";
            }
            else
            {
                for (int i = 0; i < FeatureCoordinatesOnFrame.Count; i++)
                {
                    if (FeatureCoordinatesOnFrame[i].X != 0)
                    {
                        //Feature coordinates = 0
                        FeatureCoordinatesOnFrame[i] *= 0;
                        //Point3d featureZeros = FeatureCoordinates[i];
                        //featureZeros.X = 0;
                        //featureZeros.Y = 0;
                        //featureZeros.Z = 0;
                        //FeatureCoordinates[i] = featureZeros;
                    }
                }

                return "Camera is running, features not found";
            }
        }

        private void ConvertFrames()
        {
            switch (ViewSource.FrameOfImageSource)
            {
                case FramesToView.InitialFrame:
                    InitialImage = WriteableBitmapConverter.ToWriteableBitmap(InitialFrame);
                    InitialImage.Freeze();
                    ViewSource.AppImageSource = InitialImage;
                    if (ViewSource.FrameOfMapSource == FramesToView.InitialFrame)
                    {
                        ViewSource.AppMapSource = InitialImage;
                        return;
                    }
                    break;
                case FramesToView.UndistortedFrame:
                    UndistortedImage = WriteableBitmapConverter.ToWriteableBitmap(UndistortedFrame);
                    UndistortedImage.Freeze();
                    ViewSource.AppImageSource = UndistortedImage;
                    if (ViewSource.FrameOfMapSource == FramesToView.UndistortedFrame)
                    {
                        ViewSource.AppMapSource = UndistortedImage;
                        return;
                    }
                    break;
                case FramesToView.FrameHSV:
                    HSVImage = WriteableBitmapConverter.ToWriteableBitmap(FrameHSV);
                    HSVImage.Freeze();
                    ViewSource.AppImageSource = HSVImage;
                    if (ViewSource.FrameOfMapSource == FramesToView.FrameHSV)
                    {
                        ViewSource.AppMapSource = HSVImage;
                        return;
                    }
                    break;
                case FramesToView.FrameThresholdedByHSV:
                    ImageThresholdedByHSV = WriteableBitmapConverter.ToWriteableBitmap(FrameThresholdedByHSV);
                    ImageThresholdedByHSV.Freeze();
                    ViewSource.AppImageSource = ImageThresholdedByHSV;
                    if (ViewSource.FrameOfMapSource == FramesToView.FrameThresholdedByHSV)
                    {
                        ViewSource.AppMapSource = ImageThresholdedByHSV;
                        return;
                    }
                    break;
                case FramesToView.FrameErodedAndDelated:
                    ImageErodedAndDelated = WriteableBitmapConverter.ToWriteableBitmap(FrameErodedAndDelated);
                    ImageErodedAndDelated.Freeze();
                    ViewSource.AppImageSource = ImageErodedAndDelated;
                    if (ViewSource.FrameOfMapSource == FramesToView.FrameErodedAndDelated)
                    {
                        ViewSource.AppMapSource = ImageErodedAndDelated;
                        return;
                    }
                    break;
                case FramesToView.MapFrame:
                    MapImage = DrawMap(CameraPosition, Map);
                    MapImage.Freeze();
                    ViewSource.AppImageSource = MapImage;
                    if (ViewSource.FrameOfMapSource == FramesToView.MapFrame)
                    {
                        ViewSource.AppMapSource = MapImage;
                        return;
                    }
                    break;
            }
            switch (ViewSource.FrameOfMapSource)
            {
                case FramesToView.InitialFrame:
                    InitialImage = WriteableBitmapConverter.ToWriteableBitmap(InitialFrame);
                    InitialImage.Freeze();
                    ViewSource.AppMapSource = InitialImage;
                    break;
                case FramesToView.UndistortedFrame:
                    UndistortedImage = WriteableBitmapConverter.ToWriteableBitmap(UndistortedFrame);
                    UndistortedImage.Freeze();
                    ViewSource.AppMapSource = UndistortedImage;
                    break;
                case FramesToView.FrameHSV:
                    HSVImage = WriteableBitmapConverter.ToWriteableBitmap(FrameHSV);
                    HSVImage.Freeze();
                    ViewSource.AppMapSource = HSVImage;
                    break;
                case FramesToView.FrameThresholdedByHSV:
                    ImageThresholdedByHSV = WriteableBitmapConverter.ToWriteableBitmap(FrameThresholdedByHSV);
                    ImageThresholdedByHSV.Freeze();
                    ViewSource.AppMapSource = ImageThresholdedByHSV;
                    break;
                case FramesToView.FrameErodedAndDelated:
                    ImageErodedAndDelated = WriteableBitmapConverter.ToWriteableBitmap(FrameErodedAndDelated);
                    ImageErodedAndDelated.Freeze();
                    ViewSource.AppMapSource = ImageErodedAndDelated;
                    break;
                case FramesToView.MapFrame:
                    MapImage = DrawMap(CameraPosition, Map);
                    MapImage.Freeze();
                    ViewSource.AppMapSource = MapImage;
                    break;
            }
        }

        private ImageSource DrawMap(double[,] cameraPosition, List<Point2d> map)
        {
            Pen shapeOutlinePen = new Pen(Brushes.Black, 2);
            shapeOutlinePen.Freeze();
            Pen gridPen = new Pen(Brushes.Black, 0.5);
            gridPen.Freeze();
            // Create a DrawingGroup
            DrawingGroup dGroup = new DrawingGroup();
            // Obtain a DrawingContext from the DrawingGroup
            using (DrawingContext dc = dGroup.Open())
            {
                int imageWidth = 640;
                int imageHeight = 480;
                int centerX = imageWidth / 2;
                int centerY = imageHeight / 2;
                double milimetersToPixelCoefficient = 0.1;
                double camera_X = cameraPosition[0, 0] * milimetersToPixelCoefficient;
                double camera_Z = -cameraPosition[3, 0] * milimetersToPixelCoefficient;
                double camera_Theta = cameraPosition[6, 0];
 
                double camera_ViewLine = 1000;
                double viewRange = 60 * Math.PI / 180;
                double featureSize = 45 / 2 * milimetersToPixelCoefficient;
                double cameraSize = 100 / 2 * milimetersToPixelCoefficient;

                // Draw a rectangle around the camera
                dc.DrawRectangle(Brushes.White, shapeOutlinePen, new System.Windows.Rect(camera_X - centerX, camera_Z - centerY, imageWidth, imageHeight));

                // Draw camera
                dc.DrawEllipse(Brushes.Black, shapeOutlinePen, new System.Windows.Point(camera_X, camera_Z), cameraSize, cameraSize);
                

                // Draws features located in a rectangle
                for (int i = 0; i < map.Count; i++)
                {
                    if(map[i].X * milimetersToPixelCoefficient > camera_X - imageWidth / 2 && 
                       map[i].X * milimetersToPixelCoefficient < camera_X + imageWidth / 2 && 
                       -map[i].Y * milimetersToPixelCoefficient < camera_Z + imageHeight / 2 &&
                       -map[i].Y * milimetersToPixelCoefficient > camera_Z - imageHeight / 2)
                    {
                        dc.DrawEllipse(Brushes.Red, shapeOutlinePen, new System.Windows.Point(map[i].X * milimetersToPixelCoefficient, 
                                                                                              -map[i].Y * milimetersToPixelCoefficient),
                                                                                              featureSize, 
                                                                                              featureSize);
                    }
                }
                dc.PushOpacity(0.5);
                dc.DrawLine(shapeOutlinePen, new System.Windows.Point(camera_X, camera_Z),
                                             new System.Windows.Point(camera_X + camera_ViewLine * Math.Sin(camera_Theta + viewRange / 2) * milimetersToPixelCoefficient,
                                                                      camera_Z - camera_ViewLine * Math.Cos(camera_Theta + viewRange / 2) * milimetersToPixelCoefficient));

                dc.DrawLine(shapeOutlinePen, new System.Windows.Point(camera_X, camera_Z),
                                             new System.Windows.Point(camera_X + camera_ViewLine * Math.Sin(camera_Theta - viewRange / 2) * milimetersToPixelCoefficient,
                                                                      camera_Z - camera_ViewLine * Math.Cos(camera_Theta - viewRange / 2) * milimetersToPixelCoefficient));
                //Draw grid
                double gridSize = cameraSize * 20;
                for (double i = 0; i <= imageHeight + gridSize; i += gridSize)
                {
                    if (camera_Z - camera_Z % gridSize - centerY + i < camera_Z + imageHeight / 2 &&
                        camera_Z - camera_Z % gridSize - centerY + i > camera_Z - imageHeight / 2)
                    {
                        dc.DrawLine(gridPen, new System.Windows.Point(camera_X - centerX, camera_Z - camera_Z % gridSize - centerY + i),
                                             new System.Windows.Point(camera_X - centerX + imageWidth, camera_Z - camera_Z % gridSize - centerY + i));
                    }
                }
                for (double i = 0; i <= imageWidth + gridSize; i += gridSize)
                {
                    if (camera_X - camera_X % gridSize - centerX + i > camera_X - imageWidth / 2 &&
                        camera_X - camera_X % gridSize - centerX + i < camera_X + imageWidth / 2)
                    {
                        dc.DrawLine(gridPen, new System.Windows.Point(camera_X - camera_X % gridSize - centerX + i, camera_Z - centerY),
                                             new System.Windows.Point(camera_X - camera_X % gridSize - centerX + i, camera_Z - centerY + imageHeight));
                    }
                }
            }
            
            // Display the drawing using an image control.
            MapDrawingImage = new DrawingImage(dGroup);

            int MapFrameWidth = 640;
            int MapFrameHeight = 480;

            DrawingVisual drawingVisual = new DrawingVisual();
            DrawingContext drawingContext = drawingVisual.RenderOpen();
            drawingContext.DrawImage(MapDrawingImage, new System.Windows.Rect(new System.Windows.Point(0, 0), new System.Windows.Size(MapFrameWidth, MapFrameHeight)));
            drawingContext.Close();

            RenderTargetBitmap BitmapSourceFrame = new RenderTargetBitmap(MapFrameWidth, MapFrameHeight, 96, 96, PixelFormats.Pbgra32);
            BitmapSourceFrame.Render(drawingVisual);

            return BitmapSourceFrame;
        }
    }
    public enum FramesToView
    {
        InitialFrame = 1,
        UndistortedFrame = 2,
        FrameHSV = 3,
        FrameThresholdedByHSV = 4,
        FrameErodedAndDelated = 5,
        MapFrame = 6
    }
}