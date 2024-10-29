using System.ComponentModel;
using System.Runtime.CompilerServices;
using System.Windows;
using System.Windows.Controls;
using System.Windows.Media;

namespace OpenCV_WPF_try3;

public partial class View : System.Windows.Window
{
    public ViewBindingSource ViewSource { get; set; }
    private ViewModel ViewModel { get; set; }

    public View()
    {
        InitializeComponent();

        ViewSource = (ViewBindingSource)DataContext;
        ViewModel = new ViewModel(ViewSource);

        WindowImageChooze.ItemsSource = new FramesToView[] { FramesToView.InitialFrame, FramesToView.UndistortedFrame, FramesToView.FrameHSV,
                                                 FramesToView.FrameThresholdedByHSV, FramesToView.FrameErodedAndDelated, FramesToView.MapFrame};
        WindowMapChooze.ItemsSource = WindowImageChooze.ItemsSource;

        WindowImageButton.Click += ImageButton_Click;
        WindowImageChooze.SelectionChanged += WindowImageChooze_SelectionChanged;
        WindowMapChooze.SelectionChanged += WindowMapChooze_SelectionChanged;
        CalibrationButton.Click += CalibrationButton_Click;

        WindowImageChooze.SelectedItem = FramesToView.UndistortedFrame;
        WindowMapChooze.SelectedItem = FramesToView.MapFrame;

        ViewSource.AppTextSource = "Initialization complited";
    }

    private void CalibrationButton_Click(object sender, RoutedEventArgs e)
    {
        ViewModel.CameraCalibration();
    }
    private void WindowImageChooze_SelectionChanged(object sender, SelectionChangedEventArgs e)
    {
        ViewModel.ViewSource.FrameOfImageSource = (FramesToView)WindowImageChooze.SelectedItem;
    }
    private void WindowMapChooze_SelectionChanged(object sender, SelectionChangedEventArgs e)
    {
        ViewModel.ViewSource.FrameOfMapSource = (FramesToView)WindowMapChooze.SelectedItem; ;
    }
    private void ImageButton_Click(object sender, RoutedEventArgs e)
    {
        ViewModel.StartOrStopCamera();
    }
}
public class ViewBindingSource : INotifyPropertyChanged
    {
        public event PropertyChangedEventHandler? PropertyChanged;

        public FramesToView FrameOfImageSource { get; set; }
        public FramesToView FrameOfMapSource { get; set; }

        private ImageSource? _appImageSource;
        private ImageSource? _appMapSource;
        private string? _appTextSource;
        private int _thresholdHSV_HLow;
        private int _thresholdHSV_SLow;
        private int _thresholdHSV_VLow;
        private int _thresholdHSV_HHigh;
        private int _thresholdHSV_SHigh;
        private int _thresholdHSV_VHigh;
        private int _sizeOfShapeToRemove;
        private int _sizeOfHoleToFill;
        public ImageSource AppImageSource
        {
            get
            {
                return _appImageSource!;
            }
            set
            {
                _appImageSource = value;
                OnPropertyChanged();
            }
        }

        public ImageSource AppMapSource
        {
            get
            {
                return _appMapSource!;
            }
            set
            {
                _appMapSource = value;
                OnPropertyChanged();
            }
        }
        public string AppTextSource
        {
            get
            {
                return _appTextSource!;
            }
            set
            {
                _appTextSource = value;
                OnPropertyChanged();
            }
        }

        public int ThresholdHSV_HLow
        {
            get
            {
                return _thresholdHSV_HLow;
            }
            set
            {
                _thresholdHSV_HLow = value;
                OnPropertyChanged();
            }
        }

        public int ThresholdHSV_HHigh
        {
            get
            {
                return _thresholdHSV_HHigh;
            }
            set
            {
                _thresholdHSV_HHigh = value;
                OnPropertyChanged();
            }
        }

        public int ThresholdHSV_SLow
        {
            get
            {
                return _thresholdHSV_SLow;
            }
            set
            {
                _thresholdHSV_SLow = value;
                OnPropertyChanged();
            }
        }
        public int ThresholdHSV_SHigh
        {
            get
            {
                return _thresholdHSV_SHigh;
            }
            set
            {
                _thresholdHSV_SHigh = value;
                OnPropertyChanged();
            }
        }
        public int ThresholdHSV_VLow
        {
            get
            {
                return _thresholdHSV_VLow;
            }
            set
            {
                _thresholdHSV_VLow = value;
                OnPropertyChanged();
            }
        }
        public int ThresholdHSV_VHigh
        {
            get
            {
                return _thresholdHSV_VHigh;
            }
            set
            {
                _thresholdHSV_VHigh = value;
                OnPropertyChanged();
            }
        }
        public int SizeOfShapeToRemove
        {
            get
            {
                return _sizeOfShapeToRemove;
            }
            set
            {
                _sizeOfShapeToRemove = value;
                OnPropertyChanged();
            }
        }
        public int SizeOfHoleToFill
        {
            get
            {
                return _sizeOfHoleToFill;
            }
            set
            {
                _sizeOfHoleToFill = value;
                OnPropertyChanged();
            }
        }
        public ViewBindingSource()
        {
            //355/255
            _thresholdHSV_HHigh = 30;
            _thresholdHSV_HLow = 0;
            _thresholdHSV_SHigh = 100;
            _thresholdHSV_SLow = 30;
            _thresholdHSV_VHigh = 100;
            _thresholdHSV_VLow = 20;
            _sizeOfShapeToRemove = 5;
            _sizeOfHoleToFill = 15;
        }

        protected void OnPropertyChanged([CallerMemberName] string? name = null)
        {
            PropertyChanged?.Invoke(this, new PropertyChangedEventArgs(name));
        }
    }