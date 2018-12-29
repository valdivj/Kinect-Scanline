

namespace ScanLineCamera
{
    using System;

    using System.Collections.Generic;
    using System.ComponentModel;
    using System.Diagnostics;
    using System.Globalization;
    using System.Windows.Threading;
    using System.Windows.Shapes;
    using System.IO;
    using System.Windows;
    using System.Windows.Media;
    using System.Windows.Media.Imaging;
    using System.Windows.Input;
    using Microsoft.Kinect;
    using AdvancedHMIDrivers;
    using MfgControl.AdvancedHMI;
    using System.Windows.Controls;

    /// <summary>
    /// Interaction logic for MainWindow
    /// </summary>
    public partial class MainWindow : Window
    {

      private const int MapDepthToByte = 8000 / 256;
     //   private const int MapDepthToByte = 256/256;
        private KinectSensor kinectSensor = null;
        private DepthFrameReader depthFrameReader = null;
        private ColorFrameReader _colorReader = null;
        private FrameDescription depthFrameDescription = null;
        private WriteableBitmap depthBitmap = null;
        private WriteableBitmap _bmp = null;
        private byte[] depthPixels = null;
        ushort[] _depthData = null;
        int _depthHeight;
        int _depthWidth;
       // int x;
       // int y;
        string Red_Dispaly_Result;
        string Blue_Dispaly_Result;
        string Green_Dispaly_Result;
        string Yellow_Dispaly_Result;
       
              

        EthernetIPforCLXComm CookieJar = new EthernetIPforCLXComm();
        DispatcherTimer timerReadPLC = new DispatcherTimer();
        DispatcherTimer timerReadMes = new DispatcherTimer();
        DispatcherTimer SliderLines = new DispatcherTimer();


         public MainWindow()
         {
             this.InitializeComponent();
             this.kinectSensor = KinectSensor.GetDefault();

             this.depthFrameReader = this.kinectSensor.DepthFrameSource.OpenReader();         
             this.depthFrameReader.FrameArrived += this.Reader_FrameArrived;
             this.depthFrameDescription = this.kinectSensor.DepthFrameSource.FrameDescription;
             this.depthPixels = new byte[this.depthFrameDescription.Width * this.depthFrameDescription.Height];
             this.depthBitmap = new WriteableBitmap(this.depthFrameDescription.Width, this.depthFrameDescription.Height, 96.0, 96.0, PixelFormats.Gray8, null);
          // this.rgb.Source = depthBitmap;

             this._colorReader = this.kinectSensor.ColorFrameSource.OpenReader();
             this._colorReader.FrameArrived += this._colorReader_FrameArrived;      
             var description = _colorReader.ColorFrameSource.CreateFrameDescription(ColorImageFormat.Bgra);
             this._bmp = new WriteableBitmap(description.Width, description.Height, 96.0, 96.0, PixelFormats.Bgra32, null);
            this.rgblive.Source = _bmp;  
                     
             this.rgblive.MouseLeftButtonUp += DepthImage_MouseLeftButtonUp;

             this.kinectSensor.Open();
             this._depthHeight = kinectSensor.DepthFrameSource.FrameDescription.Height;
             this._depthWidth = kinectSensor.DepthFrameSource.FrameDescription.Width;

             this.sliderYRed.Value = 300;
             this.sliderRed_L.Value = 0;
             this.sliderRed_R.Value = 1920;
             this.sliderYBlue.Value = 700;
             this.sliderBlue_L.Value = 0;
             this.sliderBlue_R.Value = 1920;     
             this.sliderXYellow.Value = 500;
             this.sliderYellow_T.Value = 0;
             this.sliderYellow_B.Value = 1080;
             this.sliderXGrn.Value = 1500;
             this.sliderGreen_T.Value = 0;
             this.sliderGreen_B.Value = 1080;

             this.timerReadPLC.Interval = TimeSpan.FromMilliseconds(100); //every 200ms update
             this.timerReadPLC.Tick += new EventHandler(timerReadPLC_Tick);
             this.timerReadMes.Interval = TimeSpan.FromMilliseconds(100); //every 200ms update
             this.timerReadMes.Tick += new EventHandler(timerReadMes_Tick);
             this.SliderLines.Interval = TimeSpan.FromMilliseconds(100); //every 200ms update
             this.SliderLines.Tick += new EventHandler(Sliderlines_Tick);

             this.radioButton1.IsChecked = true;
             this. RGB_Label.Content = 1;

             this.RedPixelSet.Text = Settings1.Default.RedPixelSet;
             this.BluePixelSet.Text = Settings1.Default.BluePixelSet;
             this.GreenPixelSet.Text = Settings1.Default.GreenPixelSet;
            this.YellowPixelSet.Text = Settings1.Default.YellowPixelSet;

             this.IP_Address.Text = Settings1.Default.IP_Address;
             this.Low.Text = "1";
             this.HIGH1.Text = "255";
             this.sliderGraph_High.Value = 255;
             CookieJar.IPAddress = Settings1.Default.IP_Address; 
             rotation.Content = "90";
             amplification.Text = "10";
         }
        private void MainWindow_Closing(object sender, CancelEventArgs e)
        {
            if (this.depthFrameReader != null)
            {
                // DepthFrameReader is IDisposable
                this.depthFrameReader.Dispose();
                this.depthFrameReader = null;
            }

            if (this.kinectSensor != null)
            {
                this.kinectSensor.Close();
                this.kinectSensor = null;
                timerReadPLC.Stop();
            }
        }
        #region ImageProcessing
        private void Reader_FrameArrived(object sender, DepthFrameArrivedEventArgs e)
        {
            bool depthFrameProcessed = false;

            using (DepthFrame depthFrame = e.FrameReference.AcquireFrame())
            {
                if (depthFrame != null)
                {
                    _depthData = new ushort[depthFrame.FrameDescription.LengthInPixels];
                    depthFrame.CopyFrameDataToArray(_depthData);

                    using (Microsoft.Kinect.KinectBuffer depthBuffer = depthFrame.LockImageBuffer())
                    {

                        if (((this.depthFrameDescription.Width * this.depthFrameDescription.Height) == (depthBuffer.Size / this.depthFrameDescription.BytesPerPixel)) &&
                            (this.depthFrameDescription.Width == this.depthBitmap.PixelWidth) && (this.depthFrameDescription.Height == this.depthBitmap.PixelHeight))
                        {
                            // Note: In order to see the full range of depth (including the less reliable far field depth)
                            // we are setting maxDepth to the extreme potential depth threshold
                            ushort maxDepth = ushort.MaxValue;
                            //ushort maxDepth = 2700;

                            // If you wish to filter by reliable depth distance, uncomment the following line:
                            // maxDepth = depthFrame.DepthMaxReliableDistance;

                            this.ProcessDepthFrameData(depthBuffer.UnderlyingBuffer, depthBuffer.Size, depthFrame.DepthMinReliableDistance, maxDepth);
                            depthFrameProcessed = true;
                        }
                    }
                }
            }

            if (depthFrameProcessed)
            {
                this.RenderDepthPixels();
            }
        }
        void _colorReader_FrameArrived(object sender, ColorFrameArrivedEventArgs e)
        {
            using (var colorFrame = e.FrameReference.AcquireFrame())
             
            {
                if (colorFrame != null)
                   
                {
           
                    FrameDescription colorFrameDescription = colorFrame.FrameDescription;
                    using (KinectBuffer colorBuffer = colorFrame.LockRawImageBuffer())
                    {
                        _bmp.Lock();

                        // verify data and write the new color frame data to the display bitmap
                        if ((colorFrameDescription.Width == _bmp.PixelWidth) && (colorFrameDescription.Height == _bmp.PixelHeight))
                        {
                            colorFrame.CopyConvertedFrameDataToIntPtr(
                                _bmp.BackBuffer,
                                (uint)(colorFrameDescription.Width * colorFrameDescription.Height * 4),
                                ColorImageFormat.Bgra);

                            _bmp.AddDirtyRect(new Int32Rect(0, 0, _bmp.PixelWidth, _bmp.PixelHeight));

                        }
                            
                        _bmp.Unlock();
                    }
                }
            }
        }

        private unsafe void ProcessDepthFrameData(IntPtr depthFrameData, uint depthFrameDataSize, ushort minDepth, ushort maxDepth)
        {
            // depth frame data is a 16 bit value
            ushort* frameData = (ushort*)depthFrameData;

            // convert depth to a visual representation
            for (int i = 0; i < (int)(depthFrameDataSize / this.depthFrameDescription.BytesPerPixel); ++i)
            {
                // Get the depth for this pixel
                ushort depth = frameData[i];

                // To convert to a byte, we're mapping the depth value to the byte range.
                // Values outside the reliable depth range are mapped to 0 (black).
                this.depthPixels[i] = (byte)(depth >= minDepth && depth <= maxDepth ? (depth / MapDepthToByte) : 0);
            }
        }

        private void RenderDepthPixels()
        {
            this.depthBitmap.WritePixels(
                new Int32Rect(0, 0, this.depthBitmap.PixelWidth, this.depthBitmap.PixelHeight),
                this.depthPixels,
                this.depthBitmap.PixelWidth,
                0);
        }
        #endregion

        private void Start_Measure(object sender, RoutedEventArgs e)
        {
            rotation.Content = "90";
            timerReadMes.Start();
            SliderLines.Start();
            
        }

        private void DepthImage_MouseLeftButtonUp(object sender, MouseButtonEventArgs e)
        {

            Point p = e.GetPosition(rgblive);
            BitmapSource img = _bmp;
            int stride = _bmp.PixelWidth * 4;
            int size = _bmp.PixelHeight * stride;
            byte[] pixels = new byte[size];
            _bmp.CopyPixels(pixels, stride, 0);

            {                     
                    int x = Convert.ToInt16(Math.Floor(p.X * _bmp.PixelWidth / this.rgblive.ActualWidth));
                    int y = Convert.ToInt16( Math.Floor(p.Y * _bmp.PixelHeight / this.rgblive.ActualHeight));
                    int index = y * stride + 4 * x;
                    var blue = pixels[index];
                    int green = pixels[index + 1];
                    int red = pixels[index + 2];
                 // byte alpha = pixels[index + 3];
                    PixelDepth.Text = "X:" + (x.ToString() + ", Y:" + y.ToString() + "  R:" + red.ToString() + "  G:" + green.ToString() + "  B:" + blue.ToString());
             }
           }

        #region MesureControl
        private void timerReadMes_Tick(object sender, EventArgs e)
            {
                BitmapSource img = _bmp;
                int stride = _bmp.PixelWidth * 4;
                int size = _bmp.PixelHeight * stride;
                byte[] pixels = new byte[size];
                _bmp.CopyPixels(pixels, stride, 0);

                {                           
                    int[] RedLine;
                    RedLine = new int[Convert.ToInt32(sliderRed_R.Value)];
                    int[] RedpixelIndexcall;
                    RedpixelIndexcall = new int[1920];
                    int[] Reddepthcall;
                    Reddepthcall = new int[1920];

                    int[] BlueLine;
                    BlueLine = new int[Convert.ToInt32(sliderBlue_R.Value)];
                    int[] BluepixelIndexcall;
                    BluepixelIndexcall = new int[1920];
                    int[] Bluedepthcall;
                    Bluedepthcall = new int[1920];

                    int[] GreenLine;
                    GreenLine = new int[Convert.ToInt32(sliderGreen_B.Value)];
                    int[] GreenpixelIndexcall;
                    GreenpixelIndexcall = new int[1078];
                    int[] Greendepthcall;
                    Greendepthcall = new int[1078];

                    int[] YellowLine;
                    YellowLine = new int[Convert.ToInt32(sliderYellow_B.Value)];
                    int[] YellowpixelIndexcall;
                    YellowpixelIndexcall = new int[1078];
                    int[] Yellowdepthcall;
                    Yellowdepthcall = new int[1078];

                    int[] RedmatchedItems;
                    int[] BluematchedItems;
                    int[] GreenmatchedItems;
                    int[] YellowmatchedItems;

                    int[] RedmatchedItemsAngle;
                    RedmatchedItemsAngle = new int[1000];
                    int[] RedmatchedItemsAngleFinal;

                    int[] BluematchedItemsAngle;
                    BluematchedItemsAngle = new int[1000];
                    int[] BluematchedItemsAngleFinal;

                    int[] GreenmatchedItemsAngle;
                    GreenmatchedItemsAngle = new int[500];
                    int[] GreenmatchedItemsAngleFinal;

                    int[] YellowmatchedItemsAngle;
                    YellowmatchedItemsAngle = new int[500];
                    int[] YellowmatchedItemsAngleFinal;

                    int lBound = Convert.ToInt32(Low.Text);
                    int uBound = Convert.ToInt32(HIGH1.Text);

                    {
                       // for (int i = xfactorColor.Length; i-- > 0; )
                       //     xfactorColor[i] = i + 1;
                        for (int r = RedLine.Length; r-- > 0; )
                            RedLine[r] = r + 1;

                        for (int b = BlueLine.Length; b-- > 0; )
                            BlueLine[b] = b + 1;

                        for (int g = 0; g != (GreenLine.Length); g++)
                            GreenLine[g] = g + 1;

                        for (int y = 0; y != (YellowLine.Length); y++)
                            YellowLine[y] = y + 1;
                    }
                            int RedY = Convert.ToInt32(sliderYRed.Value);
                            for (int r = Convert.ToInt32(sliderRed_L.Value); r != (RedLine.Length); r++)
                              RedpixelIndexcall[r] = (int)(RedY * stride + 4 * (RedLine[r]));

                            int BlueY = Convert.ToInt32(sliderYBlue.Value);
                            for (int b = Convert.ToInt32(sliderBlue_L.Value); b != (BlueLine.Length); b++)
                              BluepixelIndexcall[b] = (int)(BlueY * stride + 4 * (BlueLine[b]));

                            int GreenX = Convert.ToInt32(sliderXGrn.Value);
                            for (int g = Convert.ToInt32(sliderGreen_T.Value); g != (GreenLine.Length); g++)
                                GreenpixelIndexcall[g] = (int)((GreenLine[g]) * stride + 4 * GreenX);

                            int YellowX = Convert.ToInt32(sliderXYellow.Value);
                            for (int y = Convert.ToInt32(sliderYellow_T.Value); y != (YellowLine.Length); y++)
                                YellowpixelIndexcall[y] = (int)((YellowLine[y]) * stride + 4 * YellowX);

                    {
                        for (int r = RedpixelIndexcall.Length; r-- > 0; )
                            Reddepthcall[r] = pixels[RedpixelIndexcall[r] + Convert.ToInt32(RGB_Label.Content)];
                        RedmatchedItems = Array.FindAll(Reddepthcall, x =>
                        x >= lBound && x <= uBound);
                        for (int ctr = 0; ctr < RedmatchedItems.Length; ctr++) ;
                        int Redcount = RedmatchedItems.Length;

                        Array.Copy(Reddepthcall, RedmatchedItemsAngle, 1000);
                        RedmatchedItemsAngleFinal = Array.FindAll(RedmatchedItemsAngle, x =>
                       x >= lBound && x <= uBound);
                        for (int ctr = 0; ctr < RedmatchedItemsAngleFinal.Length; ctr++) ;
                        int RedcountAngle = RedmatchedItemsAngleFinal.Length;

                        Double Redsum = Convert.ToDouble(Settings1.Default.RedPixelSet);
                        double RedSumCount = Math.Round((Redcount * Redsum), 2);
                        RedCount.Content = Convert.ToString(Redcount);
                        RedSum.Content = Convert.ToString(RedSumCount);
                       // Red_Dispaly_Result = Convert.ToString(RedSumCount);
                   //  CookieJar.WriteData("CookieJar_Int_Array[1]", Convert.ToString(Redcount * Redsum));
                       

                        for (int b = BluepixelIndexcall.Length; b-- > 0; )
                            Bluedepthcall[b] = pixels[BluepixelIndexcall[b] + Convert.ToInt32(RGB_Label.Content)];
                        BluematchedItems = Array.FindAll(Bluedepthcall, x =>
                        x >= lBound && x <= uBound);
                        for (int ctr = 0; ctr < BluematchedItems.Length; ctr++) ;
                        int Bluecount = BluematchedItems.Length;

                        Array.Copy(Bluedepthcall, BluematchedItemsAngle, 1000);
                        BluematchedItemsAngleFinal = Array.FindAll(BluematchedItemsAngle, x =>
                       x >= lBound && x <= uBound);
                        for (int ctr = 0; ctr < BluematchedItemsAngleFinal.Length; ctr++) ;
                        int BluecountAngle = BluematchedItemsAngleFinal.Length;

                        Double Bluesum = Convert.ToDouble(Settings1.Default.BluePixelSet);
                        BlueCount.Content = Convert.ToString(Bluecount);
                        BlueSum.Content = Convert.ToString(Math.Round((Bluecount * Bluesum), 2));
                        Blue_Dispaly_Result = Convert.ToString(Bluecount * Bluesum);
                       // CookieJar.WriteData("CookieJar_Int_Array[2]", Convert.ToString(Bluecount * Bluesum));

                        for (int g = 0; g != (GreenpixelIndexcall.Length); g++)
                            Greendepthcall[g] = pixels[GreenpixelIndexcall[g] + Convert.ToInt32(RGB_Label.Content)];
                        GreenmatchedItems = Array.FindAll(Greendepthcall, x =>
                        x >= lBound && x <= uBound);
                        for (int ctr = 0; ctr < GreenmatchedItems.Length; ctr++) ;
                        int Greencount = GreenmatchedItems.Length;

                        Array.Copy(Greendepthcall, GreenmatchedItemsAngle, 500);
                        GreenmatchedItemsAngleFinal = Array.FindAll(GreenmatchedItemsAngle, x =>
                       x >= lBound && x <= uBound);
                        for (int ctr = 0; ctr < GreenmatchedItemsAngleFinal.Length; ctr++) ;
                        int GreencountAngle = GreenmatchedItemsAngleFinal.Length;

                        Double Greensum = Convert.ToDouble(Settings1.Default.GreenPixelSet);
                        GreenCount.Content = Convert.ToString(Greencount);
                        GreenSum.Content = Convert.ToString(Math.Round((Greencount * Greensum), 2));
                        Green_Dispaly_Result = Convert.ToString(Greencount * Greensum);
                       // CookieJar.WriteData("CookieJar_Int_Array[3]", Convert.ToString(Greencount * Greensum));

                        for (int y = 0; y != (YellowpixelIndexcall.Length); y++)
                            Yellowdepthcall[y] = pixels[YellowpixelIndexcall[y] + Convert.ToInt32(RGB_Label.Content)];
                        YellowmatchedItems = Array.FindAll(Yellowdepthcall, x =>
                        x >= lBound && x <= uBound);
                        for (int ctr = 0; ctr < YellowmatchedItems.Length; ctr++) ;
                        int Yellowcount = YellowmatchedItems.Length;

                        Array.Copy(Yellowdepthcall, YellowmatchedItemsAngle, 500);
                        YellowmatchedItemsAngleFinal = Array.FindAll(YellowmatchedItemsAngle, x =>
                       x >= lBound && x <= uBound);
                        for (int ctr = 0; ctr < YellowmatchedItemsAngleFinal.Length; ctr++) ;
                        int YellowcountAngle = YellowmatchedItemsAngleFinal.Length;

                        Double Yellowsum = Convert.ToDouble(Settings1.Default.YellowPixelSet);
                        YellowCount.Content = Convert.ToString(Yellowcount);
                        YellowSum.Content = Convert.ToString(Math.Round((Yellowcount * Yellowsum), 2));
                        Yellow_Dispaly_Result = Convert.ToString(Yellowcount * Yellowsum);
                       // CookieJar.WriteData("CookieJar_Int_Array[4]", Convert.ToString(Yellowcount * Yellowsum));

                        int p1x = (1920 - (RedmatchedItemsAngleFinal.Length + 920));
                        int p2x = (1920 - (BluematchedItemsAngleFinal.Length + 920));
                        int p1y = Convert.ToInt32(sliderYRed.Value);
                        int p2y = Convert.ToInt32(sliderYBlue.Value);                   
                        RedLineX.Content = Convert.ToString(p1x);
                        BlueLineX.Content = Convert.ToString(p2x);
                        Point p1 = new Point(p1x, p1y);
                        Point p2 = new Point(p2x, p2y);
                        double radians = Math.Atan2(p2.Y - p1.Y, p2.X - p1.X);
                        double Angle = (radians * (180.0 / Math.PI));
                        double RedCorrected = (Math.Sin(radians) * (Redcount * Redsum));
                        double BlueCorrected = (Math.Sin(radians) * (Bluecount * Bluesum));
                        AngleLBL.Content = Convert.ToString(Math.Round(Angle,2));
                        Corrected.Content = Convert.ToString(Math.Round(RedCorrected, 2));
                        Blue_Corrected.Content = Convert.ToString(Math.Round(BlueCorrected, 2));

                        int GYp1x = (1078 - (GreenmatchedItemsAngleFinal.Length + 578));
                        int GYp2x = (1078 - (YellowmatchedItemsAngleFinal.Length + 578));
                        int GYp1y = Convert.ToInt32(sliderXGrn.Value);
                        int GYp2y = Convert.ToInt32(sliderXYellow.Value);
                        GreenLineX.Content = Convert.ToString(GYp1x);
                        YellowLineX.Content = Convert.ToString(GYp2x);
                        Point GYp1 = new Point(GYp1x,GYp1y);
                        Point GYp2 = new Point(GYp2x, GYp2y);
                        double GYradians = Math.Atan2(GYp2.Y - GYp1.Y, GYp2.X - GYp1.X);
                        double GYAngle = (radians * (360.0 / Math.PI));
                        double GYGreenCorrected = (Math.Sin(radians) * (Greencount * Greensum));
                        double GYYellowCorrected = (Math.Sin(radians) * (Yellowcount * Yellowsum));
                        GY_AngleLBL.Content = Convert.ToString(Math.Round(GYAngle, 2));
                        Green_Corrected.Content = Convert.ToString(Math.Round(GYGreenCorrected, 2));
                        Yellow_Corrected.Content = Convert.ToString(Math.Round(GYYellowCorrected, 2));

                    }
                  
                    List<TodoItem> items = new List<TodoItem>();
                    if (radioButton1.IsChecked == true)
                        for (int r = Reddepthcall.Length; r-- > 0; r -= 3)
                            items.Add(new TodoItem() { Completion = Convert.ToInt32(Reddepthcall[r] * 10 ) });
                //      for (int r = RedmatchedItems.Length; r-- > 0; )
                //     items.Add(new TodoItem() { Completion = Convert.ToInt32(RedmatchedItems[r] * 10) });

                    else if (radioButton2.IsChecked == true)
                        for (int b = Bluedepthcall.Length; b-- > 0; b -= 3)
                            items.Add(new TodoItem() { Completion = Convert.ToInt32(Bluedepthcall[b] * 10) });
                 //     for (int b = BluematchedItems.Length; b-- > 0; )
                   //  items.Add(new TodoItem() { Completion = Convert.ToInt32(BluematchedItems[b] * 10) });

                    else if (radioButton3.IsChecked == true)
                        for (int g = Greendepthcall.Length; g-- > 0; g -= 2)
                            items.Add(new TodoItem() { Completion = Convert.ToInt32(Greendepthcall[g] * 10) });
                    //  for (int g = GreenmatchedItems.Length; g-- > 0; )
                    //items.Add(new TodoItem() { Completion = Convert.ToInt32(GreenmatchedItems[g] * 10) });

                    else if (radioButton4.IsChecked == true)
                        for (int y = Yellowdepthcall.Length; y-- > 0; y -= 2)
                            items.Add(new TodoItem() { Completion = Convert.ToInt32(Yellowdepthcall[y] * 10) });
                    // for (int y = YellowmatchedItems.Length; y-- > 0; )
                    //  items.Add(new TodoItem() { Completion = Convert.ToInt32(YellowmatchedItems[y] * 10) });
                  icTodoList.ItemsSource = items;

                 
                    }

                }

        public class TodoItem
        {
            public int Completion { get; set; }
        }

        #endregion

        #region AdvancedPLC_Comms
        private void timerReadPLC_Tick(object sender, EventArgs e)
        {
            string[] Display;
            Display =  new string[5];
            try
            {
                for (int i = 0; i != (Display.Length); i++)
                    Display[i] = CookieJar.ReadAny("Cookie_Jar_Real_Array["+ (i) +"]");
                Display1.Text = Display[1] + "  in";
                Display2.Text = Display[2] + "  in";
                Display3.Text = Display[3] + "  in";
                Display4.Text = Display[4] + "  in";

                CookieJar.WriteData("CookieJar_Int_Array[1]",Red_Dispaly_Result);
                CookieJar.WriteData("CookieJar_Int_Array[2]",Blue_Dispaly_Result);
                CookieJar.WriteData("CookieJar_Int_Array[3]",Green_Dispaly_Result);
                CookieJar.WriteData("CookieJar_Int_Array[4]",Yellow_Dispaly_Result);

                Comms_Status_Display.Text = "Comm ok";
                Comms_Status_Display.Foreground = new SolidColorBrush(Colors.Green);
            }
            catch (Exception)
            {
                Comms_Status_Display.Text = "Comm Error Check connection and Restart";
                Comms_Status_Display.Foreground = new SolidColorBrush(Colors.Red);
                timerReadPLC.Stop();
            }
        }
    

        private void Start_PLC(object sender, RoutedEventArgs e)
        {
                try
                {
                    CookieJar.WriteData("CookieJar_Bit_Array[0]", 1);
                    timerReadPLC.Start();
                    Comms_Status_Display.Text = "Comm ok";
                    Comms_Status_Display.Foreground = new SolidColorBrush(Colors.Green);
                 //   Start_Measure_Button.Visibility = Visibility.Visible;
                }
                catch (Exception)
                {
                    Comms_Status_Display.Text = "Comm Error Check I.P Address or Cable";
                    Comms_Status_Display.Foreground = new SolidColorBrush(Colors.Red);
                }
                   
        }
        #endregion
     
        #region  GraphControls  
        private void radioButton1_Checked(object sender, RoutedEventArgs e)
        {
            GraphColor.Text = "Red";
            GraphColor.Foreground = new SolidColorBrush(Colors.Red);
        }

        private void radioButton2_Checked(object sender, RoutedEventArgs e)
        {
            GraphColor.Text = "Blue";
            GraphColor.Foreground = new SolidColorBrush(Colors.Blue);
        }

        private void radioButton3_Checked(object sender, RoutedEventArgs e)
        {
            GraphColor.Text = "Green";
            GraphColor.Foreground = new SolidColorBrush(Colors.Green);
        }

        private void radioButton4_Checked(object sender, RoutedEventArgs e)
        {
            GraphColor.Text = "Yellow";
            GraphColor.Foreground = new SolidColorBrush(Colors.Yellow);
        }

        private void R_Button_Click(object sender, RoutedEventArgs e)
        {
            RGB_Label.Content = 0;
            R_Button.Background = new SolidColorBrush(Colors.Orange);
            G_Button.Background = new SolidColorBrush(Colors.LightGray);
            B_Button.Background = new SolidColorBrush(Colors.LightGray);
        }

        private void G_Button_Click(object sender, RoutedEventArgs e)
        {
            RGB_Label.Content = 1;
            G_Button.Background = new SolidColorBrush(Colors.Orange);
            B_Button.Background = new SolidColorBrush(Colors.LightGray);
            R_Button.Background = new SolidColorBrush(Colors.LightGray);
        }

        private void B_Button_Click(object sender, RoutedEventArgs e)
        {
            RGB_Label.Content = 2;
            B_Button.Background = new SolidColorBrush(Colors.Orange);
            G_Button.Background = new SolidColorBrush(Colors.LightGray);
            R_Button.Background = new SolidColorBrush(Colors.LightGray);
        }
        # endregion

        #region SliderControl
        void Sliderlines_Tick(object sender, EventArgs e)
        {
            myCanvas.Children.Clear();
            Graph_Canvas.Children.Clear();
            {
                Line line = new Line();
                Thickness thickness = new Thickness(101, -11, 362, 250);
                line.Margin = thickness;
                line.Visibility = System.Windows.Visibility.Visible;
                line.StrokeThickness = 2;
                line.Stroke = System.Windows.Media.Brushes.Red;
                line.X1 = (Convert.ToInt32(sliderRed_R.Value) / 2.25) - 725;
                line.X2 = (Convert.ToInt32(sliderRed_L.Value) / 2.25) - 725;
                line.Y1 = (Convert.ToInt32(sliderYRed.Value) / 2.25) + 4;
                line.Y2 = (Convert.ToInt32(sliderYRed.Value) / 2.25) + 4;
                RedLineY.Content = Convert.ToString(sliderYRed.Value);
                myCanvas.Children.Add(line);
            }

            {
                Line line = new Line();
                Thickness thickness = new Thickness(101, -11, 362, 250);
                line.Margin = thickness;
                line.Visibility = System.Windows.Visibility.Visible;
                line.StrokeThickness = 2;
                line.Stroke = System.Windows.Media.Brushes.Blue;
                line.X1 = (Convert.ToInt32(sliderBlue_R.Value) / 2.25) - 725;
                line.X2 = (Convert.ToInt32(sliderBlue_L.Value) / 2.25) - 725;
                line.Y1 = (Convert.ToInt32(sliderYBlue.Value) / 2.25) + 4;
                line.Y2 = (Convert.ToInt32(sliderYBlue.Value) / 2.25) + 4;
                BlueLineY.Content = Convert.ToString(sliderYBlue.Value);
                myCanvas.Children.Add(line);
            }

            {
                Line line = new Line();
                Thickness thickness = new Thickness(101, -11, 362, 250);
                line.Margin = thickness;
                line.Visibility = System.Windows.Visibility.Visible;
                line.StrokeThickness = 3;
                line.Stroke = System.Windows.Media.Brushes.Green;
                line.X1 = (Convert.ToInt32(sliderXGrn.Value) / 2.25) - 722;
                line.X2 = (Convert.ToInt32(sliderXGrn.Value) / 2.25) - 722;
                line.Y1 = (Convert.ToInt32(sliderGreen_T.Value) * 1.135) + 8;
                line.Y2 = (Convert.ToInt32(sliderGreen_B.Value) * 1.135) - 740;
                GreenLineY.Content = Convert.ToString(sliderXGrn.Value);
                myCanvas.Children.Add(line);

            }

            {
                Line line = new Line();
                Thickness thickness = new Thickness(101, -11, 362, 250);
                line.Margin = thickness;
                line.Visibility = System.Windows.Visibility.Visible;
                line.StrokeThickness = 2;
                line.Stroke = System.Windows.Media.Brushes.Yellow;
                line.X1 = (Convert.ToInt32(sliderXYellow.Value) / 2.25) - 722;
                line.X2 = (Convert.ToInt32(sliderXYellow.Value) / 2.25) - 722;
                line.Y1 = (Convert.ToInt32(sliderYellow_T.Value) * 1.135) + 8;
                line.Y2 = (Convert.ToInt32(sliderYellow_B.Value) * 1.135) - 740;
                YellowLineY.Content = Convert.ToString(sliderXYellow.Value);
                
                myCanvas.Children.Add(line);

            }

            {
                Line line = new Line();
                Thickness thickness = new Thickness(101, -11, 362, 250);
                line.Margin = thickness;
                line.Visibility = System.Windows.Visibility.Visible;
                line.StrokeThickness = 1;
                line.Stroke = System.Windows.Media.Brushes.Black;
                line.X1 = -50;
                line.X2 = 1100;
                line.Y1 = (Convert.ToInt32(sliderGraph_low.Value) / 1.15) + 10;
                line.Y2 = (Convert.ToInt32(sliderGraph_low.Value) / 1.15) + 10;
                Graph_Canvas.Children.Add(line);
            }

            {
                Line line = new Line();
                Thickness thickness = new Thickness(101, -11, 362, 250);
                line.Margin = thickness;
                line.Visibility = System.Windows.Visibility.Visible;
                line.StrokeThickness = 1;
                line.Stroke = System.Windows.Media.Brushes.Black;
                line.X1 = -50;
                line.X2 = 1100;
                line.Y1 = (Convert.ToInt32(sliderGraph_High.Value) / 1.15) + 10;
                line.Y2 = (Convert.ToInt32(sliderGraph_High.Value) / 1.15) + 10;
                Graph_Canvas.Children.Add(line);
            }

            {
                SliderLines.Stop();
            }
        }

        private void sliderGraph_High_ValueChanged(object sender, RoutedPropertyChangedEventArgs<double> e)
        {
                SliderLines.Start();
               HIGH1.Text = Convert.ToString(sliderGraph_High.Value);   
        }
        
        private void sliderGraph_lowValueChanged(object sender, RoutedPropertyChangedEventArgs<double> e)
        {
           SliderLines.Start();
           Low.Text = Convert.ToString(sliderGraph_low.Value);         
        }
     
        private void sliderRed_L_ValueChanged(object sender, RoutedPropertyChangedEventArgs<double> e)
        {
            SliderLines.Start();
        }
        private void sliderYRed_ValueChanged(object sender, RoutedPropertyChangedEventArgs<double> e)
        {
            SliderLines.Start();
            this.radioButton1.IsChecked = true;                     
        }
              
        private void sliderRed_R_ValueChanged(object sender, RoutedPropertyChangedEventArgs<double> e)
        {
            SliderLines.Start();
        }

        private void sliderXGrn_ValueChanged(object sender, RoutedPropertyChangedEventArgs<double> e)
        {
            SliderLines.Start();
            this.radioButton3.IsChecked = true;                
        }
        
        private void sliderGreen_B_ValueChanged(object sender, RoutedPropertyChangedEventArgs<double> e)
        {
            SliderLines.Start();
        }

        private void sliderGreen_T_ValueChanged(object sender, RoutedPropertyChangedEventArgs<double> e)
        {
            SliderLines.Start();
        }

        private void sliderXYellow_ValueChanged(object sender, RoutedPropertyChangedEventArgs<double> e)
        {
            SliderLines.Start();
            this.radioButton4.IsChecked = true;             
        }
        
        private void sliderYellow_T_ValueChanged(object sender, RoutedPropertyChangedEventArgs<double> e)
        {
            SliderLines.Start();
        }

        private void sliderYellow_B_ValueChanged(object sender, RoutedPropertyChangedEventArgs<double> e)
        {
            SliderLines.Start();
        }

        private void sliderBlue_R_ValueChanged(object sender, RoutedPropertyChangedEventArgs<double> e)
        {
            SliderLines.Start();
        }

        private void sliderYBlue_ValueChanged(object sender, RoutedPropertyChangedEventArgs<double> e)
        {
            SliderLines.Start();
            this.radioButton2.IsChecked = true;         
        }
        
        private void sliderBlue_L_ValueChanged(object sender, RoutedPropertyChangedEventArgs<double> e)
        {
            SliderLines.Start();
        }

        # endregion

        #region Store
        private void Save_IP_Address(object sender, RoutedEventArgs e)
        {
            // Update the value.
            Settings1.Default.IP_Address = IP_Address.Text;
            // Save the config file.
            Settings1.Default.Save();
        }

        private void Save_Button_Click(object sender, RoutedEventArgs e)
        {
            if (string.IsNullOrWhiteSpace(this.RedPixelSet.Text))
            {
                MessageBox.Show("TextBox is empty");
            }
            else
                // Update the value.
                Settings1.Default.RedPixelSet = RedPixelSet.Text;
            Settings1.Default.BluePixelSet = BluePixelSet.Text;
            Settings1.Default.GreenPixelSet = GreenPixelSet.Text;
            Settings1.Default.YellowPixelSet = YellowPixelSet.Text;
            // Save the config file.
            Settings1.Default.Save();
        }

        private void Save_Button_Red_Click(object sender, RoutedEventArgs e)
        {
            {
            if (string.IsNullOrWhiteSpace(this.RedPixelSet.Text))
            {
                MessageBox.Show("TextBox is empty");
            }
            else
                // Update the value.
                Settings1.Default.RedPixelSet = RedPixelSet.Text;
            // Save the config file.
            Settings1.Default.Save();
            }
            {
                MessageBox.Show("Pixel Dia. has been saved.");
            }
        }

        private void Save_Button_Blue_Click(object sender, RoutedEventArgs e)
        {
            {
                if (string.IsNullOrWhiteSpace(this.BluePixelSet.Text))
                {
                    MessageBox.Show("TextBox is empty");
                }
                else
                    // Update the value.
                Settings1.Default.BluePixelSet = BluePixelSet.Text;
                // Save the config file.
                Settings1.Default.Save();
            }
            {
                MessageBox.Show("Pixel Dia. has been saved.");
            }
        }


        private void Save_Button_Green_Click(object sender, RoutedEventArgs e)
        {
          {
            if (string.IsNullOrWhiteSpace(this.GreenPixelSet.Text))
            {
                MessageBox.Show("TextBox is empty");
            }
            else
                // Update the value.
            Settings1.Default.GreenPixelSet = GreenPixelSet.Text;
            // Save the config file.
            Settings1.Default.Save();
           }
       
           {
             MessageBox.Show("Pixel Dia. has been saved.");
           }
        }

        private void Save_Button_Yellow_Click(object sender, RoutedEventArgs e)
        {
            {
                if (string.IsNullOrWhiteSpace(this.YellowPixelSet.Text))
                {
                    MessageBox.Show("TextBox is empty");
                }
                else
                    // Update the value.
                Settings1.Default.YellowPixelSet = YellowPixelSet.Text;
                // Save the config file.
                Settings1.Default.Save();
            }
            {
                MessageBox.Show("Pixel Dia. has been saved.");
            }
        }

        private void RedPixelSet_TextChanged(object sender, TextChangedEventArgs e)
        {
            if (System.Text.RegularExpressions.Regex.IsMatch(RedPixelSet.Text, "[^.0-9]"))
            {
                MessageBox.Show("Please enter only numbers.");
               RedPixelSet.Text = RedPixelSet.Text.Remove(RedPixelSet.Text.Length - 1);
            }
        }

        private void BluePixelSet_TextChanged(object sender, TextChangedEventArgs e)
        {
           if (System.Text.RegularExpressions.Regex.IsMatch(BluePixelSet.Text, "[^.0-9]"))
            {
                MessageBox.Show("Please enter only numbers.");
                BluePixelSet.Text = BluePixelSet.Text.Remove(BluePixelSet.Text.Length - 1);
            }
        }

        private void GreenPixelSet_TextChanged(object sender, TextChangedEventArgs e)
        {
            if (System.Text.RegularExpressions.Regex.IsMatch(GreenPixelSet.Text, "[^.0-9]"))
            {
                MessageBox.Show("Please enter only numbers.");
                GreenPixelSet.Text = GreenPixelSet.Text.Remove(GreenPixelSet.Text.Length - 1);
            }
        }

        private void YellowPixelSet_TextChanged(object sender, TextChangedEventArgs e)
        {
            if (System.Text.RegularExpressions.Regex.IsMatch(YellowPixelSet.Text, "[^.0-9]"))
            {
                MessageBox.Show("Please enter only numbers.");
                YellowPixelSet.Text = YellowPixelSet.Text.Remove(YellowPixelSet.Text.Length - 1);
            }
        }

        private void IP_Address_TextChanged(object sender, TextChangedEventArgs e)
        {
            if (System.Text.RegularExpressions.Regex.IsMatch(IP_Address.Text, "[^.0-9]"))
            {
                MessageBox.Show("Please enter only numbers.");
                IP_Address.Text = IP_Address.Text.Remove(IP_Address.Text.Length - 1);
            }
        }

       }
     }
# endregion