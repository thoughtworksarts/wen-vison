using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using System.Windows;
using System.Windows.Controls;
using System.Windows.Data;
using System.Windows.Documents;
using System.Windows.Input;
using System.Windows.Media;
using System.Windows.Media.Imaging;
using System.Windows.Navigation;
using System.Windows.Shapes;
using System.Diagnostics;
using System.Globalization;
using System.ComponentModel;
using Microsoft.Kinect;
using WenLibrary;

namespace WenViz
{
    /// <summary>
    /// Interaction logic for MainWindow.xaml
    /// </summary>
    public partial class MainWindow : Window
    {
        /// <summary>
        /// Radius of drawn hand circles
        /// </summary>
        private const double HandSize = 30;

        /// <summary>
        /// Thickness of drawn joint lines
        /// </summary>
        private const double JointThickness = 10;

        /// <summary>
        /// Thickness of clip edge rectangles
        /// </summary>
        private const double ClipBoundsThickness = 10;

        /// <summary>
        /// Constant for clamping Z values of camera space points from being negative
        /// </summary>
        private const float InferredZPositionClamp = 0.1f;

        /// <summary>
        /// Brush used for drawing hands that are currently tracked as closed
        /// </summary>
        private readonly Brush handClosedBrush = new SolidColorBrush(Color.FromArgb(128, 255, 0, 0));

        /// <summary>
        /// Brush used for drawing hands that are currently tracked as opened
        /// </summary>
        private readonly Brush handOpenBrush = new SolidColorBrush(Color.FromArgb(128, 0, 255, 0));

        /// <summary>
        /// Brush used for drawing hands that are currently tracked as in lasso (pointer) position
        /// </summary>
        private readonly Brush handLassoBrush = new SolidColorBrush(Color.FromArgb(128, 0, 0, 255));

        /// <summary>
        /// Brush used for drawing joints that are currently tracked
        /// </summary>
        private readonly Brush trackedJointBrush = new SolidColorBrush(Color.FromArgb(255, 68, 192, 68));

        /// <summary>
        /// Brush used for drawing joints that are currently inferred
        /// </summary>        
        private readonly Brush inferredJointBrush = Brushes.Yellow;

        /// <summary>
        /// Pen used for drawing bones that are currently inferred
        /// </summary>        
        private readonly Pen inferredBonePen = new Pen(Brushes.Gray, 1);

        /// <summary>
        /// Drawing group for person body rendering output
        /// </summary>
        private DrawingGroup personDrawingGroup;

        /// <summary>
        /// Drawing group for WEN arm rendering output
        /// </summary>
        private DrawingGroup wenArmDrawingGroup;

        /// <summary>
        /// Drawing image that we will use to display from the Kinect
        /// </summary>
        private DrawingImage personImageSource;

        /// <summary>
        /// Drawing image that we will display the WEN arm in 
        /// </summary>
        private DrawingImage wenArmImageSource;

        /// <summary>
        /// Active Kinect sensor
        /// </summary>
        private KinectSensor kinectSensor = null;

        /// <summary>
        /// Coordinate mapper to map one type of point to another
        /// </summary>
        private CoordinateMapper coordinateMapper = null;

        /// <summary>
        /// Reader for body frames
        /// </summary>
        private BodyFrameReader bodyFrameReader = null;

        /// <summary>
        /// Array for the bodies
        /// </summary>
        private Body[] bodies = null;

        /// <summary>
        /// definition of person bones
        /// </summary>
        private List<Tuple<JointType, JointType>> personBones;

        /// <summary>
        /// definition of wen arm bones
        /// </summary>
        private List<Tuple<JointType, JointType>> wenArmBones;

        /// <summary>
        /// Width of display (depth space)
        /// </summary>
        private int displayWidth;

        /// <summary>
        /// Height of display (depth space)
        /// </summary>
        private int displayHeight;

        /// <summary>
        /// List of colors for each body tracked
        /// </summary>
        private List<Pen> bodyColors;

        /// <summary>
        /// Current status text to display
        /// </summary>
        private string kinectStatusText = null;

        //arm coordinates 
        List<float[]> coordinates;

        //WEN joints 
        private Dictionary<JointType, Joint> wenJointsDictionary;

        //Array that has joint types for the WEN arm 
        private JointType[] wenArmJointTypes;

        //Starting XYZ coordinates for the 6 joints of the WEN arm
        private double[,] startingPositions;

        //Starting XYZ Origins of rotations for the 6 joints of the WEN arm
        private double[,] STARTING_ORIGINS;

        //PlaneTypes array for the 6 joints of the WEN arm that tells us what plane the movement exists in
        private int[,] planeTypes;

        //private PositionUpdater positionUpdater; 

         public int repeatIndex;

        /// <summary>
        /// Initializes a new instance of the MainWindow class.
        /// </summary>
        public MainWindow()
        {
            ParseArmCoords();
            repeatIndex=1;
            SetupKinect();
            SetupWENArm();
            SetupSharedWindowProperties();
            SetUpStartingPositions();
            SetUpOrigins();
            SetUpPlaneTypes();
            SetupWenArmUpdater();
            SetUpPositionUpdater();
            DrawWenArm();
            InitializeComponent(); 
            //DrawRepeatingWenArm();

            Console.WriteLine("Status of the kinnect is "+kinectStatusText);
        }

        private void ParseArmCoords()
        {
            CoordinateParser parser = new CoordinateParser();
            this.coordinates = parser.GetCoordinates("dummy_data1.txt");

            //Left this here for debug purposes but feel free to delete this when your done
            for (int i = 0; i < coordinates.Count; i++)
            {
                var coords = coordinates[i];

                Console.WriteLine("Current set of coordinates: ");
                for (int j = 0; j < coords.Length; j++)
                {
                    Console.WriteLine(j + ". " + coords[j]);
                }

                Console.WriteLine();
            }
        }

        private void SetUpStartingPositions()
        {
           this.startingPositions = new double[,]
            {
                {1000,1000,1000},
                {1000-100,-1000+0,300},
                {1000-300,-1000+0,300},
                {1000-400,-1000+0,300},
                {1000-400,-1000+0,250},
                {1000-400,-1000+0,200}
            };
        }

        //REDO, builds but does not run (need to implement horrible loops from above)
        private void SetUpOrigins()
        {
           this.STARTING_ORIGINS = new double[,]
            {
                {0, 0, 0},
                {0, 0, 0},
                {-1, 0, 2},
                {-3, 0, 3},
                {-4, 0, 3},
                {-4, 0, 2.5}
            };
        }

        //REDO
        private void SetUpPlaneTypes()
        {
            this.planeTypes = new int[,] {
            {1,1,0},
            {1,0,1},
            {1,0,1},
            {0,1,1},
            {1,0,1},
            {0,1,1},
            };

        }

        private void SetupWENArm()
        {
            // Create an image source for our arm 
            this.wenArmDrawingGroup = new DrawingGroup();
            this.wenArmImageSource = new DrawingImage(this.wenArmDrawingGroup);

            wenArmBones = new List<Tuple<JointType, JointType>>();
        
            //IMPORTANT REDO this 
            // Reusing the right Kinect arm for our WEN arm
            this.wenArmBones.Add(new Tuple<JointType, JointType>(JointType.ShoulderRight, JointType.ElbowRight));
            this.wenArmBones.Add(new Tuple<JointType, JointType>(JointType.ElbowRight, JointType.WristRight));
            this.wenArmBones.Add(new Tuple<JointType, JointType>(JointType.WristRight, JointType.HandRight));
            this.wenArmBones.Add(new Tuple<JointType, JointType>(JointType.HandRight, JointType.HandTipRight));
            this.wenArmBones.Add(new Tuple<JointType, JointType>(JointType.HandTipRight, JointType.ThumbRight));

            //Dictionary containing a mapping joint type and the current joint (with coordinates) for the WEN arm
            this.wenJointsDictionary = new Dictionary<JointType, Joint>();

            this.wenArmJointTypes = new[]
            { JointType.ShoulderRight,
              JointType.ElbowRight,
              JointType.WristRight,
              JointType.HandRight,
              JointType.HandTipRight,
              JointType.ThumbRight
            };
        }

        private void SetupKinect()
        {
            // one sensor is currently supported
            this.kinectSensor = KinectSensor.GetDefault();

            // open the reader for the body frames
            this.bodyFrameReader = this.kinectSensor.BodyFrameSource.OpenReader();

            // a bone defined as a line between two joints
            this.personBones = new List<Tuple<JointType, JointType>>();

            // Torso
            this.personBones.Add(new Tuple<JointType, JointType>(JointType.Head, JointType.Neck));
            this.personBones.Add(new Tuple<JointType, JointType>(JointType.Neck, JointType.SpineShoulder));
            this.personBones.Add(new Tuple<JointType, JointType>(JointType.SpineShoulder, JointType.SpineMid));
            this.personBones.Add(new Tuple<JointType, JointType>(JointType.SpineMid, JointType.SpineBase));
            this.personBones.Add(new Tuple<JointType, JointType>(JointType.SpineShoulder, JointType.ShoulderRight));
            this.personBones.Add(new Tuple<JointType, JointType>(JointType.SpineShoulder, JointType.ShoulderLeft));
            this.personBones.Add(new Tuple<JointType, JointType>(JointType.SpineBase, JointType.HipRight));
            this.personBones.Add(new Tuple<JointType, JointType>(JointType.SpineBase, JointType.HipLeft));

            // Right Arm
            this.personBones.Add(new Tuple<JointType, JointType>(JointType.ShoulderRight, JointType.ElbowRight));
            this.personBones.Add(new Tuple<JointType, JointType>(JointType.ElbowRight, JointType.WristRight));
            this.personBones.Add(new Tuple<JointType, JointType>(JointType.WristRight, JointType.HandRight));
            this.personBones.Add(new Tuple<JointType, JointType>(JointType.HandRight, JointType.HandTipRight));
            this.personBones.Add(new Tuple<JointType, JointType>(JointType.WristRight, JointType.ThumbRight));

            // Left Arm
            this.personBones.Add(new Tuple<JointType, JointType>(JointType.ShoulderLeft, JointType.ElbowLeft));
            this.personBones.Add(new Tuple<JointType, JointType>(JointType.ElbowLeft, JointType.WristLeft));
            this.personBones.Add(new Tuple<JointType, JointType>(JointType.WristLeft, JointType.HandLeft));
            this.personBones.Add(new Tuple<JointType, JointType>(JointType.HandLeft, JointType.HandTipLeft));
            this.personBones.Add(new Tuple<JointType, JointType>(JointType.WristLeft, JointType.ThumbLeft));

            // Right Leg
            this.personBones.Add(new Tuple<JointType, JointType>(JointType.HipRight, JointType.KneeRight));
            this.personBones.Add(new Tuple<JointType, JointType>(JointType.KneeRight, JointType.AnkleRight));
            this.personBones.Add(new Tuple<JointType, JointType>(JointType.AnkleRight, JointType.FootRight));

            // Left Leg
            this.personBones.Add(new Tuple<JointType, JointType>(JointType.HipLeft, JointType.KneeLeft));
            this.personBones.Add(new Tuple<JointType, JointType>(JointType.KneeLeft, JointType.AnkleLeft));
            this.personBones.Add(new Tuple<JointType, JointType>(JointType.AnkleLeft, JointType.FootLeft));

            // set IsAvailableChanged event notifier
            this.kinectSensor.IsAvailableChanged += this.Sensor_IsAvailableChanged;

            // open the sensor
            this.kinectSensor.Open();

            // set the status text
            this.KinectStatusText = this.kinectSensor.IsAvailable ? Properties.Resources.RunningStatusText
                                                            : Properties.Resources.NoSensorStatusText;

            // Create the drawing group we'll use for drawing
            this.personDrawingGroup = new DrawingGroup();

            // Create an image source that we can use in our image control
            this.personImageSource = new DrawingImage(this.personDrawingGroup);
        }

        private void SetupSharedWindowProperties()
        {
           // use the window object as the view model in this simple example
            this.DataContext = this;

            // get the coordinate mapper
            this.coordinateMapper = this.kinectSensor.CoordinateMapper;
            //this.coordinateMapper = new CoordinateMapper();

            if (this.kinectSensor != null)
            {
                // get the depth (display) extents
                FrameDescription frameDescription = this.kinectSensor.DepthFrameSource.FrameDescription;

                // get size of joint space
                this.displayWidth = frameDescription.Width;
                this.displayHeight = frameDescription.Height;
            }
            else
            {
                this.displayWidth = 5120;
                this.displayHeight = 4240; 
            }

            // populate body colors, one for each BodyIndex
            this.bodyColors = new List<Pen>();

            this.bodyColors.Add(new Pen(Brushes.Red, 6));
            this.bodyColors.Add(new Pen(Brushes.Orange, 6));
            this.bodyColors.Add(new Pen(Brushes.Green, 6));
            this.bodyColors.Add(new Pen(Brushes.Blue, 6));
            this.bodyColors.Add(new Pen(Brushes.Indigo, 6));
            this.bodyColors.Add(new Pen(Brushes.Violet, 6));


        }

        /// <summary>
        /// INotifyPropertyChangedPropertyChanged event to allow window controls to bind to changeable data
        /// </summary>
        public event PropertyChangedEventHandler PropertyChanged;

        /// <summary>
        /// Gets the person bitmap to display
        /// </summary>
        public ImageSource PersonImageSource
        {
            get
            {
                return this.personImageSource;
            }
        }

        /// <summary>
        /// Gets the wen arm bitmap to display
        /// </summary>
        public ImageSource WENArmImageSource
        {
            get
            {
                return this.wenArmImageSource;
            }
        }

        /// <summary>
        /// Gets or sets the current status text to display
        /// </summary>
        public string KinectStatusText
        {
            get
            {
                return this.kinectStatusText;
            }

            set
            {
                if (this.kinectStatusText != value)
                {
                    this.kinectStatusText = value;

                    // notify any bound elements that the text has changed
                    if (this.PropertyChanged != null)
                    {
                        this.PropertyChanged(this, new PropertyChangedEventArgs("StatusText"));
                    }
                }
            }
        }

        /// <summary>
        /// Execute start up tasks
        /// </summary>
        /// <param name="sender">object sending the event</param>
        /// <param name="e">event arguments</param>
        private void MainWindow_Loaded(object sender, RoutedEventArgs e)
        {
            if (this.bodyFrameReader != null)
            {
                this.bodyFrameReader.FrameArrived += this.Reader_FrameArrived;
                Debug.Write("main window loaded");
            }
        }

        /// <summary>
        /// Execute shutdown tasks
        /// </summary>
        /// <param name="sender">object sending the event</param>
        /// <param name="e">event arguments</param>
        private void MainWindow_Closing(object sender, CancelEventArgs e)
        {
            if (this.bodyFrameReader != null)
            {
                // BodyFrameReader is IDisposable
                this.bodyFrameReader.Dispose();
                this.bodyFrameReader = null;
            }

            if (this.kinectSensor != null)
            {
                this.kinectSensor.Close();
                this.kinectSensor = null;
            }
        }

        /// <summary>
        /// Handles the body frame data arriving from the sensor
        /// </summary>
        /// <param name="sender">object sending the event</param>
        /// <param name="e">event arguments</param>
        private void Reader_FrameArrived(object sender, BodyFrameArrivedEventArgs e)
        {
            Debug.Write("reader frame arrived being called");

            DrawRepeatingWenArm(repeatIndex);
            repeatIndex++;
            bool dataReceived = false;

            using (BodyFrame bodyFrame = e.FrameReference.AcquireFrame())
            {
                if (bodyFrame != null)
                {
                    if (this.bodies == null)
                    {
                        this.bodies = new Body[bodyFrame.BodyCount];
                    }

                    // The first time GetAndRefreshBodyData is called, Kinect will allocate each Body in the array.
                    // As long as those body objects are not disposed and not set to null in the array,
                    // those body objects will be re-used.
                    bodyFrame.GetAndRefreshBodyData(this.bodies);
                    dataReceived = true;
                }
            }

            if (dataReceived)
            {
                using (DrawingContext dc = this.personDrawingGroup.Open())
                {
                    // Draw a transparent background to set the render size
                    dc.DrawRectangle(Brushes.Brown, null, new Rect(0.0, 0.0, this.displayWidth, this.displayHeight));

                    int penIndex = 0;
                    foreach (Body body in this.bodies)
                    {
                        Pen drawPen = this.bodyColors[penIndex++];

                        if (body.IsTracked)
                        {
                            this.DrawClippedEdges(body, dc);

                            IReadOnlyDictionary<JointType, Joint> joints = body.Joints;

                            // convert the joint points to depth (display) space
                            Dictionary<JointType, Point> jointPoints = new Dictionary<JointType, Point>();

                            foreach (JointType jointType in joints.Keys)
                            {
                                // sometimes the depth(Z) of an inferred joint may show as negative
                                // clamp down to 0.1f to prevent coordinatemapper from returning (-Infinity, -Infinity)
                                CameraSpacePoint position = joints[jointType].Position;
                                if (position.Z < 0)
                                {
                                    position.Z = InferredZPositionClamp;
                                }

                                DepthSpacePoint depthSpacePoint = this.coordinateMapper.MapCameraPointToDepthSpace(position);
                                jointPoints[jointType] = new Point(depthSpacePoint.X, depthSpacePoint.Y);
                            }

                            this.DrawBody(joints, jointPoints, dc, drawPen);
                            Debug.Write("drawing person");

                            this.DrawHand(body.HandLeftState, jointPoints[JointType.HandLeft], dc);
                            this.DrawHand(body.HandRightState, jointPoints[JointType.HandRight], dc);
                        }
                    }

                    // prevent drawing outside of our render area
                    this.personDrawingGroup.ClipGeometry = new RectangleGeometry(new Rect(0.0, 0.0, this.displayWidth, this.displayHeight));
                }
            }
        }

        /// <summary>
        /// Draws a body
        /// </summary>
        /// <param name="joints">joints to draw</param>
        /// <param name="jointPoints">translated positions of joints to draw</param>
        /// <param name="drawingContext">drawing context to draw to</param>
        /// <param name="drawingPen">specifies color to draw a specific body</param>
        private void DrawBody(IReadOnlyDictionary<JointType, Joint> joints, IDictionary<JointType, Point> jointPoints, DrawingContext drawingContext, Pen drawingPen, Boolean isArm = false)
        {
            if (!isArm)
            {
                // Draw the person's bones
                foreach (var bone in this.personBones)
                {
                    this.DrawBone(joints, jointPoints, bone.Item1, bone.Item2, drawingContext, drawingPen);
                }
            }
            else
            {
                // Draw the wen arm's bones 
                foreach (var bone in this.wenArmBones)
                {
                    this.DrawBone(joints, jointPoints, bone.Item1, bone.Item2, drawingContext, drawingPen);
                }
            }

            // Draw the joints
            foreach (JointType jointType in joints.Keys)
            {
                Brush drawBrush = null;
                TrackingState trackingState = joints[jointType].TrackingState;

                if (trackingState == TrackingState.Tracked)
                {
                    drawBrush = this.trackedJointBrush;
                }
                else if (trackingState == TrackingState.Inferred)
                {
                    drawBrush = this.inferredJointBrush;
                }

                if (drawBrush != null)
                {
                    drawingContext.DrawEllipse(drawBrush, null, jointPoints[jointType], JointThickness, JointThickness);
                    //Debug.WriteLine("We are drawing the joint "+jointType+" on the screen at "+jointPoints[jointType].X+" and "+jointPoints[jointType].Y);
                }
            }
        }

        /// <summary>
        /// Draws one bone of a body (joint to joint)
        /// </summary>
        /// <param name="joints">joints to draw</param>
        /// <param name="jointPoints">translated positions of joints to draw</param>
        /// <param name="jointType0">first joint of bone to draw</param>
        /// <param name="jointType1">second joint of bone to draw</param>
        /// <param name="drawingContext">drawing context to draw to</param>
        /// /// <param name="drawingPen">specifies color to draw a specific bone</param>
        private void DrawBone(IReadOnlyDictionary<JointType, Joint> joints, IDictionary<JointType, Point> jointPoints, JointType jointType0, JointType jointType1, DrawingContext drawingContext, Pen drawingPen)
        {
            Joint joint0 = joints[jointType0];
            Joint joint1 = joints[jointType1];

            // If we can't find either of these joints, exit
            if (joint0.TrackingState == TrackingState.NotTracked ||
                joint1.TrackingState == TrackingState.NotTracked)
            {
                return;
            }

            // We assume all drawn bones are inferred unless BOTH joints are tracked
            Pen drawPen = this.inferredBonePen;
            if ((joint0.TrackingState == TrackingState.Tracked) && (joint1.TrackingState == TrackingState.Tracked))
            {
                drawPen = drawingPen;
            }

            drawingContext.DrawLine(drawPen, jointPoints[jointType0], jointPoints[jointType1]);
        }

        /// <summary>
        /// Draws a hand symbol if the hand is tracked: red circle = closed, green circle = opened; blue circle = lasso
        /// </summary>
        /// <param name="handState">state of the hand</param>
        /// <param name="handPosition">position of the hand</param>
        /// <param name="drawingContext">drawing context to draw to</param>
        private void DrawHand(HandState handState, Point handPosition, DrawingContext drawingContext)
        {
            switch (handState)
            {
                case HandState.Closed:
                    drawingContext.DrawEllipse(this.handClosedBrush, null, handPosition, HandSize, HandSize);
                    break;

                case HandState.Open:
                    drawingContext.DrawEllipse(this.handOpenBrush, null, handPosition, HandSize, HandSize);
                    break;

                case HandState.Lasso:
                    drawingContext.DrawEllipse(this.handLassoBrush, null, handPosition, HandSize, HandSize);
                    break;
            }
        }

        /// <summary>
        /// Draws indicators to show which edges are clipping body data
        /// </summary>
        /// <param name="body">body to draw clipping information for</param>
        /// <param name="drawingContext">drawing context to draw to</param>
        private void DrawClippedEdges(Body body, DrawingContext drawingContext)
        {
            FrameEdges clippedEdges = body.ClippedEdges;

            if (clippedEdges.HasFlag(FrameEdges.Bottom))
            {
                drawingContext.DrawRectangle(
                    Brushes.Red,
                    null,
                    new Rect(0, this.displayHeight - ClipBoundsThickness, this.displayWidth, ClipBoundsThickness));
            }

            if (clippedEdges.HasFlag(FrameEdges.Top))
            {
                drawingContext.DrawRectangle(
                    Brushes.Red,
                    null,
                    new Rect(0, 0, this.displayWidth, ClipBoundsThickness));
            }

            if (clippedEdges.HasFlag(FrameEdges.Left))
            {
                drawingContext.DrawRectangle(
                    Brushes.Red,
                    null,
                    new Rect(0, 0, ClipBoundsThickness, this.displayHeight));
            }

            if (clippedEdges.HasFlag(FrameEdges.Right))
            {
                drawingContext.DrawRectangle(
                    Brushes.Red,
                    null,
                    new Rect(this.displayWidth - ClipBoundsThickness, 0, ClipBoundsThickness, this.displayHeight));
            }
        }

        private void SetupWenArmUpdater()
        {
            //sets up the updater
        }

        //DRAWS WEN arm from the passed in coordinate
        //AndyA - Going to put in sample coordinates from my starting position variable initialized above
        //AndyA - Need to add the startPositions as the XYZ Position attribute for each joint... how to do this? 
        private void DrawWenArm()
        {
            UpdateWenArmJointsFromCoordinate(); //this will actually update from the first row of the coordinates using the updater

            using (DrawingContext dc = this.wenArmDrawingGroup.Open())
            {
                // Draw a transparent background to set the render size
                //dc.DrawRectangle(Brushes.Brown, null, new Rect(0.0, 0.0, this.displayWidth, this.displayHeight));

                //we're going to just use a pen index of 0 for now
                int penIndex = 0;
                Pen drawPen = this.bodyColors[penIndex];

                IReadOnlyDictionary<JointType, Joint> joints = wenJointsDictionary;

                // convert the joint points to depth (display) space
                Dictionary<JointType, Point> jointPoints = new Dictionary<JointType, Point>();

                foreach (JointType jointType in joints.Keys)
                {
                    CameraSpacePoint position = joints[jointType].Position;
                    Debug.WriteLine("the position is " + position.X + " " + position.Y+ " "+position.Z);
                    DepthSpacePoint depthSpacePoint = this.coordinateMapper.MapCameraPointToDepthSpace(position);
                    Debug.WriteLine("the depth point is " + depthSpacePoint.X + " "+ depthSpacePoint.Y);
                    jointPoints[jointType] = new Point(depthSpacePoint.X/7336, depthSpacePoint.Y/7336);
                }

                this.DrawBody(joints, jointPoints, dc, drawPen, true);

                //Going to draw the hand of the WEN arm or the very last point 
                //CameraSpacePoint handSpacePoint = new CameraSpacePoint();
                //handSpacePoint.X = 0;
                //handSpacePoint.Y = coordinate[5];
                //handSpacePoint.Z = InferredZPositionClamp;

                //DepthSpacePoint handDepthSpacePoint = this.coordinateMapper.MapCameraPointToDepthSpace(handSpacePoint);
                //this.DrawHand(HandState.Closed, new Point(handDepthSpacePoint.X, handDepthSpacePoint.Y), dc);

                this.wenArmDrawingGroup.ClipGeometry = new RectangleGeometry(new Rect(0.0, 0.0, this.displayWidth, this.displayHeight));
            }
        }

        private void CreateNewPositionForOneJoint(int jointIndex, int i) {
            //a dummy class to add a second point to the arm 
            //Reality: Take the next set of rotation angle/ single angle, and generate the next points
            //If it's the end of the script, start again

            //call nextPositionFromOneJointMovement
                    
                               this.currentPositions = new double[,]
                                {
                                    //this should actually set updatedPositions
                                    //set currentPositions = updatedPositions
                                    {10, 10+i,1000},
                                    {10,-1000+0+i,300},
                                    {1000-300+10+i,-1000+0,300},
                                    {40,-1000+0+10+i,300},
                                    {10+10+i,-1000+0+10,250},
                                    {1000-400+i,-1000+0+10,200}
                                };
        }

        private void DrawRepeatingWenArm(int repeatingIndex) {
                DrawWenArm();
                //InitializeComponent();
                //System.Threading.Thread.Sleep(700);
                CreateNewPositionForOneJoint(3, repeatingIndex);
                //System.Threading.Thread.Sleep(700);
                Debug.WriteLine(this.startingPositions[1,0]);

        }

        //Updates wen joints dictionary with the current coordinate 
        private void UpdateWenArmJointsFromCoordinate()
        {
            
            //We create a list of tuples that have a mapping of a jointtype to a joint in the wen arm 
            for (int i = 0; i < wenArmJointTypes.Length; i++)
            {
                Joint wenJoint = new Joint();
                Debug.WriteLine("Index is "+i);
                var currentJointType = wenArmJointTypes[i];
            
                wenJoint.TrackingState = TrackingState.Tracked;
                CameraSpacePoint point = new CameraSpacePoint();
                point.X = (float) currentPositions[i,0]/1001;
                point.Y = (float) currentPositions[i,1]/1001;
                point.Z = (float) currentPositions[i,2]/1001;

                //point.Y = coordinate[i]; //for now let's assign our coordinate to the y axis for every arm joint
                wenJoint.Position = point;

                wenJointsDictionary[currentJointType] = wenJoint;
            }
        }

        /// <summary>
        /// Handles the event which the sensor becomes unavailable (E.g. paused, closed, unplugged).
        /// </summary>
        /// <param name="sender">object sending the event</param>
        /// <param name="e">event arguments</param>
        private void Sensor_IsAvailableChanged(object sender, IsAvailableChangedEventArgs e)
        {
            // on failure, set the status text
            this.KinectStatusText = this.kinectSensor.IsAvailable ? Properties.Resources.RunningStatusText
                                                            : Properties.Resources.SensorNotAvailableStatusText;
        }

        //* ________________________________________ POSITION UPDATER METHODS BELOW ______________________________________________ */
        // ______________ ------------------- _________________ ------------------------ ___________________ ----------------------


        //come from the coordinate parser class, maybe populate this in the parseArmCoords method?
        private float[] rotationAngles;

        private double[,] currentPositions; //maybe these should be arrays of joints who each posess their own coordinates

        private double[,] updatedPositions;

        private double[,] updatedOrigins;

        private double[,] currentOrigins;

        //private float[,] origins;
        //planeTypes from above 

        public void SetUpPositionUpdater() {
            //this.rotationAngles = return from parseArmCoords
            //set up updated positions and current positions 
            //set up an updated origins ds
            //set up current origins ds
            this.currentPositions = startingPositions;
            this.currentOrigins = STARTING_ORIGINS;

        }





    }
}
