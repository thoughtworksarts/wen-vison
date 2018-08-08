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
using Microsoft.Kinect;
using WenLibrary;

namespace WenViz
{
    /// <summary>
    /// Interaction logic for MainWindow.xaml
    /// </summary>
    public partial class MainWindow : Window
    {
        public MainWindow()
        {
            InitializeComponent();
            TestParser();
        }

        private void TestParser()
        {
            CoordinateParser parser = new CoordinateParser();
            List<float[]> coordinates = parser.GetCoordinates("dummy_data1.txt");

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
    }
}
