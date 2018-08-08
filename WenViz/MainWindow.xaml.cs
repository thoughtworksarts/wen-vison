﻿using System;
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

            //This is just an idea to get it going, to see if I could see the output, but it's not working because the build won't compile...
            CoordinateParser parser = new CoordinateParser();
            List<float[]> coordinates = parser.GetCoordinates("dummy_data1.txt");

            //print out the coordinates to test out parser
            for(int i = 0; i < coordinates.Count; i++)
            {
                var coords = coordinates[i];

                Console.WriteLine("Current set of coordinates: ");
                for(int j = 0; j < coords.Length; j++)
                {
                    Console.WriteLine(j + ". " + coords[j]);
                }

                Console.WriteLine();
            }
        }
    }
}
