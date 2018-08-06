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

namespace WenViz
{

    public class CoordinateParser
    {
        float[,] coordinates;

        public CoordinateParser()
        {
            //Array should be dynamic, but for now, 2x6 for the dummy data:
            coordinates = new float[2, 6];
        }

        public float[,] GetCoordinates(string filename)
        {
            //Read the text file
            TextReader rdr = new StreamReader("dummy_data.txt");

            //dynamic loop that while there are more lines in the text file, add to the array

            using (rdr)
            {
                string line;

                int row = -1;

                int col = 0;

                float[] currentCoords;

                while ((line = rdr.ReadLine()) != null)
                {
                    System.Diagnostics.Debug.WriteLine();
                    //logic to parse the lines in here
                    var lineMap = line.ToCharArray();
                    //Check to see if we are at the end of an interval of coordinates, then update the array and start over, reset:
                    if (lineMap.GetValue(0) == ' ') {
                        if (currentCoords!=null)
                        {
                            coordinates.SetValue(currentCoords, row); //updates the current row of data
                        }
                        row++;
                        col = 0;
                        currentCoords = new float[6]; //clears currentCoords
                    }
                    //Updates the array with the next set of coordinates, it will do this for 6 rows:
                    else
                    {
                        currentCoords.SetValue(line.Substring(3), col);
                        col++;
                    }

                }
            }
            

            return this.coordinates;
        }
    }
}
