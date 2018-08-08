using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using System.Windows;
using System.IO;
using System.Reflection;

namespace WenLibrary
{

    public class CoordinateParser
    {

        public CoordinateParser()
        {
            
        }

        public List<float[]> GetCoordinates(string filename)
        {
            //Grab the full path of the text file 
            string dir = Path.GetDirectoryName(Assembly.GetExecutingAssembly().Location);
            string fullFilePath = dir + @"\" + filename;

            List<float[]> coordinates = new List<float[]>();
            float[] currentCoords = new float[6];

            //Runs through text file line by line
            using (TextReader rdr = new StreamReader(fullFilePath))
            {
                string line;

                int coordinateIdx = 0;

                while ((line = rdr.ReadLine()) != null)
                {
                    //Check to see if the first character in the current line is whitespace. If it is, we have reached new coordinates 
                    //and add our current array of coordinates to our coordinates list. Otherwise we continue to update our current coordinates array
                    var lineMap = line.ToCharArray();
                    var firstCharacter = lineMap[0];

                    if (Char.IsWhiteSpace(firstCharacter))
                    {
                        if (coordinateIdx == currentCoords.Length)
                        {
                            coordinates.Add(currentCoords); //adds a new array of coordinates
                        }

                        //reset both the coordinate array and the coordinate index we are at in our coordinate array  
                        coordinateIdx = 0;
                        currentCoords = new float[6]; 
                    }
                    //Updates our array of coordinates, it will do this for a count of 6 coordinates
                    else
                    {
                        var currentCoord = float.Parse(line.Substring(3));
                        currentCoords[coordinateIdx] = currentCoord;
                        coordinateIdx++;
                    }
                }
            }

            return coordinates;
        }
    }
}


