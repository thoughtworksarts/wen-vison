using System;

namespace WenViz
{

    public class CoordinateParser
    {
        long[,] coordinates;

        public CoordinateParser()
        {
            coordinates = new long[2, 6];
        }

        public long[,] GetCoordinates(TextReader rdr = new StreamReader("Coordinates.txt"))
        {
            //Read the text file

            //dynamic loop that while there are more lines in the text file, add to the array

            /* using (TextReader rdr = new StreamReader(fullFilePath))
            {
                string line;

                while ((line = rdr.ReadLine()) != null)
                {
                     // use line here: add a line to the array with the 6 new coordinates
                }
            } */

            return this.coordinates;
        }
    }
}
