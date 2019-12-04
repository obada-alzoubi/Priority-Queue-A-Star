using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;

namespace PathFinder
{
    public class Map
    {
        //Decalration
        public string[] mapString;
        public int width = 0;
        public int height = 0;

        //Constructor
        public Map(string[] mapString,int width,int height)
        {
            this.mapString = mapString;
            this.width = width;
            this.height = height;

        }

        //Method to build the Map
        public State[,] MapBuild()
        {
            State[,] map=new State[height,width];
            //build the Map
            int temp = 0;
            for (int i = 0; i < height; i++)
            {
                for (int j = 0; j < width; j++)
                {
                    if (mapString[i + 4][j].ToString() == "T" || mapString[i + 4][j].ToString() == "W" ||
                mapString[i + 4][j].ToString() == "@")
                        temp = 0;
                    else
                        temp = 1;
                    map[i,j] = new State(i, j, 0, 0, 0, false,temp,-1,-1);
                }
            }

            return map;

        }


    }
}
