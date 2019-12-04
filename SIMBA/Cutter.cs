using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Diagnostics;
namespace PathFinder
{
    public class Cutter
    {
        //Define the Region of High Cost Region 
        Point Right;
        Point Left;
        Point startState;
        Point endState;
        int height;
        int width;
        //Variable to Deal with graph
        public State[,] mapR;
        public State[,] map;
        //To View Border Nodes
        //public static List<Point> BorderList = new List<Point>();
        //To Show if we Reached the Maximum Cut Size
        public static bool FullPathExplored = false;
        //Default Constructor
        public Cutter(State[,] map,int height,int width, Point startState ,Point endState)
        {
            this.map = map;
            this.startState=startState;
            this.endState=endState;
            this.height = height;
            this.width = width;
            //map = new State[height, width];
            //this.map = map;
        }

        //General Repair Method
        public State[,] Repair(Point Left,Point Right)
        {

            this.Left = Left;
            this.Right = Right;
            //Clear Border Side List for Each New Cut
            this.Left = Left;
            this.Right = Right;
            //graph Variable
            mapR = new State[height, width];


            //Copy the graph
            for (int i = 0; i < height; i++)
            {
                for (int j = 0; j < width; j++)
                {
                    //Copy objects
                    mapR[i, j] = new State(0, 0, 0, 0, 0, false, 0, -1, -1);
                    mapR[i, j].cost = map[i, j].cost;
                    mapR[i, j].heuristic = map[i, j].heuristic;
                    mapR[i, j].isBorder = map[i, j].isBorder;
                    mapR[i, j].totalCost = map[i, j].totalCost;
                    mapR[i, j].value = map[i, j].value;
                    mapR[i, j].X = map[i, j].X;
                    mapR[i, j].Y = map[i, j].Y;
                    mapR[i, j].previous_X = map[i, j].previous_X;
                    mapR[i, j].previous_Y = map[i, j].previous_Y;
                    if (i>0 & i< Left.X -1 & j>0 & j< width)
                        mapR[i, j].value = 0;
                    if (i >= Right.X + 1 & i< height & j>=0 & j< width)
                        mapR[i, j].value = 0;
                    if (i>=Left.X - 1 & i <= Right.X + 1 & j >= 0 & j < Left.Y - 1)
                        mapR[i, j].value = 0;
                    if (i>=Left.X - 1 & i <= Right.X + 1 & j >= Right.Y + 1 & j < width)
                        mapR[i, j].value = 0;
                    if (i >= Left.X - 1 & i <= Right.X + 1)
                    {
                        mapR[i, Left.Y - 1]= new State(i, Left.Y - 1, 0, 0, 0, true, 2, map[i, Left.Y - 1].previous_X, map[i, Left.Y - 1].previous_Y);
                    }

                    if (i >=Left.X - 1 & i <= Right.X + 1)
                    {
                        mapR[i, Right.Y + 1] = new State(i, Right.Y + 1, 0, 0, 0, true, 2, map[i, Right.Y + 1].previous_X, map[i, Right.Y + 1].previous_Y);
                    }

                    if ( j>= Left.Y & j <= Right.Y)
                    {
                        mapR[Left.X - 1, j] = new State(Left.X - 1, j, 0, 0, 0, true, 2, map[Left.X - 1, j].previous_X, map[Left.X - 1, j].previous_Y);

                    }
                     if (j >= Left.Y & j <= Right.Y)
                    {
                        mapR[Right.X + 1, j] = new State(Right.X + 1, j, 0, 0, 0, true, 2, map[Right.X + 1, j].previous_X, map[Right.X + 1, j].previous_Y);
                    }


                }

            }
            if (FullPathExplored)
                return map;
            else
                return mapR;
            //if (FullPathExplored)
            //{
            //    //Return the full graph if We reached The Maximum Size
            //    return mapR;
            //}
            //else
            //{
            //    //Make out of Cut untraversable
            //    for (int i = 0; i < Left.X - 1; i++)
            //    {
            //        for (int j = 0; j < width; j++)
            //        {
            //            //make it unwalkable,we used out of boundries @
            //            mapR[i, j].value = 0;
            //        }
            //    }


            //    for (int i = (int)Right.X + 1; i < height; i++)
            //    {
            //        for (int j = 0; j < width; j++)
            //        {
            //            //make it unwalkable,we used out of boundries @
            //            mapR[i, j].value = 0;
            //        }
            //    }

            //    for (int i = (int)Left.X - 1; i <= Right.X + 1; i++)
            //    {
            //        for (int j = 0; j < Left.Y - 1; j++)
            //        {

            //            //make it unwalkable,we used out of boundries @
            //            mapR[i, j].value = 0;
            //        }
            //    }

            //    for (int i = (int)Left.X - 1; i <= Right.X + 1; i++)
            //    {
            //        for (int j = (int)Right.Y + 1; j < width; j++)
            //        {
            //            //make it unwalkable,we used out of boundries @
            //            mapR[i, j].value = 0;
            //        }
            //    }


            //    //Build Border Nodes 
            //    //Build Down Side 
            //    for (int i = (int)Left.X - 1; i <= Right.X + 1; i++)
            //    {
            //        /** 
            //         * Make the Boder Node :
            //         * 1-traversable
            //         * 2-Set Cost to Minimum Cost
            //        * */
            //        //if (map[i, (int)Left.Y - 1].value == "T" || map[i, (int)Left.Y - 1].value == "W" || map[i, (int)Left.Y - 1].value == "@")
            //        //{
            //        //    mapR[i, (int)Left.Y - 1].value = map[i, (int)Left.Y - 1].value;
            //        //}
            //        //else
            //        {
            //            mapR[i, (int)Left.Y - 1].value = 2;
            //            mapR[i, (int)Left.Y - 1].isBorder = true;
            //        }
            //        Point position = new Point(i, (int)Left.Y - 1);
            //        /** Add Node to Structure to Reusing in Define Cut Size 
            //            this is A further Improvement By Obada
            //         * */
            //        //DownSide.Add(position);
            //        //Add nodes to  General list to View the Boder
            //        //BorderList.Add(position);

            //    }

            //    //Build Up Side Border
            //    for (int i = (int)Left.X - 1; i <= Right.X + 1; i++)
            //    {

            //        //if (map[i, (int)Right.Y + 1].value == "T" || map[i, (int)Right.Y + 1].value == "W" || map[i, (int)Right.Y + 1].value == "@")
            //        //{
            //        //    mapR[i, (int)Right.Y + 1].value = map[i, (int)Right.Y + 1].value;
            //        //}
            //        //else
            //        {
            //            mapR[i, (int)Right.Y + 1].value = 2;
            //            mapR[i, (int)Right.Y + 1].isBorder = true;
            //        }
        
            //        Point position = new Point(i, (int)Right.Y + 1);
            //        //UpSide.Add(position);
            //        //BorderList.Add(position);

            //    }

            //    //Build Left Side Boder
            //    for (int i = (int)Left.Y; i <= Right.Y; i++)
            //    {


            //        //if (map[(int)Left.X - 1, i].value == "T" || map[(int)Left.X - 1, i].value == "W" || map[(int)Left.X - 1, i].value == "@")
            //        //{
            //        //    mapR[(int)Left.X - 1, i].value = map[(int)Left.X - 1, i].value;
            //        //}
            //        //else
            //        {
            //            mapR[(int)Left.X - 1, i].value = 2;
            //            mapR[(int)Left.X - 1, i].isBorder = true;
            //        }
     
            //        Point position = new Point((int)Left.X - 1, i);
            //        //LeftSide.Add(position);
            //        //BorderList.Add(position);

            //    }

            //    //Build Right Side Boder
            //    for (int i = (int)Left.Y; i <= Right.Y; i++)
            //    {

            //        //if (map[(int)Right.X + 1, i].value == "T" || map[(int)Right.X + 1, i].value == "W" || map[(int)Right.X + 1, i].value == "@")
            //        //{
            //        //    mapR[(int)Right.X + 1, i].value = map[(int)Right.X + 1, i].value;
            //        //}
            //        //else
            //        {
            //            mapR[(int)Right.X + 1, i].value = 2;
            //            mapR[(int)Right.X + 1, i].isBorder = true;
            //        }
            //        Point position = new Point((int)Right.X + 1, i);
            //        //RightSide.Add(position);
            //        //BorderList.Add(position);

            //    }


            //    return mapR;
            //}
        }

    }
}
