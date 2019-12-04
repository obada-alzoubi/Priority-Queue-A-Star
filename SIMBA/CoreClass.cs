using System;
using System.Collections.Generic;
using System.Linq;
using System.Diagnostics;
using System.IO;
// Added new 
using System.Data;
using System.Drawing;
using System.ComponentModel;
using System.Windows.Forms;

namespace PathFinder
{
    class CoreClass
    {
        //Decalaration 
        //public static State[,] map;
        //Map Size 
        int width = 0;
        int height = 0;
        //Define Start & End Points
        public Point startState;
        public Point endState;
        public Point startStateOrg;
        public Point endStateOrg;
        public Point StateWithLowestTotal;
        public Point midState=new Point(0,0);
        public Point currentState;
        //Define Right & Left Points for Cuts
        public Point Right;
        public Point Left;
        //Variables for Search Method
        //Copy-Paste Declaration
        Stopwatch sw2;
        //Store Points that on the Path
        public static TimeSpan time;
        public static TimeSpan timeA;
        //public static List<Point> pathA;
        //Variable to run Timer2 for A* 
        public static bool PureAStar = false;
        public bool FindPath = false;
        public static int NumberCuts = 0;
        public static int ExpandedNodes = 0;
        public static int TotalExpanded = 0;
        public static double finalCost = 0;

        public static int PureExpandedNodes = 0;
        //Number Of Expanded Border Nodes
        public static int NumBorNode = 0;
        //Timer
        Stopwatch sw;
        //Variable to indicate that path hit border
        public static bool PathHitBorder = false;
        //Search Method parameters
        public static List<State> ProcessedList = new List<State>();
        public static List<State> pureClosedList = new List<State>();
        public static List<State> pureOpenList = new List<State>();
        public static List<State> pathStates = new List<State>();
        public List<State> openList = new List<State>();
        public List<State> closedList = new List<State>();
        public static long coutTime = 0;
        public bool largeCut = false;
        IEnumerable<MyPathNode> path;
        

        //Varibles to find the MinDifference
        //public static double Mindiff = 0;
        //public static double MaxCost = 0;
        //public static double[,] FirstCutCost;
        //public double evaluationPathCost = 0;


        //Varibles to store PAth Sides
        public int upSide = 0;
        public int downSide = 0;
        public int leftSide = 0;
        public int rightSide = 0;
        //Constructor
        public CoreClass(State [,] mapa,int width,int height,int x_start,int y_start,int x_end,int y_end)
        {
            //Initialize Variables
            this.width = width;
            this.height = height;
            State[,] map = mapa;
            startState = new Point(x_start, y_start);
            endState = new Point(x_end, y_end);
            startStateOrg = new Point(x_start, y_start);
            endStateOrg = new Point(x_end, y_end);
            //Find The mid point to find the cut size
            midState.X = (int )(0.5*startState.X + 0.5*endState.X);
            midState.Y = (int) (0.5*startState.Y +0.5* endState.Y);
            
        }

        public class MyPathNode : PathFinder.IPathNode<Object>
        {
            public Int32 X { get; set; }
            public Int32 Y { get; set; }
            public Boolean IsWall { get; set; }

            public bool IsWalkable(Object unused)
            {
                return !IsWall;
            }
        }

        public class MySolver<TPathNode, TUserContext> : PathFinder.SpatialAStar<TPathNode, TUserContext> where TPathNode : PathFinder.IPathNode<TUserContext>
        {
            protected override Double Heuristic(PathNode inStart, PathNode inEnd)
            {
                //return Math.Abs(inStart.X - inEnd.X) + Math.Abs(inStart.Y - inEnd.Y);
                return Math.Sqrt((inStart.X - inEnd.X) * (inStart.X - inEnd.X) + (inStart.Y - inEnd.Y) * (inStart.Y - inEnd.Y));
            }

            protected override Double NeighborDistance(PathNode inStart, PathNode inEnd)
            {
                return Heuristic(inStart, inEnd);
            }

            public MySolver(TPathNode[,] inGrid)
                : base(inGrid)
            {
            }
        }


        public unsafe void IMBA( State[,] map)
        {
            State[,] graphI = new State[height, width];
            //because this static we may have a prevois value
            Cutter.FullPathExplored = false;
            //Compute Execution Time
            
            Right = new Point(0, 0);
            Left = new Point(0, 0);
            // Define the Map 
            Right.X = 0;
            Right.Y = 0;
            Left.X = 0;
            Left.Y = 0;
            //Set Number of Cuts to Zero 
            NumberCuts = 0;
            //Counter To count The Number of Cuts
            NumberCuts++;

            //Compute The max distnace
            // Define the Cut Size :Staticlally
            CutDefine(ref Left, ref Right);
            //MAke sure That Statr and End states are inside the Cut
            Cutter cutter1 = new Cutter(map, height, width, startState, endState);
            graphI = cutter1.Repair(Left, Right);
            sw = Stopwatch.StartNew();
            //System.Diagnostics.Stopwatch watch = new System.Diagnostics.Stopwatch();
            //watch.Start();
            //Reset Values of Ratios for New Usage
            //Set  PathHitBorder for New Usage
            PathHitBorder = false;
            List<MySolver<MyPathNode, Object>> listSolvers = new List<MySolver<MyPathNode, Object>>();
            int pathCount = 0;

            while (true)
            {
                //if (IsWalkable(graphI))
                //    break;
                //Clear OpenList and Closed List for New Cut to make Clear Search
                //### need to be changed if we using pure IMBA
                //closedList.Clear();
                //openList.Clear();
                //Clear Path States for Final Cut 
                pathStates.Clear();
                //Set Number Expaned Nodes to Zero for Each Cut
                //TotalExpanded += ExpandedNodes;
                //ExpandedNodes = 0;

                //Console.WriteLine("We Are in the Cut Number:{0}", NumberCuts);
                int[,] borderIndex= new int[height,width];
                MyPathNode[,] grid = new MyPathNode[height, width];
                Bitmap gridBmp = new Bitmap(height, width, System.Drawing.Imaging.PixelFormat.Format32bppArgb);

                PathFinder.ImagePixelLock locked = new PathFinder.ImagePixelLock(gridBmp, false);

                using (locked)
                {
                    int* pixels = locked.Pixels;

                    for (int x = 0; x < width; x++)
                    {
                        for (int y = 0; y < height; y++)
                        {
                            Boolean isWall;

                            if (graphI[x, y].value >= 1)
                            {
                                isWall = false;
                                if (map[x, y].value==2)
                                    borderIndex[x, y] = 1;

                            }
                            else
                            {
                                isWall = true;
                                borderIndex[x, y] = 0;
                            }

                            if (isWall)
                                *pixels = unchecked((int)0xFF000000);
                            else
                                *pixels = unchecked((int)0xFFFFFFFF);

                            grid[x, y] = new MyPathNode()
                            {
                                IsWall = isWall,

                                X = x,
                                Y = y,
                            };
                            pixels++;


                        }
                    }
                }
                // compute and display path
           

                MySolver<MyPathNode, Object> aStar = new MySolver<MyPathNode, Object>(grid);

                if (NumberCuts >= 2)
                {
                    //if (NumberCuts == 3)
                    //    Console.WriteLine("here");
                    aStar.m_CameFrom = listSolvers[NumberCuts-2].Cm_CameFrom;
                    
                    aStar.m_ClosedSet = listSolvers[NumberCuts-2].Cm_ClosedSet;
                    aStar.m_OpenSet= listSolvers[NumberCuts-2].Cm_OpenSet;
                    aStar.m_RuntimeGrid = listSolvers[NumberCuts - 2].Cm_RuntimeGrid;
                    //aStar.m_SearchSpace = listSolvers[NumberCuts - 2].Cm_SearchSpace;
                    aStar.m_OrderedOpenSet = listSolvers[NumberCuts - 2].Cm_OrderedOpenSet;

                    //aStar.Cm_CameFrom = listSolvers[NumberCuts - 2].Cm_CameFrom;

                    //aStar.Cm_ClosedSet = listSolvers[NumberCuts - 2].Cm_ClosedSet;
                    //aStar.Cm_OpenSet = listSolvers[NumberCuts - 2].Cm_OpenSet;
                    //aStar.Cm_RuntimeGrid = listSolvers[NumberCuts - 2].Cm_RuntimeGrid;
                    ////aStar.Cm_SearchSpace = listSolvers[NumberCuts - 2].Cm_SearchSpace;
                    //aStar.Cm_OrderedOpenSet = listSolvers[NumberCuts - 2].Cm_OrderedOpenSet;



                }
                
                try
                {

                    path = aStar.Search(new System.Drawing.Point(startState.X, startState.Y), new System.Drawing.Point(endState.X, endState.Y), null, borderIndex, NumberCuts);
                    listSolvers.Add(aStar);
                    finalCost = aStar.pathCost;
                    pathCount = path.Count();
                    ExpandedNodes = aStar.m_ClosedSet.Count;

                    foreach (Point node in aStar.resultList)
                    {
                        if (borderIndex[node.X, node.Y] == 1)
                        {
                            if (node.Y == Left.Y -1)
                                leftSide++;
                            if (node.Y == Right.Y +1)
                                rightSide++;
                            if (node.X == Right.X + 1)
                                upSide++;
                            if (node.X == Left.X -1)
                                downSide++;
                            PathHitBorder = true;
                        }

                    }

                    //if (aStar.pathHitBorder)
                    //{
                    //    Console.WriteLine("Path hit border !!");
                    //    PathHitBorder = true;
                    //}

                }
                catch
                {
                }




                //watch.Stop();

                //watch.Start();
                //{
                //    aStar.Search(new System.Drawing.Point(startState.X, startState.Y), new System.Drawing.Point(endState.X, endState.Y), null);
                //}
                //watch.Stop();
                //int count = 0;
                //foreach (MyPathNode node in path)
                //{
                //    gridBmp.SetPixel(node.X, node.Y, Color.Red);

                //    count = count + 1;
                //}
                //System.Windows.Forms.MessageBox.Show("Pathfinding length " + count + "units.");
                //System.Windows.Forms.MessageBox.Show("Pathfinding took " + watch.ElapsedMilliseconds + "ms to complete.");

                //System.Diagnostics.Stopwatch watch2 = new System.Diagnostics.Stopwatch();

                //watch2.Start();

                //Search2(graphI, startState, endState);
                //watch2.Stop();
                //System.Windows.Forms.MessageBox.Show("Pathfinding2 took " + watch2.ElapsedMilliseconds + "ms to complete.");
                //foreach (Point node in pathA)
                //    {
                //    if (node.X!=-1 && node.Y!=-1)
                //    gridBmp.SetPixel(node.X, node.Y, Color.Green);


                //}

                //gridBmp.Save(".\\dump.png");

                //If we ddin't find any path in cut , Tgere is no path at all
                if (pathCount==0)
                {
                    //Console.WriteLine("There is no way to Reach The Goal State");
                    finalCost = -1;
                    break;


                }

                //foreach (MyPathNode node in path)
                //{
                //    if (graphI[node.X, node.Y].isBorder)
                //    {
                //        PathHitBorder = true;
                //        break;
                //    }
                //}

                if (pathCount > 0 && !PathHitBorder)
                {
                    //Console.WriteLine("The Alogrithm finally found the final optimal path");
                    break;
                }

                if (PathHitBorder)
                {
                    //To Re Make a Search
                    PathHitBorder = false;
                    NumberCuts++;
                    //increase cut size 
                    CutDefine(ref Left, ref Right);

                    //Repair cut
                    //Modification******
                    sw.Stop();
                    //graphI = cutter1.Repair(Left, Right);
                    sw.Start();
                    if (NumberCuts > 6)
                    {
                        StreamWriter ErrorWrite = new StreamWriter("log.txt", true);

                        ErrorWrite.Write("Error Number Cuts large");

                        ErrorWrite.Write(startState.X);
                        ErrorWrite.Write("-");
                        ErrorWrite.Write(startState.Y);
                        ErrorWrite.Write(")");
                        ErrorWrite.Write(",");
                        ErrorWrite.Write("(");
                        ErrorWrite.Write(endState.X);
                        ErrorWrite.Write("-");
                        ErrorWrite.Write(endState.Y);
                        ErrorWrite.Write(")");
                        ErrorWrite.Write(",");
                        ErrorWrite.WriteLine(finalCost);
                        ErrorWrite.Close();
                        largeCut = true;
                        break;
                    }
                    //Set Flags that cut Size will not Excessed the graph size
                }
            }
            if (pathCount>0)
            {

                //Compare performance with Pure A* Algorithm
                //To tell Search Method to Run time to compute Pure A* Performance
        
                //Stop the Timer of Execution time
                sw.Stop();
                time = sw.Elapsed;
                coutTime = sw.ElapsedMilliseconds;
                //Console.WriteLine("IMBA* Execution time:{0}", time);
                //Show Path 

            }
            //If We didn't Find the Path,We Should Stop the Timer
            sw.Stop();
            coutTime = sw.ElapsedMilliseconds;
            
            WriteResults();
            largeCut = false;
            FindPath = false;
            CoreClass.NumberCuts = 0;
            CoreClass.NumBorNode = 0;
            CoreClass.PathHitBorder = false;
            CoreClass.pathStates.Clear();
            CoreClass.ProcessedList.Clear();
            //CoreClass.pureClosedList.Clear();
            //CoreClass.pureOpenList.Clear();
            CoreClass.TotalExpanded = 0;
            CoreClass.coutTime = 0;
            openList.Clear();
            closedList.Clear();
            sw.Reset();

           

        }



        /**
         * The Core Method of Search (A* Algorithm)
         * 1- Do the Search in BackWard Direction 
         * 2- Later Do the Search in Forward Direction 
         * **********************************************
         * Method returs 1 if it found the path 
         *               0 if no optimal path
         *              -1 if there is an Error 
         *              
         **/

        public int Search2(State[,] mapFromIMBA, Point startState, Point endState)
        {
            //DrawMap2(mapFromIMBA);
            //###Stop copying States to temporary List when reaching A border State
            bool REACHBORDER=false;
            //ReSet The Map Values if any change happen due to previous Searches 
            for (int i = 0; i < height; i++)
            {
                for (int j = 0; j < width; j++)
                {
                    //Copy objects
                    mapFromIMBA[i, j].cost = 0;
                    mapFromIMBA[i, j].heuristic = 0;
                    mapFromIMBA[i, j].totalCost = 0;
                    mapFromIMBA[i, j].previous_X = -1;
                    mapFromIMBA[i, j].previous_Y = -1;


                }
            }
            //Check The Walkability of Start and End States
            if (IsWalkable(mapFromIMBA))
                return -1;
            //Set The Timer
            sw2 = new Stopwatch();
            sw2 = Stopwatch.StartNew();
            //copy States from temporary Lists satarting from the second cut 
            if (NumberCuts >= 2)
            {
              //### NEED TO BE CHANGED IF WE USING PURE IMBA
                //Stop timers for this method to make comparison with Pure IMBA* Fair since this Method consume time.
                sw.Stop();
                modifyLists(mapFromIMBA);
                //ReRun the Timers again 
                sw.Start();

            }
            //Console.WriteLine("The Start State is({0},{1}) & the End State({2},{3})", startState.X, startState.Y, endState.X, endState.Y);
            //For the First cut we need to add Start State since it will be included in oprnlist later by copying 
            //### Need to be changed if we using pure IMBA
            if (NumberCuts==1)
            openList.Add(mapFromIMBA[startState.X,startState.Y]);
            //Compute the Number of Expanded States
            ExpandedNodes++;
            /////////////////////////////////////////////////////////////
            /**Special parameters to Apply IMBA* and The Modification**/
            //Add Strart Node to Processed List
            //Add Start Node toFirst Cut Processed List 
            //Just Store Costs in Arry for IMBA*
            /////////////////////////////////////////////////////////////
            //For the new modifications closed list should includes the staes from temporary lists
            //closedList.Clear();
            currentState = new Point(-1, -1);
            while (!(currentState.X == endState.X && currentState.Y == endState.Y))
            {
                //Get State with samllest F-Value
                openList=openList.OrderBy(state => state.totalCost).ToList();
                if (openList.Count == 0)
                {
                    FindPath = false;
                    return -1;
                }
                currentState.X = openList.ElementAt(0).X;
                currentState.Y = openList.ElementAt(0).Y;
                //If the currentState is the endState, then we can stop searching
                if (currentState.X == endState.X && currentState.Y == endState.Y)
                {
                    //For A* to stop n the cut this:We found a path
                    FindPath = true;
                    //Console.WriteLine("The End State is found ");
                    //Console.WriteLine("The Number Of Expanded States {0}", ExpandedNodes);
                    //Stop timers for the Search Method 
                    sw2.Stop();
                    timeA = sw2.Elapsed;
                    // ShowPath(graphS);
                    //Console.WriteLine("Time of Execution {0}", time);
                    // construct the Path
                    sw.Stop();
                    FindthePath(mapFromIMBA);
                    sw.Start();
                    return 1;
                }
                else
                {

                    ////### Set flag when first border state reached
                    //if (mapFromIMBA[currentState.X, currentState.Y].isBorder)
                    //    REACHBORDER = true;
                    //### Stop copying States to temporary states when reaching Border State

                    //check if this works in the orginal code 
                    openList.RemoveAll(state => state.X == currentState.X && state.Y == currentState.Y);
                    //A* (ADD State to closed list and remove it from open list
                    closedList.Add(mapFromIMBA[currentState.X,currentState.Y]);
                    if (!REACHBORDER)
                    {
                        pureClosedList.Add(mapFromIMBA[currentState.X, currentState.Y]);
                        pureOpenList.Remove(mapFromIMBA[currentState.X, currentState.Y]);


                    }


                    //Get all the adjacent States
                    List<Point> adjacentStates = GetAdjacentStates(currentState, mapFromIMBA);
                    foreach (Point adjacentState in adjacentStates)
                    {



                        //### Set flag when first border state reached
                        if (mapFromIMBA[adjacentState.X, adjacentState.Y].isBorder)
                            REACHBORDER = true;
                        //Here find the cost,we take into consider the Digonal States
                        //Digonal States with sqrt(2) cost moving 
                        //Also for IMBA* Algorithm, We need to define differnet cost for Border Nodes 
                        //Check The Author's Paper for more details

                        //Compute Cost
                        double Cost = 0;
                        if (Math.Abs(currentState.X - adjacentState.X) + Math.Abs(currentState.Y - adjacentState.Y) == 1)
                        {

                            if (mapFromIMBA[currentState.X, currentState.Y].isBorder && mapFromIMBA[adjacentState.X, adjacentState.Y].isBorder)
                                Cost = 1;
                            else
                                Cost = 1;
                        }

                        else
                        {
                            if (mapFromIMBA[currentState.X, currentState.Y].isBorder && mapFromIMBA[adjacentState.X, adjacentState.Y].isBorder)
                                Cost = 1.4142;
                            else
                                Cost =  1.4142;

                        }
                        //Compute Tentative Cost
                        float tentative_cost = (float)( mapFromIMBA[currentState.X, currentState.Y].cost + Cost);
                        //adjacent State can not be in the closed list
                        if (closedList.Exists(point => point.X == adjacentState.X && point.Y == adjacentState.Y))
                            if (tentative_cost > mapFromIMBA[adjacentState.X, adjacentState.Y].cost)
                                continue;
                        if ((!openList.Exists(po => po.X == adjacentState.X && po.Y == adjacentState.Y) || tentative_cost < mapFromIMBA[adjacentState.X, adjacentState.Y].cost))
                        {
                            
                            //A* (Set the State's Father , g-value ,h-value and f-value) 
                            mapFromIMBA[adjacentState.X, adjacentState.Y].previous_X = currentState.X;
                            mapFromIMBA[adjacentState.X, adjacentState.Y].previous_Y = currentState.Y;
                            mapFromIMBA[adjacentState.X, adjacentState.Y].cost = tentative_cost;
                            //use costs from first cut as Heuristic 
                            mapFromIMBA[adjacentState.X, adjacentState.Y].heuristic = EucDistance(adjacentState);
                            mapFromIMBA[adjacentState.X, adjacentState.Y].totalCost = mapFromIMBA[adjacentState.X, adjacentState.Y].cost
                                + mapFromIMBA[adjacentState.X, adjacentState.Y].heuristic;
                            ////For IMBA*
                            //Add cots to the Arry of first cut Costs
                            if (!openList.Exists(poi => poi.X == adjacentState.X && poi.Y == adjacentState.Y))
                            {
                                openList.Add(mapFromIMBA[adjacentState.X, adjacentState.Y]);
                                ExpandedNodes++;
                                //=============
                                if (!REACHBORDER)
                                {
                                    // if (!pureOpenList.Exists(poi => poi.X == adjacentState.X && poi.Y == adjacentState.Y))
                                    pureOpenList.Add(mapFromIMBA[adjacentState.X, adjacentState.Y]);
                                }
                            }
                            //### Copy State to temporary lists (open List Just)

                        }
                    }

                }
            }
            return 0;

        }


        // List of all functions
        /**
         * modifylists :copy All temporary list to closed and open losts 
         * IsWalkable : check the walkability of Start and end States
         * FindthePath
         * DrawMap
         * GetStateWithLowestTotal
         * ManhattanDistance
         * GetAdjacentStates
         * CutDefine
         
         * */
        #region 

        void modifyLists(State[,] mapfromSearch)
        {
            closedList.Clear();
            openList.Clear();
            foreach (State state in pureOpenList)
            {
                openList.Add(state);
                mapfromSearch[state.X, state.Y].cost = state.cost;
                mapfromSearch[state.X, state.Y].heuristic = state.heuristic;
                mapfromSearch[state.X, state.Y].totalCost = state.totalCost;
                mapfromSearch[state.X, state.Y].previous_X = state.previous_X;
                mapfromSearch[state.X, state.Y].previous_Y = state.previous_Y;


            }
            foreach (State state in pureClosedList)
            {
                closedList.Add(state);
                mapfromSearch[state.X, state.Y].cost = state.cost;
                mapfromSearch[state.X, state.Y].heuristic = state.heuristic;
                mapfromSearch[state.X, state.Y].totalCost = state.totalCost;
                mapfromSearch[state.X, state.Y].previous_X = state.previous_X;
                mapfromSearch[state.X, state.Y].previous_Y = state.previous_Y;
            }
            pureClosedList.Clear();
            pureOpenList.Clear();
            //openList=openList.Distinct().ToList();
            //closedList = closedList.Distinct().ToList();
            //openList.RemoveAll(state => state.inopen == false);
            //closedList.RemoveAll(state => state.inclose == false);
            
        }
        //Check The Walkability of Start and The End States
        public bool IsWalkable(State[,] mapFromSearch)
        {
            bool wakableStart = false;
            bool wakableEnd = false;
            //Make Sure That Start and the End States are Walkable
            if (mapFromSearch[startState.X, startState.Y].value == 0)
            {
                Console.WriteLine("The Start State is not walkable Area, it should be eaither '.' or 'S' while it is {0}", mapFromSearch[endState.X, endState.Y].value);
                wakableStart = true;
            }
            if (mapFromSearch[endState.X, endState.Y].value == 0)
            {
                Console.WriteLine("The End State is not walkable Area, it should be eaither '.' or 'S', while it is {0}", mapFromSearch[endState.X, endState.Y].value);
                wakableEnd = true;
            }
            return ((wakableEnd || wakableStart));
        }

        //Find The Path after the Search is done
        public List<Point> FindthePath(State[,] mapFromSearch)
        {
            //Console.WriteLine("The Optimal Path was found with Cost={0}",  mapFromSearch[endState.X, endState.Y].cost);
            //DrawMap2(mapFromSearch);
            finalCost = mapFromSearch[endState.X, endState.Y].cost;
            List<Point> the_Path = new List<Point>();
            int x = endState.X;
            int y = endState.Y;
            int x_temp = 0;
            int y_temp = 0;
            int borderinPath = 0;
            upSide = 0;
            downSide = 0;
            leftSide = 0;
            rightSide = 0;
            
            while (true)
            {
                if (the_Path.Count > 10000)
                {
                    StreamWriter ErrorWrite = new StreamWriter("log.txt", true);
                    ErrorWrite.Write(startState.X);
                    ErrorWrite.Write("-");
                    ErrorWrite.Write(startState.Y);
                    ErrorWrite.Write(")");
                    ErrorWrite.Write(",");
                    ErrorWrite.Write("(");
                    ErrorWrite.Write(endState.X);
                    ErrorWrite.Write("-");
                    ErrorWrite.Write(endState.Y);
                    ErrorWrite.Write(")");
                    ErrorWrite.Write(",");
                    ErrorWrite.WriteLine(finalCost);
                    ErrorWrite.Close();
                    break;
                }
                x_temp = x;
                y_temp = y;
                x = mapFromSearch[x_temp, y_temp].previous_X;

                y = mapFromSearch[x_temp, y_temp].previous_Y;
                Point pathS = new Point(x, y);

                the_Path.Add(pathS);
                if (x == -1 & y == -1)
                    break;
                ////For IMBA* 
                //Check wether the path hit the border or not 
                if (mapFromSearch[x, y].isBorder)
                {
                    //Console.WriteLine("The is a border Point ({0},{1})",x,y);
                    borderinPath++;
                    PathHitBorder = true;
                }
                if (y == Left.Y-1)
                    leftSide++;
                if (y == Right.Y + 1)
                    rightSide++;
                if (x == Right.X + 1)
                    upSide++;
                if (x == Left.X - 1)
                    downSide++;
            }
            Console.WriteLine("We are not using dynamic expanding" );
            //Draw the Map
            //DrawMap(mapFromSearch, the_Path);
            //MinDiff(mapFromSearch);
            Console.WriteLine("The Number of sates  in  Path:{0}", the_Path.Count);
            //pathA = the_Path; 
            return the_Path;

        }

        //Draw the Map
        public void DrawMap(State[,] mapFromSearch ,List<Point> the_Path)
        {
            StreamWriter file = new StreamWriter("map.map", true);

            for (int i = 0; i < height; i++)
            {
                for (int j = 0; j < width; j++)
                {
                    if (j < width - 1)
                    {
                        if (the_Path.Exists(point => point.X == i && point.Y == j))
                        {
                            if (mapFromSearch[i, j].isBorder)
                                file.Write("X");
                            else
                                file.Write("#");


                        }
                        else
                            file.Write(mapFromSearch[i, j].value);

                    }
                    else
                    {
                        file.WriteLine(mapFromSearch[i, j].value);

                    }
                }
            }
            file.Close();
        }
        public void DrawMap2(State[,] mapFromSearch)
        {
            StreamWriter file = new StreamWriter("map.map", true);

            for (int i = 0; i < height; i++)
            {
                for (int j = 0; j < width; j++)
                {
                    if (j < width - 1)
                    {
   
                            file.Write(mapFromSearch[i, j].value);

                    }
                    else
                    {
                        file.WriteLine(mapFromSearch[i, j].value);

                    }
                }
            }
            file.Close();
        }

  
  

        //Get State with lowest cost 
        public  Point GetStateWithLowestTotal( List<State> openList, State[,] graphG)
        {
            //temp variables
            //openList.Reverse();
           StateWithLowestTotal=new Point(-1,-1);

            double lowestTotal = double.MaxValue;

            //search all the open States and get the State with the lowest evaluation cost
            foreach (State openState in openList)
            {
                if (openState.totalCost < lowestTotal)
                {
                    lowestTotal = graphG[openState.X, openState.Y].totalCost;
                    
                    StateWithLowestTotal.X = openState.X;
                    StateWithLowestTotal.Y = openState.Y;
                }
            }

            if (StateWithLowestTotal.X == -1 && StateWithLowestTotal.Y == -1)
            {
                StateWithLowestTotal.X = endState.X;
                StateWithLowestTotal.Y = endState.Y;
            }

            return  StateWithLowestTotal;
        }

        //Get Adjacent States 
        public List<Point> GetAdjacentStates(Point currentState, State[,] graph)
        {
            bool Right  = false;
            bool Left = false;
            bool Up = false;
            bool down = false;

            List<Point> adjacentStates = new List<Point>();
            Point adjacentState;
            //State Right
            adjacentState = new Point(currentState.X, currentState.Y + 1);
            if (adjacentState.Y < width && (graph[adjacentState.X, adjacentState.Y].value >=1 ))
            {
                adjacentStates.Add(adjacentState);
                Right = true;
            }

            //State Left
            adjacentState = new Point(currentState.X, currentState.Y - 1);
            if (adjacentState.Y >= 0 && (graph[adjacentState.X, adjacentState.Y].value >= 1))
            {
                adjacentStates.Add(adjacentState);
                Left = true;
            }

            //State to the Down
            adjacentState = new Point(currentState.X + 1, currentState.Y);
            if (adjacentState.X < height && (graph[adjacentState.X, adjacentState.Y].value >= 1))
            {
                adjacentStates.Add(adjacentState);
                down = true;
            }

            //State to the Up
            adjacentState = new Point(currentState.X - 1, currentState.Y);
            if (adjacentState.X >= 0 && (graph[adjacentState.X, adjacentState.Y].value >= 1))
            {
                adjacentStates.Add(adjacentState);
                Up = true;
            }
            //Diagonal States 
            adjacentState = new Point(currentState.X+1, currentState.Y + 1);
            if (adjacentState.Y < width && adjacentState.X < height && (graph[adjacentState.X, adjacentState.Y].value >= 1))
                if(down&&Right)
                adjacentStates.Add(adjacentState);
            adjacentState = new Point(currentState.X - 1, currentState.Y - 1);
            if (adjacentState.Y >= 0 && adjacentState.X >= 0 && (graph[adjacentState.X, adjacentState.Y].value >= 1))
                if(Up&&Left)
                adjacentStates.Add(adjacentState);
            adjacentState = new Point(currentState.X - 1, currentState.Y + 1);
            if (adjacentState.Y < width && adjacentState.X >= 0 && (graph[adjacentState.X, adjacentState.Y].value >= 1))
                if(Up&&Right)
                adjacentStates.Add(adjacentState);
            adjacentState = new Point(currentState.X + 1, currentState.Y - 1);
            if (adjacentState.Y >= 0 && adjacentState.X < height && (graph[adjacentState.X, adjacentState.Y].value >= 1))
                if(down&&Left)
                adjacentStates.Add(adjacentState);

            return adjacentStates;
        }

        //Metrics Computations 
        public float EucDistance(Point adjacentState)
        {

                //Heuristic for Forward Search
              return (float)Math.Sqrt((endState.X-adjacentState.X)*(endState.X - adjacentState.X) + (endState.Y - adjacentState.Y)*(endState.Y-adjacentState.Y));
               
            

        }
        public double ManhattanDistance1(Point adjacentState)
        {

            //Heuristic for Forward Search
            double manhattan = Math.Abs(endState.X - adjacentState.X) + Math.Abs(endState.Y - adjacentState.Y);
            return manhattan;


        }


        //Method to define the cut borders
        public void CutDefine(ref Point leftP, ref Point rightP)
        {

            int temp_x =Math.Abs( Left.X - Right.X);
            int temp_y = Math.Abs(Left.Y - Right.Y);
            int temp_sum=leftSide+upSide+rightSide+downSide;

            //if (NumberCuts == 1)
            //{

            //    leftP.X = height - 1;
            //    leftP.Y = width - 1;
            //    int min_X = -1, min_Y = -1, max_X = -1, max_Y = -1;
            //    if (startState.X < endState.X)
            //    {
            //        min_X = startState.X;
            //        max_X = endState.X;
            //    }
            //    else
            //    {
            //       max_X = startState.X;
            //        min_X = endState.X;
            //    }
            //    if (startState.Y < endState.Y)
            //    {
            //        min_Y = startState.Y;
            //        max_Y = endState.Y;
            //    }
            //    else
            //    {
            //        max_Y = startState.Y;
            //        min_Y = endState.Y;
            //    }

            //    leftP.X = min_X - 50;
            //    leftP.Y = min_Y - 50;
            //    rightP.X = max_X + 50;
            //    rightP.Y = max_Y + 50;

            //}
            //else
            //{

            //    Console.WriteLine("We are not using dynamic expanding");


            //    leftP.X -= 25 + NumberCuts * temp_sum * (downSide) / (temp_sum);
            //    leftP.Y -= 25 + NumberCuts * temp_sum * (leftSide) / (temp_sum); ;
            //    rightP.X += 25 + NumberCuts * temp_sum * (upSide) / (temp_sum); ;
            //    rightP.Y += 25 + NumberCuts * temp_sum * (rightSide) / (temp_sum);

            //    //leftP.X -= 50;
            //    //leftP.Y -= 50;
            //    //rightP.X += 50;
            //    //rightP.Y += 50;
            //}

            //if (leftP.X <= 1)
            {
                leftP.X = 1;
            }
            //if (leftP.Y <=1)
            {
                leftP.Y = 1;
            }
            //Soleve Cut Size Out of graph Range
            //if (rightP.X > height - 2)
            {
                rightP.X = height - 2;
            }
            //Soleve Cut Size Out of graph Range
            //if (rightP.Y > width - 2)
            {
                rightP.Y = width - 2;
            }

            //Console.WriteLine("The Left Point is({0},{1}) & the Righ Point({2},{3})", leftP.X, leftP.Y, rightP.X, rightP.Y);
            
        }
        void WriteResults()
        {
            StreamWriter Writer = new StreamWriter("c:\\benchmark2\\results\\"+"APQ_"+Program.mapName+".txt", true);
            Writer.Write(coutTime);
            Writer.Write(",");
            Writer.Write(ExpandedNodes);
            Writer.Write(",");
            Writer.Write(NumberCuts);
            Writer.Write(",");
            Writer.Write("(");
            Writer.Write(startState.X);
            Writer.Write("-");
            Writer.Write(startState.Y);
            Writer.Write(")");
            Writer.Write(",");
            Writer.Write("(");
            Writer.Write(endState.X);
            Writer.Write("-");
            Writer.Write(endState.Y);
            Writer.Write(")");
            Writer.Write(",");
            if (path.Count()>0)
                if (!largeCut)
                    Writer.WriteLine(finalCost);
                else
                    Writer.WriteLine("Large Cut");

            else
                Writer.WriteLine("No path");
            Writer.Close();

        }

        //Method to Compute the MinDifference
        //public void MinDiff(State [,] mapAfterResearch)
        //{
        //    double Min = int.MaxValue;
        //    double MinLoop = int.MaxValue;

        //    foreach (State position in ProcessedFirstCut)
        //    {

        //        if (mapAfterResearch[position.X, position.Y].heuristic >= 150)
        //        {

        //            MinLoop = Math.Abs(mapAfterResearch[position.X, position.Y].cost - mapAfterResearch[position.X, position.Y].heuristic);
        //            if (MinLoop <= Min)
        //            {
        //                Min = MinLoop;
        //            }
        //        }
        //    }
        //    //Make Sure that MinDiff will Not be Affect by the Later Cuts 
        //    if (NumberCuts == 1)
        //        Mindiff = Min;
        //    //Compute The Max Cost
        //    double Max = int.MaxValue;
        //    double MaxLoop = int.MaxValue;
        //    foreach (State position in ProcessedFirstCut)
        //    {


        //        if (mapAfterResearch[position.X, position.Y].isBorder)
        //        {
        //            MaxLoop = mapAfterResearch[position.X, position.Y].cost;
        //            if (MaxLoop <= Max)
        //            {
        //                Max = MaxLoop;
        //            }
        //        }
        //    }
        //    if (NumberCuts == 1)
        //        MaxCost = Max;
        //}


    }
    #endregion


    

}