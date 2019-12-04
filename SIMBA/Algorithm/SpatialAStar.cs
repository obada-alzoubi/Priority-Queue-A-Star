/*
The MIT License

Copyright (c) 2010 Christoph Husse

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in
all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
THE SOFTWARE.
*/

using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;

namespace PathFinder
{
    public interface IPathNode<TUserContext>
    {
        Boolean IsWalkable(TUserContext inContext);
    }

    public interface IIndexedObject
    {
        int Index { get; set; }
    }

    /// <summary>
    /// Uses about 50 MB for a 1024x1024 grid.
    /// </summary>
    public class SpatialAStar<TPathNode, TUserContext> where TPathNode : IPathNode<TUserContext>
    {
        public OpenCloseMap m_ClosedSet;
        public OpenCloseMap m_OpenSet;
        public PriorityQueue<PathNode> m_OrderedOpenSet;
        public PathNode[,] m_CameFrom;
        public OpenCloseMap m_RuntimeGrid;
        public PathNode[,] m_SearchSpace;

        public OpenCloseMap Cm_ClosedSet;
        public OpenCloseMap Cm_OpenSet;
        public PriorityQueue<PathNode> Cm_OrderedOpenSet;
        public PathNode[,] Cm_CameFrom;
        public OpenCloseMap Cm_RuntimeGrid;
        public PathNode[,] Cm_SearchSpace;


        public TPathNode[,] SearchSpace { get; set; }
        public int Width { get;  set; }
        public int Height { get;  set; }
        public bool pathHitBorder { get;  set; }
        public double pathCost = 0;
        public List <Point> resultList=new List<Point>();


        public class PathNode : IPathNode<TUserContext>, IComparer<PathNode>, IIndexedObject
        {
            public static readonly PathNode Comparer = new PathNode(0, 0, default(TPathNode));

            public TPathNode UserContext { get; internal set; }
            public Double G { get; internal set; }
            public Double H { get; internal set; }
            public Double F { get; internal set; }
            public int Index { get; set; }


            public Boolean IsWalkable(TUserContext inContext)
            {
                return UserContext.IsWalkable(inContext);
            }

            public int X { get; internal set; }
            public int Y { get; internal set; }

            public int Compare(PathNode x, PathNode y)
            {
                if (x.F < y.F)
                    return -1;
                else if (x.F > y.F)
                    return 1;
                return 0;
            }

            public PathNode(int inX, int inY, TPathNode inUserContext)
            {
                X = inX;
                Y = inY;
                UserContext = inUserContext;
            }
        }

        public SpatialAStar(TPathNode[,] inGrid)
        {
            SearchSpace = inGrid;
            Width = inGrid.GetLength(0);
            Height = inGrid.GetLength(1);
            m_SearchSpace = new PathNode[Width, Height];
            m_ClosedSet = new OpenCloseMap(Width, Height);
            m_OpenSet = new OpenCloseMap(Width, Height);
            m_CameFrom = new PathNode[Width, Height];
            m_RuntimeGrid = new OpenCloseMap(Width, Height);
            m_OrderedOpenSet = new PriorityQueue<PathNode>(PathNode.Comparer);

            Cm_SearchSpace = new PathNode[Width, Height];
            Cm_ClosedSet = new OpenCloseMap(Width, Height);
            Cm_OpenSet = new OpenCloseMap(Width, Height);
            Cm_CameFrom = new PathNode[Width, Height];
            Cm_RuntimeGrid = new OpenCloseMap(Width, Height);
            Cm_OrderedOpenSet = new PriorityQueue<PathNode>(PathNode.Comparer);



            for (int x = 0; x < Width; x++)
            {
                for (int y = 0; y < Height; y++)
                {
                    if (inGrid[x, y] == null)
                        throw new ArgumentNullException();

                    m_SearchSpace[x, y] = new PathNode(x, y, inGrid[x, y]);
                    Cm_SearchSpace[x, y] = new PathNode(x, y, inGrid[x, y]);


                }
            }
        }

        protected virtual Double Heuristic(PathNode inStart, PathNode inEnd)
        {
            return Math.Sqrt((inStart.X - inEnd.X) * (inStart.X - inEnd.X) + (inStart.Y - inEnd.Y) * (inStart.Y - inEnd.Y));
        }

        public static readonly Double SQRT_2 = Math.Sqrt(2);

        protected virtual Double NeighborDistance(PathNode inStart, PathNode inEnd)
        {
            int diffX = Math.Abs(inStart.X - inEnd.X);
            int diffY = Math.Abs(inStart.Y - inEnd.Y);

            switch (diffX + diffY)
            {
                case 1: return 1;
                case 2: return SQRT_2;
                case 0: return 0;
                default:
                    throw new ApplicationException();
            }
        }

        //private List<Int64> elapsed = new List<long>();

        /// <summary>
        /// Returns null, if no path is found. Start- and End-Node are included in returned path. The user context
        /// is passed to IsWalkable().
        /// </summary>
        public LinkedList<TPathNode> Search(System.Drawing.Point inStartNode, System.Drawing.Point inEndNode, TUserContext inUserContext,int [,] borderindex,int numCuts)
        {
            PathNode startNode = m_SearchSpace[inStartNode.X, inStartNode.Y];
            PathNode endNode = m_SearchSpace[inEndNode.X, inEndNode.Y];

            //System.Diagnostics.Stopwatch watch = new System.Diagnostics.Stopwatch();
            //watch.Start();

            if (startNode == endNode)
                return new LinkedList<TPathNode>(new TPathNode[] { startNode.UserContext });

            PathNode[] neighborNodes = new PathNode[8];

            int nodes = 0;
            //Console.WriteLine("WE did not update node variable yet");
            bool locker = false;
            if (numCuts == 1)
            {
               
                m_ClosedSet.Clear();
                m_OpenSet.Clear();
                m_RuntimeGrid.Clear();
                m_OrderedOpenSet.Clear();

                Cm_ClosedSet.Clear();
                Cm_OpenSet.Clear();
                Cm_RuntimeGrid.Clear();
                Cm_OrderedOpenSet.Clear();

                for (int x = 0; x < Width; x++)
                {
                    for (int y = 0; y < Height; y++)
                    {
                        m_CameFrom[x, y] = null;
                        Cm_CameFrom[x, y] = null;
                    }
                }
                startNode.G = 0;
                startNode.H = Heuristic(startNode, endNode);
                startNode.F = startNode.H;

                m_OpenSet.Add(startNode);
                Cm_OpenSet.Add(startNode);
                m_OrderedOpenSet.Push(startNode);
                Cm_OrderedOpenSet.Push(startNode);
                m_RuntimeGrid.Add(startNode);
                Cm_RuntimeGrid.Add(startNode);


            }
            else
            {
                //for (int i=0;i < m_OpenSet.Count;i++)
                //{
                //    PathNode x = m_OpenSet.
                //    Cm_OpenSet.Add()
                //}

            }

            while (!m_OpenSet.IsEmpty)
            {

                PathNode x = m_OrderedOpenSet.Pop();
                if (borderindex[x.X, x.Y] == 1 && !locker)
                {
                    pathHitBorder = true;
                    //Cm_OrderedOpenSet.InnerList = m_OrderedOpenSet.items;
                    //for (int i = 0; i < m_OrderedOpenSet.Count; i++)
                        Cm_OrderedOpenSet.InnerList = m_OrderedOpenSet.items.ToList();
                    //    Cm_OpenSet.m_Map = m_OpenSet.items();
                    //Cm_RuntimeGrid.m_Map = m_RuntimeGrid.items();
                    //Cm_SearchSpace = m_SearchSpace;
                    //Cm_CameFrom = m_CameFrom;
                    //Cm_ClosedSet.m_Map = m_ClosedSet.items();

                    //foreach (PathNode t in  m_OrderedOpenSet.items )
                    //{
                    //    Cm_OrderedOpenSet.InnerList.Add(t);
                    //}
                    for (int k = 0; k < Width; k++)
                    {
                        for (int l = 0; l < Height; l++)
                        {
                            if (m_ClosedSet.m_index[k, l] != 0)
                            {
                                Cm_ClosedSet.Add(m_ClosedSet.itemAt(k, l));
                               // TCm_ClosedSet.Add(m_ClosedSet.itemAt(k, l));

                            }
                            if (m_OpenSet.m_index[k, l] != 0)
                            {
                                Cm_OpenSet.Add(m_OpenSet.itemAt(k, l));
                            }

                            if (m_RuntimeGrid.m_index[k, l] != 0)
                            {
                                Cm_RuntimeGrid.Add(m_RuntimeGrid.itemAt(k, l));
                            }
                            //if (m_CameFrom!=null)
                            //{
                            //    Cm_CameFrom=
                            //}

                        }
                    }

                    locker = true;
                }
                //PathNode xxx;



                if (x == endNode)
                {
                    // watch.Stop();

                    //elapsed.Add(watch.ElapsedMilliseconds);
                    //Console.WriteLine("Cost {0}", x.F);
                    pathCost = x.F;
                    LinkedList<TPathNode> result = ReconstructPath(m_CameFrom, m_CameFrom[endNode.X, endNode.Y]);

                    if (pathHitBorder)
                    {
                        //for (int k = 0; k < Width; k++)
                        //{
                        //    for (int l = 0; l < Height; l++)
                        //    {
                        //        if (m_ClosedSet.m_index[k, l] != 0)
                        //        {
                        //            Cm_ClosedSet.Add(m_ClosedSet.itemAt(k, l));
                        //        }
                        //        if (m_OpenSet.m_index[k, l] != 0)
                        //        {
                        //            Cm_OpenSet.Add(m_OpenSet.itemAt(k, l));
                        //        }

                        //        if (m_RuntimeGrid.m_index[k, l] != 0)
                        //        {
                        //            Cm_RuntimeGrid.Add(m_RuntimeGrid.itemAt(k, l));
                        //        }
                        //        //if (m_CameFrom!=null)
                        //        //{
                        //        //    Cm_CameFrom=
                        //        //}

                        //    }
                        //}




                    }

                    
                    


              //  }

                result.AddLast(endNode.UserContext);


                    return result;
                }
                //if (!pathHitBorder )
                //{
                //    Cm_OpenSet.Remove(x);
                //    Cm_ClosedSet.Add(x);
                //}
                m_OpenSet.Remove(x);
                m_OpenSet.m_index[x.X, x.Y] = 0;
                m_ClosedSet.Add(x);

                StoreNeighborNodes(x, neighborNodes);

                for (int i = 0; i < neighborNodes.Length; i++)
                {
                    PathNode y = neighborNodes[i];
                    Boolean tentative_is_better;

                    if (y == null)
                        continue;

                    if (!y.UserContext.IsWalkable(inUserContext))
                        continue;

                    if (m_ClosedSet.Contains(y))
                        continue;

                    nodes++;

                    Double tentative_g_score = m_RuntimeGrid[x].G + NeighborDistance(x, y);
                    Boolean wasAdded = false;

                    if (!m_OpenSet.Contains(y))
                    {
                        m_OpenSet.Add(y);
                        tentative_is_better = true;
                        wasAdded = true;
                        //if (!pathHitBorder &&numCuts == 1)
                        //{
                        //    Cm_OpenSet.Add(y);
                        //}
                    }
                    else if (tentative_g_score < m_RuntimeGrid[y].G)
                    {
                        tentative_is_better = true;
                    }
                    else
                    {
                        tentative_is_better = false;
                    }

                    if (tentative_is_better)
                    {
                        m_CameFrom[y.X, y.Y] = x;
                        //if (!pathHitBorder)
                        //{
                        //    Cm_CameFrom[y.X, y.Y] = x;
                        //}

                        if (!m_RuntimeGrid.Contains(y))
                        {
                            m_RuntimeGrid.Add(y);
                            //if (!pathHitBorder && numCuts == 1)
                            //{
                            //    Cm_RuntimeGrid.Add(y);
                            //}
                        }

                        m_RuntimeGrid[y].G = tentative_g_score;
                        m_RuntimeGrid[y].H = Heuristic(y, endNode);
                        m_RuntimeGrid[y].F = m_RuntimeGrid[y].G + m_RuntimeGrid[y].H;
                        //if (!pathHitBorder)
                        //{
                        //    Cm_RuntimeGrid[y].G = tentative_g_score;
                        //    Cm_RuntimeGrid[y].H = Heuristic(y, endNode);
                        //    Cm_RuntimeGrid[y].F = m_RuntimeGrid[y].G + m_RuntimeGrid[y].H;
                        //}

                        if (wasAdded)
                        {
                            m_OrderedOpenSet.Push(y);
                            //if (!pathHitBorder)
                            //{
                            //    Cm_OrderedOpenSet.Push(y);
                            //}
                        }

                        else
                        {
                            m_OrderedOpenSet.Update(y);
                            //if (!pathHitBorder)
                            //    Cm_OrderedOpenSet.Update(y);
                        }
                    }
                }
            }

            return null;
        }

        public LinkedList<TPathNode> ReconstructPath(PathNode[,] came_from, PathNode current_node)
        {
            LinkedList<TPathNode> result = new LinkedList<TPathNode>();

            ReconstructPathRecursive(came_from, current_node, result);

            return result;
        }

        public void ReconstructPathRecursive(PathNode[,] came_from, PathNode current_node, LinkedList<TPathNode> result)
        {
            PathNode item = came_from[current_node.X, current_node.Y];

            if (item != null)
            {

                ReconstructPathRecursive(came_from, item, result);

                result.AddLast(current_node.UserContext);
                resultList.Add(new Point(item.X,item.Y));
            }
            else
            {
                result.AddLast(current_node.UserContext);
//                resultList.Add(new Point(item.X, item.Y));

            }

        }

        public void StoreNeighborNodes(PathNode inAround, PathNode[] inNeighbors)
        {
            int x = inAround.X;
            int y = inAround.Y;
            bool Right = false;
            bool Left = false;
            bool Up = false;
            bool down = false;

            if (x < Width - 1)
                inNeighbors[4] = m_SearchSpace[x + 1, y];
            else
            {
                inNeighbors[4] = null;
                down = true;
            }

            if (x > 0)
                inNeighbors[3] = m_SearchSpace[x - 1, y];
            else
            {
                inNeighbors[3] = null;
                Up = true;
            }

            if (y < Height - 1)
                inNeighbors[6] = m_SearchSpace[x, y + 1];
            else
            {
                inNeighbors[6] = null;
                Right = true;
            }
            if (y > 0)
                inNeighbors[1] = m_SearchSpace[x, y - 1];
            else
            {
                inNeighbors[1] = null;
                Left = true;
            }


            if ((x > 0) && (y > 0))
                inNeighbors[0] = m_SearchSpace[x - 1, y - 1];
            else
                inNeighbors[0] = null;



            if ((x < Width - 1) && (y > 0))
                inNeighbors[2] = m_SearchSpace[x + 1, y - 1];
            else
                inNeighbors[2] = null;

            

            if ((x > 0) && (y < Height - 1))
                inNeighbors[5] = m_SearchSpace[x - 1, y + 1];
            else
                inNeighbors[5] = null;


            if ((x < Width - 1) && (y < Height - 1))
                inNeighbors[7] = m_SearchSpace[x + 1, y + 1];
            else
                inNeighbors[7] = null;
        }

        public class OpenCloseMap
        {
            public PathNode[,] m_Map;

            public int[,] m_index;
            public int Width { get;  set; }
            public int Height { get;  set; }
            public int Count { get;  set; }
            
            public PathNode this[Int32 x, Int32 y]
            {
                get
                {
                    return m_Map[x, y];
                }
            }

            public PathNode this[PathNode Node]
            {
                get
                {
                    return m_Map[Node.X, Node.Y];
                }

            }

            public bool IsEmpty
            {
                get
                {
                    return Count == 0;
                }
            }

            public OpenCloseMap(int inWidth, int inHeight)
            {
                m_Map = new PathNode[inWidth, inHeight];
                Width = inWidth;
                Height = inHeight;
                m_index = new int[Width, Height];

            }

            public void Add(PathNode inValue)
            {

                PathNode item = m_Map[inValue.X, inValue.Y];

#if DEBUG
                //if (item != null)
                //    throw new ApplicationException();
#endif

                Count++;
                m_Map[inValue.X, inValue.Y] = inValue;
                m_index[inValue.X, inValue.Y] = 1;
            }

            public bool Contains(PathNode inValue)
            {
                PathNode item = m_Map[inValue.X, inValue.Y];

                if (item == null)
                    return false;

#if DEBUG
                //if (!inValue.Equals(item))
                //    throw new ApplicationException();
#endif

                return true;
            }

            public void Remove(PathNode inValue)
            {
                PathNode item = m_Map[inValue.X, inValue.Y];

#if DEBUG
                if (!inValue.Equals(item))
                    throw new ApplicationException();
#endif

                Count--;
                m_Map[inValue.X, inValue.Y] = null;
            }

            public void Clear()
            {
                Count = 0;

                for (int x = 0; x < Width; x++)
                {
                    for (int y = 0; y < Height; y++)
                    {
                        m_Map[x, y] = null;
                    }
                }
            }
            public PathNode itemAt(int x,int y)
                {
                PathNode item = m_Map[x, y];
                return item;
            }
        }
    }
}
