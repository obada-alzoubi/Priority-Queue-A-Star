using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;

namespace PathFinder
{
    public class State
    {
        //Decalaration
        public float cost=0;
        public float heuristic=0;
        public float totalCost=0;
        public bool isBorder=false;

        public int X=0;
        public int Y=0;
        public int value = 1;
        public int previous_X = -1;
        public int previous_Y = -1;
        

        //Constructor
        public State(int X, int Y, float cost, float heuristic, float totalCost,bool isBorder,int value ,int prevoius_X,int prevoius_Y)
        {
            this.X = X;
            this.Y = Y;
            this.cost = cost;
            this.heuristic = heuristic ;
            this.totalCost = totalCost ;
            this.isBorder = isBorder ;
            this.value = value;
            previous_X = prevoius_X;
            previous_Y = prevoius_Y;
        }  

    }
}
