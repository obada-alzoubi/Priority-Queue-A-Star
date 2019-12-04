/** Obada Al Zoubi (oma25@mail.aub.edu.lb)
 * Amerian University of Beirut
 * This Work is under Path Finding Algorithm project 
 * */

using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;

namespace PathFinder
{
    public class ScenarioLoader
    {
        //Declarations 
        public static string[] scen;
        public static string[,] scenInfo; 
        //Constructor 
        public ScenarioLoader(string scenPath)
        {
            scen = System.IO.File.ReadAllLines(scenPath);
            //Verify the Version,The First line of Scenario file                        
            if (scen[0]=="version 1")
                Console.WriteLine("The Version of the Map is Verified: {0}",scen[0]);
            //Load information from scenario
            scenInfo = new string[scen.Length - 1, 9];
            //Load All Scenarios
            /**ScenInfo  
             * 1-Bucket
             * 2-Map Name
             * 3-Map width
             * 4-Map height
             * 5-Start X
             * 6-Start Y
             * 7-End X
             * 8-End Y
             * 9-Optimal Length
             * */
            for (int i = 1; i < scen.Length; i++)
            {
                string [] tempScen =scen[i].Split('\t');
                int j=0;
                foreach (string str in tempScen)
                {
                    scenInfo[i - 1, j] = str;
                    j++;
                }

            }
        }


    }

}
