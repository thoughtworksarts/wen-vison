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

//Something helpful for this class will be to make set constants for the startpositions and origins
namespace WenLibrary
{
    public class PositionUpdater
    {
        //this is the 1x6 array from the parser, one row
        private float[] rotationAngles;

        private float[,] currentPositions; //maybe these should be arrays of joints who each posess their own coordinates

        private float[,] updatedPositions;

        private float[,] origins;

        private int[,] planeTypes;

        public PositionUpdater(float[] rotations, float[,] startPositions, float[,] origins, float[,] planeTypes)
        {
            this.rotationAngles = rotations; //maybe take this out and put it in the method 
            this.currentPositions = startPositions;
            this.origins = origins;
            this.planeTypes = planeTypes;

        }

        //should generate a list of positions in x,y,z coords for each joint 


        //Right now, this method updates the endPositions array
        public float[,] nextPositionsFromOneJointMovement(float[] rotationsAngles, int jointIndex, float[,] currentOrigins)
        {
            float[] jointPositions = new float[3];

            this.origins = currentOrigins;

            float angle = rotationAngles.GetValue(jointIndex);

            float[] origin = origins.GetValue(jointIndex);

            float[] currentPositions = this.currentPositions.GetValue(jointIndex); //will be an [X,Y,Z] array

            int planeType = determineRotationAxis(planeTypes.GetValue(jointIndex));
            //assign X, Y, and Z according to the planeType,, where X and Y become the relevent axes in the plane (could be X and Z) and Z is assigned to the rotation axis
            if (planeType == 0) //Y,Z plane
            {
                //nextPositions = rotateByAngle(angle, assign x=y and y=z, z=x )
                jointPositions = rotateByAngle(angle, origin.GetValue(1), origin.GetValue(2), origin.GetValue(0), currentPositions.GetValue(1), currentPositions.GetValue(2), currentPositions.GetValue(0));
            }

            if (planeType == 1) //X,Z plane:
            {
                //assign x=x, y=z, z=y
                jointPositions = rotateByAngle(angle, origin.GetValue(0), origin.GetValue(2), origin.GetValue(1), currentPositions.GetValue(0), currentPositions.GetValue(2), currentPositions.GetValue(1));
            }

            if (planeType == 2) //x, y PLANE
            {
                //ASSIGN x=x. y=y, z=z
                jointPositions = rotateByAngle(angle, origin.GetValue(0), origin.GetValue(1), origin.GetValue(2), currentPositions.GetValue(0), currentPositions.GetValue(1), currentPositions.GetValue(2));
            }

            //update endPositions
            //update origins
            updatedPositions[jointIndex, 0] = jointPositions.GetValue(0);
            updatedPositions[jointIndex, 1] = jointPositions.GetValue(1);
            updatedPositions[jointIndex, 2] = jointPositions.GetValue(2);
            //updatedPositions.SetValue(jointPositions, jointIndex);
            //set startPositions = updatedPositions
            //set origins = updatedOrigins

            return updatedPositions;
        }

        public float[] rotateByAngle(float angle, float OriginX, float OriginY, float OriginZ, float currentPointX, float currentPointY, float currentPointZ)
        {
            float[] nextPointCoordinates = new float[3];

            float nextPointX;
            float nextPointY;
            float nextPointZ;

            nextPointX = OriginX + Math.Cos(angle) * (currentPointX - OriginX) - Math.Sin(angle) * (currentPointY - OriginY);
            nextPointY = OriginY + Math.Sin(angle) * (currentPointX - OriginX) - Math.Cos(angle) * (currentPointY - OriginY);
            nextPointZ = currentPointZ;

            nextPointCoordinates.SetValue(nextPointX, 0);
            nextPointCoordinates.SetValue(nextPointY, 1);
            nextPointCoordinates.SetValue(nextPointZ, 2);

            return nextPointCoordinates;
        }

        public int determineRotationAxis(float[] planeType)
        {
            //returns the index of the axis that the rotation occurs around, the 1's mean movement is visible in that plane
            if (planeType.GetValue(0) == 0) { return 0; } //movement happens in the (y,z) plane
            if (planeType.GetValue(1) == 0) { return 1; } //movement happens in the (x,z) plane
            if (planeType.GetValue(2) == 0) { return 2; } //movement happens in the (x,y) plane
        }

        //This method should update endPositions for each joint's desired angle movement
        //Will be slightly more complicated because it should do so based on startPositions = endPositions for the last update from the last joint
        //Will also need to update the origins (the adjacent joint's new center)
        public float[,] nextPositionsFromAllJointMovement()
        {
            //TODO
            //For the ball-in-socket joints, this method will have to update all joints that follow them to move their x coordinates
            return null;
        }

        //returns the new positions so that we can then translate them to the depth points and visualize them

        //rotation equation for x and y coordinates rotated in the plane by angle t:
        //origin points: ox, oy
        //current points: px, py
        //next points: qx, qy
        //qx = ox + cos(t)(px-ox) - sin(t)(py-oy)
        //qy = oy + sin(t)(px-ox) + cos(t)(py-oy)
    }
}

