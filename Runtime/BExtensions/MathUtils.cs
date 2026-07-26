using System;
using UnityEngine;
using UnityEngine.Assertions;

namespace Bloodthirst.Core.Utils
{
    public static class MathUtils
    {
        /// <summary>
        /// Is val between min (inclusive) and max (inclusive) ?
        /// </summary>
        /// <param name="val"></param>
        /// <param name="min"></param>
        /// <param name="max"></param>
        /// <returns></returns>
        public static bool IsBetween(float val, float min, float max)
        {
            Assert.IsTrue(min < max);

            bool isValid = val <= max && val >= min;

            return isValid;
        }

        /// <summary>
        /// Is val between min (inclusive) and max (inclusive) ?
        /// </summary>
        /// <param name="val"></param>
        /// <param name="min"></param>
        /// <param name="max"></param>
        /// <returns></returns>
        public static bool IsBetween(int val, int min, int max)
        {
            Assert.IsTrue(min < max);

            bool isValid = val <= max && val >= min;

            return isValid;
        }

        public static float Remap(float val, float oldMin, float oldMax, float newMin, float newMax)
        {
            float ratio = (val - oldMin) / (oldMax - oldMin);

            return ((newMax - newMin) * ratio) + newMin;

        }

        /// <summary>
        /// Take a positive angle [0 , 360 , 720 , etc ...] and remaps it to [-180 , 180]
        /// </summary>
        /// <param name="angle"></param>
        /// <returns></returns>
        public static float RoundAngle(float angle)
        {
            while (angle < 0)
            {
                angle += 360;
            }

            angle %= 360;

            if(angle > 180)
            {
                angle = angle - 360;
            }

            return angle;
        }
    }
}
