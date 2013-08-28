using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using BulletXNA;
using System.Threading;

namespace BulletXNADemos.Demos
{
    public class PoolTest
    {
        static void Main(string[] args)
        {
            PooledTypeManager pooledTypeManager = new PooledTypeManager(null);
            int iterations = 100000000;
            float[] result = null;
            double total = 0;
            for (int i = 0; i < iterations; ++i)
            {
                result = pooledTypeManager.FloatArray24Pool.Get();
                for(int j=0;j<result.Length;++j)
                {
                    result[j] = j;
                    total += result[j];
                }
                //pooledTypeManager.FloatArray24Pool.Free(result);
            }
            Thread.Sleep(1000 * 60 * 1);
            System.Console.Write("Result : "+total);
        }
    }
}
