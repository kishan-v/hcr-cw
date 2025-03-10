using UnityEngine;
using System.Collections.Generic;
using Newtonsoft.Json;
using System.Diagnostics;
using System.Text;
using Unity.Mathematics;
using Unity.Collections;
using System.Text.Json;
using Unity.Burst;
using Unity.Jobs;

using Debug = UnityEngine.Debug;

public class LidarProcessor : MonoBehaviour
{
    public GameObject boxPrefab;
    private List<GameObject> boxes = new List<GameObject>();

    static private List<int> FindTrueIndices(byte[] data)
    {
        List<int> trueIndices = new List<int>();

        /* We are sending all the header information (depth, width, height
         * etc) even though we are apparently hardcoding all of them. We
         * can't create a copy of the whole array otherwise this would be
         * expensive, so we just skip the first 20 bytes. Yes. */
        int headerSize = 20;
        for (int i = headerSize; i < data.Length; i++)
        {
            for (int j = 0; i < 8; i++) 
            {
               // Extract a bit and shift along
               // remember this is big-endian!!
               bool bit = (data[i] & (1 << j)) != 0;
               // Multiply byte index (i) by 8, then add bit index (j)
               if (bit) trueIndices.Add((i - headerSize)*8 + j);
            }
        } 
        return trueIndices;
    }

    public void ProcessLidarData(byte[] lidarJson)
    {

        Stopwatch stopwatch = new Stopwatch();
        stopwatch.Start();

        List<Vector3> positions = decompressData(lidarJson);
        DrawBoxes3(positions);

        stopwatch.Stop();

        System.TimeSpan ts = stopwatch.Elapsed;
        string elapsedTime = string.Format("{0:00}:{1:00}:{2:00}.{3:000}", ts.Hours, ts.Minutes, ts.Seconds, ts.Milliseconds);
        //Debug.Log("Function Execution Time: " + elapsedTime);
        //Debug.Log("Function Execution Time: " + elapsedTime);
        //Debug.Log("Successfully deserialised the json");
    }


    static private List<Vector3> decompressData(byte[] data) {
        // extract data
        List<int> trueIndices = FindTrueIndices(data); // Convert list to array

        // No move semantics in C# !! Very expensive copy into a native array
        NativeArray<float4> indices = new NativeArray<float4>(trueIndices.Count, Allocator.Persistent);
        for (int i = 0; i < trueIndices.Count; i++)
        {
            indices[i] = (float)trueIndices[i]; // Explicit cast to float
        }

        NativeArray<float4> z = new NativeArray<float4>(indices.Length, Allocator.Persistent);
        NativeArray<float4> y = new NativeArray<float4>(indices.Length, Allocator.Persistent);
        NativeArray<float4> x = new NativeArray<float4>(indices.Length, Allocator.Persistent);
 
        var job = new Decompress{
            indices=indices,
            x=x,
            y=y, 
            z=z
        };

        job.Schedule().Complete();

        List<Vector3> coord = ConvertToVector3List(ref x, ref z, ref y); // remember to convert to the unity coord frame 

        // dispose to avoid mem leak
        x.Dispose();
        y.Dispose();
        z.Dispose();
        indices.Dispose();

        return coord;
    }

    [BurstCompile(CompileSynchronously = true)]
    private struct Decompress: IJob 
    {
        public NativeArray<float4> x;
        public NativeArray<float4> y;
        public NativeArray<float4> z;


        [ReadOnly]
        public NativeArray<float4> indices;

        public void Execute()
        {
            // Okay so hard-coded width, depth, height and step_size!
            // set up constants
            const float x_area = (float) (4.0 * (1.0 / 0.1));
            const float y_area = (float) ((float) 4.0 * (1.0 / 0.1)) * x_area;
            const float step_size = (float) 0.1;

            // calculate_z
            NativeArray<float4>.Copy(indices, z);

            for(int i = 0; i < z.Length; i++)
            {
                z[i] = math.floor(z[i] / y_area) * step_size;
            }


            // calculate y
            NativeArray<float4>.Copy(indices, y);

            for(int i = 0; i < y.Length; i++)
            {
                y[i] = math.floor((y[i] % y_area) / x_area) * step_size - ((float) 2.0);
            }

            // calculate x
            NativeArray<float4>.Copy(indices, x);

            for(int i = 0; i < x.Length; i++)
            {
                x[i] = math.floor((x[i] % y_area) % x_area) * step_size - ((float) 2.0);
            }

        }
    }

    private void DrawBoxes3(List<Vector3> positions)
    {
        ClearBoxes();

        foreach (Vector3 position in positions)
        {
            GameObject box = Instantiate(boxPrefab, position, Quaternion.identity);
            boxes.Add(box);
        }
    }

    static private List<Vector3> ConvertToVector3List(ref NativeArray<float4> xVectors, ref NativeArray<float4> yVectors, ref NativeArray<float4> zVectors)
    {
        List<Vector3> positions = new List<Vector3>();
        for (int i = 0; i < xVectors.Length; i++)
        {
            positions.Add(new Vector3(xVectors[i].x, yVectors[i].x, zVectors[i].x));
        }
        return positions;
    }

    void OnDestroy()
    {
        ClearBoxes();
    }

    void OnApplicationPause(bool pauseStatus)
    {
        if (pauseStatus)
        {
            ClearBoxes();
        }
    }


    private void ClearBoxes()
    {
        foreach (GameObject box in boxes)
        {
            Destroy(box);
        }
        boxes.Clear();
    }
}
