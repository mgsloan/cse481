using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.IO;
using Accord.Statistics.Distributions.Univariate;
using Accord.Statistics.Models.Markov;
using Accord.Statistics.Models.Markov.Topology;
using Accord.Statistics.Models.Markov.Learning;
using System.Collections;
using System.Runtime.Serialization;
using System.Runtime.Serialization.Formatters.Binary;
using Accord.Statistics.Distributions.Multivariate;
using Accord.Statistics.Distributions.Fitting;
using Accord.Statistics.Analysis;

namespace HMMTest
{
    public class HMMClassifier
    {
        private static BinaryFormatter bin = new BinaryFormatter();

        // fraction of samples to use for training a given HMM
        private static readonly double TRAIN_PROPORTION = .8; 

        // the series of positions for a gesture is condensed into NUM_BLOCKS values
        // (after applying PCA), using averaging (see 'shrink')
        private static readonly int NUM_BLOCKS = 6;

        // minimum MLE probability for accepting a gesture for the HMM
        // in PCA, the relative weights of the eigenvals correspond to how much of
        // the variance each dimension accounts for, the classifier retains enough
        // dimensions to account for INFO_PROPORTION of the variance
        private static readonly double INFO_PROPORTION = .999;

        // the number of states for the underlying HMM (it may be best to keep this
        // equal to the number of blocks, since this creates a state for each 'timestep')
        private static readonly int NUM_STATES = 6;

        private double threshold;
        private HiddenMarkovModel<MultivariateNormalDistribution> hMM;
        private PrincipalComponentAnalysis pca;
        private int dimension;
        private int reducedDimension;

        private String name; public String getName() { return name; }

        static void Main(string[] args)
        {
            String directory = Directory.GetCurrentDirectory();
            String motion_dir = directory + "\\motion_data";

            // should all be true
            var classifier1 = new HMMClassifier();
            double[][][] motions1 = getMotions(motion_dir + "\\raise2");
            classifier1.Initialize(motions1);

            // should all be true
            var classifier2 = new HMMClassifier();
            double[][][] motions2 = getMotions(motion_dir + "\\raise");
            classifier2.Initialize(motions2);

            // should all be false
            int pos = 0;
            for (int i = 0; i < motions2.Length; i++)
            {
                bool class1 = classifier1.isMember(motions2[i]);
                if (class1) pos++;
            }
            Console.WriteLine("" + pos + " of " + motions2.Length);

            // should all be false
            pos = 0;
            for (int i = 0; i < motions1.Length; i++)
            {
                bool class1 = classifier2.isMember(motions1[i]);
                if (class1) pos++;
            }
            Console.WriteLine("" + pos + " of " + motions1.Length);

            Console.ReadLine();
        }

        public void Initialize(String dirName)
        {
            String[] path = dirName.Split('\\');
            this.name = path[path.Length - 1];
            double[][][] motions = getMotions(dirName);
            Initialize(motions);
        }

        void Initialize(double[][][] motions)
        {
            this.dimension = motions[0][0].Length; // use any data point
            double[][][] reducedMotions = pcaReduction(motions);

            int train = (int) (reducedMotions.Length * TRAIN_PROPORTION);
            int test = reducedMotions.Length - train;

            double[][][] trainMotions = new double[train][][];
            int[] trainIndexes = HMMClassifier.getRandomIndexes(train, reducedMotions.Length);

            Console.Write("Train Indexes: ");
            for (int i = 0; i < train; i++) Console.Write("[" + trainIndexes[i] + "]");
            Console.WriteLine();

            for (int i = 0; i < train; i++) trainMotions[i] = shrink(reducedMotions[trainIndexes[i]], NUM_BLOCKS);

            this.hMM = HMMClassifier.createHMM(trainMotions);

            // allow for some (!) slop, other gestures should score extremely low
            // so this shouldn't be a problem
            this.threshold = 1e-10; 

            // the rest is printing stats
            double[] probs = new double[reducedMotions.Length]; // for information purposes
            for (int i = 0; i < reducedMotions.Length; i++)
            {
                double prob = hMM.Evaluate(shrink(reducedMotions[i], NUM_BLOCKS));
                probs[i] = prob; // for information purposes
            }

            int yesTrain = 0, yesTest = 0;
            int tI = 0;
            for (int i = 0; i < reducedMotions.Length; i++)
            {
                if (probs[i] >= threshold)
                {
                    if (tI < trainIndexes.Length && i == trainIndexes[tI]) yesTrain++;
                    else yesTest++;
                }
                if (tI < trainIndexes.Length && i == trainIndexes[tI]) tI++;
            }
            Console.WriteLine("" + yesTrain + " of " + train + " train.");
            Console.WriteLine("" + yesTest + " of " + test + " test.");
            Console.WriteLine("" + (yesTrain + yesTest) + " of " + (train + test) + " total.");
        }

        public double evaluate(double[][] motion)
        {
            return hMM.Evaluate(shrink(reduce(motion), NUM_BLOCKS));
        }

        public bool isMember(double[][] motion)
        {
            return evaluate(motion) > threshold;
        }

        static HiddenMarkovModel<MultivariateNormalDistribution> createHMM(double[][][] motions)
        {
            int dimension = motions[0][0].Length; // use 1st observation (any would suffice)
            var density = new MultivariateNormalDistribution(dimension);
            var hMM = new HiddenMarkovModel<MultivariateNormalDistribution>(NUM_STATES, density);

            // May have to specify a regularization constant here
            var bWL = new BaumWelchLearning<MultivariateNormalDistribution>(hMM)
            {
                Tolerance = 0.0001,
                Iterations = 0,

                // Specify a regularization constant
                FittingOptions = new NormalOptions() { Regularization = 1e-8 }
            };
            
            double logLikelihood = bWL.Run(motions);
            return hMM;
        }

        // form a PCA transformation to account for INFO_PROPORTION of the variance
        // (include more informative dimensions first), discarding the remainder of
        // the dimensions
        private double[][][] pcaReduction(double[][][] motions)
        {
            double[,] mtrx = concatenate(motions);
            int[] blockSizes = getBlockSizes(motions);
            this.pca = new PrincipalComponentAnalysis(mtrx);
            pca.Compute();

            double[] E = pca.Eigenvalues;
            double sum = 0;
            for (int i = 0; i < E.Length; i++) sum += E[i];
            Array.Sort(E);
            double info = 0;
            double infoThreshold = INFO_PROPORTION * sum;
            reducedDimension = 0;
            for (int i = E.Length - 1; i > 0 && info < infoThreshold; i--)
            {
                reducedDimension++;
                info += E[i];
            }

            double[,] components = pca.Transform(mtrx, reducedDimension);
            double[][][] reducedMotions = split(components, blockSizes, reducedDimension);
            return reducedMotions;
        }

        // apply the derived PCA transformation to an input matrix (reducing
        // the dimension from 'dimension' to 'reducedDimension')
        private double[][] reduce(double[][] mtrx)
        {
            double[,] mdArr = new double[mtrx.Length, dimension];
            for (int i = 0; i < mtrx.Length; i++)
            {
                for (int j = 0; j < dimension; j++)
                {
                    mdArr[i, j] = mtrx[i][j];
                }
            }
            double[,] reducedMDArr = pca.Transform(mdArr, reducedDimension);
            double[][] reducedMtrx = new double[mtrx.Length][];
            for (int i = 0; i < mtrx.Length; i++)
            {
                reducedMtrx[i] = new double[reducedDimension];
                for (int j = 0; j < reducedDimension; j++)
                {
                    reducedMtrx[i][j] = reducedMDArr[i, j];
                }
            }
            return reducedMtrx;
        }

        // evenly split a double[][] into contiguous blocks of double[] and
        // merge each contiguous block into 1 point
        static private double[][] shrink(double[][] mtrx, int numBlocks)
        {
            double blockAvg = mtrx.Length / (double) numBlocks;
            int ceil = (int) Math.Ceiling(blockAvg);
            int floor = (int) Math.Floor(blockAvg);

            int accountedFor = 0;
            Random random = new Random();
            double[][] shrink = new double[numBlocks][];
            for (int i = 0; i < numBlocks - 1 && accountedFor <= mtrx.Length - ceil; i++)
            {
                int span;
                if (accountedFor <= blockAvg * i) span = ceil;
                else span = floor;
                shrink[i] = average(mtrx, accountedFor, span);
                accountedFor += span;
            }
            shrink[numBlocks - 1] = average(mtrx, accountedFor, mtrx.Length - accountedFor);

            return shrink;
        }

        // average a set of multidimensional points into one point
        static double[] average(double[][] points, int startIndex, int span)
        {
            int dimension = points[0].Length;
            double[] sum = new double[dimension];
            for (int i = 0; i < span; i++)
            {
                for (int j = 0; j < dimension; j++) sum[j] += points[startIndex + i][j];
            }
            for (int j = 0; j < dimension; j++) sum[j] /= span;
            return sum;
        }

        // expects a file to contain data for 1 example of 1 motion, where
        // each line contains all the angle values for 1 instant in time.
        public static double[][] getMotion(String fileName)
        {
            StreamReader reader = new StreamReader(fileName);
            ArrayList observations = new ArrayList();

            String line;
            String[] values;
            while (reader.Peek() >= 0)
            {
                line = reader.ReadLine();
                values = line.Split(',');
                if (values.Length > 1)
                {
                    double[] obs_values = new double[values.Length - 1];
                    for (int i = 0; i < values.Length - 1; i++)
                    {
                        obs_values[i] = Double.Parse(values[i + 1]);
                    }
                    observations.Add(obs_values);
                }
            }
            double[][] motion = new double[observations.Count][];
            for (int i = 0; i < observations.Count; i++)
            {
                motion[i] = (double[]) observations[i];
            }

            return motion;
        }

        // return an array of double[][] where each double[][] represents 
        // a gesture, or series of poses
        static double[][][] getMotions(String dirName)
        {
            String[] files = Directory.GetFiles(dirName, "*.rec");
            double[][][] motions = new double[files.Length][][];
            for (int i = 0; i < files.Length; i++)
            {
                motions[i] = getMotion(files[i]);
            }
            return motions;
        }

        // join together each member in an array of double[][] to form a
        // single double[][]
        static double[,] concatenate(double[][][] mtrxList)
        {
            int numRows = 0;
            for (int i = 0; i < mtrxList.Length; i++) numRows += mtrxList[i].Length;
            double[,] concat = new double[numRows, mtrxList[0][0].Length];
            int rowIndex = 0;
            for (int i = 0; i < mtrxList.Length; i++)
            {
                for (int j = 0; j < mtrxList[i].Length; j++)
                {
                    for (int k = 0; k < mtrxList[i][j].Length; k++)
                    {
                        concat[rowIndex, k] = mtrxList[i][j][k];
                    }
                    rowIndex++;
                }
            }
            return concat;
        }

        // split a double[,] into an array of double[][] using blockSizes as
        // the delineators (there should be blockSizes.Length double[][])
        static double[][][] split(double[,] mtrx, int[] blockSizes, int dimension)
        {
            double[][][] mtrxList = new double[blockSizes.Length][][];
            int rowIndex = 0;
            for (int i = 0; i < blockSizes.Length; i++)
            {
                mtrxList[i] = new double[blockSizes[i]][];
                for (int j = 0; j < blockSizes[i]; j++)
                {
                    mtrxList[i][j] = new double[dimension];
                    for (int k = 0; k < dimension; k++)
                    {
                        mtrxList[i][j][k] = mtrx[rowIndex, k];
                    }
                    rowIndex++;
                }
            }
            return mtrxList;
        }

        // used to convert a double[,] back to a double[][][] (assuming the
        // double[,] was obtained by calling concatenate on 'blocks')
        static int[] getBlockSizes(double[][][] blocks)
        {
            int[] blockSizes = new int[blocks.Length];
            for (int i = 0; i < blocks.Length; i++)
            {
                blockSizes[i] = blocks[i].Length;
            }
            return blockSizes;
        }

        // given a zero indexed array example gestures of length 'total'
        // randomly select 'train' of the indexes (train should be <= total)
        static int[] getRandomIndexes(int train, int total)
        {
            int[] indexes = new int[train];
            Random random = new Random();
            int numLeft = total;
            int numNeeded = train;
            for (int i = 0; i < total && numNeeded > 0; i++)
            {
                if (random.NextDouble() < (numNeeded / (double)numLeft))
                {
                    indexes[train - numNeeded] = i;
                    numNeeded--;
                }
                numLeft--;
            }
            return indexes;
        }

        // TODO: include PCA matrix

        void serializeModel(Stream stream)
        {
            bin.Serialize(stream, hMM.Transitions);
            bin.Serialize(stream, hMM.Emissions);
            bin.Serialize(stream, hMM.Probabilities);
        }

        void parseModel(Stream stream)
        {
            var transitions   = (double[,])                        bin.Deserialize(stream);
            var emissions     = (MultivariateNormalDistribution[]) bin.Deserialize(stream);
            var probabilities = (double[])                         bin.Deserialize(stream);
            hMM = new HiddenMarkovModel<MultivariateNormalDistribution>(transitions, emissions, probabilities);
        }

        public double[][] getPerformMotion()
        {
            String directory = Directory.GetCurrentDirectory() + "\\motion_data\\" + this.name;
            String[] files = Directory.GetFiles(directory, "*.rec");
            
            double[][] motions = getMotion(files[7]);
            
            return motions;
        }
    }
}
