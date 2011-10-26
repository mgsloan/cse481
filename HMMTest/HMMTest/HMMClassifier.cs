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
    class HMMClassifier
    {
        private static readonly double TRAIN_PROPORTION = .9; // fraction of samples to use for training
        private double threshold;
        private HiddenMarkovModel<MultivariateNormalDistribution> hMM;
        private PrincipalComponentAnalysis pca;
        private int dimension;
        private int reducedDimension;
        
        static void Main(string[] args)
        {
            String directory = Directory.GetCurrentDirectory();
            String motion_dir = directory + "\\motion_data";

            // should all be true
            var classifier1 = new HMMClassifier();
            double[][][] motions1 = getMotions(motion_dir + "\\raise");
            classifier1.Initialize(motions1);
            int pos = 0;
            for (int i = 0; i < motions1.Length; i++)
            {
                bool class1 = classifier1.isMember(motions1[i]);
                if (class1) pos++;
            }
            Console.WriteLine("" + pos + " of " + motions1.Length);

            /*
            // should all be true
            var classifier2 = new HMMClassifier();
            double[][][] motions2 = getMotions(motion_dir + "\\raise2");
            classifier2.Initialize(motions2);
            for (int i = 0; i < motions2.Length; i++)
            {
                bool class1 = classifier2.isMember(motions2[i]);
                Console.Write("[" + class1 + "]");
            }
            Console.WriteLine();

            // should all be false
            for (int i = 0; i < motions2.Length; i++)
            {
                bool class1 = classifier1.isMember(motions2[i]);
                Console.Write("[" + class1 + "]");
            }
            Console.WriteLine();

            // should all be false
            for (int i = 0; i < motions1.Length; i++)
            {
                bool class1 = classifier2.isMember(motions1[i]);
                Console.Write("[" + class1 + "]");
            }
            Console.WriteLine();
            */
            Console.WriteLine("got");
        }

        void Initialize(String dirName)
        {
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

            for (int i = 0; i < train; i++) trainMotions[i] = reducedMotions[trainIndexes[i]];

            this.hMM = HMMClassifier.createHMM(trainMotions);
            double sum = 0;
            for (int i = 0; i < reducedMotions.Length; i++)
            {
                double prob = hMM.Evaluate(reducedMotions[i]);
                sum += prob;
            }

            this.threshold = .8 * (sum / reducedMotions.Length);
        }

        bool isMember(double[][] motion)
        {
            double likelihood = hMM.Evaluate(reduce(motion));
            return (likelihood > threshold);
        }

        static HiddenMarkovModel<MultivariateNormalDistribution> createHMM(double[][][] motions)
        {
            int dimension = motions[0][0].Length; // use 1st observation (any would suffice)
            var density = new MultivariateNormalDistribution(dimension);
            var hMM = new HiddenMarkovModel<MultivariateNormalDistribution>(8, density);

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
            double infoThreshold = .999 * sum;
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

        // expects a file to contain data for 1 example of 1 motion, where
        // each line contains all the angle values for 1 instant in time.
        static double[][] getMotion(String fileName)
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

        static int[] getBlockSizes(double[][][] blocks)
        {
            int[] blockSizes = new int[blocks.Length];
            for (int i = 0; i < blocks.Length; i++)
            {
                blockSizes[i] = blocks[i].Length;
            }
            return blockSizes;
        }

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
    }
}
