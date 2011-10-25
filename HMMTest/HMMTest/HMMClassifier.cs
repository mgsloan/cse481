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
        private static readonly double TRAIN_PROPORTION = .8; // fraction of samples to use for training
        private double threshold;
        private HiddenMarkovModel<MultivariateNormalDistribution> hMM;
        
        static void Main(string[] args)
        {
            String directory = Directory.GetCurrentDirectory();
            String motion_dir = directory + "\\motion_data";

            // should all be true
            var classifier1 = new HMMClassifier();
            double[][][] motions1 = getMotions(motion_dir + "\\raise");
            classifier1.Initialize(motions1);
            for (int i = 0; i < motions1.Length; i++)
            {
                bool class1 = classifier1.isMember(motions1[i]);
                Console.Write("[" + class1 + "]");
            }
            Console.WriteLine();

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

            Console.WriteLine("got");
        }

        void Initialize(String dirName)
        {
            double[][][] motions = getMotions(dirName);
            Initialize(motions);
        }

        void Initialize(double[][][] motions)
        {
            int train = (int) (motions.Length * TRAIN_PROPORTION);
            int test = motions.Length - train;

            double[][][] trainMotions = new double[train][][];
            int[] trainIndexes = HMMClassifier.getRandomIndexes(train, motions.Length);

            Console.Write("Train Indexes: ");
            for (int i = 0; i < train; i++) Console.Write("[" + trainIndexes[i] + "]");
            Console.WriteLine();

            for (int i = 0; i < train; i++) trainMotions[i] = motions[trainIndexes[i]];

            this.hMM = HMMClassifier.createHMM(trainMotions);
            double sum = 0;
            for (int i = 0; i < motions.Length; i++)
            {
                double prob = hMM.Evaluate(motions[i]);
                sum += prob;
            }

            this.threshold = .8 * (sum / motions.Length);
        }

        bool isMember(double[][] motion)
        {
            double likelihood = hMM.Evaluate(motion);
            return (likelihood > threshold);
        }

        static HiddenMarkovModel<MultivariateNormalDistribution> createHMM(double[][][] motions)
        {
            int dimension = motions[0][0].Length; // use 1st observation (any would suffice)
            var density = new MultivariateNormalDistribution(dimension);
            var hMM = new HiddenMarkovModel<MultivariateNormalDistribution>(4, density);

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
                double[][] motion = getMotion(files[i]);
                motions[i] = pcmReduction(motion);
            }
            return motions;
        }

        static double[][] pcmReduction(double[][] motion)
        {
            double[,] mtrx = new double[motion.Length, motion[0].Length];
            for (int i = 0; i < motion.Length; i++)
            {
                for (int j = 0; j < motion[0].Length; j++)
                {
                    mtrx[i, j] = motion[i][j];
                }
            }

            PrincipalComponentAnalysis pca = new PrincipalComponentAnalysis(mtrx);
            pca.Compute();
            double[,] components = pca.Transform(mtrx, 5);
            double[][] reduced = new double[motion.Length][];
            for (int i = 0; i < motion.Length; i++)
            {
                reduced[i] = new double[5];
                for (int j = 0; j < 5; j++)
                {
                    reduced[i][j] = components[i, j];
                }
            }
            return reduced;
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
