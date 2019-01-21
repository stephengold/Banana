/*
 Copyright (c) 2019, Stephen Gold
 All rights reserved.

 Redistribution and use in source and binary forms, with or without
 modification, are permitted provided that the following conditions are met:
 * Redistributions of source code must retain the above copyright
 notice, this list of conditions and the following disclaimer.
 * Redistributions in binary form must reproduce the above copyright
 notice, this list of conditions and the following disclaimer in the
 documentation and/or other materials provided with the distribution.
 * Neither the name of the copyright holder nor the names of its contributors
 may be used to endorse or promote products derived from this software without
 specific prior written permission.

 THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */
package banana;

import com.jme3.math.FastMath;
import java.io.PrintStream;
import java.util.logging.Level;
import java.util.logging.Logger;

/**
 * Sampled performance data from a single measurement phase.
 *
 * @author Stephen Gold sgold@sonic.net
 */
class PerfData {
    // *************************************************************************
    // constants and loggers

    /**
     * message logger for this class
     */
    final public static Logger logger
            = Logger.getLogger(PerfData.class.getName());
    // *************************************************************************
    // fields

    /**
     * sum of all sample values, for computing the mean
     */
    private double sum;
    /**
     * width of each histogram bin (&gt;0)
     */
    final private float binWidth;
    /**
     * boundary value between the 1st and 2nd histogram bins
     */
    final private float firstBreak;
    /**
     * highest sample value
     */
    private float max;
    /**
     * lowest sample value
     */
    private float min;
    /**
     * histogram data
     */
    final private int[] counts;
    // *************************************************************************
    // constructors

    /**
     * Instantiate data with the specified breakpoints.
     *
     * @param firstBreak the boundary value between the 1st and 2nd bins
     * @param binWidth the width of each bin (&gt;0)
     * @param numBins the number of bins (&gt;0)
     */
    PerfData(float firstBreak, float binWidth, int numBins) {
        assert binWidth > 0f : binWidth;
        assert numBins > 0 : numBins;

        counts = new int[numBins];
        this.firstBreak = firstBreak;
        this.binWidth = binWidth;
        clear();
    }
    // *************************************************************************
    // new methods exposed

    /**
     * Add a sample.
     *
     * @param sampleValue the sample value
     */
    void add(float sampleValue) {
        logger.log(Level.FINER, "sampleValue={0}", sampleValue);

        sum += sampleValue;
        if (sampleValue > max) {
            max = sampleValue;
            logger.log(Level.FINE, "new max={0}", max);
        }
        if (sampleValue < min) {
            min = sampleValue;
        }
        int binIndex = findBinIndex(sampleValue);
        ++counts[binIndex];
    }

    /**
     * Clear all the data.
     */
    void clear() {
        sum = 0.0;
        max = Float.MIN_VALUE;
        min = Float.MAX_VALUE;
        for (int binIndex = 0; binIndex < numBins(); binIndex++) {
            counts[binIndex] = 0;
        }
    }

    /**
     * Determine the total number of samples.
     *
     * @return count (&ge;0)
     */
    int count() {
        int numSamples = 0;
        for (int count : counts) {
            numSamples += count;
        }

        assert numSamples >= 0 : numSamples;
        return numSamples;
    }

    /**
     * Read the number of samples in the specified histogram bin.
     *
     * @param binIndex (&ge;0, &lt;numBins)
     * @return count (&ge;0)
     */
    int count(int binIndex) {
        validateBinIndex(binIndex);

        int numSamples = counts[binIndex];

        assert numSamples >= 0 : numSamples;
        return numSamples;
    }

    /**
     * Read the number of samples in the last histogram bin.
     *
     * @return count (&ge;0)
     */
    int countLastBin() {
        int lastBin = lastBinIndex();
        int numSamples = counts[lastBin];

        assert numSamples >= 0 : numSamples;
        return numSamples;
    }

    /**
     * Dump statistics to a PrintStream.
     *
     * @param out the output stream (not null)
     */
    void dump(PrintStream out) {
        int numSamples = count();
        if (numSamples < 1) {
            out.println(" No data.");
            return;
        }

        out.printf(" %d total sample%s:%n", numSamples,
                numSamples == 1 ? "" : "s");
        for (int binIndex = 0; binIndex < numBins(); binIndex++) {
            int count = count(binIndex);
            if (count > 0) {
                float percent = percent(binIndex);
                out.printf("%7d (%5.1f%%) ", count, percent);
                float lower = lowerLimit(binIndex);
                float upper = upperLimit(binIndex);
                if (lower == Float.NEGATIVE_INFINITY) {
                    out.printf("under %.3f", upper);
                } else if (upper != Float.POSITIVE_INFINITY) {
                    out.printf("between %.3f and %.3f", lower, upper);
                } else {
                    out.printf("over %.3f", lower);
                }
                out.println();
            }
        }
        float mean = mean();
        float percentLastBin = percentLastBin();
        out.printf("  min=%.3f  mean=%.4f  pctLast=%.1f  max=%.3f%n", min,
                mean, percentLastBin, max);
        out.flush();
    }

    /**
     * Determine which histogram bin for a particular sample.
     *
     * @param value the sample value
     * @return the bin index (&ge;0, &lt;numBins)
     */
    int findBinIndex(float value) {
        if (value < firstBreak) {
            return 0;
        }

        float ratio = (value - firstBreak) / binWidth;
        int binIndex = 1 + (int) FastMath.floor(ratio);
        assert binIndex >= 1 : binIndex;
        int last = lastBinIndex();
        if (binIndex > last) {
            binIndex = last;
        }

        validateBinIndex(binIndex);
        return binIndex;
    }

    /**
     * Determine the index of the last bin.
     *
     * @return index (&ge;0)
     */
    int lastBinIndex() {
        int lastBinIndex = numBins() - 1;
        assert lastBinIndex > 0 : lastBinIndex;
        return lastBinIndex;
    }

    /**
     * Determine the lower limit of the specified histogram bin.
     *
     * @param the bin index (&ge;0, &lt;numBins)
     * @return the limit value (may be infinite)
     */
    float lowerLimit(int binIndex) {
        validateBinIndex(binIndex);

        float result;
        if (binIndex == 0) {
            result = Float.NEGATIVE_INFINITY;
        } else {
            result = firstBreak + binWidth * (binIndex - 1);
        }

        return result;
    }

    /**
     * Determine the arithmetic mean of all samples.
     *
     * @return the mean value
     */
    float mean() {
        int numSamples = count();
        assert numSamples > 0 : numSamples;
        float mean = (float) (sum / numSamples);
        return mean;
    }

    /**
     * Read the number of bins.
     *
     * @return count (&gt;0)
     */
    int numBins() {
        int numBins = counts.length;
        assert numBins > 0 : numBins;
        return numBins;
    }

    /**
     * Calculate the percentage of samples in the specified histogram bin.
     *
     * @param binIndex (&ge;0, &lt;numBins)
     * @return percentage (&ge;0, &le;100)
     */
    float percent(int binIndex) {
        int numSamples = count();
        assert numSamples > 0 : numSamples;

        int count = count(binIndex);
        float percent = (100f * count) / numSamples;

        return percent;
    }

    /**
     * Calculate the percentage of samples in the last histogram bin.
     *
     * @return percentage (&ge;0, &le;100)
     */
    float percentLastBin() {
        int numSamples = count();
        assert numSamples > 0 : numSamples;

        int count = countLastBin();
        float percent = (100f * count) / numSamples;

        return percent;
    }

    /**
     * Determine the upper limit of the specified histogram bin.
     *
     * @param the bin index (&ge;0, &lt;numBins)
     * @return the limit value (may be infinite)
     */
    float upperLimit(int binIndex) {
        validateBinIndex(binIndex);

        float result;
        if (binIndex == lastBinIndex()) {
            result = Float.POSITIVE_INFINITY;
        } else {
            result = firstBreak + binWidth * binIndex;
        }

        return result;
    }

    /**
     * Ensure that a bin index is valid.
     *
     * @param the bin index (&ge;0, &lt;numBins)
     */
    void validateBinIndex(int binIndex) {
        assert binIndex >= 0 : binIndex;
        assert binIndex < counts.length : binIndex;
    }
}
