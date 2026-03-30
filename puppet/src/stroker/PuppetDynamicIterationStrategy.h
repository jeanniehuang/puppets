/*************************************************************************
 *
 * ADOBE CONFIDENTIAL
 * ___________________
 *
 *  Copyright 2025 Adobe Systems Incorporated
 *  All Rights Reserved.
 *
 * NOTICE:  All information contained herein is, and remains
 * the property of Adobe Systems Incorporated and its suppliers,
 * if any.  The intellectual and technical concepts contained
 * herein are proprietary to Adobe Systems Incorporated and its
 * suppliers and are protected by trade secret or copyright law.
 * Dissemination of this information or reproduction of this material
 * is strictly forbidden unless prior written permission is obtained
 * from Adobe Systems Incorporated.
 **************************************************************************/

#ifndef EXTENSIONS_PUPPET_PUPPETDYNAMICITERATIONSTRATEGY
#define EXTENSIONS_PUPPET_PUPPETDYNAMICITERATIONSTRATEGY

#include <atomic>
#include <optional>

#include "PuppetStrokeSample.h"

namespace extensions
{
    namespace puppet
    {

        /**
         * Helper class for tracking the puppet stroke latency and dynamically adjusting the following
         * properties to keep up:
         *  - the number of iterations to find an acceptable solution
         *  - the time spacing between samples
         *
         * In simple terms, if the stroke is lagging, this class reduces the number of iterations performed
         * per sample, then starts dropping samples.
         *
         * This class assumes that samples will be received on one thread but processed on another.
         * It is thread-safe so long as this threading model is followed.
         *
         * This class tracks sample rate across its entire lifetime. If there is reason to believe that the
         * sample rate will change significantly, it is best to recreate the strategy. Since it is assumed
         * the iteration rate will change with each stroke, the tracked data is reset in beginStroke().
         *
         * Nomenclature:
         *  - Received: a sample is received by the client on the main thread and passed to this strategy.
         *    Not all received samples are returned to the client to be processed, since some may be
         *    dropped or altered to maintain an acceptable latency.
         *  - Processed: a sample is processed by the client on the background thread. This strategy must
         *    be notified before and after processing.
         *  - Queued: a sample that has been returned from this class to the client for handling, but has
         *    yet to be processed.
         */
        class PuppetDynamicIterationStrategy
        {
        public:
            class Tests;
            
            PuppetDynamicIterationStrategy();

            /**
             * Signify to the strategy that a new stroke has begun.
             * This resets the tracked values for iteration rate but not sample rate, since the latter
             * is assumed to be fairly constant across strokes. It also resets properties that are stroke-specific,
             * such as the previous stroke samples.
             *
             * Should be called from the main thread
             */
            void beginStroke();

            /**
             * Indicate that to the strategy that a stroke sample has been received. Note that since this
             * method produces at most one queued sample per sample received, we will never produce
             * more samples for processing than the number of sample received.
             *
             * Should be called from the main thread
             *
             * @param sample The input sample received
             * @return The sample that should be processed by the client. It is possible that the input sample
             *         was modified or dropped completely (nullopt) by the strategy to reduce latency.
             */
            std::optional<PuppetStrokeSample> sampleReceived(const PuppetStrokeSample& sample);

            /**
             * Inform the strategy that a sample is about to be processed.
             *
             * Should be called from the background thread
             *
             * @return The maximum number of iterations to set on the warper to achieve the desired latency
             */
            double sampleWillProcess();

            /**
             * Inform the strategy that a sample has been processed in the provided number of iterations. The number
             * of iterations is used to compute a running average of iterations per time (iteration rate).
             *
             * Should be called from the background thread
             *
             * @param iterations Number of iterations used to process the sample
             */
            void sampleProcessed(int64_t iterations);

            /**
             * 'Set' methods for parameters tuning, see the member variables for explanation of each parameter
             */
            void setMaxTimeSpacing(double maxTimeSpacing) { _maxTimeSpacing = maxTimeSpacing; }
            void setMinIterations(int64_t minIterations) { _minIterations = minIterations; }
            void setMaxIterations(double maxIterations) { _maxIterations = maxIterations; }
            void setCatchupTime(double catchupTime) { _catchupTime = catchupTime; }
            
        private:
            /**
             * Calculate the optimal time spacing based on current performance metrics, under
             * the assumption that we are using the minimum iterations per sample. This assumption ensures that
             * we prioritize dropping the number of iterations per sample before dropping samples themselves.
             *
             * @param iterationTimeForCurrentStroke Total time spent iterating in current stroke
             * @param iterationsForCurrentStroke Total iterations performed in current stroke
             * @param queuedSamples Number of currently queued samples
             * @return The calculated time spacing value (in seconds)
             */
            double calculateTimeSpacing(double iterationTimeForCurrentStroke,
                                        int64_t iterationsForCurrentStroke,
                                        size_t queuedSamples) const;

            /**
             * Calculate the next sample based on time spacing and update internal state
             * This method first checks if the current sample can be returned directly based on timing,
             * then linearly interpolates the sample and the previously received sample, if needed. It
             * handles incrementing _queuedSamples and updating _previousSamples internally.

             * @param sample The current input sample
             * @param previousReceived The previously received sample
             * @param previousQueued The previously queued sample
             * @param timeSpacing The time spacing (in seconds) that should be maintained
             * @return Optional next sample to queue, or nullopt if no sample should be queued
             */
            std::optional<PuppetStrokeSample> nextSampleForTimeSpacing(const PuppetStrokeSample& sample,
                                                                       const PuppetStrokeSample& previousReceived,
                                                                       const PuppetStrokeSample& previousQueued,
                                                                       double timeSpacing);

            /**
             * A convenience struct for storing the last received and queued samples according
             * to the 'sampleReceived' method
             */
            struct PreviousSamples
            {
                // The most recent sample received as input to the strategy
                PuppetStrokeSample received;
                // The most recent sample produced as output by this strategy
                PuppetStrokeSample queued;
            };
            std::optional<PreviousSamples> _previousSamples;

            // Sum of all time deltas between samples received by this class over its lifetime
            std::atomic<double> _totalStrokeTime;
            // Sum of all samples received by this class over its lifetime
            std::atomic<size_t> _totalSamplesReceived;

            // Current number of samples that have been returned from 'samplesReceived' but not
            // yet processed. These samples are considered 'queued'.
            std::atomic<size_t> _queuedSamples;

            // Sum of all iterations performed by the client for the current stroke
            std::atomic<int64_t> _iterationsForCurrentStroke;
            // Sum of all time spent actually iterating for the current stroke, aka the time
            // between sampleWillProcess and sampleProcessed calls
            std::atomic<double> _iterationTimeForCurrentStroke;
            // Start time of current iteration (non-atomic since only used on background thread)
            double _sampleProcessStartTime;

            // Tunable parameters

            // The maximum time spacing (in seconds) between samples that must be maintained when considering whether we
            // can drop or alter samples to maintain latency. It is technically possible to violate
            // this constraint if the input samples are very far apart in the time domain
            double _maxTimeSpacing{0.20};
            // The floor for the maximum number of iterations to perform for a given sample
            double _minIterations{100};
            // The ceiling on the max number of iterations to perform for a given sample
            double _maxIterations{400};
            // The amount of time (in seconds) alloted for this class to catch up to real time processing
            double _catchupTime{0.10};
        };

    } // namespace puppet
} // namespace extensions

#endif /* EXTENSIONS_PUPPET_PUPPETDYNAMICITERATIONSTRATEGY */ 
