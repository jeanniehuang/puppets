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

#include <algorithm>

#include "PuppetDynamicIterationStrategy.h"
#include "PuppetStrokeSample.h"

#include "base/Base"

namespace extensions
{
    namespace puppet
    {
        PuppetDynamicIterationStrategy::PuppetDynamicIterationStrategy() :
            _totalStrokeTime(0),
            _iterationsForCurrentStroke(0),
            _totalSamplesReceived(0),
            _queuedSamples(0),
            _iterationTimeForCurrentStroke(0),
            _sampleProcessStartTime(0.0),
            _previousSamples(std::nullopt)
        {
        }

        void PuppetDynamicIterationStrategy::beginStroke()
        {
            // Reset the tracked sample values
            _previousSamples = std::nullopt;
            _queuedSamples = 0;

            // Reset the tracked iteration values
            _iterationTimeForCurrentStroke = 0.0;
            _iterationsForCurrentStroke = 0;
        }

        std::optional<PuppetStrokeSample> PuppetDynamicIterationStrategy::sampleReceived(const PuppetStrokeSample& sample)
        {
            _totalSamplesReceived++;

            // If this is the first sample, initialize the previous sample values and return it
            if (!_previousSamples.has_value())
            {
                _previousSamples = { /* received = */ sample, /* queued = */ sample };
                _queuedSamples++;
                return sample;
            }

            // Retreive local values of the previous samples and update the received sample
            const PuppetStrokeSample previousReceived = _previousSamples.value().received;
            const PuppetStrokeSample previousQueued = _previousSamples.value().queued;
            _previousSamples.value().received = sample;

            // Add delta T to the running tally of total time
            const double dt = sample.time - previousReceived.time;
            _totalStrokeTime = _totalStrokeTime + dt;

            // If there are no queued samples, return the received sample for immediate processing
            const size_t queuedSamples = _queuedSamples;
            if (queuedSamples == 0)
            {
                _previousSamples.value().queued = sample;
                _queuedSamples++;
                return sample;
            }

            // Retrieve local values of the atomic member variables for thread safety
            const double totalStrokeTime = _totalStrokeTime;
            const int64_t iterationsForCurrentStroke = _iterationsForCurrentStroke;
            const size_t totalSamplesReceived = _totalSamplesReceived;
            const double iterationTimeForCurrentStroke = _iterationTimeForCurrentStroke;

            // If we don't have enough information to make a decision yet, omit the sample unless we've exceeded the max time spacing
            if (totalStrokeTime <= 0.0 || iterationTimeForCurrentStroke <= 0.0 || iterationsForCurrentStroke <= 0 || totalSamplesReceived <= 0)
            {
                // Return the received sample if it's beyond the max time spacing
                if (sample.time - previousQueued.time > _maxTimeSpacing)
                {
                    _previousSamples.value().queued = sample;
                    _queuedSamples++;
                    return sample;
                }

                // Otherwise, omit the sample
                return std::nullopt;
            }

            // Calculate the time spacing required to maintain low latency assuming that we are using the minimum iterations per sample
            const double timeSpacing = calculateTimeSpacing(iterationTimeForCurrentStroke, iterationsForCurrentStroke, queuedSamples);

            // Use the time spacing to determine the next sample to queue
            return nextSampleForTimeSpacing(sample, previousReceived, previousQueued, timeSpacing);
        }

        double PuppetDynamicIterationStrategy::sampleWillProcess()
        {
            // Retrieve local values of the atomic member variables for thread safety
            const double totalStrokeTime = _totalStrokeTime;
            const int64_t iterationsForCurrentStroke = _iterationsForCurrentStroke;
            const size_t totalSamplesReceived = _totalSamplesReceived;
            const size_t queuedSamples = _queuedSamples;
            const double iterationTimeForCurrentStroke = _iterationTimeForCurrentStroke;

            // Record the start time of iteration (in seconds)
            _sampleProcessStartTime = base::time::elapsed();

            // If we don't have enough information to make a decision yet, return the average of the min and max iterations per sample
            if (totalStrokeTime <= 0.0 || iterationTimeForCurrentStroke <= 0.0 || iterationsForCurrentStroke <= 0 || totalSamplesReceived <= 0)
            {
                return (_maxIterations - _minIterations) / 2 + _minIterations;
            }

            // Find the expected number of samples that will be processed in the next catchup period
            const double sampleRate = totalSamplesReceived / totalStrokeTime;
            const double expectedNewSamples = sampleRate * _catchupTime;
            const double expectedSamplesToProcess = queuedSamples + expectedNewSamples;

            // Find the expected number of iterations we can execute in the catchup period
            const double iterationRate = iterationsForCurrentStroke / iterationTimeForCurrentStroke;
            const double expectedIterations = iterationRate * _catchupTime;

            // Spread the expected iterations out evenly across the expected samples
            const double iterationsPerSample = expectedIterations / expectedSamplesToProcess;

            // Clamp to our min / max bounds
            return std::clamp(iterationsPerSample, _minIterations, _maxIterations);
        }

        void PuppetDynamicIterationStrategy::sampleProcessed(int64_t iterations)
        {
            // Calculate time spent iterating and add to cumulative total (in seconds)
            const double currentTime = base::time::elapsed();
            const double iterationTime = currentTime - _sampleProcessStartTime;
            _iterationTimeForCurrentStroke = _iterationTimeForCurrentStroke + iterationTime;

            // Update tracked values for the current stroke
            _iterationsForCurrentStroke += iterations;
            _queuedSamples--;
        }

        double PuppetDynamicIterationStrategy::calculateTimeSpacing(double iterationTimeForCurrentStroke,
                                                                    int64_t iterationsForCurrentStroke,
                                                                    size_t queuedSamples) const
        {
            // Use the tracked iteration rate to predict how many spare iterations we have for processing
            // new samples during the catchup period. Since we want to start dropping samples after
            // dropping iterations, assume that we are using the minimum iterations per sample.
            const double iterationRate = iterationsForCurrentStroke / iterationTimeForCurrentStroke;
            const double expectedIterations = iterationRate * _catchupTime;
            const double expectedIterationsForQueuedSamples = queuedSamples * _minIterations;
            const double expectedIterationsForFutureSamples = expectedIterations - expectedIterationsForQueuedSamples;

            // Use the number of spare iterations to determine how many new samples we can support, which
            // we space out evenly across the catchup period.
            const double supportedNewSamples = expectedIterationsForFutureSamples / _minIterations;
            const double timeSpacing = supportedNewSamples <= 0 ? _maxTimeSpacing : _catchupTime / supportedNewSamples;
            return std::min(timeSpacing, _maxTimeSpacing);
        }

        std::optional<PuppetStrokeSample> PuppetDynamicIterationStrategy::nextSampleForTimeSpacing(const PuppetStrokeSample& sample,
                                                                                                   const PuppetStrokeSample& previousReceived,
                                                                                                   const PuppetStrokeSample& previousQueued,
                                                                                                   double timeSpacing)
        {
            // If the previously received sample was queued (i.e. it was not dropped) and the time between samples meets
            // the time spacing requirement, then we don't need to interpolate. We are already keeping up with the input rate,
            // so continue to queue the samples as they come in. 
            const bool queuedPreviousSample = previousQueued == previousReceived;
            const double timeSinceLastQueuedSample = sample.time - previousQueued.time;
            if (queuedPreviousSample && timeSpacing <= timeSinceLastQueuedSample)
            {
                _previousSamples.value().queued = sample;
                _queuedSamples++;
                return sample;
            }

            // If the time of the next sample is beyond the current sample, then we drop the current sample
            const double nextSampleTime = previousQueued.time + timeSpacing;
            if (nextSampleTime > sample.time)
            {
                return std::nullopt;
            }

            // If the time of next sample is before the previously received sample, then return the previous received sample
            if (nextSampleTime < previousReceived.time)
            {
                _previousSamples.value().queued = previousReceived;
                _queuedSamples++;
                return previousReceived;
            }

            // Otherwise, interpolate between the previous received sample and the current sample
            const double timeFromPreviousToNext = nextSampleTime - previousReceived.time;
            const double timeFromPreviousToCurrent = sample.time - previousReceived.time;
            const double lerpT = std::clamp(timeFromPreviousToNext / timeFromPreviousToCurrent, 0.0, 1.0);
            const PuppetStrokeSample next = PuppetStrokeSample::lerp(previousReceived, sample, lerpT);
            _previousSamples.value().queued = next;
            _queuedSamples++;
            return next;
        }
    } // namespace puppet
} // namespace extensions 
