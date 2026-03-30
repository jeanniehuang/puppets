//
//  ADOBE CONFIDENTIAL
//  __________________
//
//  Copyright 2025 Adobe
//  All Rights Reserved.
//
//  NOTICE:  All information contained herein is, and remains
//  the property of Adobe and its suppliers, if any. The intellectual
//  and technical concepts contained herein are proprietary to Adobe
//  and its suppliers are protected by all applicable intellectual
//  property laws, including trade secret and copyright laws.
//  Dissemination of this information or reproduction of this material
//  is strictly forbidden unless prior written permission is obtained
//  from Adobe.
//

#include "CPTestCase.h"

#include "base/Base"
#include "base/threads/Threads"
#include "geom/Geom"

#include "stroker/PuppetDynamicIterationStrategy.h"
#include "stroker/PuppetStrokeSample.h"

namespace extensions
{
    namespace puppet
    {
        /**
         * Test fixture for PuppetDynamicIterationStrategy testing.
         * 
         * This fixture simulates stroke processing scenarios with configurable sample and process rates
         * to test how the dynamic iteration strategy adapts to different performance conditions.
         * 
         * Usage:
         * 1. Construct with performance parameters:
         *    - sampleRate: Samples per second being queued (simulates user input speed)
         *    - processRate: Samples per second being processed (simulates warping performance)  
         *    - processIterations: Actual iterations reported back (implies the interation rate)
         *    - maxTimeSpacing, minIterations, maxIterations, catchupTime: Strategy configuration
         * 
         * 2. Call queueSamples() to simulate stroke input with the configured sample rate
         * 
         * 3. Call waitUntilCompletion() to let all queued processing jobs finish
         * 
         * 4. Examine results via accessors:
         *    - queuedSamples(): All samples that were queued for processing
         *    - processedSamples(): Samples that actually got processed (may be subset if dropping occurred)
         *    - suggestedIterations(): Iteration counts suggested by the strategy over time
         */
        class Fixture
        {
            PuppetDynamicIterationStrategy _strategy;
            std::shared_ptr<volatile base::JobDispatcher> _jobDispatcher;
            base::JobGroup _jobGroup;
            double _sampleRate;
            double _processRate;
            int64_t _processIterations;
            std::vector<PuppetStrokeSample> _queuedSamples;
            std::vector<PuppetStrokeSample> _processedSamples;
            std::vector<double> _suggestedIterations;
            
        public:
            Fixture(double sampleRate, double processRate, int64_t processIterations,
                    double maxTimeSpacing, int64_t minIterations, double maxIterations, double catchupTime) : 
                _sampleRate(sampleRate),
                _processRate(processRate),
                _processIterations(processIterations)
            {
                _jobDispatcher = base::JobDispatcher::createAsyncQueue("TestQueue", base::Thread::QOS::normal);
                _strategy.setMaxTimeSpacing(maxTimeSpacing);
                _strategy.setMinIterations(minIterations);
                _strategy.setMaxIterations(maxIterations);
                _strategy.setCatchupTime(catchupTime);
            }

            PuppetDynamicIterationStrategy& strategy()
            {
                return _strategy;
            }
            
            void queueSamples(int numSamples)
            {
                _strategy.beginStroke();

                for (int i = 0; i < numSamples; ++i)
                {
                    geom::Vector2f position{float(i), float(i)};
                    PuppetStrokeSample sample{position, base::time::elapsed()};
                    _queuedSamples.push_back(sample);
                    
                    if (auto processableSample = _strategy.sampleReceived(sample))
                    {
                        base::JobGroup::Member member(_jobGroup);
                        _jobDispatcher->push(member, [this, processableSample]()
                        {
                            processSample(*processableSample);
                        });
                    }
                    base::ThisThread::sleep(1.0 / _sampleRate);
                }
            }
            
            void waitUntilCompletion() 
            { 
                _jobGroup.wait(); 
            }
            
            const std::vector<PuppetStrokeSample>& queuedSamples() const
            {
                return _queuedSamples;
            }
            
            const std::vector<PuppetStrokeSample>& processedSamples() const
            {
                return _processedSamples;
            }
            
            const std::vector<double>& suggestedIterations() const
            {
                return _suggestedIterations;
            }
            
        private:
            void processSample(const PuppetStrokeSample& sample)
            {
                _processedSamples.push_back(sample);
                double iterations = _strategy.sampleWillProcess();
                _suggestedIterations.push_back(iterations);
                base::ThisThread::sleep(1.0 / _processRate);
                _strategy.sampleProcessed(_processIterations);
            }
        };
        
        class PuppetDynamicIterationStrategy::Tests
        {
        public:
            void testBadPerformance()
            {
                // Configure for minimum iterations: fast sampling, slow processing
                Fixture fixture(
                    1000,   // Fast sample rate (1000 samples/sec)
                    100,    // Slow process rate (100 samples/sec)
                    5,      // Report low actual iterations (simulates slow processing)
                    0.01,   // maxTimeSpacing
                    10,     // minIterations (target value we expect)
                    100,    // maxIterations  
                    0.05    // catchupTime
                );
                
                fixture.queueSamples(100);
                fixture.waitUntilCompletion();
                
                // Should converge to minimum iterations due to slow processing
                const auto& iterations = fixture.suggestedIterations();
                CPAssertTrue(iterations.back() == 10.0);

                // Should be dropping samples due to slow processing
                const auto& queuedSamples = fixture.queuedSamples();
                const auto& processedSamples = fixture.processedSamples();
                CPAssertTrue(processedSamples.back().position != queuedSamples.back().position);
                
                // Last two samples should have max spacing due to sample dropping
                double spacing = processedSamples.back().time - processedSamples[processedSamples.size() - 2].time;
                const double epsilon = 0.0000001;
                CPAssertTrue(std::abs(spacing - 0.01) < epsilon);
            }
            
            void testGoodPerformance()
            {
                // Configure for maximum iterations: slow sampling, fast processing
                Fixture fixture(
                    25,     // Slow sample rate (25 samples/sec)
                    500,    // Fast process rate (500 samples/sec)
                    100,    // Report high actual iterations (simulates fast processing)
                    0.2,    // maxTimeSpacing
                    5,      // minIterations
                    20,     // maxIterations (target value we expect)
                    0.1     // catchupTime
                );
                
                fixture.queueSamples(10);
                fixture.waitUntilCompletion();
                
                // Should converge to maximum iterations due to fast processing
                const auto& iterations = fixture.suggestedIterations();
                CPAssertTrue(iterations.back() == 20.0);

                // Should not be dropping samples due to fast processing
                const auto& queuedSamples = fixture.queuedSamples();
                const auto& processedSamples = fixture.processedSamples();
                CPAssertTrue(processedSamples.back().position == queuedSamples.back().position);
            }
            
            void testBalancedPerformance()
            {
                // Configure for balanced performance: equal sample and process rates
                Fixture fixture(
                    100,    // Sample rate (100 samples/sec)
                    100,    // Process rate (100 samples/sec - same as sampling)
                    25,     // Report moderate actual iterations
                    0.2,    // maxTimeSpacing
                    10,     // minIterations
                    40,     // maxIterations
                    0.1     // catchupTime
                );
                
                fixture.queueSamples(10);
                fixture.waitUntilCompletion();
                
                // Should settle between min and max iterations due to balanced performance
                const auto& iterations = fixture.suggestedIterations();
                CPAssertTrue(iterations.back() > 10.0 && iterations.back() < 40.0);

                // Should not be dropping samples due to balanced performance
                const auto& queuedSamples = fixture.queuedSamples();
                const auto& processedSamples = fixture.processedSamples();
                CPAssertTrue(processedSamples.back().position == queuedSamples.back().position);
            }
        };
    }
}

CPSUITE(PuppetDynamicIterationStrategyTests, extensions::puppet::PuppetDynamicIterationStrategy::Tests)
CPTEST(testBadPerformance)
CPTEST(testGoodPerformance)
CPTEST(testBalancedPerformance)
CPENDSUITE
