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

#include "composition/Composition"

#include "base/Base"

#include "../tests/PuppetTestHelpers.h"
#include "stroker/PuppetStroker.h"
#include "stroker/PuppetStrokeSample.h"

namespace extensions
{
    namespace puppet
    {
        using namespace optimtools;
        USING_OPTIMTOOLS_MESH_TYPES(typename,double,2);
        
        using PuppetTestHelpers = extensions::puppet::PuppetTestHelpers;
        
        class PuppetStroker::Tests
        {
            static std::vector<std::vector<int>> triangles()
            {
                std::vector<std::vector<int>> expectedTriangles =
                {
                    {3, 0, 1},
                    {1, 2, 3}
                };
                return expectedTriangles;
            }
            
            static std::vector<optimtools::mesh_traits<double,2>::Point> initialPoints()
            {
                std::vector<optimtools::mesh_traits<double,2>::Point> expectedInitialPoints =
                {
                    optimtools::mesh_traits<double,2>::Point(0.0, 0.0),
                    optimtools::mesh_traits<double,2>::Point(200.0, 0.0),
                    optimtools::mesh_traits<double,2>::Point(200.0, 200.0),
                    optimtools::mesh_traits<double,2>::Point(0.0, 200.0)
                };
                return expectedInitialPoints;
            }
            
            static std::vector<optimtools::mesh_traits<double,2>::Point> deformedPointsAfterFirstDrag()
            {
                std::vector<optimtools::mesh_traits<double,2>::Point> expectedDeformedPoints =
                {
                    optimtools::mesh_traits<double,2>::Point(50.0, 0.0),
                    optimtools::mesh_traits<double,2>::Point(200.0, 0.0),
                    optimtools::mesh_traits<double,2>::Point(200.0, 200.0),
                    optimtools::mesh_traits<double,2>::Point(0.0, 200.0)
                };
                return expectedDeformedPoints;
            }
            
            static std::vector<optimtools::mesh_traits<double,2>::Point> deformedPointsAfterSecondDrag()
            {
                std::vector<optimtools::mesh_traits<double,2>::Point> expectedDeformedPoints =
                {
                    optimtools::mesh_traits<double,2>::Point(50.0, 0.0),
                    optimtools::mesh_traits<double,2>::Point(200.0, 0.0),
                    optimtools::mesh_traits<double,2>::Point(150.0, 200.0),
                    optimtools::mesh_traits<double,2>::Point(0.0, 200.0)
                };
                return expectedDeformedPoints;
            }

        public:

            void testBeginAddEndSamples()
            {
                const bool PRINT_LOCAL_DEV_DEBUG = false;
                
                // Create a simple puppet
                auto puppet = PuppetTestHelpers::createRectanglePuppet(200, 200, 200);
                CPAssertTrue(puppet != nullptr);
                
                // Create job dispatcher and stroker
                auto jobDispatcher = base::JobDispatcher::createAsyncQueue("TestQueue", base::Thread::QOS::normal);
                PuppetStroker stroker(jobDispatcher);
                
                // Initialize stroker with the puppet
                std::vector<std::shared_ptr<PuppetT>> puppets = { puppet };
                stroker.initialize(puppets);
                
                // Create the two samples for the first stroke
                PuppetStrokeSample startSample;
                startSample.position = geom::Vector2f{1.0f, 5.0f};
                startSample.time = 1.0;

                PuppetStrokeSample dragSample;
                dragSample.position = geom::Vector2f{51.0f, 5.0f};
                dragSample.time = 2.0;
                
                // Initiate the first warp
                CPAssertTrue(stroker.beginSamples(startSample, 20.0f));
                stroker.addSample(dragSample);
                base::ThisThread::sleep(0.01); // Make sure the samples have been processed
                stroker.endSamples();
                
                // Get puppet reference for validation and debug
                const PuppetT& selectedPuppetAfterFirstDrag = stroker.selectedPuppet();
                
                // Debug output for first stroke
                if (PRINT_LOCAL_DEV_DEBUG) {
                    PuppetTestHelpers::printPuppetGeometryValidation(selectedPuppetAfterFirstDrag,
                                                                    initialPoints(),
                                                                    deformedPointsAfterFirstDrag(),
                                                                    triangles());
                }
                
                // Validate puppet geometry after first stroke
                PuppetTestHelpers::validatePuppetGeometry(selectedPuppetAfterFirstDrag,
                                                          initialPoints(),
                                                          deformedPointsAfterFirstDrag(),
                                                          triangles());

                // Create the two samples for the second stroke
                PuppetStrokeSample startSample2;
                startSample2.position = geom::Vector2f{250.0f, 250.0f};
                startSample2.time = 3.0;
                
                PuppetStrokeSample dragSample2;
                dragSample2.position = geom::Vector2f{200.0f, 250.0f};
                dragSample2.time = 4.0;
                
                // Initiate the second warp
                // Note: make sure that the first vertex is not within the threshold range
                bool secondStrokeStarted = stroker.beginSamples(startSample2, 100.0f);
                CPAssertTrue(secondStrokeStarted);
                
                stroker.addSample(dragSample2);
                base::ThisThread::sleep(0.01); // Make sure the samples have been processed
                stroker.endSamples();
                
                
                const PuppetT& selectedPuppetAfterSecondDrag = stroker.selectedPuppet();
                
                // Debug stroke info for second stroke
                if (PRINT_LOCAL_DEV_DEBUG) {
                    PuppetTestHelpers::printStrokeInfo(startSample2.position, dragSample2.position, selectedPuppetAfterFirstDrag);
                    
                    // Debug geometry validation for second stroke
                    PuppetTestHelpers::printPuppetGeometryValidation(selectedPuppetAfterSecondDrag,
                                                                    initialPoints(),
                                                                    deformedPointsAfterSecondDrag(),
                                                                    triangles());
                }
                
                // Validate puppet geometry after second stroke
                PuppetTestHelpers::validatePuppetGeometry(selectedPuppetAfterSecondDrag,
                                                          initialPoints(),
                                                          deformedPointsAfterSecondDrag(),
                                                          triangles());
                
                // Deinitialize the stroker
                stroker.releasePuppets();
                
                // Validate that the puppets are gone
                CPAssertTrue(stroker.puppets().size() == 0);
            }
        };
    }
}

CPSUITE(PuppetStrokerTests, extensions::puppet::PuppetStroker::Tests)
CPTEST(testBeginAddEndSamples)
CPENDSUITE
