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

#ifndef EXTENSIONS_PUPPET_PUPPETVECTORSKINNER
#define EXTENSIONS_PUPPET_PUPPETVECTORSKINNER

#include <memory>

#include "graphics/Group.h"

#include "PuppetTypes.h"

namespace extensions
{
    namespace puppet
    {
        /** Main entry point to the vector sculpting functionality. This class is responsible for taking in a series of sketchlib paths,
            converting them to BezierCurve instances, running them thru a SkinnerLSQ instance to deform them, and then converting
            them back to sketchlib paths.
         */
        class PuppetVectorSkinner
        {
        public:
            using PuppetMeshRef = std::shared_ptr<optimtools::PuppetMesh<Scalar>>;
            
            static std::shared_ptr<PuppetVectorSkinner> create();
            
            virtual ~PuppetVectorSkinner() = default;
            
            /** Initialize the instance with source vectors and a collection of puppet meshes. The source vectors are
                continuously updated by calls to applyPuppets.
                This method binds the source vectors to their corresponding triangles. A progress object is provided
                in order to support cancellation. This method will throw a Cancel exeception if the given progress object
                indicates it wants to cancel.
             */
            virtual void initialize(graphics::GroupRef rootGroup,
                                    const std::vector<PuppetMeshRef>& puppetMeshes,
                                    const base::Progress& progress) = 0;
            
            /** Deforms the source vectors using the given puppet meshes. Note that all output
                vectors are cubic. Any linear curves in the source are upgraded to cubic curves after deformation.
             */
            virtual void applyPuppets(const std::vector<PuppetMeshRef>& puppetMeshes) = 0;
            
            /** Retrieve the output vectors that were deformed by the skinner.
             */
            virtual graphics::GroupRef outputVectors() const = 0;
        };
    }
}

#endif // EXTENSIONS_PUPPET_PUPPETVECTORSKINNER
