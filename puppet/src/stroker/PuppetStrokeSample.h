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

#ifndef EXTENSIONS_PUPPET_PUPPETSTROKESAMPLE
#define EXTENSIONS_PUPPET_PUPPETSTROKESAMPLE

#include "geom/Geom"
#include "numerics/Numerics"

namespace extensions
{
    namespace puppet
    {

        // Stroke sample consisting of position and timestamp
        struct PuppetStrokeSample
        {
            geom::Vector2f position;
            double time{0.0};

            static PuppetStrokeSample lerp(const PuppetStrokeSample& a,
                                           const PuppetStrokeSample& b,
                                           double t)
            {
                PuppetStrokeSample result;
                result.position = numerics::lerp(a.position, b.position, t);
                result.time = numerics::lerp(a.time, b.time, t);
                return result;
            }

            bool operator==(const PuppetStrokeSample& other) const
            {
                return position == other.position && time == other.time;
            }

            bool operator!=(const PuppetStrokeSample& other) const
            {
                return !(*this == other);
            }
        };

    } // namespace puppet
} // namespace extensions

#endif /* EXTENSIONS_PUPPET_PUPPETSTROKESAMPLE */ 
