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
//  and its suppliers and are protected by all applicable intellectual
//  property laws, including trade secret and copyright laws.
//  Dissemination of this information or reproduction of this material
//  is strictly forbidden unless prior written permission is obtained
//  from Adobe.
//

#ifndef MODEL_PUPPETSETTINGS_H
#define MODEL_PUPPETSETTINGS_H

namespace model
{
    /**
     * Settings for configuring the mesh and warper of a Sculpting Puppet.
     * Currently, these are used only for developer debugging and experimentation, and are not exposed to users.
     */
    struct PuppetSettings
    {
        bool showMesh {false};
        bool showPin {false};
        bool autoPinEnabled {true};
        double autoPinFurthestFraction {0.20}; // Fraction of furthest X% vertices to auto-pin
        bool useAutoZone {true};
        double minTriangles {1000}; // Default minimum triangles
        double maxTriangles {15000}; // Default maximum triangles  
        double maxTrianglesPerPuppet {4000}; // Default max triangles per puppet
        double maxComplexity {30}; // Default max complexity
        double s {0.5};
        double w {1.0};
        
        // Dynamic iteration strategy settings
        double maxTimeSpacing {0.20};
        double minIterations {100};
        double maxIterations {400};
        double catchupTime {0.10};
    };

}

#endif /* MODEL_PUPPETSETTINGS_H */
