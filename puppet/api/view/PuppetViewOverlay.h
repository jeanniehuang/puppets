//
//  ADOBE CONFIDENTIAL
//  __________________
//
//  Copyright 2021 Adobe
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

#ifndef VIEW_PUPPETVIEWOVERLAY_H
#define VIEW_PUPPETVIEWOVERLAY_H

#include "sketchlib/view/DrawingView.h"

namespace view
{

    /** Puppet overlay to draw the pins.
     */
    class PuppetViewOverlay : public DrawingViewOverlay
    {
    public:
        static std::shared_ptr<PuppetViewOverlay> create();
        
        /** Set layer being viewed.
         */
        virtual void setLayer(const model::ConstLayerRef& layer) = 0;
    };

}

#endif // VIEW_PUPPETVIEWOVERLAY_H
