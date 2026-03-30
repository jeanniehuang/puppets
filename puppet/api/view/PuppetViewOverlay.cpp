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


#include "composition/Composition"

#include "sketchlib/model/Layer.h"
#include "sketchlib/model/RasterLayerContents.h"
#include "sketchlib/view/VectorDrawingContext.h"

#include "PuppetEffect.h"
#include "PuppetViewOverlay.h"

namespace view
{

    class ConcretePuppetViewOverlay : public PuppetViewOverlay
    {
        model::ConstLayerRef _layer;
        std::vector<util::Point> _handles;

    public:
        void setLayer(const model::ConstLayerRef& layer) override
        {
            _layer = layer;
            setNeedsUpdate(util::Rect::Infinite());
        }

        /** Return the desired API to use for drawing (i.e API::vector).
            The appropriate DrawingContext will be passed with drawRect() using this method.
        */
        DrawingViewOverlay::API api() const override
        {
            return DrawingViewOverlay::API::vector;
        }
        
        bool updateWantsCorrectedTransform() const override
        {
            return true;
        }

        /** Called (on main thread) to update the contents of the overlay.
         */
        void update(const util::AffineTransform& drawingToView) override
        {
            using namespace extensions::puppet;
            using namespace optimtools;
            USING_OPTIMTOOLS_MESH_TYPES(typename,Scalar,2);
            
            _handles.clear();
            
            // Check if we have a valid layer
            if (!_layer || !_layer->contents()->is<model::RasterLayerContents>())
                return;
            
            // Get the effect from the layer (it may have been attached since setLayer was called)
            auto contents = _layer->contents()->as<model::RasterLayerContents>();
            const composition::DrawingLayerNode& node = contents->node();
            
            auto effect = node.effect();
            if (!(effect && effect->is<PuppetEffect>()))
                return;
            
            std::shared_ptr<const PuppetEffect> puppetEffect = std::static_pointer_cast<const PuppetEffect>(effect);
            const PuppetT& puppet = puppetEffect->selectedPuppet();
            
            if (!puppet.base_mesh().is_init())
                return;
              
            // Get position of all handles in the current view
            const std::vector<PointVector>& handle_positions = puppet.get_handle_points();
            for (const auto& handle : handle_positions) {
                for(int i = 0; i < handle.size(); ++i) {
                    const auto& point = handle[i];
                    const util::Point handle_in_view = drawingToView.transform(util::Point(point.x(), point.y()));
                    _handles.push_back(handle_in_view);
                }
            }
        }


        /** Called (on render thread) to draw the contents of the overlay.
        */
        void drawRect(VectorDrawingContext& context) override
        {
            // red circle outline for active handles
            const util::Color color = util::Color::RGBColor(1, 0, 0);
            const double radius = 8.0;
            const double strokeWidth = 2.0;

            for (const util::Point& p : _handles)
                context.drawCircle(p, radius, color, strokeWidth);
        }
    };

    std::shared_ptr<PuppetViewOverlay> PuppetViewOverlay::create()
    {
        return std::make_shared<ConcretePuppetViewOverlay>();
    }

}
