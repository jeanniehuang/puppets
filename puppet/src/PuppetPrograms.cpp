/*************************************************************************
 *
 * ADOBE CONFIDENTIAL
 * ___________________
 *
 *  Copyright 2021 Adobe Systems Incorporated
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

#include "gpu/src/GPU"

#include "renderer/gpu/ScratchBuffers.h"

#include "PuppetPrograms.h"


namespace renderer
{

    //--------------------------------------------------------------------------------
    // PuppetPreviewProgram
    //--------------------------------------------------------------------------------
        
    std::unique_ptr<PuppetPreviewProgram> PuppetPreviewProgram::construct(GPUTarget& target)
    {
        auto program = std::make_unique<PuppetPreviewProgram>();
        if (!program->loadProgram(target))
            ASK_THROW_NO_PII(std::bad_alloc());
        return program;
    }

    void PuppetPreviewProgram::destroyResource(GPUTarget& target)
    {
        super::destroy(target.renderContext());
    }

    bool PuppetPreviewProgram::loadProgram(GPUTarget& target)
    {
        bool result = false;

        switch(target.renderContext().engine())
        {
            case gpu::Engine::DX11:
            case gpu::Engine::MANTADX12:
            {
            }
            break;
            case gpu::Engine::MANTAMETAL:
            {
                gpu::ProgramSources sources;
                sources.setLibrary(target.getProgramLibrary("sketch-extension.bundle/default.metallib"));
                result = super::loadSource(target.renderContext(), sources, *this, "puppet_preview_vs", "", "puppet_preview_fs");
            }
            break;
            default:
            {
            }
            break;
        }

        return result;
    }

    bool PuppetPreviewProgram::preprocess(std::string& vtxshader, std::string& frgshader)
    {
        return true;
    }
    
    void PuppetPreviewProgram::bindUAV()
    {
    }

    void PuppetPreviewProgram::bindAttributes()
    {
        static const gpu::ProgramBridge::AttributeDesc layout[] =
        {
            { "vertexPosDst",  gpu::ProgramBridge::kFloat2, ATTRIB_XY, XY_INDEX * sizeof(float) },
            { "vertexPosSrc",    gpu::ProgramBridge::kFloat2, ATTRIB_UV, UV_INDEX * sizeof(float) }
        };
        int numEntries = sizeof(layout) / sizeof(gpu::ProgramBridge::AttributeDesc);
        
        impl().declareAttributes(layout, numEntries);
    }
    
    void PuppetPreviewProgram::getUniforms()
    {
        static const gpu::ProgramBridge::UniformDesc vsUniforms [] = {
            { "vertexToDst", gpu::ProgramBridge::kMat4x4 },
            { "vertexToSrc", gpu::ProgramBridge::kMat4x4 }
        };
        static const gpu::ProgramBridge::UniformDesc fsUniforms [] = {
            { "dstUnmultiplied", gpu::ProgramBridge::kInt1 },
            { "invertSrc", gpu::ProgramBridge::kInt1 },
            { "showMesh", gpu::ProgramBridge::kInt1 }
        };

        impl().declareVSUniforms(vsUniforms, sizeof(vsUniforms) / sizeof(gpu::ProgramBridge::UniformDesc));
        impl().declareFSUniforms(fsUniforms, sizeof(fsUniforms) / sizeof(gpu::ProgramBridge::UniformDesc));
        
        impl().declareTexture("texSourceImage", 0);
    }

    bool PuppetPreviewProgram::use()
    {
        super::use();

        return true;
    }

    void PuppetPreviewProgram::setUp(const geom::Affinity2f& vertexToDst,
                                     const geom::Affinity2f& vertexToSrc,
                                     const gpu::Sampler& texSourceImage,
                                     AlphaFormat dstAlphaFmt,
                                     bool invertSrc,
                                     bool showMesh)
    {
        const float flipy = context().ndcOrientation();
        impl().setUniformMat4f("vertexToDst",
                               flipy > 0 ? vertexToDst : geom::Affinity2f::scale(1, -1) * vertexToDst);
        impl().setUniformMat4f("vertexToSrc", vertexToSrc);
        impl().setUniform1i("dstUnmultiplied", dstAlphaFmt == AlphaFormat::unmul ? 1 : 0);
        impl().setUniform1i("invertSrc", invertSrc ? 1 : 0);
        impl().setUniform1i("showMesh", showMesh ? 1 : 0);
        impl().setTexture("texSourceImage", texSourceImage);
    }

    void PuppetPreviewProgram::draw(const gpu::VertexBuffer& vertices,
                                    const gpu::IndexBuffer* indices)
    {
        constexpr size_t stride = FLOATS_PER_VERTEX * sizeof(float);
        impl().setAttributes(vertices, stride);
        
        if (indices)
        {
            // Use the number of indices from the index buffer
            uint32_t numIndices = indices->numIndices();
            context().drawIndexed(gpu::RenderContext::kTriangles, *indices, numIndices);
        }
        else
        {
            // Calculate the number of vertices from buffer size
            uint32_t numVertices = vertices.filledBytes() / stride;
            context().draw(gpu::RenderContext::kTriangles, numVertices);
        }
    }

    //--------------------------------------------------------------------------------
    // PuppetPreviewProgramDrawer
    //--------------------------------------------------------------------------------

    void PuppetPreviewProgramDrawer::draw(renderer::GPUTarget& target,
                                          const geom::Affinity2f& vertexToDst,
                                          const geom::Affinity2f& vertexToSrc,
                                          const gpu::Sampler& sourceSampler,
                                          const gpu::VertexBuffer& vertices,
                                          const gpu::IndexBuffer* indices,
                                          bool treatAlphaAsInverted,
                                          bool clear,
                                          bool showMesh) const
    {
        if (treatAlphaAsInverted)
        {
            drawAsInvertedAlpha(target, vertexToDst, vertexToSrc, sourceSampler, vertices, indices, clear, showMesh);
        }
        else
        {
            drawNormal(target, vertexToDst, vertexToSrc, sourceSampler, vertices, indices, clear, showMesh);
        }
    }

    void PuppetPreviewProgramDrawer::drawPreview(renderer::GPUTarget& target,
                                                 const geom::Affinity2f& vertexToDst,
                                                 const geom::Affinity2f& vertexToSrc,
                                                 const gpu::Sampler& sourceSampler,
                                                 const gpu::VertexBuffer& vertices,
                                                 const gpu::IndexBuffer* indices,
                                                 bool treatAlphaAsInverted,
                                                 bool showMesh) const
    {
        // draw preview
        auto& previewProgram = target.get<renderer::PuppetPreviewProgram>();
        target.useProgram(previewProgram);
        
        // Set up the program with the provided matrices, treatAlphaAsInverted flag, and showMesh flag
        // We assume that hardware blending is in effect so always output to premultiplied alpha
        previewProgram.setUp(vertexToDst, vertexToSrc, sourceSampler, renderer::AlphaFormat::premul, treatAlphaAsInverted, showMesh);

        previewProgram.draw(vertices, indices);
    }

    void PuppetPreviewProgramDrawer::drawAsInvertedAlpha(renderer::GPUTarget& target,
                                                         const geom::Affinity2f& vertexToDst,
                                                         const geom::Affinity2f& vertexToSrc,
                                                         const gpu::Sampler& sourceSampler,
                                                         const gpu::VertexBuffer& vertices,
                                                         const gpu::IndexBuffer* indices,
                                                         bool clear,
                                                         bool showMesh) const
    {
        // Create a scratch buffer
        gpu::FloatRect dstbounds = target.renderContext().colorBufferBounds();
        renderer::ScratchBuffer tmp(target, gpu::kRGBA, gpu::kUnsignedByte, dstbounds.w, dstbounds.h);

        {
            // Save current render state and bind scratch buffer
            gpu::AutoSaveState state(target.renderContext());
            state.saveColorBuffer();
            tmp.bind(target, /*clear = */ true);

            // Draw the preview to the scratch buffer while sampling the source as 'inverted'
            drawPreview(target, vertexToDst, vertexToSrc, sourceSampler, vertices, indices, /* treatAlphaAsInverted = */ true, showMesh);
        }

        // Clear the destination to white (opaque background for inversion)
        if (clear)
        {
            target.renderContext().clearColorBuffer(1.0f, 1.0f, 1.0f, 1.0f);
        }

        // Composite the scratch buffer back using erase blend mode. Note that since our scratch buffer contains
        // the results of the inverted preview, we 'undo' this inversion by erasing from a purely white background.
        const geom::Box2f srcuv(0, 1, 0, 1);
        const auto sampler = target.sampler_nn(tmp.get());
        const renderer::GPUTextureSourceBinding src(sampler, srcuv);
        const geom::Projectivity2f modelTransform = geom::Projectivity2f(vertexToDst).inverse();
        
        renderer::GPUBlendCompositor compositor(1.0f, renderer::BlendMethod::kErase);
        compositor.draw(target, src, geom::Projectivity2f::identity(), modelTransform, geom::Box2f(-1,1,-1,1),
                       false, renderer::AlphaFormat::premul, renderer::AlphaFormat::premul);
    }

    void PuppetPreviewProgramDrawer::drawNormal(renderer::GPUTarget& target,
                                                const geom::Affinity2f& vertexToDst,
                                                const geom::Affinity2f& vertexToSrc,
                                                const gpu::Sampler& sourceSampler,
                                                const gpu::VertexBuffer& vertices,
                                                const gpu::IndexBuffer* indices,
                                                bool clear,
                                                bool showMesh) const
    {
        // Clear destination to transparent background if requested
        if (clear)
        {
            target.renderContext().clearColorBuffer(0.0f, 0.0f, 0.0f, 0.0f);
        }

        drawPreview(target, vertexToDst, vertexToSrc, sourceSampler, vertices, indices, /* treatAlphaAsInverted = */ false, showMesh);
    }

    //--------------------------------------------------------------------------------
    // PuppetDilateProgram
    //--------------------------------------------------------------------------------
        
    std::unique_ptr<PuppetDilateProgram> PuppetDilateProgram::construct(GPUTarget& target)
    {
        auto program = std::make_unique<PuppetDilateProgram>();
        if (!program->loadProgram(target))
            ASK_THROW_NO_PII(std::bad_alloc());
        return program;
    }

    void PuppetDilateProgram::destroyResource(GPUTarget& target)
    {
        super::destroy(target.renderContext());
    }

    bool PuppetDilateProgram::loadProgram(GPUTarget& target)
    {
        bool result = false;

        switch(target.renderContext().engine())
        {
            case gpu::Engine::DX11:
            case gpu::Engine::MANTADX12:
            {
                // TODO: Add DX implementation if needed
            }
            break;
            case gpu::Engine::MANTAMETAL:
            {
                gpu::ProgramSources sources;
                sources.setLibrary(target.getProgramLibrary("sketch-extension.bundle/default.metallib"));
                result = super::loadSource(target.renderContext(), sources, *this, "puppet_dilate_vs", "", "puppet_dilate_fs");
            }
            break;
            default:
            {
            }
            break;
        }

        return result;
    }

    bool PuppetDilateProgram::preprocess(std::string& vtxshader, std::string& frgshader)
    {
        return true;
    }
    
    void PuppetDilateProgram::bindUAV()
    {
    }

    void PuppetDilateProgram::bindAttributes()
    {
        static const gpu::ProgramBridge::AttributeDesc layout[] =
        {
            { "position",  gpu::ProgramBridge::kFloat2, ATTRIB_XY, XY_INDEX * sizeof(float) }
        };
        int numEntries = sizeof(layout) / sizeof(gpu::ProgramBridge::AttributeDesc);
        
        impl().declareAttributes(layout, numEntries);
    }
    
    void PuppetDilateProgram::getUniforms()
    {
        static const gpu::ProgramBridge::UniformDesc vsUniforms [] =
        {
            { "imageToDestNDC", gpu::ProgramBridge::kMat4x4 },
            { "imageToSrcUV", gpu::ProgramBridge::kMat4x4 }
        };
        static const gpu::ProgramBridge::UniformDesc fsUniforms [] =
        {
            { "dilationRadius", gpu::ProgramBridge::kInt1 },
            { "isHorizontalPass", gpu::ProgramBridge::kInt1 },
            { "sourceWidth", gpu::ProgramBridge::kInt1 },
            { "sourceHeight", gpu::ProgramBridge::kInt1 },
            { "singleChannelSrc", gpu::ProgramBridge::kInt1 },
            { "invertSrc", gpu::ProgramBridge::kInt1 }
        };

        impl().declareVSUniforms(vsUniforms, sizeof(vsUniforms) / sizeof(gpu::ProgramBridge::UniformDesc));
        impl().declareFSUniforms(fsUniforms, sizeof(fsUniforms) / sizeof(gpu::ProgramBridge::UniformDesc));
        
        impl().declareTexture("texSourceImage", 0);
    }

    bool PuppetDilateProgram::use()
    {
        super::use();
        return true;
    }

    void PuppetDilateProgram::setUp(GPUTarget& target,
                                    const gpu::Sampler& texSourceImage,
                                    int dilationRadius,
                                    bool isHorizontalPass,
                                    int sourceWidth,
                                    int sourceHeight,
                                    bool singleChannelSrc,
                                    bool invertSrc,
                                    const geom::Affinity2f& imageToDestNDC,
                                    const geom::Affinity2f& imageToSrcUV)
    {
        // Set vertex shader uniforms - transformation matrices
        impl().setUniformMat4f("imageToDestNDC", imageToDestNDC);
        impl().setUniformMat4f("imageToSrcUV", imageToSrcUV);
        
        // set fragment shader uniforms
        impl().setUniform1i("dilationRadius", dilationRadius);
        impl().setUniform1i("isHorizontalPass", isHorizontalPass ? 1 : 0);
        impl().setUniform1i("sourceWidth", sourceWidth);
        impl().setUniform1i("sourceHeight", sourceHeight);
        impl().setUniform1i("singleChannelSrc", singleChannelSrc ? 1 : 0);
        impl().setUniform1i("invertSrc", invertSrc ? 1 : 0);
        
        // bind texture
        impl().setTexture("texSourceImage", texSourceImage);
    }

    void PuppetDilateProgram::draw(const gpu::VertexBuffer& vertices,
                                   const gpu::IndexBuffer& indices,
                                   uint32_t nIndices)
    {
        constexpr size_t stride = FLOATS_PER_VERTEX * sizeof(float);
        impl().setAttributes(vertices, stride);
        context().drawIndexed(gpu::RenderContext::kTriangles, indices, nIndices);
    }

    //--------------------------------------------------------------------------------
    // PuppetDilateProgramDrawer
    //--------------------------------------------------------------------------------

    void PuppetDilateProgramDrawer::draw(renderer::GPUTarget& target,
                                         int dilationRadius,
                                         const gpu::Texture& src,
                                         gpu::Texture& dst,
                                         bool invertSrc) const
    {
        // Derive parameters from texture properties
        const bool singleChannelSrc = (src.format() == gpu::kRed);
        const bool singleChannelDst = (dst.format() == gpu::kRed);
        
        // Save the GPU state and disable blending to overwrite pixels
        gpu::AutoSaveState state(target.renderContext());
        state.saveColorBuffer();
        target.renderContext().disableBlending();
        
        // Pass 1: Horizontal dilation
        auto horizontalResult = drawHorizontal(target, dilationRadius, src, singleChannelSrc, singleChannelDst, invertSrc);
        
        // Pass 2: Vertical dilation  
        drawVertical(target, dilationRadius, src, *horizontalResult, dst, singleChannelDst);
    }

    std::unique_ptr<renderer::ScratchBuffer> PuppetDilateProgramDrawer::drawHorizontal(renderer::GPUTarget& target,
                                                                                       int dilationRadius,
                                                                                       const gpu::Texture& src,
                                                                                       bool singleChannelSrc,
                                                                                       bool singleChannelDst,
                                                                                       bool invertSrc) const
    {
        // Create sampler for source texture
        const gpu::Sampler sourceSampler = target.sampler_ll(src);
        
        // Derive parameters from texture properties
        const int sourceWidth = src.width();
        const int sourceHeight = src.height();
        
        // Float versions for vertex data to avoid casting
        const float dilationRadiusF = dilationRadius;
        const float sourceWidthF = sourceWidth;
        const float sourceHeightF = sourceHeight;
        
        // Create vertex data for horizontal pass (only width expanded)
        std::vector<float> vertexData =
        {
            // x,  y  
            -dilationRadiusF,                   0.0f,                // top-left
            sourceWidthF + dilationRadiusF,     0.0f,                // top-right  
            sourceWidthF + dilationRadiusF,     sourceHeightF,       // bottom-right
            -dilationRadiusF,                   sourceHeightF        // bottom-left
        };
        
        std::vector<uint16_t> indexData =
        {
            0, 1, 2,  // first triangle
            0, 2, 3   // second triangle
        };
        
        auto vertices = target.renderContext().createVertexBuffer(vertexData.data(), vertexData.size() * sizeof(float));
        auto indices = target.renderContext().createIndexBuffer(indexData.data(), indexData.size());

        // Create scratch buffer for horizontal pass (only width is expanded)  
        auto scratchBuffer = std::make_unique<renderer::ScratchBuffer>(target,
                                                                       singleChannelDst ? gpu::kRed : gpu::kRGBA,
                                                                       gpu::kUnsignedByte,
                                                                       sourceWidth + 2 * dilationRadius,
                                                                       sourceHeight);
        
        // The toDstNDC matrix needs to convert from image coordinates (x:[-dilation, width + dilation], y:[0, height]) to
        // NDC coordinates (x:[-1, 1], y:[-1, 1]). In the x dimension, we start by translating by the dilation, then scaling
        // by 2 / (width + 2 * dilation). In the y dimension, we just scale by 2 / height. After doing so, our coordinate
        // system is 2 units wide with the origin at (0,0). Thus, the final step is to offset the origin by translating by
        // (-1, -1), then accounting for any platform specific y-flip that may be present.
        float ndcOrientation = target.renderContext().ndcOrientation();
        geom::Affinity2f toDstNDC = geom::Affinity2f::scale(1.0f, -ndcOrientation) *
                                    geom::Affinity2f::translate(-1.0f, -1.0f) *
                                    geom::Affinity2f::scale(2.0f / (sourceWidth + 2 * dilationRadius), 2.0f / sourceHeight) *
                                    geom::Affinity2f::translate(dilationRadius, 0.0f);
        
        // toSrcUV converts from image coordinates to texture coordinates (x:[0,1], y:[0,1]). Since the vertices are already
        // relative to the unexpanded source image, all we need to do here is normalize. Note that pixels that are within the
        // expanded edge for the dilation in the dst will sample outside of the source image, producing (0,0,0,0), as intended.
        geom::Affinity2f toSrcUV = geom::Affinity2f::scale(1.0f / sourceWidth, 1.0f / sourceHeight);

        auto& dilateProgram = target.get<renderer::PuppetDilateProgram>();
        target.useProgram(dilateProgram);

        // Perform horizontal dilation
        scratchBuffer->bind(target, /* clear = */ true);
        dilateProgram.setUp(target, sourceSampler, dilationRadius,
                            /* isHorizontalPass = */ true,
                            sourceWidth, sourceHeight, singleChannelSrc, invertSrc,
                            toDstNDC, toSrcUV);
        dilateProgram.draw(*vertices, *indices, indexData.size());
        
        return scratchBuffer;
    }

    void PuppetDilateProgramDrawer::drawVertical(renderer::GPUTarget& target,
                                                 int dilationRadius,
                                                 const gpu::Texture& src,
                                                 renderer::ScratchBuffer& horizontalResult,
                                                 gpu::Texture& dst,
                                                 bool singleChannelDst) const
    {
        // Derive parameters from original source texture properties
        const int sourceWidth = src.width();
        const int sourceHeight = src.height();
        
        // Float versions for vertex data to avoid casting
        const float dilationRadiusF = dilationRadius;
        const float sourceWidthF = sourceWidth;
        const float sourceHeightF = sourceHeight;
        
        // Create vertex data for vertical pass (rendering to fully expanded destination)  
        std::vector<float> vertexData =
        {
            // x,  y
            -dilationRadiusF,                   -dilationRadiusF,                      // top-left
            sourceWidthF + dilationRadiusF,     -dilationRadiusF,                      // top-right
            sourceWidthF + dilationRadiusF,     sourceHeightF + dilationRadiusF,       // bottom-right
            -dilationRadiusF,                   sourceHeightF + dilationRadiusF        // bottom-left
        };
        
        std::vector<uint16_t> indexData =
        {
            0, 1, 2,  // first triangle
            0, 2, 3   // second triangle
        };
        
        auto vertices = target.renderContext().createVertexBuffer(vertexData.data(), vertexData.size() * sizeof(float));
        auto indices = target.renderContext().createIndexBuffer(indexData.data(), indexData.size());
        
        // The toDstNDC matrix needs to convert from image coordinates (x:[-dilation, width + dilation],
        // y:[-dilation, height + dilation]) to NDC coordinates (x:[-1, 1], y:[-1, 1]). We start by translating by the
        // dilation, then scaling by 2 / (dimension + 2 * dilation). After doing so, our coordinate system is 2 units wide
        // with the origin at (0,0). Thus, the final step is to offset the origin by translating by (-1, -1), then accounting
        // for any platform specific y-flip that may be present.
        float ndcOrientation = target.renderContext().ndcOrientation();
        geom::Affinity2f toDstNDC = geom::Affinity2f::scale(1.0f, -ndcOrientation) *
                                    geom::Affinity2f::translate(-1.0f, -1.0f) *
                                    geom::Affinity2f::scale(2.0f / (sourceWidth + 2 * dilationRadius), 2.0f / (sourceHeight + 2 * dilationRadius)) *
                                    geom::Affinity2f::translate(dilationRadius, dilationRadius);
        
        // toSrcUV converts from image coordinates to texture coordinates (x:[0,1], y:[0,1]). The vertices are already in
        // image coordinates relative to the unexpanded source image. Since the y dimension is not expanded by the previous
        // horizontal pass, all we need to do there is normalize. However, since the x dimension is expanded, we need to offset
        // by the dilation then normalize. Said another way, for the x dimension, we want -dilation -> 0 and width + dilation -> 1.
        geom::Affinity2f toSrcUV = geom::Affinity2f::scale(1.0f / (sourceWidth + 2 * dilationRadius), 1.0f / sourceHeight) *
                                   geom::Affinity2f::translate(dilationRadius, 0.0f);

        auto& dilateProgram = target.get<renderer::PuppetDilateProgram>();
        target.useProgram(dilateProgram);

        // Perform vertical dilation from horizontally-dilated buffer to final destination
        target.renderContext().bindColorBuffer(dst);
        const auto horizontalSampler = target.sampler_nn(horizontalResult.get());
        // Note: Use invertSrc=false for pass 2 since inversion was already applied in pass 1
        dilateProgram.setUp(target, horizontalSampler, dilationRadius,
                            /* isHorizontalPass = */ false,
                            sourceWidth + 2 * dilationRadius, sourceHeight, singleChannelDst,
                            /* invertSrc = */ false, toDstNDC, toSrcUV);
        dilateProgram.draw(*vertices, *indices, indexData.size());
    }

}
