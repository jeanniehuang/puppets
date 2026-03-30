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

#ifndef EXTENSIONS_PUPPET_PUPPETPROGRAMS
#define EXTENSIONS_PUPPET_PUPPETPROGRAMS

#include <map>
#include <string>
#include "renderer/gpu/GPU"

namespace renderer
{
    
    /** Program that renders a preview of puppet results.
     */
    class PuppetPreviewProgram :
        public GPUTarget::Resource,
        public gpu::Program,
        public gpu::Program::Binder
    {
        enum Attribute
        {
            // vertex position (x,y).
            ATTRIB_XY,
            // (u,v) coordinates within texture to blit.
            ATTRIB_UV
        };

        enum
        {
            FLOATS_PER_VERTEX = 4,
            XY_INDEX = 0,
            UV_INDEX = 2,
        };

    public:
        using super = gpu::Program;
        
        /** Make an instance of the program. */
        static std::unique_ptr<PuppetPreviewProgram> construct(GPUTarget& target);

        void destroyResource(GPUTarget& target) override;

        bool loadProgram(GPUTarget& target);
        bool preprocess(std::string& vtxshader, std::string& frgshader)override;
        void bindUAV() override;
		void bindAttributes()override;
        void getUniforms() override;
        
        bool use() override;

        /** Set up for drawing the source image.
            - vertexToDst maps vertex coordinates to destination NDC coordinates.
            - vertexToSrc maps vertex coordinates to source texture coordinates [0, 1].
            - when 'invertSrc' is true, the source will be sampled as 1 - r,g,b,a.
            Source is assumed to be un-multiplied.
         */
        void setUp(const geom::Affinity2f& vertexToDst,
                   const geom::Affinity2f& vertexToSrc,
                   const gpu::Sampler& texSourceImage,
                   AlphaFormat dstAlphaFmt,
                   bool invertSrc,
                   bool showMesh);
        
        /** Draw mesh.
         
            Each vertex is four floats {dx,dy,sx,sy} where dx,dy are the destination
            coordinates and sx,sy are source pixel coordinates for sampling the image.
         
            The index buffer is an array of triangles. If nullptr, draws without indices.
         */
        void draw(const gpu::VertexBuffer& vertices,
                  const gpu::IndexBuffer* indices);
    };

    /** Program that dilates a source image into a binarized results (255 for content, 0 for background)
     *  This program is designed to be run in two passes - one vertical dilation and one horizontal dilation.
     *  Since this is a two-pass approach, the dilation is a box rather than a circle.
     */
    class PuppetDilateProgram :
        public GPUTarget::Resource,
        public gpu::Program,
        public gpu::Program::Binder
    {
        enum Attribute
        {
            // vertex position (x,y) in NDC space
            ATTRIB_XY
        };

        enum
        {
            FLOATS_PER_VERTEX = 2,
            XY_INDEX = 0,
        };

    public:
        using super = gpu::Program;
        
        /** Make an instance of the program. */
        static std::unique_ptr<PuppetDilateProgram> construct(GPUTarget& target);

        void destroyResource(GPUTarget& target) override;

        bool loadProgram(GPUTarget& target);
        bool preprocess(std::string& vtxshader, std::string& frgshader)override;
        void bindUAV() override;
        void bindAttributes() override;
        void getUniforms() override;
        
        bool use() override;

        /** Set up for dilating the source image.
            - dilationRadius is the radius of dilation in pixels
            - isHorizontalPass determines if this is the horizontal (true) or vertical (false) pass
            - sourceWidth and sourceHeight are the dimensions of the source texture
            - singleChannelSrc determines content detection channel (true=red, false=alpha)
            - invertSrc inverts the source sample if true (1.0 - sample)
            - Direct 1:1 mapping from source to destination (no transformations)
            - flipY is determined from context NDC orientation
         */
        void setUp(GPUTarget& target,
                   const gpu::Sampler& texSourceImage,
                   int dilationRadius,
                   bool isHorizontalPass,
                   int sourceWidth,
                   int sourceHeight,
                   bool singleChannelSrc,
                   bool invertSrc,
                   const geom::Affinity2f& imageToDestNDC,
                   const geom::Affinity2f& imageToSrcUV);
        
        /** Draw mesh for dilation.
         
            Each vertex is two floats {x,y} representing image pixel coordinates.
            Transformation matrices convert these to NDC and source UV coordinates in the vertex shader.
         
            The index buffer is an array of triangles.
         */
        void draw(const gpu::VertexBuffer& vertices,
                  const gpu::IndexBuffer& indices,
                  uint32_t nIndices);
    };

    /** Helper class that invokes the puppet preview program
     */
    class PuppetPreviewProgramDrawer
    {
    public:
        /** Draws the provided vertices / indices of the puppet preview to the destination
            currently bound to the target under the provided transformations.

            - vertexToDst maps vertex coordinates to destination NDC coordinates.
            - vertexToSrc maps vertex coordinates to source texture coordinates [0, 1].
            - the source is treated as unmultiplied
            - the destination is drawn as premultiplied to allow for hardware blending
            - when 'treatAlphaAsInverted' is true, the program will invert the source image (aka 1 - r,g,b,a),
              draw to an intermediate buffer, then invert once more when copying the
              buffer to the final destination. This effectively unlocks different hardware
              blend modes.
            - when 'clear is true, the destination will be cleared to transparent before drawing
            - when 'showMesh' is true, the mesh triangles will be drawn with random colors (for debugging purposes)
         */
        void draw(renderer::GPUTarget& target,
                  const geom::Affinity2f& vertexToDst,
                  const geom::Affinity2f& vertexToSrc,
                  const gpu::Sampler& sourceSampler,
                  const gpu::VertexBuffer& vertices,
                  const gpu::IndexBuffer* indices,
                  bool treatAlphaAsInverted,
                  bool clear,
                  bool showMesh) const;

    private:
        void drawPreview(renderer::GPUTarget& target,
                         const geom::Affinity2f& vertexToDst,
                         const geom::Affinity2f& vertexToSrc,
                         const gpu::Sampler& sourceSampler,
                         const gpu::VertexBuffer& vertices,
                         const gpu::IndexBuffer* indices,
                         bool treatAlphaAsInverted,
                         bool showMesh) const;
        
        void drawAsInvertedAlpha(renderer::GPUTarget& target,
                                 const geom::Affinity2f& vertexToDst,
                                 const geom::Affinity2f& vertexToSrc,
                                 const gpu::Sampler& sourceSampler,
                                 const gpu::VertexBuffer& vertices,
                                 const gpu::IndexBuffer* indices,
                                 bool clear,
                                 bool showMesh) const;
        
        void drawNormal(renderer::GPUTarget& target,
                        const geom::Affinity2f& vertexToDst,
                        const geom::Affinity2f& vertexToSrc,
                        const gpu::Sampler& sourceSampler,
                        const gpu::VertexBuffer& vertices,
                        const gpu::IndexBuffer* indices,
                        bool clear,
                        bool showMesh) const;
    };

    /** Helper class that performs two-pass dilation using PuppetDilateProgram
     */
    class PuppetDilateProgramDrawer
    {
    public:
        /** Performs two-pass dilation on the source image to the destination.
            - The source image is sampled and dilated by dilationRadius pixels
            - First pass dilates horizontally into a scratch buffer, second pass dilates vertically
            - Uses binarization: any non-zero pixel within radius returns (1,1,1,1), otherwise (0,0,0,0)
            - Sampler is created internally using target.sampler_ll()
            - invertSrc inverts the source sample if true (1.0 - sample)
            - The dst texture should be expanded by 'dilation' on all sides compared to the src texture.
              Thus, the dimensions of the dst texture should be (srcWidth + 2*dilation) x (srcHeight + 2*dilation).
         */
        void draw(renderer::GPUTarget& target,
                  int dilationRadius,
                  const gpu::Texture& src,
                  gpu::Texture& dst,
                  bool invertSrc) const;

    private:
        /** Performs the horizontal pass as a substep of the above 'draw' method. Allocates
            and returns a scratch buffer that is expanded by 'dilation' on both sides of the
            x dimension. */
        std::unique_ptr<renderer::ScratchBuffer> drawHorizontal(renderer::GPUTarget& target,
                                                                int dilationRadius,
                                                                const gpu::Texture& src,
                                                                bool singleChannelSrc,
                                                                bool singleChannelDst,
                                                                bool invertSrc) const;

        /** Performs the vertical pass as a substep of the above 'draw' method. Consumes
            the output of the previous horizontal pass, then dilates directly to the dst.
            The horizontal result should be expanded by dilation on both sides of the
            x dimension, while the dst should be expanded by dilation on all sides. */
        void drawVertical(renderer::GPUTarget& target,
                          int dilationRadius,
                          const gpu::Texture& src,
                          renderer::ScratchBuffer& horizontalResult,
                          gpu::Texture& dst,
                          bool singleChannelDst) const;
    };
    
}

#endif
