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


#include "CPTestCase.h"

#include "geom/Geom"

#include "tests/SketchTestHelpers.h"

#include "PuppetPrograms.h"

namespace renderer
{

    class PuppetProgramsTests
    {
        class Fixture
        {
        public:
            GPUTargetFactory::LocalTarget target;
            std::unique_ptr<gpu::Texture> sourceTexture;
            std::unique_ptr<gpu::Texture> destTexture;
            std::unique_ptr<gpu::VertexBuffer> vertices;
            std::unique_ptr<gpu::IndexBuffer> indices;

            Fixture() : target(setup())
            {
                (*target).renderContext().disableScissor();
                createResources();
            }

            ~Fixture()
            {
                gpu::destroy((*target).renderContext(), sourceTexture);
                gpu::destroy((*target).renderContext(), destTexture);
                gpu::destroy((*target).renderContext(), vertices);
                gpu::destroy((*target).renderContext(), indices);
            }

            // Helper method to read pixels back from destination texture
            // Returns pixels in row-major order: [0][1]
            //                                    └─row 0─┘
            std::vector<test::Pixel> getDestinationPixels()
            {
                std::vector<test::Pixel> pixels(destTexture->width() * destTexture->height());
                destTexture->getPixels((*target).renderContext(), 0, 0,
                                       destTexture->width(), destTexture->height(),
                                       pixels.data());
                return pixels;
            }

            // Helper method to verify pixel values with tolerance
            bool verifyPixel(const test::Pixel& actual, const test::Pixel& expected, uint8_t tolerance = 1)
            {
                return std::abs(actual.r - expected.r) <= tolerance &&
                       std::abs(actual.g - expected.g) <= tolerance &&
                       std::abs(actual.b - expected.b) <= tolerance &&
                       std::abs(actual.a - expected.a) <= tolerance;
            }
            
            // Helper method to run puppet drawer test with specified parameters
            std::vector<test::Pixel> runPuppetDrawerTest(bool overlapping, uint8_t alpha, bool treatAlphaAsInverted)
            {
                createResources(overlapping, alpha);
                gpu::RenderContext& ctx = (*target).renderContext();
                
                // Bind destination texture as render target
                ctx.bindColorBuffer(*destTexture);
                
                // Set up blending mode (hardware overblending)
                ctx.enableBlending();
                ctx.setBlendMode(gpu::RenderContext::kOne, gpu::RenderContext::kOneMinusSrcAlpha);
                
                // Create drawer and set up transformation matrices
                PuppetPreviewProgramDrawer drawer;
                
                // Transform from pixel coordinates to NDC: [0,2] x [0,1] -> [-1,1] x [-1,1]
                const geom::Affinity2f vertexToDst = geom::Affinity2f::translate(-1.0f, -1.0f) * geom::Affinity2f::scale(2.0f/2.0f, 2.0f/1.0f);
                
                // Transform from pixel coordinates to texture coordinates: [0,2] x [0,1] -> [0,1] x [0,1]  
                const geom::Affinity2f vertexToSrc = geom::Affinity2f::scale(1.0f/2.0f, 1.0f/1.0f);
                const gpu::Sampler sourceSampler = (*target).sampler_nn(*sourceTexture);
                
                // Draw with specified parameters
                drawer.draw(*target, vertexToDst, vertexToSrc, sourceSampler,
                           *vertices, indices.get(), 
                           treatAlphaAsInverted,
                           /* clear = */ true,
                           /* showMesh = */ false);
                
                // Read back and return the result pixels
                return getDestinationPixels();
            }

        private:
            GPUTargetFactory& setup()
            {
                return GPUTargetFactory::instance();
            }

            void createResources(bool overlapping = false, uint8_t alpha = 255)
            {
                // Create 2x1 test textures (width=2, height=1) - simplest possible case
                sourceTexture = (*target).renderContext().createTexture(2, 1, nullptr);
                destTexture = (*target).renderContext().createTexture(2, 1, nullptr);
                
                // Simple test: single triangle covering right pixel, sampling from left pixel
                // For 2x1 texture: copy left pixel (0,0) to right pixel (1,0)
                // Vertex format: {pixel_x, pixel_y, pixel_u, pixel_v} - all in pixel coordinates
                float vertexData[] =
                {
                    // Triangle that moves the left pixel over one to the right
                    1.0f, 0.0f, 0.0f, 0.0f,  // vertex 0: dest (1,0) <- src (0,0)
                    2.0f, 0.0f, 1.0f, 0.0f,  // vertex 1: dest (2,0) <- src (1,0)
                    1.5f, 1.0f, 0.5f, 0.5f   // vertex 2: dest (1.5,1) <- src (0.5,0.5)
                };
            
                // Create index data - two identical triangles
                uint16_t indexData[] =
                {
                    0, 1, 2,  // First triangle
                    0, 1, 2   // Second triangle (same vertices)
                };

                // If overlapping, draw the same triangle on top of itself
                int indexCount = overlapping ? 6 : 3;
                
                vertices = (*target).renderContext().createVertexBuffer(vertexData, sizeof(vertexData));
                indices = (*target).renderContext().createIndexBuffer(indexData, indexCount * sizeof(uint16_t));
                
                // Fill source texture with simple pattern: [White][Black]
                // Layout: [W][B]  ← Single row
                uint8_t testPattern[2 * 1 * 4] =
                {
                    // (0,0) (1,0)
                    255, 255, 255, alpha,       // Left: White pixel with specified alpha
                    0, 0, 0, 255                // Right: Black pixel (always opaque)
                };
                sourceTexture->setPixels((*target).renderContext(), 0, 0, 2, 1, 4, testPattern);
            }
        };

        class DilateFixture
        {
        public:
            GPUTargetFactory::LocalTarget target;
            std::unique_ptr<gpu::Texture> sourceTexture;
            std::unique_ptr<gpu::Texture> destTexture;
            uint32_t width, height;

            DilateFixture(uint32_t w = 5, uint32_t h = 5, bool singleChannel = false) : 
                target(setup()), width(w), height(h)
            {
                (*target).renderContext().disableScissor();
                createTextures(singleChannel);
            }

            ~DilateFixture()
            {
                gpu::destroy((*target).renderContext(), sourceTexture);
                gpu::destroy((*target).renderContext(), destTexture);
            }

            // Set source texture data (RGBA format)
            void setSourceData(const std::vector<test::Pixel>& pixels)
            {
                if (pixels.size() != width * height)
                {
                    throw std::invalid_argument("Pixel count doesn't match texture dimensions");
                }
                
                sourceTexture->setPixels((*target).renderContext(), 0, 0, width, height, 4 * width, 
                                         reinterpret_cast<const uint8_t*>(pixels.data()));
            }

            // Set source texture data (single-channel R format)
            void setSourceDataR(const std::vector<uint8_t>& pixels)
            {
                if (pixels.size() != width * height)
                {
                    throw std::invalid_argument("Pixel count doesn't match texture dimensions");
                }
                
                sourceTexture->setPixels((*target).renderContext(), 0, 0, width, height, 1 * width, 
                                         pixels.data());
            }

            // Helper method to create a simple pixel
            test::Pixel pixel(uint8_t r, uint8_t g, uint8_t b, uint8_t a)
            {
                return test::Pixel(r, g, b, a);
            }

            // Helper to create clear (transparent) pixel
            test::Pixel clear() { return pixel(0, 0, 0, 0); }

            // Helper to create solid white pixel
            test::Pixel white() { return pixel(255, 255, 255, 255); }

            // Helper to create solid black pixel
            test::Pixel black() { return pixel(0, 0, 0, 255); }

            // Run dilation and validate against expected expanded output (RGBA format)
            bool validate(const std::vector<test::Pixel>& expectedExpandedOutput,
                          int dilationRadius = 1, 
                          bool invertSrc = false)
            {
                const uint32_t expandedWidth = width + 2 * dilationRadius;
                const uint32_t expandedHeight = height + 2 * dilationRadius;
                
                if (expectedExpandedOutput.size() != expandedWidth * expandedHeight)
                {
                    return false;
                }

                // Create expanded destination texture for dilation
                std::unique_ptr<gpu::Texture> expandedDestTexture = (*target).renderContext().createTexture(expandedWidth, expandedHeight, nullptr);

                // Bind expanded destination texture as render target
                gpu::RenderContext& ctx = (*target).renderContext();
                ctx.bindColorBuffer(*expandedDestTexture);
                ctx.clearColorBuffer(0.0f, 0.0f, 0.0f, 0.0f);

                // Run dilation on expanded buffer
                PuppetDilateProgramDrawer drawer;
                drawer.draw(*target, dilationRadius, *sourceTexture, *expandedDestTexture, invertSrc);

                // Read back expanded result
                std::vector<test::Pixel> resultPixels(expandedWidth * expandedHeight);
                expandedDestTexture->getPixels(ctx, 0, 0, expandedWidth, expandedHeight, resultPixels.data());

                // Compare with expected expanded output
                for (size_t i = 0; i < expectedExpandedOutput.size(); ++i)
                {
                    if (!verifyPixel(resultPixels[i], expectedExpandedOutput[i]))
                    {
                        return false;
                    }
                }

                return true;
            }

            // Run dilation and validate against expected expanded output (single-channel R format)
            bool validateR(const std::vector<uint8_t>& expectedExpandedOutputR,
                           int dilationRadius = 1,
                           bool invertSrc = false)
            {
                const uint32_t expandedWidth = width + 2 * dilationRadius;
                const uint32_t expandedHeight = height + 2 * dilationRadius;
                
                if (expectedExpandedOutputR.size() != expandedWidth * expandedHeight)
                {
                    return false;
                }

                // Create expanded destination texture for dilation
                std::unique_ptr<gpu::Texture> expandedDestTexture = (*target).renderContext().createTexture(expandedWidth, expandedHeight, nullptr);

                // Bind expanded destination texture as render target
                gpu::RenderContext& ctx = (*target).renderContext();
                ctx.bindColorBuffer(*expandedDestTexture);
                ctx.clearColorBuffer(0.0f, 0.0f, 0.0f, 0.0f);

                // Run dilation on expanded buffer
                PuppetDilateProgramDrawer drawer;
                drawer.draw(*target, dilationRadius, *sourceTexture, *expandedDestTexture, invertSrc);

                // Read back expanded result (comes back as RGBA pixels from GPU)
                std::vector<test::Pixel> resultPixels(expandedWidth * expandedHeight);
                expandedDestTexture->getPixels(ctx, 0, 0, expandedWidth, expandedHeight, resultPixels.data());

                // Compare with expected expanded output - check only the R channel
                for (size_t i = 0; i < expectedExpandedOutputR.size(); ++i)
                {
                    uint8_t expectedR = expectedExpandedOutputR[i];
                    uint8_t actualR = resultPixels[i].r;
                    
                    // For single-channel dilation: if expectedR > 0, we expect white (255,255,255,255)
                    // if expectedR == 0, we expect clear (0,0,0,0)
                    test::Pixel expectedPixel;
                    if (expectedR > 0) {
                        expectedPixel = white(); // (255,255,255,255)
                    } else {
                        expectedPixel = clear(); // (0,0,0,0)
                    }
                    
                    if (!verifyPixel(resultPixels[i], expectedPixel))
                    {
                        return false;
                    }
                }

                return true;
            }

        private:
            GPUTargetFactory& setup()
            {
                return GPUTargetFactory::instance();
            }

            void createTextures(bool singleChannel = false)
            {
                if (singleChannel)
                {
                    // Create single-channel (R) source texture
                    sourceTexture = (*target).renderContext().createTexture(width, height, gpu::kRed, gpu::kUnsignedByte, nullptr);
                }
                else
                {
                    // Create RGBA source texture (default)
                    sourceTexture = (*target).renderContext().createTexture(width, height, nullptr);
                }
                
                // Destination is always RGBA since we read back as test::Pixel
                destTexture = (*target).renderContext().createTexture(width, height, nullptr);
            }

            // Helper method to read pixels back from destination texture
            std::vector<test::Pixel> getDestinationPixels()
            {
                std::vector<test::Pixel> pixels(width * height);
                destTexture->getPixels((*target).renderContext(), 0, 0, width, height, pixels.data());
                return pixels;
            }

            // Helper method to verify pixel values with tolerance
            bool verifyPixel(const test::Pixel& actual, const test::Pixel& expected, uint8_t tolerance = 1)
            {
                return std::abs(actual.r - expected.r) <= tolerance &&
                       std::abs(actual.g - expected.g) <= tolerance &&
                       std::abs(actual.b - expected.b) <= tolerance &&
                       std::abs(actual.a - expected.a) <= tolerance;
            }
        };

    public:
        
        void testPuppetDrawer()
        {
            Fixture f;
            
            // Traditional test: single triangle, full alpha, no inversion
            std::vector<test::Pixel> resultPixels = f.runPuppetDrawerTest(
                /* overlapping = */ false, 
                /* alpha = */ 255, 
                /* treatAlphaAsInverted = */ false
            );
            
            // Expected: triangle should copy left pixel (white) to right pixel  
            test::Pixel expectedWhite(255, 255, 255, 255);
            
            // Verify that triangle copied white from left side to right side  
            CPAssertTrue(f.verifyPixel(resultPixels[1], expectedWhite)); // (1,0) - should be white from triangle
        }
        
        void testPuppetDrawerNormalBlending()
        {
            Fixture f;
            
            // Test overlapping normal blending: alpha=64 (25%), no inversion
            std::vector<test::Pixel> resultPixels = f.runPuppetDrawerTest(
                /* overlapping = */ true, 
                /* alpha = */ 64, 
                /* treatAlphaAsInverted = */ false
            );
            
            // Expected: overlapping triangles with alpha=64 should blend
            // First triangle: alpha = 64
            // Second triangle: alpha = 64 + 64 * (1 - 64/255) = 64 + 64 * (191/255) ≈ 112
            test::Pixel expectedBlended(112, 112, 112, 112);  // Blended alpha result
            
            // Verify the blended result matches expected alpha blending
            CPAssertTrue(f.verifyPixel(resultPixels[1], expectedBlended)); // Should match blended alpha
        }
    
        void testPuppetDrawerInvertedBlending()
        {
            Fixture f;
            
            // Test overlapping inverted blending: alpha=191 (75%), with inversion
            std::vector<test::Pixel> resultPixels = f.runPuppetDrawerTest(
                /* overlapping = */ true, 
                /* alpha = */ 191, 
                /* treatAlphaAsInverted = */ true
            );
            
            // Expected: overlapping inverted triangles with alpha=191 should blend
            // Process: invert → draw → invert back
            // This should be mathematically equivalent to inverting the normal blending result
            // If normal blending gives (112, 112, 112, 112), inverted gives 255 minus those values
            test::Pixel expectedInvertedBlended(143, 143, 143, 143);  // (143, 143, 143, 143)

            // Verify the inverted blended result  
            CPAssertTrue(f.verifyPixel(resultPixels[1], expectedInvertedBlended)); // Should match inverted blended result
        }

        void testDilateRGBA()
        {
            // Test 5x5 dilation with plus sign input, expecting 7x7 expanded output
            DilateFixture f(5, 5);
            
            // Create source image: plus sign in center
            std::vector<test::Pixel> sourceData = {
                f.clear(), f.clear(), f.clear(), f.clear(), f.clear(),  // Row 0
                f.clear(), f.clear(), f.white(), f.clear(), f.clear(),  // Row 1
                f.white(), f.white(), f.white(), f.white(), f.white(),  // Row 2: horizontal bar
                f.clear(), f.clear(), f.white(), f.clear(), f.clear(),  // Row 3
                f.clear(), f.clear(), f.clear(), f.clear(), f.clear()   // Row 4
            };
            f.setSourceData(sourceData);
            
            // Expected 7x7 expanded output: two-pass separable dilation with radius 1
            // Dilation acts as a box filter, not circular
            std::vector<test::Pixel> expectedExpandedOutput = {
                f.clear(), f.clear(), f.clear(), f.clear(), f.clear(), f.clear(), f.clear(),  // Row -1
                f.clear(), f.clear(), f.white(), f.white(), f.white(), f.clear(), f.clear(),  // Row 0
                f.white(), f.white(), f.white(), f.white(), f.white(), f.white(), f.white(),  // Row 1
                f.white(), f.white(), f.white(), f.white(), f.white(), f.white(), f.white(),  // Row 2
                f.white(), f.white(), f.white(), f.white(), f.white(), f.white(), f.white(),  // Row 3
                f.clear(), f.clear(), f.white(), f.white(), f.white(), f.clear(), f.clear(),  // Row 4
                f.clear(), f.clear(), f.clear(), f.clear(), f.clear(), f.clear(), f.clear()   // Row 5
            };
            
            // Run dilation and validate expanded output
            CPAssertTrue(f.validate(expectedExpandedOutput, /* dilationRadius = */ 1));
        }

        void testDilateR()
        {
            // Test 5x5 single-channel dilation with plus sign input, expecting 7x7 expanded output
            DilateFixture f(5, 5, /* singleChannel = */ true);
            
            // Create source image: plus sign in center (single-channel data)
            // 0 = no content, 255 = white content
            std::vector<uint8_t> sourceData = {
                0,   0,   0,   0,   0,     // Row 0
                0,   0,   255, 0,   0,     // Row 1
                255, 255, 255, 255, 255,   // Row 2: horizontal bar
                0,   0,   255, 0,   0,     // Row 3  
                0,   0,   0,   0,   0      // Row 4
            };
            f.setSourceDataR(sourceData);
            
            // Expected 7x7 expanded output: two-pass separable dilation with radius 1
            // Dilation acts as a box filter, not circular
            std::vector<uint8_t> expectedExpandedOutputR = {
                0,   0,   0,   0,   0,   0,   0,     // Row -1
                0,   0,   255, 255, 255, 0,   0,     // Row 0
                255, 255, 255, 255, 255, 255, 255,   // Row 1
                255, 255, 255, 255, 255, 255, 255,   // Row 2
                255, 255, 255, 255, 255, 255, 255,   // Row 3
                0,   0,   255, 255, 255, 0,   0,     // Row 4
                0,   0,   0,   0,   0,   0,   0      // Row 5
            };
            
            // Run dilation and validate expanded output (R = single-channel)
            CPAssertTrue(f.validateR(expectedExpandedOutputR, /* dilationRadius = */ 1));
        }

        void testDilateInvertedRGBA()
        {
            // Test 5x5 RGBA dilation with inversion enabled, expecting 7x7 expanded output
            DilateFixture f(5, 5);
            
            // Create inverted source image: clear holes in white background (opposite of testDilateRGBA)
            // The shader will invert this back to the normal plus pattern before dilation
            std::vector<test::Pixel> sourceData = {
                f.white(), f.white(), f.white(), f.white(), f.white(),  // Row 0: white background
                f.white(), f.white(), f.clear(), f.white(), f.white(),  // Row 1: clear hole
                f.clear(), f.clear(), f.clear(), f.clear(), f.clear(),  // Row 2: horizontal clear bar
                f.white(), f.white(), f.clear(), f.white(), f.white(),  // Row 3: clear hole
                f.white(), f.white(), f.white(), f.white(), f.white()   // Row 4: white background
            };
            f.setSourceData(sourceData);
            
            // Expected 7x7 expanded output: two-pass separable dilation with radius 1
            // Dilation acts as a box filter, not circular
            std::vector<test::Pixel> expectedExpandedOutput = {
                f.clear(), f.clear(), f.clear(), f.clear(), f.clear(), f.clear(), f.clear(),  // Row -1
                f.clear(), f.clear(), f.white(), f.white(), f.white(), f.clear(), f.clear(),  // Row 0
                f.white(), f.white(), f.white(), f.white(), f.white(), f.white(), f.white(),  // Row 1
                f.white(), f.white(), f.white(), f.white(), f.white(), f.white(), f.white(),  // Row 2
                f.white(), f.white(), f.white(), f.white(), f.white(), f.white(), f.white(),  // Row 3
                f.clear(), f.clear(), f.white(), f.white(), f.white(), f.clear(), f.clear(),  // Row 4
                f.clear(), f.clear(), f.clear(), f.clear(), f.clear(), f.clear(), f.clear()   // Row 5
            };
            
            // Run dilation with inversion enabled
            CPAssertTrue(f.validate(expectedExpandedOutput, /* dilationRadius = */ 1, /* invertSrc = */ true));
        }

        void testDilateInvertedR()
        {
            // Test 5x5 single-channel dilation with inversion enabled, expecting 7x7 expanded output
            DilateFixture f(5, 5, /* singleChannel = */ true);
            
            // Create inverted source image: 0 holes in 255 background (opposite of testDilateR)
            // The shader will invert this back to the normal plus pattern before dilation
            std::vector<uint8_t> sourceData = {
                255, 255, 255, 255, 255,   // Row 0: white background
                255, 255, 0,   255, 255,   // Row 1: clear hole
                0,   0,   0,   0,   0,     // Row 2: horizontal clear bar
                255, 255, 0,   255, 255,   // Row 3: clear hole
                255, 255, 255, 255, 255    // Row 4: white background
            };
            f.setSourceDataR(sourceData);
            
            // Expected 7x7 expanded output: two-pass separable dilation with radius 1
            // Dilation acts as a box filter, not circular
            std::vector<uint8_t> expectedExpandedOutputR = {
                0,   0,   0,   0,   0,   0,   0,     // Row -1
                0,   0,   255, 255, 255, 0,   0,     // Row 0
                255, 255, 255, 255, 255, 255, 255,   // Row 1
                255, 255, 255, 255, 255, 255, 255,   // Row 2
                255, 255, 255, 255, 255, 255, 255,   // Row 3
                0,   0,   255, 255, 255, 0,   0,     // Row 4
                0,   0,   0,   0,   0,   0,   0      // Row 5
            };
            
            // Run dilation with inversion enabled
            CPAssertTrue(f.validateR(expectedExpandedOutputR, /* dilationRadius = */ 1, /* invertSrc = */ true));
        }
    };
}

CPSUITE(PuppetProgramsTests, renderer::PuppetProgramsTests)
CPTEST(testPuppetDrawer)
CPTEST(testPuppetDrawerNormalBlending)
CPTEST(testPuppetDrawerInvertedBlending)
CPTEST(testDilateRGBA)
CPTEST(testDilateR)
CPTEST(testDilateInvertedRGBA)
CPTEST(testDilateInvertedR)
CPENDSUITE
