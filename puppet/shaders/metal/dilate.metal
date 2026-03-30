#include <metal_config>
#include <metal_stdlib>
#include <simd/simd.h>
using namespace metal;

namespace puppet
{
    
    struct DilateVSUniforms
    {
        float4x4 imageToDestNDC;    // Transform from image pixels to destination NDC
        float4x4 imageToSrcUV;      // Transform from image pixels to source UV coordinates
    };
    
    struct DilateVSInput
    {
        float2 position;  // Image pixel coordinates
    };

    struct DilateFSUniforms
    {
        int dilationRadius;
        int isHorizontalPass;  // 1 for horizontal, 0 for vertical
        int sourceWidth;
        int sourceHeight;
        int singleChannelSrc;  // 1 for single channel (use .r), 0 for multi-channel (use .a)
        int invertSrc;         // 1 to invert source sample, 0 for normal sampling
    };

    struct DilateFSInput
    {
        float4 position [[position]];
        float2 uv;
    };
    
}
using namespace puppet;

vertex DilateFSInput
puppet_dilate_vs(constant DilateVSUniforms& u [[buffer(0)]],
                 const device DilateVSInput* vertices [[buffer(1)]],
                 unsigned int vid [[vertex_id]])
{
    const device DilateVSInput& v = vertices[vid];
    DilateFSInput out;
    
    // Convert from image pixel coordinates to destination NDC
    float4 imagePos = float4(v.position, 0.0, 1.0);
    out.position = u.imageToDestNDC * imagePos;
    
    // Convert from image pixel coordinates to source UV coordinates
    float4 srcUVPos = u.imageToSrcUV * imagePos;
    out.uv = srcUVPos.xy;
    
    return out;
}

fragment float4
puppet_dilate_fs(DilateFSInput in [[stage_in]],
                 constant DilateFSUniforms& u [[buffer(1)]],
                 texture2d<float> texSourceImage [[texture(0)]],
                 sampler samplerSourceImage [[sampler(0)]])
{
    float2 uv = in.uv;
    
    // Get texture dimensions from uniforms
    float2 texelSize = float2(1.0 / float(u.sourceWidth), 1.0 / float(u.sourceHeight));
    
    // Determine sampling direction based on pass type
    float2 sampleOffset = (u.isHorizontalPass == 1) ? float2(texelSize.x, 0.0) : float2(0.0, texelSize.y);
    
    // Perform dilation in the specified direction
    for (int offset = -u.dilationRadius; offset <= u.dilationRadius; offset++)
    {
        float2 sampleUV = uv + float(offset) * sampleOffset;
        
        float4 sample = texSourceImage.sample(samplerSourceImage, sampleUV);
        
        // Apply source inversion if requested
        if (u.invertSrc == 1)
        {
            sample = float4(1.0) - sample;
        }
        
        // Check if this sample has content - if so, return immediately
        bool hasContent = (u.singleChannelSrc == 1) ? (sample.r > 0.0) : (sample.a > 0.0);
        if (hasContent)
        {
            return float4(1.0, 1.0, 1.0, 1.0);
        }
    }
    
    // No content found in dilation radius
    return float4(0.0, 0.0, 0.0, 0.0);
}
