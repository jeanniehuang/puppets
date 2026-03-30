#include <metal_config>
#include <metal_stdlib>
#include <simd/simd.h>
using namespace metal;

// Controls whether source color data is assumed to be pre- or un-multiplied
#define UNMULTIPLIED_SRC 1

// Enable to debug triangle mesh
#define RANDOM_TRIANGLE_COLORS 1

namespace puppet
{
    
    struct PreviewVSUniforms
    {
        // transforms vertex locations to destination ndc
        float4x4 vertexToDst;
        // transforms vertex coordinates to source texture coordinates [0, 1]
        float4x4 vertexToSrc;
    };
    
    struct PreviewVSInput
    {
        float2 vertexPosDst;
        float2 vertexPosSrc;
    };

    struct PreviewFSUniforms
    {
        int dstUnmultiplied;
        int invertSrc;
        int showMesh;
    };

    struct PreviewFSInput
    {
        float4 position [[position]];
        float2 uv;
#if RANDOM_TRIANGLE_COLORS
        uint triangleID [[flat]];
#endif
    };
    
    /** Parameters for filtered sampling of a texture. */
    struct SamplingParams
    {
        // Location to use when gathering the texture pixel values
        float2 gatherUV;
        // Factors to use for interpolating the gathered values
        float2 factor;
    };
    
    /** Utility function that returns parameters to be used for filtered
        sampling of a texture.
     */
    SamplingParams samplingParams(texture2d<float> texture, float2 uv)
    {
        uint2 texSize;
        texSize.x = texture.get_width();
        texSize.y = texture.get_height();
        
        const float2 floatTexSize = float2(texSize.x, texSize.y);

        // Convert UV coordinates to pixel coordinates and get pixel index of top left
        // pixel (assuming UVs are relative to top left corner of texture)
        const float2 pixelCoord = uv * floatTexSize - 0.5f;

        // this is the index of the top left pixel within the set of 4 pixels we want to use
        const float2 pixelCoordIndex = floor(pixelCoord);

        SamplingParams result;
        
        // For Gather we want UV coordinates of bottom right corner of top left pixel
        result.gatherUV = (pixelCoordIndex + 1.0f) / floatTexSize;

        // The factor to use for combining the pixel values
        result.factor = fract(pixelCoord);
        
        return result;
    }
    
    /** Sampling of the source color texture. When the source is un-multiplied
        hardware filtering cannot be used. The result is always pre-multiplied.
        When the source is pre-multiplied we're assuming the sampler will apply
        filtering.
     */
    float4 sampleColorPremul(texture2d<float> texture, sampler sampler, float2 uv)
    {
#if UNMULTIPLIED_SRC
        SamplingParams p = samplingParams(texture, uv);
        
        const float4 reds = texture.gather(sampler, p.gatherUV, int2(0), component::x);
        const float4 greens = texture.gather(sampler, p.gatherUV, int2(0), component::y);
        const float4 blues = texture.gather(sampler, p.gatherUV, int2(0), component::z);
        const float4 alphas = texture.gather(sampler, p.gatherUV, int2(0), component::w);

        float4 color1 = float4(reds.x, greens.x, blues.x, alphas.x);
        float4 color2 = float4(reds.y, greens.y, blues.y, alphas.y);
        float4 color3 = float4(reds.z, greens.z, blues.z, alphas.z);
        float4 color4 = float4(reds.w, greens.w, blues.w, alphas.w);

        // convert these colors from unmultiplied to premultiplied
        color1.rgb *= color1.a;
        color2.rgb *= color2.a;
        color3.rgb *= color3.a;
        color4.rgb *= color4.a;

        const float4 topRow = mix(color4, color3, p.factor.x);
        const float4 bottomRow = mix(color1, color2, p.factor.x);

        const float4 result = mix(topRow, bottomRow, p.factor.y);
        return result;
#else
        return texture.sample(sampler, uv);
#endif
    }

}
using namespace puppet;

vertex PreviewFSInput
puppet_preview_vs(constant PreviewVSUniforms& u [[buffer(0)]],
                  const device PreviewVSInput* vertices [[buffer(1)]],
                  unsigned int vid [[vertex_id]])
{
    const device PreviewVSInput& v = vertices[vid];
    PreviewFSInput out;
    
    out.position = u.vertexToDst * float4(v.vertexPosDst, 0.0, 1.0);
    out.uv = (u.vertexToSrc * float4(v.vertexPosSrc, 0.0, 1.0)).xy;

#if RANDOM_TRIANGLE_COLORS
    out.triangleID = vid;
#endif
    
    return out;
}

fragment float4
puppet_preview_fs(PreviewFSInput in [[stage_in]],
                  constant PreviewFSUniforms& u [[buffer(1)]],
                  texture2d<float> texSourceImage [[texture(0)]],
                  sampler samplerSourceImage [[sampler(0)]])
{
#if RANDOM_TRIANGLE_COLORS
    if (u.showMesh)
    {
        uint id = in.triangleID;
        
        // Simple hash to get a pseudo-random color
        float r = fmod(float((id * 16807) % 256), 256.0) / 255.0;
        float g = fmod(float((id * 48271) % 256), 256.0) / 255.0;
        float b = fmod(float((id * 69691) % 256), 256.0) / 255.0;
        
        return float4(r, g, b, 1.0);
    }
    else
#endif
    {
        float2 uv = in.uv;
        
        if (!(0.0 <= uv.x && uv.x <= 1.0 && 0.0 <= uv.y && uv.y <= 1.0))
            return float4(0.0);
        
        float4 src = sampleColorPremul(texSourceImage, samplerSourceImage, uv);
        
        // Apply source inversion if requested
        if (u.invertSrc == 1)
        {
            src = max(float4(1.0) - src, float4(0.0));
        }

        // src is pre-mul, remove if needed
        if (u.dstUnmultiplied == 1 && src.a > 0.0)
            src.rgb /= src.a;
        
        return src;
    }
    
}
