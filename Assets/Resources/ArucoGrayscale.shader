// Converts the passthrough camera image to 8-bit luminance on the GPU so only
// 1 byte per pixel has to be read back to the CPU (4x less than RGBA).
// Loaded at runtime via Resources.Load<Shader>("ArucoGrayscale").
Shader "Hidden/ArucoGrayscale"
{
    Properties
    {
        _MainTex ("Texture", 2D) = "white" {}
        // 1 = mirror the image vertically. Same result as flipping it in OpenCV, but free.
        _FlipY ("Flip Y", Float) = 0
    }
    SubShader
    {
        Cull Off ZWrite Off ZTest Always

        Pass
        {
            CGPROGRAM
            #pragma vertex vert_img
            #pragma fragment frag
            #include "UnityCG.cginc"

            sampler2D _MainTex;
            float _FlipY;

            fixed4 frag (v2f_img i) : SV_Target
            {
                float2 uv = float2(i.uv.x, _FlipY > 0.5 ? 1.0 - i.uv.y : i.uv.y);
                float3 color = tex2D(_MainTex, uv).rgb;
            #if !defined(UNITY_COLORSPACE_GAMMA)
                // Match the bytes the old ARGB32 (sRGB) readback produced.
                color = LinearToGammaSpace(color);
            #endif
                float luminance = dot(color, float3(0.299, 0.587, 0.114));
                return fixed4(luminance, luminance, luminance, 1);
            }
            ENDCG
        }
    }
}
