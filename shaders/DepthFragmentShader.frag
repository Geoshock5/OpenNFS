// Ouput data
layout(location = 0) out mediump float fragmentdepth;

// Discard pixels for depth buffer based on Alpha (This might nerf perf)
in mediump vec2 UV;
flat in uint texIndex;
uniform mediump sampler2DArray texture_array;

void main(){
    mediump vec4 tempColor = texture(texture_array, vec3(UV, texIndex)).rgba;
    if (tempColor.a <= 0.5)
         discard;

    // Not really needed, OpenGL does it anyway
    fragmentdepth = gl_FragCoord.z;
}
