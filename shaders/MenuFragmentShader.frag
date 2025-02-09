in mediump vec2 TexCoords;
out mediump vec4 color;

uniform sampler2D menuTextureSampler;
uniform mediump vec3 colour;

void main()
{
    mediump vec4 sampled = texture(menuTextureSampler, TexCoords).rgba;
    color = vec4(colour, 1.0) * sampled;
}