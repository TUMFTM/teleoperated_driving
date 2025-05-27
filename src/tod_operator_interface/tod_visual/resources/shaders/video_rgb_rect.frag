#version 330 core


uniform sampler2DRect RGBtex; 
out vec4 colour;
in vec2 TexCoord;

void main(void) {
    vec3 rgb = texture2DRect(RGBtex, TexCoord).rgb;
    colour = vec4(rgb, 1.0);
}