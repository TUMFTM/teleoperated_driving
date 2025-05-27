#version 330 core
out vec4 FragColor;
in vec2 TexCoord;
in vec3 Color;
uniform sampler2D texture_diffuse;
void main()
{
    FragColor = texture(texture_diffuse, TexCoord);
}
