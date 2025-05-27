#version 330 core
layout (location = 0) in vec3 aPos;
layout (location = 1) in vec2 aTexCoord; // unused
layout (location = 2) in vec3 aColor;
uniform mat4 ProjectionView; 
uniform mat4 Model;
out vec3 Color;
void main()
{
    gl_Position = ProjectionView * Model * vec4(aPos, 1.0);
    Color = aColor;
}
