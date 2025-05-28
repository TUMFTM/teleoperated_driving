#version 330 core
//layout (location = 0) in vec4 vertex; // <vec2 pos, vec2 tex>
layout (location = 0) in vec3 aPos;
layout (location = 1) in vec2 aTexCoord;
layout (location = 2) in vec3 aColor; // unused
out vec2 TexCoords;

uniform mat4 ProjectionView;
uniform mat4 Model; 

void main()
{
    gl_Position = ProjectionView * Model * vec4(aPos.xyz, 1.0);
    TexCoords = aTexCoord;
}