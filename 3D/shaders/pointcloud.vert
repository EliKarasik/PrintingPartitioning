#version 150 core

in vec3 vertexPosition;
in vec3 vertexColor;

out vec4 color;

uniform mat4 modelView;
uniform mat3 modelViewNormal;
uniform mat4 mvp;
uniform mat4 projectionMatrix;
uniform mat4 viewportMatrix;

void main()
{
    //normal = normalize(modelViewNormal * vertexNormal);
    //color = vertexColor;
    gl_Position = mvp * vec4(vertexPosition, 1.0);
    gl_PointSize = 5;
    color = vec4(1,1,1,1);
    //gl_PointSize = viewportMatrix[1][1] * projectionMatrix[1][1] / gl_Position.w;
}