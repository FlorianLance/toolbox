#version 440 core

in vec3 LightIntensity;
layout(location = 0) out vec4 FragColor;

void main() {
    FragColor = vec4(LightIntensity, 1.0);
//    FragColor = vec4(1.0,0,0, 1.0);
}
