#version 440

in vec3 Position;
in vec3 Normal;

uniform struct LightInfo{
    vec4 Position; // Light position in eye coords.
    vec3 La;       // Ambient light intensity
    vec3 L;        // Diffuse and Specular light intensity
} Light;


layout (binding = 0) uniform MaterialInfo {
  vec3 Ka;            // Ambient reflectivity
  vec3 Kd;            // Diffuse reflectivity
  vec3 Ks;            // Specular reflectivity
  float Shininess;    // Specular shininess factor
} Material;

uniform int levels = 5;
const float scaleFactor = 1.0 / levels;

layout( location = 0 ) out vec4 FragColor;

vec3 toonShade() {
    vec3 n = normalize(Normal);
    vec3 s = normalize(Light.Position.xyz - Position);
    vec3 ambient = Light.La * Material.Ka;
    float sDotN = max(dot(s,n), 0.0);
    vec3 diffuse = Material.Kd * floor(sDotN * levels) * scaleFactor;
    return ambient + Light.L * diffuse;
}


void main() {

    FragColor = vec4(toonShade(), 1.0);
}
