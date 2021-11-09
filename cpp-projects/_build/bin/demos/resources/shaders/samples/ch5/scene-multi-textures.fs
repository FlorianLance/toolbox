#version 440

in vec3 Position;
in vec3 Normal;
in vec2 TexCoord;

layout (binding=0) uniform sampler2D Tex1;
layout (binding=1) uniform sampler2D Tex2;

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


layout( location = 0 ) out vec4 FragColor;

vec3 blinnphong(vec3 position, vec3 n ) {

    vec4 texColor1 = texture(Tex1, TexCoord);
    vec4 texColor2 = texture(Tex2, TexCoord);
    vec3 mixTexColor = mix(texColor1.rgb,texColor2.rgb, texColor2.a);

    vec3 ambient = Light.La * Material.Ka * mixTexColor;

    vec3 s = normalize(Light.Position.xyz - position);
    float sDotN = max (dot(s,n), 0.0);
    vec3 diffuse = Material.Kd * sDotN;

    vec3 spec = vec3(0.0);
    if( sDotN > 0.0 ) {
        vec3 v = normalize(-position.xyz);
        vec3 h = normalize(v+s);
        spec = Material.Ks * pow( max( dot(h,n), 0.0 ), Material.Shininess );
    }

    return ambient + Light.L * (diffuse + spec);
}


void main() {

    FragColor = vec4(blinnphong(Position, normalize(Normal) ), 1.0);
}
