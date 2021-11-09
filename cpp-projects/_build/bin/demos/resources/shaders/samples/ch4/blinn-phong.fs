#version 440

in vec3 Position;
in vec3 Normal;

layout (binding = 0) uniform LightInfo {
  vec4 Position; // Light position in eye coords.
  vec3 La;       // Ambient light intensity
  vec3 Ld;       // Diffuse light intensity
  vec3 Ls;       // Specular light intensity
} Light;


layout (binding = 1) uniform MaterialInfo {
  vec3 Ka;            // Ambient reflectivity
  vec3 Kd;            // Diffuse reflectivity
  vec3 Ks;            // Specular reflectivity
  float Shininess;    // Specular shininess factor
} Material;


layout( location = 0 ) out vec4 FragColor;

vec3 blinnphong(vec3 position, vec3 n ) {

    vec3 ambient = Light.La * Material.Ka;
    vec3 s = normalize(Light.Position.xyz - position);
    float sDotN = max (dot(s,n), 0.0);
    vec3 diffuse = Light.Ld * Material.Kd * sDotN;
    vec3 spec = vec3(0.0);
    if( sDotN > 0.0 ) {
        vec3 v = normalize(-position.xyz);
        vec3 h = normalize(v+s);
        spec = Light.Ls * Material.Ks * pow( max( dot(h,n), 0.0 ), Material.Shininess );
    }

    return ambient + diffuse + spec;
}


void main() {

    FragColor = vec4(blinnphong(Position, Normal ), 1.0);
}
