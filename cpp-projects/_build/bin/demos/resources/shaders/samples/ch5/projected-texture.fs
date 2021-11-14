#version 430

in vec3 EyeNormal;     // normal in eye coordinates
in vec4 EyePosition;   // position in eye coordinates
in vec4 ProjTexCoord;

layout(binding=0) uniform sampler2D ProjectorTex;


layout( location = 0 ) out vec4 FragColor;

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

vec3 blinnphong(vec3 position, vec3 n ) {

    vec3 ambient = Light.La * Material.Ka;
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

    vec3 color = blinnphong(EyePosition.xyz, normalize(EyeNormal));

    vec3 projTexColor = vec3(0.0);
    if(ProjTexCoord.z > 0.0){
        projTexColor = textureProj(ProjectorTex, ProjTexCoord).rgb;
    }

    FragColor = vec4(color + projTexColor * 0.5, 1);
    //FragColor = ProjTexCoord;//vec4(color + projTexColor * 0.5, 1);
//    FragColor = vec4(projTexColor, 1);


}
