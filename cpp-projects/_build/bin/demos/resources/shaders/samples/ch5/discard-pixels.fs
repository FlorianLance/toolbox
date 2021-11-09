#version 440

in vec3 Position;
in vec3 Normal;
in vec2 TexCoord;

layout (binding=0) uniform sampler2D BaseTex;
layout (binding=1) uniform sampler2D AlphaTex;

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
uniform float decay_factor;

vec3 blinnphong(vec3 position, vec3 n ) {

    vec3 texColor = texture(BaseTex, TexCoord).rgb;
    vec3 ambient = Light.La * Material.Ka * texColor;

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

    vec4 alphaMap = texture(AlphaTex, TexCoord);
    if(alphaMap.a < decay_factor){
        discard;
    }else{
        if(gl_FrontFacing){
            FragColor = vec4(blinnphong(Position, normalize(Normal) ), 1.0);
        }else{
            FragColor = vec4(blinnphong(Position, normalize(-Normal) ), 1.0);
        }
    }



//    FragColor = vec4(normalize(Normal).rgb,1);
}
