#version 430

//struct Light {
//    vec3 Position;
//    vec3 Color;

//    float Linear;
//    float Quadratic;
//    float Radius;
//};
//const int NR_LIGHTS = 32;
//uniform Light lights[NR_LIGHTS];



in vec3 Position;
in vec3 Normal;
in vec2 TexCoord;

layout (location = 0) out vec4 FragColor;
layout (location = 1) out vec3 PositionData;
layout (location = 2) out vec3 NormalData;
layout (location = 3) out vec3 DiffuseColorData;
layout (location = 4) out vec3 AmbiantColorData;
layout (location = 5) out vec3 SpecularColorData;

layout(binding=0) uniform sampler2D PositionTex;
layout(binding=1) uniform sampler2D NormalTex;
layout(binding=2) uniform sampler2D DiffuseColorTex;
layout(binding=3) uniform sampler2D AmbiantColorTex;
layout(binding=4) uniform sampler2D SpecularColorTex;

uniform int Pass;   // Pass number
uniform int LightCount;

uniform struct LightInfo {
    vec4 Position;  // Light position in eye coords.
    vec3 L;   // D,S intensity
    vec3 La;  // Ambient
} Light[50];


uniform struct MaterialInfo {
  vec3 Ka;
  vec3 Kd;            // Diffuse reflectivity
  vec3 Ks;
  float Shininess;
} Material;



vec3 diffuseModel( vec3 pos, vec3 norm, vec3 diff ) {
    vec3 s = normalize( vec3(Light[0].Position) - pos);
    float sDotN = max( dot(s,norm), 0.0 );
    return Light[0].L * diff * sDotN;
}


vec3 blinnPhong( vec3 position, vec3 n, vec3 kd, vec3 ka, vec3 ks, int idx ) {

    vec3 ambient = Light[idx].La * ka;
    vec3 s = normalize( Light[idx].Position.xyz - position );

    vec3 l = Light[idx].Position.xyz - position;
    float dist = length(l);
    l = normalize(l);

    float sDotN = max( dot(s,n), 0.0 );
    vec3 diffuse = kd * sDotN;
    vec3 spec = vec3(0.0);
    if( sDotN > 0.0 ) {
        vec3 v = normalize(-position.xyz);
        vec3 h = normalize( v + s );
        spec = ks * pow( max( dot(h,n), 0.0 ), Material.Shininess );
    }

    vec3 res = ambient + Light[idx].L * (diffuse + spec);
    res /= (dist*dist );
    return res;
}


void pass1() {

    // Store position, normal, and diffuse color in textures
    PositionData = Position;
    NormalData = normalize(Normal);

    // Store Material
    DiffuseColorData  = Material.Kd;
    AmbiantColorData  = Material.Ka;
    SpecularColorData = Material.Ks;

    // texture(Tex1, TexCoord).rgb;
}

void pass2() {
    // Retrieve position and normal information from textures
    vec3 pos            = vec3( texture( PositionTex, TexCoord ) );
    vec3 norm           = vec3( texture( NormalTex, TexCoord ) );
    vec3 diffColor      = vec3( texture( DiffuseColorTex, TexCoord) );
    vec3 ambiantColor   = vec3( texture( AmbiantColorTex, TexCoord) );
    vec3 specularColor  = vec3( texture( SpecularColorTex, TexCoord) );

    vec3 sum = vec3(0);
    for(int ii = 0; ii < LightCount; ++ii){
        sum += blinnPhong(pos, norm, diffColor, ambiantColor, specularColor, ii).xyz;
    }

    if(norm.x == 1 && norm.y == 1 && norm.z == 1){
        FragColor = vec4(1,1,1, 1.0 );
    }else{
        FragColor = vec4(sum, 1.0);
    }

    //FragColor = vec4(blinnPhong(pos, norm, diffColor, ambiantColor, specularColor, 0).xyz, 1.0);
    //FragColor = vec4(vec3(texture(PositionTex, TexCoord)),1);
}

void main() {
    if( Pass == 1) {
        pass1();
    }else if(Pass==2) {
        pass2();
    }
}
