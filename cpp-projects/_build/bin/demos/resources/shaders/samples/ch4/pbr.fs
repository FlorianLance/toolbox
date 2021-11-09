#version 440

in vec3 Position;
in vec3 Normal;
in vec2 TexCoords;

layout( location = 0 ) out vec4 FragColor;

struct LightInfo{
    vec4 Position; // Light position in cam coords
    vec3 La;
    vec3 Ld;
    vec3 Ls;
};

struct MaterialInfo{
    vec4 Color;
    float Rough;
    float Metal;
};

layout (std140, binding = 0) uniform LightsInfos {
    LightInfo lights[3];
};

layout (std140, binding = 1) uniform MaterialsInfos {
    MaterialInfo materials[1];
};

uniform int id = 0;

const float PI = 3.1415926535897932384626433832795;
float ggxDistribution( float nDotH ) {
    float alpha2 = materials[0].Rough * materials[0].Rough * materials[0].Rough * materials[0].Rough;
    float d = (nDotH * nDotH) * (alpha2 - 1) + 1;
    return alpha2 / (PI * d * d);
}

float geomSmith( float dotProd ) {
    float k = (materials[0].Rough + 1.0) * (materials[0].Rough + 1.0) / 8.0;
    float denom = dotProd * (1 - k) + k;
    return 1.0 / denom;
}

vec3 schlickFresnel( float lDotH ) {
    vec3 f0 = vec3(0.04);
    f0 = materials[0].Metal * materials[0].Color.rgb;
    return f0 + (1 - f0) * pow(1.0 - lDotH, 5);
}

vec3 microfacetModel( int lightIdx, vec3 position, vec3 n ) {

    vec3 diffuseBrdf = vec3(0.0);  // Metallic
    diffuseBrdf = (1 - materials[0].Metal) * materials[0].Color.rgb;

    vec3 l = vec3(0.0),
    lightI = lights[lightIdx].La;
    if( lights[lightIdx].Position.w == 0.0 ) { // Directional light
        l = normalize(lights[lightIdx].Position.xyz);
    } else {                                  // Positional light
        l = lights[lightIdx].Position.xyz - position;
        float dist = length(l);
        l = normalize(l);
        lightI /= (dist * dist);
    }

    vec3 v = normalize( -position );
    vec3 h = normalize( v + l );
    float nDotH = dot( n, h );
    float lDotH = dot( l, h );
    float nDotL = max( dot( n, l ), 0.0 );
    float nDotV = dot( n, v );
    vec3 specBrdf = 0.25 * ggxDistribution(nDotH) * schlickFresnel(lDotH) * geomSmith(nDotL) * geomSmith(nDotV);

    return (diffuseBrdf + PI * specBrdf) * lightI * nDotL;
}

void main() {

    vec3 sum = vec3(0);
    vec3 n = normalize(Normal);
    for(int ii = 0; ii < 3; ++ii){
        sum += microfacetModel(ii, Position, n);
    }

    // Gamma
    sum = pow(sum, vec3(1.0/2.2));

    FragColor = vec4(sum, 1.0);
}


