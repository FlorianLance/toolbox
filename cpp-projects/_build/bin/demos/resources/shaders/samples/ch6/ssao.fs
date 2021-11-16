#version 430

in vec3 Position;
in vec3 Normal;
in vec2 TexCoord;

layout (location = 0) out vec4 FragColor;
layout (location = 1) out vec3 PositionData;
layout (location = 2) out vec3 NormalData;
layout (location = 3) out vec3 ColorData;
layout (location = 4) out float AoData;

uniform mat4 ProjectionMatrix;

const int kernelSize = 64;

uniform bool doBlurPass = true;

uniform int Pass;   // Pass number
uniform vec3 SampleKernel[kernelSize];
uniform float Radius = 0.55;

uniform struct LightInfo {
    vec4 Position;  // Light position in eye coords.
    vec3 L;   // D,S intensity
    vec3 La;  // Ambient
} Light ;

uniform struct MaterialInfo {
    vec3 Kd;            // Diffuse reflectivity
    bool UseTex;        // Use texture
} Material;

layout(binding=0) uniform sampler2D PositionTex;
layout(binding=1) uniform sampler2D NormalTex;
layout(binding=2) uniform sampler2D ColorTex;
layout(binding=3) uniform sampler2D AoTex;
layout(binding=4) uniform sampler2D RandTex;
layout(binding=5) uniform sampler2D DiffTex;


uniform vec2 randScale = vec2( 800.0 / 4.0, 600.0 / 4.0 ); // screen = 800x600
// tile noise texture over screen, based on screen dimensions divided by noise size
// const vec2 randScale = vec2( 800.0 / 4.0, 600.0 / 4.0 ); // screen = 800x600
//const vec2 randScale = vec2( 2560.0 / 4.0, 1444.0 / 4.0 ); // screen = 800x600

vec3 ambAndDiffuse( vec3 pos, vec3 norm, vec3 diff, float ao ) {
    ao = pow(ao, 4);
    vec3 ambient = Light.La * diff * ao;
    vec3 s = normalize( vec3(Light.Position) - pos);
    float sDotN = max( dot(s,norm), 0.0 );
    return ambient + Light.L * diff * sDotN;
}

void pass1() {
    // Store position, normal, and diffuse color in textures
    PositionData = Position;
    NormalData   = normalize(Normal);
    if( Material.UseTex ) {
        ColorData = pow( texture(DiffTex, TexCoord.xy).rgb, vec3(2.2) );
    } else {
        ColorData = Material.Kd;
    }
}

// SSAO pass
void pass2() {

    // Create the random tangent space matrix
    vec3 randDir = normalize( texture(RandTex, TexCoord.xy * randScale).xyz );
    vec3 normal = normalize( texture(NormalTex, TexCoord).xyz );
    vec3 biTangent = cross( normal, randDir );
    if( length(biTangent) < 0.0001 ){  // If n and randDir are parallel, n is in x-y plane
        biTangent = cross( normal, vec3(0,0,1));
    }
    biTangent = normalize(biTangent);

    vec3 tang = cross(biTangent, normal);
    mat3 toCamSpace = mat3(tang, biTangent, normal);

    float occlusionSum = 0.0;
    vec3 camPos = texture(PositionTex, TexCoord).xyz;

    for( int i = 0; i < kernelSize; i++ ) {

        vec3 samplePos = camPos + Radius * (toCamSpace * SampleKernel[i]);

        // Project point
        vec4 p = ProjectionMatrix * vec4(samplePos,1);
        p *= 1.0 / p.w;
        p.xyz = p.xyz * 0.5 + 0.5;

        // Access camera space z-coordinate at that point
        float surfaceZ = texture(PositionTex, p.xy).z;
        float zDist = surfaceZ - camPos.z;

        // Count points that ARE occluded
        if( zDist >= 0.0 && zDist <= Radius && surfaceZ > samplePos.z ){
            occlusionSum += 1.0;
        }

    }

    float occ = occlusionSum / kernelSize;
    AoData = 1.0 - occ;
    //FragColor = vec4(AoData, AoData, AoData, 1);
}


// Blur pass
void pass3() {

    ivec2 pix = ivec2( gl_FragCoord.xy );
    if(doBlurPass){
        float sum = 0.0;
        for( int x = -1; x <= 1; ++x ) {
            for( int y = -1; y <= 1; y++ ) {
                sum += texelFetchOffset( AoTex, pix, 0, ivec2(x,y) ).r;
            }
        }
        AoData = sum * (1.0 / 9.0);
    }else{
        AoData = texelFetchOffset( AoTex, pix, 0, ivec2(0,0) ).r;
    }

    // FragColor = vec4(ao, ao, ao, 1);
}

// Final color pass
void pass4() {
    // Retrieve position and normal information from textures
    vec3 pos        = texture(PositionTex, TexCoord).xyz;
    vec3 norm       = texture(NormalTex,   TexCoord).xyz;
    vec3 diffColor  = texture(ColorTex,    TexCoord).rgb;
    float aoVal     = texture(AoTex,       TexCoord).r;

    vec3 col = ambAndDiffuse(pos, norm, diffColor, aoVal);
    col = pow(col, vec3(1.0/2.2));

    if(norm.x == 1 && norm.y == 1 && norm.z == 1){
        FragColor = vec4(1,1,1, 1.0 );
    }else{
        FragColor = vec4(col, 1.0 );
        //FragColor = vec4(aoVal,aoVal,aoVal, 1.0 );
    }
}

void main() {
    if( Pass == 1)      pass1();
    else if(Pass==2)    pass2();
    else if(Pass == 3)  pass3();
    else if(Pass == 4)  pass4();
}
