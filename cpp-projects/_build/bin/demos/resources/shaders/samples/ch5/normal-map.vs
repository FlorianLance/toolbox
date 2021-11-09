#version 440

layout (location = 0) in vec3 VertexPosition;
layout (location = 1) in vec3 VertexNormal;
layout (location = 2) in vec2 VertexTexCoord;
layout (location = 3) in vec4 VertexTangent;
layout (location = 4) in ivec4 BoneIDs;
layout (location = 5) in vec4 Weights;


uniform struct LightInfo {
    vec4 Position;  // Light position in cam. coords.
    vec3 L;         // D,S intensity
    vec3 La;        // Amb intensity
} Light ;



out vec3 LightDir;
out vec2 TexCoord;
out vec3 ViewDir;

out vec3 TCol;

uniform mat4 ModelViewMatrix;
uniform mat3 NormalMatrix;
uniform mat4 ProjectionMatrix;
uniform mat4 MVP;

uniform bool animate = false;

const int MAX_BONES = 100;
uniform mat4 BonesM[MAX_BONES];

void main(){

    mat4 BoneTransform =BonesM[BoneIDs[0]] * Weights[0];
    BoneTransform += BonesM[BoneIDs[1]] * Weights[1];
    BoneTransform += BonesM[BoneIDs[2]] * Weights[2];
    BoneTransform += BonesM[BoneIDs[3]] * Weights[3];

    vec3 PosL     = (BoneTransform * vec4(VertexPosition, 1.0)).xyz;
    vec3 NormalL  = (BoneTransform * vec4(VertexNormal, 0.0)).xyz;
    vec4 TangentL = BoneTransform * VertexTangent;

    // transform normal and tangent to eye space
    vec3 norm = normalize( NormalMatrix * NormalL );
    vec3 tang = normalize( NormalMatrix * vec3(TangentL) );

    // compute the binormal
    vec3 binormal = normalize( cross( norm, tang ) ) * TangentL.w;

    // matrix for transformation to tangent space
    mat3 toObjectLocal = mat3(
        tang.x, binormal.x, norm.x,
        tang.y, binormal.y, norm.y,
        tang.z, binormal.z, norm.z ) ;

    TexCoord = VertexTexCoord;

    if(animate){
        // transform light direction and view direction to tangent space
        vec3 pos = vec3( ModelViewMatrix * vec4(PosL,1.0) );
        LightDir = toObjectLocal * (Light.Position.xyz - pos);
        ViewDir = toObjectLocal * normalize(-pos);
        gl_Position = MVP * vec4(PosL,1.0);
    }else{
    // transform light direction and view direction to tangent space
        vec3 pos = vec3( ModelViewMatrix * vec4(VertexPosition,1.0) );
        LightDir = toObjectLocal * (Light.Position.xyz - pos);
        ViewDir = toObjectLocal * normalize(-pos);
        gl_Position = MVP * vec4(VertexPosition,1.0);
    }
}


