#version 430

in vec3 Position;
in vec3 Normal;

uniform int Pass;   // Pass number

layout(binding=0) uniform sampler2D Texture0;

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

uniform float Weight[5];
vec3 blinnPhong(vec3 position, vec3 n ) {

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

vec4 pass1() {
    return vec4(blinnPhong( Position, normalize(Normal) ),1.0);
}

vec4 pass2() {
    ivec2 pix = ivec2( gl_FragCoord.xy );
    vec4 sum = texelFetch(Texture0, pix, 0) * Weight[0];
    sum += texelFetchOffset( Texture0, pix, 0, ivec2(0,1) ) * Weight[1];
    sum += texelFetchOffset( Texture0, pix, 0, ivec2(0,-1) ) * Weight[1];
    sum += texelFetchOffset( Texture0, pix, 0, ivec2(0,2) ) * Weight[2];
    sum += texelFetchOffset( Texture0, pix, 0, ivec2(0,-2) ) * Weight[2];
    sum += texelFetchOffset( Texture0, pix, 0, ivec2(0,3) ) * Weight[3];
    sum += texelFetchOffset( Texture0, pix, 0, ivec2(0,-3) ) * Weight[3];
    sum += texelFetchOffset( Texture0, pix, 0, ivec2(0,4) ) * Weight[4];
    sum += texelFetchOffset( Texture0, pix, 0, ivec2(0,-4) ) * Weight[4];
    return sum;
}

vec4 pass3() {
    ivec2 pix = ivec2( gl_FragCoord.xy );
    vec4 sum = texelFetch(Texture0, pix, 0) * Weight[0];
    sum += texelFetchOffset( Texture0, pix, 0, ivec2(1,0) ) * Weight[1];
    sum += texelFetchOffset( Texture0, pix, 0, ivec2(-1,0) ) * Weight[1];
    sum += texelFetchOffset( Texture0, pix, 0, ivec2(2,0) ) * Weight[2];
    sum += texelFetchOffset( Texture0, pix, 0, ivec2(-2,0) ) * Weight[2];
    sum += texelFetchOffset( Texture0, pix, 0, ivec2(3,0) ) * Weight[3];
    sum += texelFetchOffset( Texture0, pix, 0, ivec2(-3,0) ) * Weight[3];
    sum += texelFetchOffset( Texture0, pix, 0, ivec2(4,0) ) * Weight[4];
    sum += texelFetchOffset( Texture0, pix, 0, ivec2(-4,0) ) * Weight[4];
    return sum;
}

void main() {
    if( Pass == 1 ){
        FragColor = pass1();
    } else if( Pass == 2 ) {
        FragColor = pass2();
    }else if( Pass == 3 ) {
        FragColor = pass3();
    }
}
