#version 430

in vec3 LightDir;
in vec2 TexCoord;
in vec3 ViewDir;

layout(binding=0) uniform sampler2D ColorTex;
layout(binding=1) uniform sampler2D NormalMapTex;
layout(binding=2) uniform sampler2D HeightMapTex;

uniform bool showHeightMap = false;
uniform float bumpFactor = 0.015;

uniform struct LightInfo {
    vec4 Position;  // Light position in cam. coords.
    vec3 L;         // D,S intensity
    vec3 La;        // Amb intensity
} Light ;

layout (binding = 0) uniform MaterialInfo {
    vec3 Ka;            // Ambient reflectivity
    vec3 Kd;            // Diffuse reflectivity
    vec3 Ks;            // Specular reflectivity
    float Shininess;    // Specular shininess factor
} Material;

layout( location = 0 ) out vec4 FragColor;


vec3 blinnPhong( ) {
    vec3 v = normalize(ViewDir);
    vec3 s = normalize( LightDir );


    float height = 1 - texture(HeightMapTex, TexCoord).r;
    vec2 delta = vec2(v.x, v.y) * height * bumpFactor / v.z;
    vec2 tc = TexCoord.xy - delta;
    tc = TexCoord.xy;

    vec3 n = texture(NormalMapTex, tc).xyz;
    n.xy = 2.0 * n.xy - 1.0;
    n  = normalize(n);

    float sDotN = max( dot(s,n), 0.0 );

    vec3 texColor = texture(ColorTex, tc).rgb;
    vec3 ambient = Light.La * texColor;
    vec3 diffuse = texColor * sDotN;
    vec3 spec = vec3(0.0);
    if( sDotN > 0.0 ) {
    vec3 h = normalize( v + s );
        spec = Material.Ks * pow( max( dot(h,n), 0.0 ), Material.Shininess );
    }
    return ambient + Light.L * (diffuse + spec);
}

void main() {

    if(showHeightMap){
        FragColor = vec4(texture(HeightMapTex, TexCoord ));
    }else{
        FragColor = vec4(blinnPhong(), 1.0 );
    }

}
