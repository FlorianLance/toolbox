#version 440

in vec3 Color;
in vec2 TexCoord;
layout( location = 0 ) out vec4 FragColor;

uniform float discardV = 0.2f;

void main() {    

    const float scale = 15.0;
    bvec2 toDiscard = greaterThan(fract(TexCoord * scale), vec2(discardV,discardV));

    if(all(toDiscard)){
        discard;
    }

    FragColor = vec4(Color, 1.0);
}
