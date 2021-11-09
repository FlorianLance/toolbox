#version 440

in vec3 Color;
in vec2 TexCoord;
layout( location = 0 ) out vec4 FragColor;

uniform float discardV = 0.2;
uniform float scaleV = 15.0;

void main() {    

    bvec2 toDiscard = greaterThan(fract(TexCoord * scaleV), vec2(discardV,discardV));

    if(all(toDiscard)){
        discard;
    }

    FragColor = vec4(Color, 1.0);
}
