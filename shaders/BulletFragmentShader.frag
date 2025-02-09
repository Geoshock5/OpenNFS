in  lowp vec3 vertColour;
out lowp vec4 color;

void main(){
    color = vec4(vertColour, 1.0f);
}