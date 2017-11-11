#version 450

in vec2 st;
uniform sampler2D tex;
out vec4 FragColor;  
  
void main()  
{  
    vec4 C = texture2D(tex, st);
	float gray = C.r*0.299 + C.g*0.587 + C.b*0.114;
	FragColor = vec4(gray, gray, gray, C.a);
}  