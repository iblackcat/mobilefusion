#version 450

in vec2 st;

uniform float		scale;
uniform sampler2D	tex;

out vec4 FragColor;  
  
void main()  
{  
	vec4 Depth = texture2D(tex, st);
	vec4 Gray;
	if (Depth.a < 1.0/255.0) {Gray = vec4(0.0, 0.0, 0.0, 0.0);}
	else {
		float d = float((double(Depth.r)*255.0 *256.0*256.0 
						+ double(Depth.g)*255.0 *256.0 
						+ double(Depth.b)*255.0) 
					/ (256*256) * scale);
		Gray = vec4(d/255.0, d/255.0, d/255.0, 1.0);
	}

	FragColor = Gray;
}  