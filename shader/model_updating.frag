#version 450

in vec2 st;
out vec4 FragColor;

uniform int m_w;
uniform int m_h;
uniform int ModelSize;
uniform int ModelTexSize;
uniform int Mu;
uniform float size;
uniform int isC_flag;
uniform mat4 Q;
uniform sampler2D tex_image;  //Image
uniform sampler2D tex_depth; //Depth
uniform sampler2D model;

vec3 WorldCoord(float x, float y, float z) {
	return vec3((x - ModelSize / 2)*size / ModelSize, (y - ModelSize / 2)*size / ModelSize, (z - ModelSize / 2)*size / ModelSize);
}

void main()
{
	int SmallSize = ModelTexSize / ModelSize;

	float mdx = 1.0 / ModelTexSize;
	float stx = (st.x - mdx/2.0) * ModelTexSize;
	float sty = (st.y - mdx/2.0) * ModelTexSize;

	float k = floor(sty / ModelSize) * SmallSize + floor(stx / ModelSize);
	float i = sty - floor(sty / ModelSize) * ModelSize;
	float j = stx - floor(stx / ModelSize) * ModelSize;

	float dx = 1.0 / m_w;
	float dy = 1.0 / m_h;

	vec4 wP = vec4(WorldCoord(j, ModelSize - i - 1.0, k), 1.0);
	//vec4 cP = Rot * wP; // ?!
	vec4 vP = Q   * wP;
	float x = (vP.x / vP.z + 0.5) * dx;
	float y = (m_h - vP.y / vP.z - 1.0 + 0.5) * dy;

	float ix = (floor(vP.x / vP.z) + 0.5) * dx;
	float iy = (floor(m_h - vP.y / vP.z - 1.0) + 0.5) * dy;
	float ix_ = ix + dx;//(floor(vP.x / vP.z) + 1.5) * dx;
	float iy_ = iy + dy;//(floor(m_h - vP.y / vP.z - 1.0) + 1.5) * dy;
	vec4 d1 = texture2D(tex_depth, vec2(ix, iy));
	vec4 d2 = texture2D(tex_depth, vec2(ix, iy_));
	vec4 d3 = texture2D(tex_depth, vec2(ix_, iy));
	vec4 d4 = texture2D(tex_depth, vec2(ix_, iy_));

	//FragColor = vec4((wP.z*10.0+128.0)/255.0, (wP.z*10.0+128.0)/255.0, (wP.z*10.0+128.0)/255.0, 1.0);
	//FragColor = vec4((i)/255.0, (i)/255.0, (i)/255.0, 1.0);
	FragColor = vec4(((vP.z - 10.0) + 128.0) / 255.0, 0.0, 0.0, 1.0);//SW;
	//return;
	
	if (x < 0.0 || x > 1.0 || y < 0.0 || y > 1.0) FragColor = texture2D(model, st);//vec4(0.6, 0.5, 0.0, 1.0);//
	else if (d1.a < 1.0/255.0 || d2.a < 1.0/255.0 || d3.a < 1.0/255.0 || d4.a < 1.0/255.0) //todo: 1e-6?
		FragColor = texture2D(model, st);
	else {
		vec4 D = texture2D(tex_depth, vec2(x, y));
		float di = float((double(D.r)*255.0 *256.0*256.0 + double(D.g)*255.0 *256.0 + double(D.b)*255.0) / (256*256));
		//di = D.r*8.0;
		
		float si = (di - vP.z) * ModelSize / size;
		vec4 ci = texture2D(tex_image, vec2(x, y));
		float wi = 1.0;

		//di = D.r*8.0;
		//si = (di - 10.0);
		//if (0 == 1) {ci = vec4(0.0, 0.0, 0.0, 0.0); di = 0.0;}

		if (si <= -Mu || si > Mu) FragColor = texture2D(model, st);
		else {
			// R G B W
			if (isC_flag == 1) {
				vec4 C  = texture2D(model, st);
				float W = C.a * 255;
				// todo: W > 50 ?
				if (W < 50) {
					C.r = (C.r * W + ci.r * wi) / (W + wi);
					C.g = (C.g * W + ci.g * wi) / (W + wi);
					C.b = (C.b * W + ci.b * wi) / (W + wi);
					C.a = (W + wi) / 255;
				}
				FragColor = C;
			}
			// S x W F
			else {
				vec4 SW = texture2D(model, st);
				float S = SW.r * 255.0 - 128.0;
				float W = SW.b * 255.0;

				if (W < 50) {
					SW.r = ((S * W + si * wi) / (W + wi) + 128.0) / 255.0;
					SW.b = (W + wi) / 255;
					SW.a = 1.0 / 255;
				}
				FragColor = vec4(SW.r, 0.0, SW.b, SW.a);//SW;
			}
		}
	}
}  