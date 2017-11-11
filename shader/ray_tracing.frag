#version 450

in vec2 st;
out vec4 FragColor;

uniform int m_w;
uniform int m_h;
uniform int ModelSize;
uniform int tex_size;
uniform float size;
uniform int flag;
uniform int Mu;
uniform int Axis;
uniform mat3 invQ;
uniform vec3 q;

uniform sampler2D modelC; 
uniform sampler2D modelSW;

float MAX_FLOAT = 10000000.0; 

vec3 WorldCoord(float x, float y, float z) {
	return vec3((x - ModelSize / 2)*size / ModelSize, (y - ModelSize / 2)*size / ModelSize, (z - ModelSize / 2)*size / ModelSize);
}

float WorldAxis(float tmp) {
	return (tmp - ModelSize / 2.0)*size / ModelSize;
}

float LocalAxis(float tmp) {
	return tmp * ModelSize / size + ModelSize / 2.0;
}

float vec3Multi(vec3 a, vec3 b) {
	return a.x*b.x + a.y*b.y + a.z*b.z;
}

void main()  
{
	int SmallSize = tex_size / ModelSize;

	float tmp = ModelSize;
	vec4 C, SW, last_C, last_SW;
	C = vec4(0.0, 0.0, 0.0, 0.0);
	SW = vec4(0.0, 0.0, 0.0, 0.0);
	float depth = 0.0, last_depth;
	
	float dx = 1.0 / m_w;
	float dy = 1.0 / m_h;

	float j = (st.x-dx/2) * m_w;
	float i = (st.y-dy/2) * m_h;
	

	float xx, yy, weight;
	float weight_tmp = 0.0;
	
	int start, end, step;
	if (Axis % 2 == 0) {start = 0; end = ModelSize; step = 1;}
	else {start = ModelSize-1; end = -1; step = -1;}

	float x, y, z, xm, ym, zm, test;
	for (int k = start; k != end; k += step) 
	{
		if (Axis / 2 == 0) {x = WorldAxis(float(k)); test = x;}
		else if (Axis / 2 == 1) {y = WorldAxis(float(k)); test = y;}
		else {z = WorldAxis(float(k)); test = z;}

		zm = (test + vec3Multi(invQ[Axis/2], q)) / vec3Multi(invQ[Axis/2], vec3(j, m_h - i - 1.0, 1.0));
		xm = j * zm;
		ym = (m_h - i - 1.0) * zm;

		x = vec3Multi(invQ[0], vec3(xm, ym, zm)) - vec3Multi(invQ[0], q); x = LocalAxis(x);
		y = vec3Multi(invQ[1], vec3(xm, ym, zm)) - vec3Multi(invQ[1], q); y = ModelSize - LocalAxis(y) - 1.0;
		z = vec3Multi(invQ[2], vec3(xm, ym, zm)) - vec3Multi(invQ[2], q); z = LocalAxis(z);
		
		float s_tmp = ModelSize;
		//20170531
		if (x < 0.0 || x > ModelSize-1 || y < 0.0 || y > ModelSize-1 || z < 0.0 || z > ModelSize-1) s_tmp = ModelSize;
		else {
			xx = ((int(z)%SmallSize) * ModelSize + x + 0.5) / tex_size;
			yy = ( int(z)/SmallSize  * ModelSize + y + 0.5) / tex_size;
			float xx1 = (((int(z)+1)%SmallSize) * ModelSize + x + 0.5) / tex_size;
			float yy1 = ( (int(z)+1)/SmallSize  * ModelSize + y + 0.5) / tex_size;
			float tmp_z = z - floor(z);
			last_SW = SW;
			last_C  = C ;
			SW = texture2D(modelSW, vec2(xx, yy)) * (1.0 - tmp_z) + texture2D(modelSW, vec2(xx1, yy1)) * tmp_z;
			C  = texture2D(modelC , vec2(xx, yy)) * (1.0 - tmp_z) + texture2D(modelC , vec2(xx1, yy1)) * tmp_z;

			s_tmp = SW.r*255.0 - 128.0;
			weight = SW.b*255.0;

			//vec4 p = Rot * vec4(WorldCoord(x,ModelSize-y-1.0,z), 1.0);
			last_depth = depth;
			//depth = p.z;
			depth = zm;

			if (SW.a*255.0 + 1e-6 < 1.0) {
				s_tmp = ModelSize;
			}
		}
		if ((tmp > 0.0 && s_tmp <= 0.0) && (tmp < Mu && s_tmp > -Mu)) {// && tmp < Mu && s_tmp > -Mu)) { //|| (tmp < 0.0 && s_tmp >= 0.0 && s_tmp < Mu)) {

			//if (tmp < 0.0 || weight/2 < weight_tmp || weight < 5) {tmp = s_tmp; continue; }
			if (flag == 0) { //I
				if (tmp != ModelSize && s_tmp != 0.0) {
					FragColor = (vec4(C.rgb, 1.0)*(-tmp) + vec4(last_C.rgb, 1.0)*(s_tmp)) / (s_tmp-tmp);
				}
				else FragColor = vec4(C.rgb, 1.0);
			}
			else if (flag == 1) { //Y
				if (tmp != ModelSize && s_tmp != 0.0) {
					FragColor = (vec4(SW.b, SW.b, SW.b, 1.0)*(-tmp) + vec4(last_SW.b, last_SW.b, last_SW.b, 1.0)*(s_tmp)) / (s_tmp-tmp);
				}
				else FragColor = vec4(SW.b, SW.b, SW.b, 1.0);
			}
			else { //D
				float dinter = depth;
				if (tmp != ModelSize && s_tmp != 0.0) {
					dinter = (depth*(-tmp) + last_depth*(s_tmp) ) / (s_tmp - tmp);
				}
				int test = int(dinter * 256 * 256);
				float A = float(1.0) / 255.0;
				float B = float((test) % 256) / 255.0;
				float G = float((test/ 256) % 256) / 255.0;
				float R = float(test / 256 / 256) / 255.0;
				FragColor = vec4(R, G, B, A);
			}
			weight_tmp = weight;
			return ;
		} /*else if (tmp < 0.0 && s_tmp >= 0.0 && s_tmp < Mu) {
			return ;
		}*/
		tmp = s_tmp;
	}
}  