#version 450

in vec2 st;

out vec4 FragColor;

void main() {
	FragColor = vec4(st.x, st.y, 0.0, 1.0);
	return ;
}