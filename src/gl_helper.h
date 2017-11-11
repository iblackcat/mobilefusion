#ifndef GL_HELPER_H
#define GL_HELPER_H

#include <stdio.h>
#include <type_traits>
#include <glew.h>
#include <freeglut.h>

namespace jhw_gl {

class GLProgram {
public:
	GLProgram() {
		m_programID = 0;
	}
	~GLProgram() {
		DeleteProgram();
	}
	bool CreateProgram() {
		m_programID = glCreateProgram();
		if (m_programID == 0) {
			printf("Error creating shader program\n");
			return false;
		}
		return true;
	}
	GLuint GetUniformLocation(char* name) {
		return glGetUniformLocation(m_programID, name);
	}
	bool DeleteProgram() {
		glDeleteProgram(m_programID);
		m_programID = 0;
		return true;
	}
	bool UseProgram() {
		glUseProgram(m_programID);
		return true;
	}
	bool DisuseProgram() {
		glUseProgram(0);
		return true;
	}
	GLuint GetProgramID() {
		return m_programID;
	}

	bool Compile() {
		GLint Success = 0;
		GLchar ErrorLog[1024] = { 0 };
		glLinkProgram(m_programID);
		glGetProgramiv(m_programID, GL_LINK_STATUS, &Success);
		if (Success == 0) {
			glGetProgramInfoLog(m_programID, sizeof(ErrorLog), NULL, ErrorLog);
			printf("Error linking shader program: '%s'\n", ErrorLog);
			return false;
		}
		// 检查验证在当前的管线状态程序是否可以被执行
		glValidateProgram(m_programID);
		glGetProgramiv(m_programID, GL_VALIDATE_STATUS, &Success);
		if (!Success) {
			glGetProgramInfoLog(m_programID, sizeof(ErrorLog), NULL, ErrorLog);
			printf("Invalid shader program: '%s'\n", ErrorLog);
			return false;
		}
		// 设置到管线声明中来使用上面成功建立的shader程序
		glUseProgram(m_programID);
		return true;
	}

	bool UseShaders(const char *vert_shader, const char *frag_shader) {
		if (m_programID == 0) m_programID = glCreateProgram();

		m_vertex_shaderID = glCreateShader(GL_VERTEX_SHADER);
		m_fragment_shaderID = glCreateShader(GL_FRAGMENT_SHADER);

		if (m_vertex_shaderID == 0 || m_fragment_shaderID == 0) {
			printf("Error creating shader.\n");
			return false;
		}
		glShaderSource(m_vertex_shaderID, 1, (const char**)&vert_shader, NULL);
		glShaderSource(m_fragment_shaderID, 1, (const char**)&frag_shader, NULL);
		glCompileShader(m_vertex_shaderID);// 编译shader对象
		glCompileShader(m_fragment_shaderID);// 编译shader对象


		GLint success;
		glGetShaderiv(m_vertex_shaderID, GL_COMPILE_STATUS, &success);
		if (!success) {
			GLchar InfoLog[1024];
			glGetShaderInfoLog(m_vertex_shaderID, 1024, NULL, InfoLog);
			printf("Error compiling 'vertex shader': '%s'\n", InfoLog);
			return false;
		}
		glGetShaderiv(m_fragment_shaderID, GL_COMPILE_STATUS, &success);
		if (!success) {
			GLchar InfoLog[1024];
			glGetShaderInfoLog(m_fragment_shaderID, 1024, NULL, InfoLog);
			printf("Error compiling 'fragment shader': '%s'\n", InfoLog);
			return false;
		}

		glAttachShader(m_programID, m_vertex_shaderID);
		glAttachShader(m_programID, m_fragment_shaderID);

		return Compile();
	}

	const char* ReadShaderFile(const char* filepath) {
		FILE *fp = fopen(filepath, "rb");
		if (!fp) return NULL;
		fseek(fp, 0, SEEK_END);
		int size = ftell(fp);
		fseek(fp, 0, SEEK_SET);
		char *content = (char*)malloc(size + 1);
		fread(content, sizeof(char), size, fp);
		content[size] = '\0';
		fclose(fp);
		return (const char*)content;
	}

	bool UseShaders_file(const char *vert_path, const char *frag_path) {
		const char *vert_shader = ReadShaderFile(vert_path);
		const char *frag_shader = ReadShaderFile(frag_path);

		return UseShaders(vert_shader, frag_shader);
	}

private:
	GLuint m_programID;
	GLuint m_vertex_shaderID;
	GLuint m_fragment_shaderID;
};

class GLRTT {
	/*
	1. setFrameSize
	2. creatFrameBuffer
	3. creatColorBuffer
	4. creatDepthBuffer
	5. bindFrameBuffer
	6. draw.......
	7. unbindFrameBuffer
	8. getColorData
	using glCheckFramebufferStatus to check the status of present frame buffer.
	*/
public:
	GLRTT() : m_width(0), m_height(0), m_frameBuffer(0u), m_colorBuffer(0u), m_depthBuffer(0u) {}
	~GLRTT() {
		DeleteRTT();
	}
	void CreateRTT(int width, int height, GLuint internalformat, GLuint inputformat = GL_RGB) {
		SetFrameSize(width, height);
		glGenFramebuffers(1, &m_frameBuffer);
		CreateColorBuffer(internalformat, inputformat);
		CreateDepthBuffer();

		/*SetFrameSize(width, height);
		CreatFrameBuffer();
		CreatColorBuffer(internalformat);
		CreatDepthBuffer();*/
	}
	void UseRTT() {
		glBindFramebuffer(GL_FRAMEBUFFER, m_frameBuffer);
		glViewport(0, 0, m_width, m_height);

		/*GLenum DrawBuffers[1] = { GL_COLOR_ATTACHMENT0 };
		glDrawBuffers(1, DrawBuffers);
		if (glCheckFramebufferStatus(GL_FRAMEBUFFER) != GL_FRAMEBUFFER_COMPLETE) {
		printf("Wrong drawing framebuffer.\n");
		}*/
	}
	void DeleteRTT() {
		if (m_frameBuffer) {
			glDeleteFramebuffers(1, &m_frameBuffer);
			m_frameBuffer = 0u;
		}
		if (m_colorBuffer) {
			glDeleteTextures(1, &m_colorBuffer);
			m_colorBuffer = 0u;
		}
		if (m_depthBuffer) {
			glDeleteRenderbuffers(1, &m_depthBuffer);
			m_depthBuffer = 0u;
		}

	}
	void SetFrameSize(int _width, int _height) {
		m_width = _width;
		m_height = _height;
	}
	void BindFrameBuffer() {
		glBindFramebuffer(GL_FRAMEBUFFER, m_frameBuffer);
	}
	void UnbindFrameBuffer() {
		glBindFramebuffer(GL_FRAMEBUFFER, 0u);
	}
	void CreateFrameBuffer() {
		glGenFramebuffers(1, &m_frameBuffer);
	}
	//internalFormat, GL_RGBA8
	bool CreateColorBuffer(GLuint internalFormat, GLuint inputformat) {
		if (m_width == 0 || m_height == 0 || m_frameBuffer == 0u)
			return false;
		//creat color buffer
		glGenTextures(1, &m_colorBuffer);
		glBindTexture(GL_TEXTURE_2D, m_colorBuffer);
		glTexImage2D(GL_TEXTURE_2D, 0, internalFormat, m_width, m_height, 0, inputformat, GL_UNSIGNED_BYTE, 0);
		glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
		glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
		glBindTexture(GL_TEXTURE_2D, 0u);
		//attach color buffer
		BindFrameBuffer();
		glFramebufferTexture(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, m_colorBuffer, 0);
		UnbindFrameBuffer();

		return true;
	}
	bool CreateDepthBuffer() {
		if (m_width == 0 || m_height == 0 || m_frameBuffer == 0u)
			return false;
		//creat depth buffer
		glGenRenderbuffers(1, &m_depthBuffer);
		glBindRenderbuffer(GL_RENDERBUFFER, m_depthBuffer);
		glRenderbufferStorage(GL_RENDERBUFFER, GL_DEPTH_COMPONENT, m_width, m_height);
		glBindRenderbuffer(GL_RENDERBUFFER, 0u);
		//attach depth buffer
		BindFrameBuffer();
		glFramebufferRenderbuffer(GL_FRAMEBUFFER, GL_DEPTH_ATTACHMENT, GL_RENDERBUFFER, m_depthBuffer);
		UnbindFrameBuffer();

		return true;
	}
	//external format
	void GetColorData(GLvoid* pixels, GLenum format, GLenum type) {
		glBindTexture(GL_TEXTURE_2D, m_colorBuffer);
		glGetTexImage(GL_TEXTURE_2D, 0, format, type, pixels);
		glBindTexture(GL_TEXTURE_2D, 0u);
	}
	GLuint GetFrameBuffer() {
		return m_frameBuffer;
	}
	GLuint GetColorBuffer() {
		return m_colorBuffer;
	}
	GLuint GetDepthBuffer() {
		return m_depthBuffer;
	}
	void GetFrameSize(int &_width, int &_height) {
		_width = m_width;
		_height = m_height;
	}
private:
	int            m_width;
	int            m_height;
	GLuint         m_frameBuffer;
	GLuint         m_colorBuffer;
	GLuint         m_depthBuffer;
};


class GLTex2d {
public:
	GLTex2d() :
		m_w(0),
		m_h(0),
		tex_id(0),
		format(0),
		type(0)
	{ };
	GLTex2d(int w, int h, GLuint id, GLenum f, GLenum t) :
		m_w(w),
		m_h(h),
		tex_id(id),
		format(f),
		type(t)
	{ };

	int			m_w;
	int			m_h;
	GLuint		tex_id;
	GLenum		format;
	GLenum		type;
};

class GLRenderer {
public:
	/*
	0. create opengl context
	1. initiate
	2. setup program and shader
	3. create vbo
	4. create textures if need
	5. render 
	changing vertices and add ebo by inherit this class.
	*/

	bool init(int w, int h, int rtt_channel = 4, GLenum rtt_data_type = GL_UNSIGNED_BYTE) {
		switch (rtt_channel) {
		case 1: m_rtt_type = GL_R; break;
		case 3: m_rtt_type = GL_RGB; break;
		case 4: m_rtt_type = GL_RGBA; break;
		default: return false;
		}
		m_rtt_channel = rtt_channel;
		m_rtt_data_type = rtt_data_type;

		m_width = w;
		m_height = h;

		m_vertice = 6;
		m_triangle = 2;
		vertices = (float*)malloc(sizeof(float)*m_vertice * 5);
		vertices[0] =-1.0f; vertices[1] = 1.0f; vertices[2] = 0.0f;
		vertices[3] = 1.0f; vertices[4] = 1.0f; vertices[5] = 0.0f;
		vertices[6] =-1.0f; vertices[7] =-1.0f; vertices[8] = 0.0f;

		vertices[9] = 1.0f; vertices[10]= 1.0f; vertices[11]= 0.0f;
		vertices[12]= 1.0f; vertices[13]=-1.0f; vertices[14]= 0.0f;
		vertices[15]=-1.0f; vertices[16]=-1.0f; vertices[17]= 0.0f;

		vertices[18]= 0.0f; vertices[19]= 1.0f;
		vertices[20]= 1.0f; vertices[21]= 1.0f;
		vertices[22]= 0.0f; vertices[23]= 0.0f;

		vertices[24]= 1.0f; vertices[25]= 1.0f;
		vertices[26]= 1.0f; vertices[27]= 0.0f;
		vertices[28]= 0.0f; vertices[29]= 0.0f;
		
		glutReshapeWindow(w, h);
		glClearColor(0.0f, 0.0f, 0.0f, 0.0f);

		m_rtt.CreateRTT(w, h, m_rtt_type, m_rtt_type);
		m_program.CreateProgram();
		return true;
	}

	bool destroy() {
		m_rtt.DeleteRTT();
		m_program.DeleteProgram();
		if (!vertices) {
			free(vertices);
			vertices = NULL;
		}
		return true;
	}
	
	bool setShader(const char *vert_shader, const char *frag_shader) {
		return m_program.UseShaders(vert_shader, frag_shader);
	}
	bool setShaderFile(const char *vert_path, const char *frag_path) {
		return m_program.UseShaders_file(vert_path, frag_path);
	}
	
	void CreateVertexBuffer() {
		glGenBuffers(1, &m_vertexBuffer);
		glBindBuffer(GL_ARRAY_BUFFER, m_vertexBuffer);
		glBufferData(GL_ARRAY_BUFFER, sizeof(float)*m_vertice * 5, vertices, GL_STATIC_DRAW);
	}
	
	GLTex2d CreateTexture(GLuint *tex_id, int w, int h, GLenum image_type = GL_RGBA, GLenum image_format = GL_UNSIGNED_BYTE) {
		glGenTextures(1, tex_id);
		glBindTexture(GL_TEXTURE_2D, *tex_id);
		glTexImage2D(GL_TEXTURE_2D, 0, image_type, w, h, 0, image_type, image_format, NULL);
		glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
		glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
		glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_BORDER); // use const edge color
		glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_BORDER);
		glBindTexture(GL_TEXTURE_2D, 0u);
		return GLTex2d(w, h, *tex_id, image_format, image_type);
	}
	
	bool useRenderer() {
		glutReshapeWindow(m_width, m_height);
		m_program.UseProgram();
		m_rtt.UseRTT();

		return true;
	}
	
	//this type must correspond to rtt_data_type
	template<typename outT>
	outT *RenderScence() {
		glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT | GL_STENCIL_BUFFER_BIT);

		glEnableVertexAttribArray(0);
		glEnableVertexAttribArray(1);
		glBindBuffer(GL_ARRAY_BUFFER, m_vertexBuffer);
		m_rtt.UseRTT();

		glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 0, 0);
		glVertexAttribPointer(1, 2, GL_FLOAT, GL_FALSE, 0, (const GLvoid*)(m_vertice * 3 * sizeof(float)));

		glDrawArrays(GL_TRIANGLES, 0, 3 * m_triangle);

		glDisableVertexAttribArray(0);
		glDisableVertexAttribArray(1);

		glBindBuffer(GL_ARRAY_BUFFER, 0);
		outT *new_data = (outT*)malloc(sizeof(outT) * m_width * m_height * m_rtt_channel);
		m_rtt.GetColorData(new_data, m_rtt_type, m_rtt_data_type);
		return new_data;
	}

protected:
	int			m_width;
	int			m_height;
	int			m_vertice;
	int			m_triangle;
	int			m_rtt_channel;
	GLuint		m_vertexBuffer;
	GLenum		m_rtt_type;
	GLenum		m_rtt_data_type;
	GLRTT		m_rtt;
	GLProgram	m_program;
	float*		vertices;
};


class MyGLRenderer : public GLRenderer {
public:
	template<typename T>
	bool setUniform1(const char *uniform_name, T uniform_value) {
		int uniformLocation = glGetUniformLocation(m_program.GetProgramID(), uniform_name);
		if (uniformLocation == -1) return false;

		if(std::is_same<T, int>::value || std::is_same<T, GLint>::value) 
			glUniform1i(uniformLocation, uniform_value);
		else if (std::is_same<T, float>::value || std::is_same<T, GLfloat>::value) 
			glUniform1f(uniformLocation, uniform_value);
		else if (std::is_same<T, double>::value || std::is_same<T, GLdouble>::value) 
			glUniform1d(uniformLocation, uniform_value);
		else return false;

		return true;
	}
	template<typename T>
	bool setUniform3(const char *uniform_name, T value1, T value2, T value3) {
		int uniformLocation = glGetUniformLocation(m_program.GetProgramID(), uniform_name);
		if (uniformLocation == -1) return false;

		if(std::is_same<T, int>::value || std::is_same<T, GLint>::value) 
			glUniform3i(uniformLocation, value1, value2, value3);
		else if (std::is_same<T, float>::value || std::is_same<T, GLfloat>::value) 
			glUniform3f(uniformLocation, value1, value2, value3);
		else if (std::is_same<T, double>::value || std::is_same<T, GLdouble>::value) 
			glUniform3d(uniformLocation, value1, value2, value3);
		else return false;

		return true;
	}
	template<typename T>
	bool setUniform4(const char *uniform_name, T value1, T value2, T value3, T value4) {
		int uniformLocation = glGetUniformLocation(m_program.GetProgramID(), uniform_name);
		if (uniformLocation == -1) return false;

		if (std::is_same<T, int>::value || std::is_same<T, GLint>::value) 
			glUniform4i(uniformLocation, value1, value2, value3, value4);
		else if (std::is_same<T, float>::value || std::is_same<T, GLfloat>::value) 
			glUniform4f(uniformLocation, value1, value2, value3, value4);
		else if (std::is_same<T, double>::value || std::is_same<T, GLdouble>::value) 
			glUniform4d(uniformLocation, value1, value2, value3, value4);
		else return false;

		return true;
	}
	template<typename T>
	bool setUniform4v(const char *uniform_name, T *value) {
		int uniformLocation = glGetUniformLocation(m_program.GetProgramID(), uniform_name);
		if (uniformLocation == -1) return false;

		if (std::is_same<T, float>::value || std::is_same<T, GLfloat>::value)
			glUniformMatrix4fv(uniformLocation, 1, GL_FALSE, (const GLfloat *)value);
		else if (std::is_same<T, double>::value || std::is_same<T, GLdouble>::value)
			glUniformMatrix4dv(uniformLocation, 1, GL_FALSE, (const GLdouble *)value);
		else return false;

		return true;
	}
	template<typename T>
	bool setUniform3v(const char *uniform_name, T *value) {
		int uniformLocation = glGetUniformLocation(m_program.GetProgramID(), uniform_name);
		if (uniformLocation == -1) return false;

		if (std::is_same<T, float>::value || std::is_same<T, GLfloat>::value)
			glUniformMatrix3fv(uniformLocation, 1, GL_FALSE, (const GLfloat *)value);
		else if (std::is_same<T, double>::value || std::is_same<T, GLdouble>::value)
			glUniformMatrix3dv(uniformLocation, 1, GL_FALSE, (const GLdouble *)value);
		else return false;

		return true;
	}

	template<typename T>
	bool setTexSub2D(const char *tex_name, GLTex2d tex, int num, GLenum texture_num, T *value) {
		glActiveTexture(texture_num);
		glBindTexture(GL_TEXTURE_2D, tex.tex_id);
		glTexSubImage2D(GL_TEXTURE_2D, 0, 0, 0, tex.m_w, tex.m_h, tex.type, tex.format, value);

		int texLocation = glGetUniformLocation(m_program.GetProgramID(), tex_name);
		glUniform1i(texLocation, num);
		//glBindTexture(GL_TEXTURE_2D, 0u);
		return true;
	}
};

//create GL context
bool GLInit(int w, int h, const char * title_name = "title");

} //namespace jhw_gl

#endif //GL_HELPER_H
