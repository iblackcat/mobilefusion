#include "gl_helper.h" 

namespace jhw_gl {

bool GLInit(int w, int h, const char * title_name) {
	int argc = 0;
	char **argv = NULL;
	glutInit(&argc, argv);
	glutInitDisplayMode(GLUT_DOUBLE | GLUT_RGBA);

	glutInitWindowSize(w, h);      // 窗口尺寸
	glutInitWindowPosition(100, 100);  // 窗口位置
	glutCreateWindow(title_name);   // 窗口标题

								 // 开始渲染

								 // 检查GLEW是否就绪，必须要在GLUT初始化之后！
	GLenum res = glewInit();
	if (res != GLEW_OK) {
		printf("Error: '%s'\n", glewGetErrorString(res));
		return false;
	}
	return true;
}

} //namespace jhw_gl