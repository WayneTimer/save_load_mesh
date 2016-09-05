#include <GL/gl.h>
#include <GL/glu.h>
#include <GL/glut.h>
#include <cstdio>
#include <cmath>

#define WIDTH 752
#define HEIGHT 480

char file_path[] = "/home/timer/catkin_ws/ply_points.txt";
GLuint meshes;
FILE *file;

// Draw triangles
static void triangles()
{
    file = fopen(file_path,"r");

    double x,y,z,r,g,b;
    int cnt = 0;
    glBegin(GL_TRIANGLES);
    while (fscanf(file,"%lf %lf %lf %lf %lf %lf",&x,&y,&z,&r,&g,&b)==6)
    {
        glColor3f(r,g,b);
        glVertex3f(x,y,z);
        cnt++;
    }
    glEnd();
    fclose(file);
    printf("total points = %d\n",cnt);
}

// Create display list with triangles(meshes) and initialize state
static void init()
{
    meshes = glGenLists(1);
    glNewList(meshes, GL_COMPILE);
    triangles();
    glEndList();

    glShadeModel(GL_FLAT);
    glClearColor(0.0, 0.0, 0.0, 0.0);
}

void display()
{
    glClear(GL_COLOR_BUFFER_BIT);
    glColor3f(1.0,1.0,1.0);
    glCallList(meshes);
    glFlush();
}

void reshape(int w, int h)
{
    glViewport(0, 0, (GLsizei) w, (GLsizei) h);
    printf("w=%d h=%d\n",w,h);
    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
   
    double FOV = atan2(1.0,(1.330001636*2))*2;
    FOV = FOV*180.0/M_PI; // tan(FOV/2) = h/(2*zNear)
    printf("FOV = %lf, PI = %lf\n",FOV,M_PI);
    FOV = 55; // 90
    gluPerspective(FOV, (GLfloat) w/(GLfloat) h, 1.0, 300.0);  // FOV of the height (tan (FOV/2)=(height-c_y)/f_y)

    glMatrixMode(GL_MODELVIEW);
    glLoadIdentity();
    gluLookAt(0, 0, 0, 0, 0, 1, 0, -1, 0);    //  last three (, shangxia, )
}

/* Rotate about x-axis when "x" typed; rotate about y-axis
   when "y" typed; "i" returns torus to original view */
void keyboard(unsigned char key, int x, int y)
{
    switch (key)
    {
        case 'z':
            glRotatef(0.3,1.0,0.0,0.0);
            glutPostRedisplay();
            break;
       case 'Z':
            glRotatef(3.,1.0,0.0,0.0);
            glutPostRedisplay();
            break;
       case 'c':
            glRotatef(0.3,-1.0,0.0,0.0);
            glutPostRedisplay();
            break;
       case 'C':
            glRotatef(3.,-1.0,0.0,0.0);
            glutPostRedisplay();
            break;
       case 'q':
            glRotatef(0.3,0.0,1.0,0.0);
            glutPostRedisplay();
            break;
       case 'Q':
            glRotatef(3.,0.0,1.0,0.0);
            glutPostRedisplay();
            break;
       case 'e':
            glRotatef(0.3,0.0,-1.0,0.0);
            glutPostRedisplay();
            break;
       case 'E':
            glRotatef(3.,0.0,-1.0,0.0);
            glutPostRedisplay();
            break;
       case 'i':
       case 'I':
            glLoadIdentity();
            gluLookAt(0, 0, 0, 0, 0, 1, 0, -1, 0);
            glutPostRedisplay();
            break;
       case 'w':
            glTranslated(0, 0, -0.01);
            glutPostRedisplay();
            break;
       case 'W':
            glTranslated(0, 0, -0.1);
            glutPostRedisplay();
            break;
       case 's':
            glTranslated(0, 0, 0.01);
            glutPostRedisplay();
            break;
       case 'S':
            glTranslated(0, 0, 0.1);
            glutPostRedisplay();
            break;
       case 'a':
            glTranslated(0.01, 0, 0);
            glutPostRedisplay();
            break;
       case 'A':
            glTranslated(0.1, 0, 0);
            glutPostRedisplay();
            break;
       case 'd':
            glTranslated(-0.01, 0, 0);
            glutPostRedisplay();
            break;
       case 'D':
            glTranslated(-0.1, 0, 0);
            glutPostRedisplay();
            break;
       case 27:  // key: 'ESC'
            exit(0);
            break;
    }
}

int main(int argc, char **argv)
{
    glutInitWindowSize(WIDTH,HEIGHT);
    glutInit(&argc, argv);
    glutInitDisplayMode(GLUT_SINGLE | GLUT_RGB);
    glutCreateWindow(argv[0]);
    init();
    glutReshapeFunc(reshape);
    glutKeyboardFunc(keyboard);
    glutDisplayFunc(display);
    glutMainLoop();
    return 0;
}
