#include <GL/gl.h>
#include <GL/glu.h>
#include <GL/glut.h>
#include <cstdio>
#include <cmath>
#include <ros/ros.h>
#include <opencv2/opencv.hpp>
#include <iostream>
#include <fstream>
#include <eigen3/Eigen/Dense>

#include "utils.h"

using namespace std;

#define OUTPUT_DIR "/home/timer/catkin_ws/gl_outputs"

const int calc_level = 0;
//const int calc_level = 2;
int WIDTH = 752;
int HEIGHT = 480;
double fy = 520.0; // visensor left cam1 (P[1][1])
double cy = 240.0; // visensor left cam1 (P[1][2])

char file_path[100];
GLuint meshes;
FILE *file;
int cnt;
bool has_color;
int file_cnt = 0;
double near = 1.0;
double far = 50.0;
double FOV;

// skip useless things in input file
int file_init()
{
    char str[100];
    printf("Opening file: %s\n",file_path);
    file = fopen(file_path,"r");

    if (!file)
    {
        ROS_ERROR("File open error!");
        return 1;
    }
    // ignore rubbish
    for (int i=0;i<3;i++)
        fgets(str,100,file);
    // read cnt
    sscanf(str,"element vertex %d",&cnt);
    ROS_INFO("cnt = %d",cnt);
    if (cnt==0)
    {
        ROS_ERROR("No mesh!");
        return 1;
    }
    // ignore rubbish
    for (int i=0;i<4;i++)
        fgets(str,100,file);
    // has color ?
    if (str[0]=='p') has_color = true;
    else has_color = false;
    if (has_color)
    {
        ROS_INFO("has_color = true");
        for (int i=0;i<3;i++)
            fgets(str,100,file);
    }
    else
        ROS_INFO("has_color = false");
    // ignore rubbish
    for (int i=0;i<2;i++)
        fgets(str,100,file);
    return 0;
}

// Draw triangles
static void triangles()
{
    char str[100];
    double x,y,z,r,g,b;
    glBegin(GL_TRIANGLES);
    for (int i=0;i<cnt;i++)
    {
        fgets(str,100,file);
        if (has_color)
        {
            sscanf(str,"%lf %lf %lf %lf %lf %lf",&x,&y,&z,&r,&g,&b);
            r/=255.0;
            g/=255.0;
            b/=255.0;
        }
        else
        {
            sscanf(str,"%lf %lf %lf",&x,&y,&z);
            r = g = b = 1.0;  // white
        }
        glColor3f(r,g,b);
        glVertex3f(x,y,z);
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
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
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
   
    // FOV of the height (  tan (FOV/2)=(height-c_y)/f_y  )
    FOV = atan2(HEIGHT-cy,fy)*2.0;
    FOV = FOV*180.0/M_PI; // tan(FOV/2) = h/(2*zNear)
    printf("FOV = %lf, PI = %lf\n",FOV,M_PI);
//    FOV = 73; // 55,  90
    gluPerspective(FOV, (GLfloat) w/(GLfloat) h, near, far);  // FOV of the height (tan (FOV/2)=(height-c_y)/f_y)

    glEnable(GL_DEPTH_TEST);

    glMatrixMode(GL_MODELVIEW);
    glLoadIdentity();
    gluLookAt(0, 0, 0, 0, 0, 1, 0, -1, 0);    //  last three (, shangxia, )
}

bool save_img(const char *file_name)
{
    char* pixels = new char[3*WIDTH*HEIGHT];  // left-corner start,   (HEIGHT,0) -> (HEIGHT,1) ...
    glPixelStorei(GL_UNPACK_ALIGNMENT,1);
    glReadPixels(0,0,WIDTH,HEIGHT,GL_RGB,GL_UNSIGNED_BYTE,pixels);

    cv::Mat img_mat = cv::Mat::zeros(HEIGHT,WIDTH,CV_8UC3);
    int pixel_cnt = 0;
    for (int v=HEIGHT-1;v>=0;v--)
        for (int u=0;u<WIDTH;u++)
        {
            cv::Vec3b tmp(pixels[pixel_cnt+2],pixels[pixel_cnt+1],pixels[pixel_cnt]);
            img_mat.at<cv::Vec3b>(v,u) = tmp;
            pixel_cnt += 3;
        }
    cv::imwrite(file_name,img_mat);

    delete [] pixels;
    return true;
}

bool save_depth(const char *file_name)
{
    float* pixels = new float[WIDTH*HEIGHT];
    glPixelStorei(GL_UNPACK_ALIGNMENT,1);
    glReadPixels(0,0,WIDTH,HEIGHT,GL_DEPTH_COMPONENT,GL_FLOAT,pixels);

    cv::Mat depth_img = cv::Mat::zeros(HEIGHT,WIDTH,CV_32FC1);
    int pixel_cnt = 0;

    double z_c = (far-near)/256.0;

    Eigen::MatrixXd depth_matrix = Eigen::MatrixXd::Zero(HEIGHT,WIDTH);
    double d_max,d_min;
    d_max = pixels[0];
    d_min = d_max;

    for (int v=HEIGHT-1;v>=0;v--)
        for (int u=0;u<WIDTH;u++)
        {
            double z_buffer = pixels[pixel_cnt];

            depth_matrix(v,u) = (2.0*far*near) / ( far + near - (far-near)*(2.0*z_buffer-1.0) );

            if (double_equ_check(depth_matrix(v,u),far) == 0)
                depth_matrix(v,u) = 0.0;

            depth_img.at<float>(v,u) = depth_matrix(v,u);  // if no depth -> z_buffer = 1.0,  z_real = far

            if (depth_img.at<float>(v,u) > d_max)
                d_max = depth_img.at<float>(v,u);
            if (depth_img.at<float>(v,u) < d_min)
                d_min = depth_img.at<float>(v,u);
            pixel_cnt++;
        }
    for (int v=HEIGHT-1;v>=0;v--)
        for (int u=0;u<WIDTH;u++)
        {
            double tmp;
            tmp = (depth_img.at<float>(v,u) - d_min)*255.0/d_max;
            depth_img.at<float>(v,u) = tmp;
        }
    printf("d_max = %lf, d_min = %lf\n",d_max,d_min);
    cv::Mat depth_out = cv::Mat::zeros(HEIGHT,WIDTH,CV_8UC1);
    depth_img.convertTo(depth_out,CV_8UC1);
    cv::imwrite(file_name,depth_out);

    // save to file
    ofstream fout;
    char tmp[100];
    sprintf(tmp,"%s/depth_%.2lf.depth",OUTPUT_DIR,FOV);
    fout.open(tmp);
    fout << depth_matrix << endl;
    fout.close();

    delete [] pixels;
    return true;    
}

/* Rotate about x-axis when "x" typed; rotate about y-axis
   when "y" typed; "i" returns torus to original view */
void keyboard(unsigned char key, int x, int y)
{
    switch (key)
    {
        case 'z':
            glRotatef(0.3,1.0,0.0,0.0);
            break;
       case 'Z':
            glRotatef(3.,1.0,0.0,0.0);
            break;
       case 'c':
            glRotatef(0.3,-1.0,0.0,0.0);
            break;
       case 'C':
            glRotatef(3.,-1.0,0.0,0.0);
            break;
       case 'q':
            glRotatef(0.3,0.0,1.0,0.0);
            break;
       case 'Q':
            glRotatef(3.,0.0,1.0,0.0);
            break;
       case 'e':
            glRotatef(0.3,0.0,-1.0,0.0);
            break;
       case 'E':
            glRotatef(3.,0.0,-1.0,0.0);
            break;
       case 'f':
            glRotatef(0.3,0.0,0.0,-1.0);
            break;
       case 'F':
            glRotatef(3.,0.0,0.0,-1.0);
            break;
       case 'g':
            glRotatef(0.3,0.0,0.0,1.0);
            break;
       case 'G':
            glRotatef(3.,0.0,0.0,1.0);
            break;
       case 'i':
       case 'I':
            glLoadIdentity();
            gluLookAt(0, 0, 0, 0, 0, 1, 0, -1, 0);
            break;
       case 'y':
            glTranslated(0, 0, -0.01);
            break;
       case 'Y':
            glTranslated(0, 0, -0.1);
            break;
       case 'h':
            glTranslated(0, 0, 0.01);
            break;
       case 'H':
            glTranslated(0, 0, 0.1);
            break;
       case 'w':
            glTranslated(0, 0.01, 0);
            break;
       case 'W':
            glTranslated(0, 0.1, 0);
            break;
       case 's':
            glTranslated(0, -0.01, 0);
            break;
       case 'S':
            glTranslated(0, -0.1, 0);
            break;
       case 'a':
            glTranslated(-0.01, 0, 0);
            break;
       case 'A':
            glTranslated(-0.1, 0, 0);
            break;
       case 'd':
            glTranslated(0.01, 0, 0);
            break;
       case 'D':
            glTranslated(0.1, 0, 0);
            break;
       case 'p':
            char file_name[100];
            sprintf(file_name,"%s/key_frame_%02d_%.2lf.bmp",OUTPUT_DIR,file_cnt,FOV);
            if (save_img(file_name))
                ROS_INFO("save img: %s done",file_name);
            else
                ROS_ERROR("save img: %s fail",file_name);
            sprintf(file_name,"%s/key_frame_depth_%02d_%.2lf.bmp",OUTPUT_DIR,file_cnt,FOV);
            if (save_depth(file_name))
                ROS_INFO("save depth: %s done",file_name);
            else
                ROS_ERROR("save depth: %s fail",file_name);
            file_cnt++;
            break;
       case '`':
            double modelview[16];
            glGetDoublev(GL_MODELVIEW_MATRIX,modelview);
            for (int i=0;i<4;i++)
            {
                for (int j=0;j<4;j++)
                {
                    printf("%lf ",modelview[i*4+j]);
                }
                puts("");
                puts("-------------------------------");
            }
            break;
       case 27:  // key: 'ESC'
            exit(0);
            break;
    }
    glutPostRedisplay();
}

int main(int argc, char **argv)
{
    ros::init(argc,argv,"gl_viewer");
    fy = fy / (1<<calc_level);    // down sample to calc_level
    cy = cy / (1<<calc_level);
    //cy = (cy+0.5) / (1<<calc_level) - 0.5;
    WIDTH = WIDTH >> calc_level;
    HEIGHT = HEIGHT >> calc_level;

    sprintf(file_path,"%s",argv[1]);
    if (file_init())
    {
        ROS_ERROR("Something error happend!");
        return 1;
    }
    glutInitWindowSize(WIDTH,HEIGHT);
    glutInit(&argc, argv);
    glutInitDisplayMode(GLUT_SINGLE | GLUT_RGB | GLUT_DEPTH);
    glutCreateWindow(argv[0]);
    init();

    glutReshapeFunc(reshape);
    glutKeyboardFunc(keyboard);
    glutDisplayFunc(display);
    glutMainLoop();

    return 0;
}
