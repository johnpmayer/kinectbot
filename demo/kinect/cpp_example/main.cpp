/*
 * This file is part of the OpenKinect Project. http://www.openkinect.org
 *
 * Copyright (c) 2010 individual OpenKinect contributors. See the CONTRIB file
 * for details.
 *
 * This code is licensed to you under the terms of the Apache License, version
 * 2.0, or, at your option, the terms of the GNU General Public License,
 * version 2.0. See the APACHE20 and GPL2 files for the text of the licenses,
 * or the following URLs:
 * http://www.apache.org/licenses/LICENSE-2.0
 * http://www.gnu.org/licenses/gpl-2.0.txt
 *
 * If you redistribute this file in source form, modified or unmodified, you
 * may:
 *   1) Leave this header intact and distribute it under the same terms,
 *      accompanying it with the APACHE20 and GPL20 files, or
 *   2) Delete the Apache 2.0 clause and accompany it with the GPL2 file, or
 *   3) Delete the GPL v2 clause and accompany it with the APACHE20 file
 * In all cases you must keep the copyright notice intact and include a copy
 * of the CONTRIB file.
 *
 * Binary distributions must follow the binary distribution requirements of
 * either License.
 */

#include "libfreenect.hpp"
#include "libfreenect.h"
#include <iostream>
#include <vector>
#include <cmath>
#include <pthread.h>

/*
  #if defined(__APPLE__)
  #include <GLUT/glut.h>
  #include <OpenGL/gl.h>
  #include <OpenGL/glu.h>
  #else
  #include <GL/glut.h>
  #include <GL/gl.h>
  #include <GL/glu.h>
  #endif
*/

#define 	FREENECT_FRAME_W   640
#define 	FREENECT_FRAME_H   480
#define 	FREENECT_FRAME_PIX   (FREENECT_FRAME_H*FREENECT_FRAME_W)
#define 	FREENECT_IR_FRAME_W   640
#define 	FREENECT_IR_FRAME_H   488
#define 	FREENECT_IR_FRAME_PIX   (FREENECT_IR_FRAME_H*FREENECT_IR_FRAME_W)
#define 	FREENECT_VIDEO_RGB_SIZE   (FREENECT_FRAME_PIX*3)

using namespace std;

//define MyFreenectDevice and Mutex class
class Mutex {
public:
  Mutex() {
    pthread_mutex_init( &m_mutex, NULL );
  }
  void lock() {
    pthread_mutex_lock( &m_mutex );
  }
  void unlock() {
    pthread_mutex_unlock( &m_mutex );
  }
private:
  pthread_mutex_t m_mutex;
};

class MyFreenectDevice : public Freenect::FreenectDevice {
public:
  MyFreenectDevice(freenect_context *_ctx, int _index) : Freenect::FreenectDevice(_ctx, _index), m_buffer_depth(FREENECT_VIDEO_RGB_SIZE),m_buffer_video(FREENECT_VIDEO_RGB_SIZE), m_gamma(2048), m_new_rgb_frame(false), m_new_depth_frame(false) {
    
    for( unsigned int i = 0 ; i < 2048 ; i++) {
      float v = i/2048.0;
      v = pow(v, 3)* 6;
      m_gamma[i] = v*6*256;
    }
  }
  void VideoCallback(void* _rgb, uint32_t timestamp) {
    m_rgb_mutex.lock();
    uint8_t* rgb = static_cast<uint8_t*>(_rgb);
    copy(rgb, rgb+getVideoBufferSize(), m_buffer_video.begin());
    m_new_rgb_frame = true;
    m_rgb_mutex.unlock();
  };
  void DepthCallback(void* _depth, uint32_t timestamp) {
    m_depth_mutex.lock();
    uint16_t* depth = static_cast<uint16_t*>(_depth);
    for( unsigned int i = 0 ; i < FREENECT_FRAME_PIX ; i++) {
      int pval = m_gamma[depth[i]];
      int lb = pval & 0xff;
      switch (pval>>8) {
      case 0:
	m_buffer_depth[3*i+0] = 255;
	m_buffer_depth[3*i+1] = 255-lb;
	m_buffer_depth[3*i+2] = 255-lb;
	break;
      case 1:
	m_buffer_depth[3*i+0] = 255;
	m_buffer_depth[3*i+1] = lb;
	m_buffer_depth[3*i+2] = 0;
	break;
      case 2:
	m_buffer_depth[3*i+0] = 255-lb;
	m_buffer_depth[3*i+1] = 255;
	m_buffer_depth[3*i+2] = 0;
	break;
      case 3:
	m_buffer_depth[3*i+0] = 0;
	m_buffer_depth[3*i+1] = 255;
	m_buffer_depth[3*i+2] = lb;
	break;
      case 4:
	m_buffer_depth[3*i+0] = 0;
	m_buffer_depth[3*i+1] = 255-lb;
	m_buffer_depth[3*i+2] = 255;
	break;
      case 5:
	m_buffer_depth[3*i+0] = 0;
	m_buffer_depth[3*i+1] = 0;
	m_buffer_depth[3*i+2] = 255-lb;
	break;
      default:
	m_buffer_depth[3*i+0] = 0;
	m_buffer_depth[3*i+1] = 0;
	m_buffer_depth[3*i+2] = 0;
	break;
      }
    }
    m_new_depth_frame = true;
    m_depth_mutex.unlock();
  }
  bool getRGB(vector<uint8_t> &buffer) {
    m_rgb_mutex.lock();
    if(m_new_rgb_frame) {
      buffer.swap(m_buffer_video);
      m_new_rgb_frame = false;
      m_rgb_mutex.unlock();
      return true;
    } else {
      m_rgb_mutex.unlock();
      return false;
    }
  }
  bool getDepth(vector<uint8_t> &buffer) {
    m_depth_mutex.lock();
    if(m_new_depth_frame) {
      buffer.swap(m_buffer_depth);
      m_new_depth_frame = false;
      m_depth_mutex.unlock();
      return true;
    } else {
      m_depth_mutex.unlock();
      return false;
    }
  }
private:
  vector<uint8_t> m_buffer_depth;
  vector<uint8_t> m_buffer_video;
  vector<uint16_t> m_gamma;
  Mutex m_rgb_mutex;
  Mutex m_depth_mutex;
  bool m_new_rgb_frame;
  bool m_new_depth_frame;
};

/*
//define OpenGL variables
GLuint gl_depth_tex;
GLuint gl_rgb_tex;
int g_argc;
char **g_argv;
int got_frames(0);
int window(0);
*/

//define libfreenect variables
Freenect::Freenect freenect;
MyFreenectDevice* device;
double freenect_angle(0);
freenect_video_format requested_format(FREENECT_VIDEO_RGB);

/*
//define Kinect Device control elements
//glutKeyboardFunc Handler
void keyPressed(unsigned char key, int x, int y)
{
if (key == 27) {
device->setLed(LED_OFF);
freenect_angle = 0;
glutDestroyWindow(window);
}
if (key == '1') {
device->setLed(LED_GREEN);
}
if (key == '2') {
device->setLed(LED_RED);
}
if (key == '3') {
device->setLed(LED_YELLOW);
}
if (key == '4') {
device->setLed(LED_BLINK_GREEN);
}
if (key == '5') {
// 5 is the same as 4
device->setLed(LED_BLINK_GREEN);
}
if (key == '6') {
device->setLed(LED_BLINK_RED_YELLOW);
}
if (key == '0') {
device->setLed(LED_OFF);
}
if (key == 'f') {
if (requested_format == FREENECT_VIDEO_IR_8BIT) {
requested_format = FREENECT_VIDEO_RGB;
} else if (requested_format == FREENECT_VIDEO_RGB){
requested_format = FREENECT_VIDEO_YUV_RGB;
} else {
requested_format = FREENECT_VIDEO_IR_8BIT;
}
device->setVideoFormat(requested_format);
}

if (key == 'w') {
freenect_angle++;
if (freenect_angle > 30) {
freenect_angle = 30;
}
}
if (key == 's' || key == 'd') {
freenect_angle = 10;
}
if (key == 'x') {
freenect_angle--;
if (freenect_angle < -30) {
freenect_angle = -30;
}
}
if (key == 'e') {
freenect_angle = 10;
}
if (key == 'c') {
freenect_angle = -10;
}
device->setTiltDegrees(freenect_angle);
}
//define OpenGL functions
void DrawGLScene()
{
static std::vector<uint8_t> depth(640*480*4);
static std::vector<uint8_t> rgb(640*480*4);

// using getTiltDegs() in a closed loop is unstable
//if(device->getState().m_code == TILT_STATUS_STOPPED){
//	freenect_angle = device->getState().getTiltDegs();
//}
device->updateState();
printf("\r demanded tilt angle: %+4.2f device tilt angle: %+4.2f", freenect_angle, device->getState().getTiltDegs());
fflush(stdout);

device->getDepth(depth);
device->getRGB(rgb);

got_frames = 0;

glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
glLoadIdentity();

glEnable(GL_TEXTURE_2D);

glBindTexture(GL_TEXTURE_2D, gl_depth_tex);
glTexImage2D(GL_TEXTURE_2D, 0, 4, 640, 480, 0, GL_RGB, GL_UNSIGNED_BYTE, &depth[0]);

glBegin(GL_TRIANGLE_FAN);
glColor4f(255.0f, 255.0f, 255.0f, 255.0f);
glTexCoord2f(0, 0); glVertex3f(0,0,0);
glTexCoord2f(1, 0); glVertex3f(640,0,0);
glTexCoord2f(1, 1); glVertex3f(640,480,0);
glTexCoord2f(0, 1); glVertex3f(0,480,0);
glEnd();
glutSwapBuffers();
}

void InitGL()
{
glClearColor(0.0f, 0.0f, 0.0f, 0.0f);
glClearDepth(1.0);
glDepthFunc(GL_LESS);
glDisable(GL_DEPTH_TEST);
glEnable(GL_BLEND);
glBlendFunc (GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
glShadeModel(GL_SMOOTH);
glGenTextures(1, &gl_depth_tex);
glBindTexture(GL_TEXTURE_2D, gl_depth_tex);
glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
glMatrixMode(GL_PROJECTION);
glLoadIdentity();
glOrtho (0, 640, 480, 0, 0.0f, 1.0f);
glMatrixMode(GL_MODELVIEW);
}

void displayKinectData(MyFreenectDevice* device){
glutInit(&g_argc, g_argv);
glutInitDisplayMode(GLUT_RGBA | GLUT_DOUBLE | GLUT_ALPHA | GLUT_DEPTH);
glutInitWindowSize(640, 480);
glutInitWindowPosition(0, 0);
window = glutCreateWindow("c++ wrapper example");
glutDisplayFunc(&DrawGLScene);
glutIdleFunc(&DrawGLScene);
glutKeyboardFunc(&keyPressed);
InitGL();
glutMainLoop();
}
*/

//define main function
int main(int argc, char **argv) {
  //Get Kinect Device
  device = &freenect.createDevice<MyFreenectDevice>(0);
  //Start Kinect Device
  device->setTiltDegrees(10);
  //device->startVideo();
  device->startDepth();
  //handle Kinect Device Data
  device->setLed(LED_RED);
  //displayKinectData(device);
  //Stop Kinect Device
  //device->stopVideo();
  device->stopDepth();
  device->setLed(LED_OFF);
  return 0;
}
