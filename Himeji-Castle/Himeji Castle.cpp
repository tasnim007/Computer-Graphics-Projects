#include<windows.h>
#include<stdio.h>
#include<stdlib.h>
#include<math.h>
//#include "camera.cpp"
//#include "hemisferic.cpp"

#include<GL/glut.h>

#define BLACK 0.8, 0.9, 1
#define BASE_COLOR 0.5, 0.5, 0
#define WALL_COLOR 1, .99, .82
#define SHADE_COLOR 0.5, 0.5, 0.5
#define CYL_COLOR .2,.2,.2
#define PILLAR_COLOR 0.37, 0.37, 0.37

#define SASHICOLOR .2,.2,.2
#define SASHIWINGSCOLOR .1,.1,.1
//using namespace std;


bool nightMode=true;
bool l_0,l_1,l_2;
int lightOn=0;
bool twoLightOnly=false;

//make a global variable -- for tracking the anglular position of camera
double cameraAngle;			//in radian
double cameraAngleDelta;
//Camera cam;
//hemisferic hem;

double cameraHeight,h,a,b;	
double cameraRadius;

double cameraMovX,cameraMovY;
double x=17.181,z=-117.978,y=8.1;

//Point3 eye1(500, -700,700);
//Point3 look(0, 10, 0);
//Vector3 up(0, 0, 1);

double rectAngle,hl,lp,road;	//in degree

bool canDrawGrid,hlOn,carMoving;

int option;


void loadImage();
int LoadBitmapImage(char *filename);

/********************************************************/


class point{
public:
	float x;
	float y;
	float z;

	point(float x1, float y1, float z1){
		x = x1;
		y = y1;
		z = z1;
	}

	void reInit(float x1, float y1, float z1){
		x = x1;
		y = y1;
		z = z1;
	}

	void increment(float x1, float y1, float z1){
		x += x1;
		y += y1;
		z += z1;
	}
};


/********************************************************/

/***********vector**************/

#define CO(V)	V.x,V.y,V.z
#define COXY(V)	V.x,V.y

#define S(x)		((x)*(x))

#define ABS(x)		(((x)>0)?(x):-(x))
#define MAX(x,y)	(((x)>(y))?(x):(y))
#define MIN(x,y)	(((x)<(y))?(x):(y))

#define EPS			1e-8
#define Z(x)		(ABS(x)  < EPS)

#define det(a,b,c,d)	((a)*(d)-(b)*(c))

double pi = 2.*acos(0.);
double sqrt2 = sqrt(2.);
double sqrt3 = sqrt(3.);

double c30 = cos(pi/6);
double s30 = sin(pi/6);
double t30 = tan(pi/6);

double c15 = cos(pi/12);
double s15 = sin(pi/12);
double t15 = tan(pi/12);

double mysqrt(double x){if(x < 0)	return 0;return sqrt(x);}
double myasin(double x){if(x < -1)	return -pi/2;if(x > 1)	return pi/2;return asin(x);}
double myacos(double x){if(x < -1)	return -pi;if(x > 1)	return 0;return acos(x);}

struct V;

V operator+(V a,V b);
V operator*(V a,V b);
V operator*(V b,double a);
V operator*(double a,V b);

struct V{
	double x,y,z;

	V(){}
	V(double _x,double _y){x=_x;y=_y;z=0;}
	V(double _x,double _y,double _z){x=_x;y=_y;z=_z;}

	double	mag2(){	return S(x)+S(y)+S(z);	}
	double	mag(){	return sqrt(mag2());	}
	
	void 	norm(){	double d = mag();x/=d;y/=d;	z/=d;}
	V 		unit(){	V ret = *this;	ret.norm(); return ret;}

	double	dot(V b){		return x*b.x + y*b.y + z*b.z;}
	V		cross(V b){		return V( y*b.z - z*b.y , z*b.x - x*b.z , x*b.y - y*b.x );}
	double	box(V b, V c){	return this->dot(b.cross(c));	}

	double	projL(V on){	on.norm();	return this->dot(on);}
	V		projV(V on){	on.norm();	return on * projL(on);}

	V rot(V axis, double angle){
		return this->rot(axis, cos(angle), sin(angle));
	}
	
	V rot(V axis, double ca, double sa){
		V rotatee = *this;
		axis.norm();
		V normal = (axis * rotatee).unit();
		V mid = (normal * axis).unit();
		double r = rotatee.projL(mid);
		V ret=r*mid*ca + r*normal*sa + rotatee.projV(axis);
		return ret.unit();
	}
};

V operator+(V a,V b){		return V(a.x+b.x, a.y+b.y, a.z+b.z);	}
V operator-(V a){			return V (-a.x, -a.y, -a.z);			}
V operator-(V a,V b){		return V(a.x-b.x, a.y-b.y, a.z-b.z);	}
V operator*(V a,V b){		return a.cross(b);						}
V operator*(double a,V b){	return V(a*b.x, a*b.y, a*b.z);			}
V operator*(V b,double a){	return V(a*b.x, a*b.y, a*b.z);			}
V operator/(V b,double a){	return V(b.x/a, b.y/a, b.z/a);			}




// calculation normals /////////////////////////
void doNormal(double x1,double y1,double z1,double x2,double y2,double z2,double x3,double y3,double z3)
{
	V d1(x1-x2,y1-y2,z1-z2);
	V d2(x1-x3,y1-y3,z1-z3);

	V d=d1*d2;
	d.norm();

	glNormal3f(d.x , d.y , d.z);
}


void dofaruknormal(point a, point b, point c){
	V d1(a.x-b.x, a.y-b.y, a.z-b.z);
	V d2(a.x-c.x, a.y-c.y, a.z-c.z);

	V d=d1*d2;
	d.norm();

	glNormal3f(d.x , d.y , d.z);
}



V I(1,0,0);
V J(0,1,0);
V K(0,0,1);
V INEG(-1,0,0);
V JNEG(0,-1,0);
V KNEG(0,0,-1);
V Origin(0,0,0);

V loc,dir,perp;


V  _L(400,-700,520);
V  _D(-1,0,0);
V  _P(0,0,1);

bool bird=false;

double speed=30,ang_speed=.1;


/********************************************************************************/


/********************Lighting and texture ******************************************************/


int num_texture  = -1;
GLuint wallimage, baseimg, grassimg;
float ar,ag,ab,dr,dg,db,sr,sg,sb;
float dl;

//void lighting(void);

void loadImage()
{
	baseimg = LoadBitmapImage("image/base.bmp");
	//sky = LoadBitmap("images/sky.bmp");
	grassimg = LoadBitmapImage("image/grass.bmp");
	wallimage = LoadBitmapImage("image/wall.bmp");
	printf("Load successful");
}


int LoadBitmapImage(char *filename)
{
    int i, j=0;
    FILE *l_file;
    unsigned char *l_texture;

    BITMAPFILEHEADER fileheader;
    BITMAPINFOHEADER infoheader;
    RGBTRIPLE rgb;

    num_texture++;

    if( (l_file = fopen(filename, "rb"))==NULL) return (-1);

    fread(&fileheader, sizeof(fileheader), 1, l_file);

    fseek(l_file, sizeof(fileheader), SEEK_SET);
    fread(&infoheader, sizeof(infoheader), 1, l_file);

    l_texture = (byte *) malloc(infoheader.biWidth * infoheader.biHeight * 4);
    memset(l_texture, 0, infoheader.biWidth * infoheader.biHeight * 4);
	for (i=0; i < infoheader.biWidth*infoheader.biHeight; i++)
		{
				fread(&rgb, sizeof(rgb), 1, l_file);

				l_texture[j+0] = rgb.rgbtRed;
				l_texture[j+1] = rgb.rgbtGreen;
				l_texture[j+2] = rgb.rgbtBlue;
				l_texture[j+3] = 255;
				j += 4;
		}
    fclose(l_file);

    glBindTexture(GL_TEXTURE_2D, num_texture);

    glTexParameterf(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_REPEAT);
    glTexParameterf(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_REPEAT);
    glTexParameterf(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
    glTexParameterf(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR_MIPMAP_NEAREST);

// glTexParameterf(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
    glTexEnvf(GL_TEXTURE_ENV, GL_TEXTURE_ENV_MODE, GL_MODULATE);
    glTexImage2D(GL_TEXTURE_2D, 0, 4, infoheader.biWidth, infoheader.biHeight, 0, GL_RGBA, GL_UNSIGNED_BYTE, l_texture);
     gluBuild2DMipmaps(GL_TEXTURE_2D, 4, infoheader.biWidth, infoheader.biHeight, GL_RGBA, GL_UNSIGNED_BYTE, l_texture);

    free(l_texture);

    return (num_texture);

}

void lighting(void)
{
	glEnable(GL_LIGHTING);
	glEnable(GL_LIGHT0);
	glShadeModel(GL_SMOOTH);
	GLfloat ambientLight[] = {ar,ag,ab,1.0f};
	GLfloat diffuseLight[] = {dr,dg,db,1.0f};
	GLfloat specularLight[] = {sr,sg,sb,1.0f};
	GLfloat position[] = {550.0f , 550.0f , 100.0f , 1.0f};

	glLightfv(GL_LIGHT0 , GL_AMBIENT , ambientLight);
	glLightfv(GL_LIGHT0 , GL_DIFFUSE , diffuseLight);
	glLightfv(GL_LIGHT0 , GL_SPECULAR ,specularLight);
	glLightfv(GL_LIGHT0 , GL_POSITION , position);

	//----------------------Incomplete------------------------

	//glDisable(GL_LIGHT0);
	//glDisable(GL_LIGHTING);
}




void initlights(void)
{

	glEnable(GL_COLOR_MATERIAL);
	glEnable(GL_LIGHTING);
	glEnable(GL_NORMALIZE);


	l_0=l_1=l_2=false;
	l_2=false;
	
	glEnable(GL_LIGHTING);
	if(l_0){
		glEnable(GL_LIGHT0);
		glEnable(GL_LIGHT5);
	}

	if(l_1){
		glEnable(GL_LIGHT1);
		glEnable(GL_LIGHT2);
	}

	if(l_2){
		glEnable(GL_LIGHT3);
		glEnable(GL_LIGHT4);
	}
	double ambientFact =0.1;
	double directedFact = 0.6;
	double directedFactSpec = 0.5;
	GLfloat ambientColor[] = {ambientFact, ambientFact, ambientFact, 1.0f}; //Color (0.2, 0.2, 0.2)
	glLightModelfv(GL_LIGHT_MODEL_AMBIENT, ambientColor);

	////glColor3f(1,1,1);
	//glutSolidCube(1500);

	GLfloat lightColor[] = {directedFact, directedFact, directedFact, 1.0f}; //Color (0.5, 0.2, 0.2)
	GLfloat lightColorSpec[] = {directedFactSpec, directedFactSpec, directedFactSpec, 1.0f}; //Color (0.5, 0.2, 0.2)

	//Coming from the direction (-1, 0.5, 0.5)
	GLfloat lightPos0[] = {0, 0, -1, 0.0f};
	glLightfv(GL_LIGHT0, GL_DIFFUSE, lightColor);
	glLightfv(GL_LIGHT0, GL_SPECULAR, lightColorSpec);
	glLightfv(GL_LIGHT0, GL_POSITION, lightPos0);

	GLfloat lightPos5[] = {0, 0, 1, 0.0f};
	glLightfv(GL_LIGHT5, GL_DIFFUSE, lightColor);
	glLightfv(GL_LIGHT5, GL_SPECULAR, lightColorSpec);
	glLightfv(GL_LIGHT5, GL_POSITION, lightPos5);


	GLfloat lightPos1[] = {0, -1, 0, 0.0f};
	glLightfv(GL_LIGHT1, GL_DIFFUSE, lightColor);
	glLightfv(GL_LIGHT1, GL_SPECULAR, lightColorSpec);
	glLightfv(GL_LIGHT1, GL_POSITION, lightPos1);

	GLfloat lightPos2[] = {0, 1, 0, 0.0f};
	glLightfv(GL_LIGHT2, GL_DIFFUSE, lightColor);
	glLightfv(GL_LIGHT2, GL_SPECULAR, lightColorSpec);
	glLightfv(GL_LIGHT2, GL_POSITION, lightPos2);

	GLfloat lightPos3[] = {-1, 0, 0, 0.0f};
	glLightfv(GL_LIGHT3, GL_DIFFUSE, lightColor);
	glLightfv(GL_LIGHT3, GL_SPECULAR, lightColorSpec);
	glLightfv(GL_LIGHT3, GL_POSITION, lightPos3);
	
	GLfloat lightPos4[] = {1, 0, 0, 0.0f};
	glLightfv(GL_LIGHT4, GL_DIFFUSE, lightColor);
	glLightfv(GL_LIGHT4, GL_SPECULAR, lightColorSpec);
	glLightfv(GL_LIGHT4, GL_POSITION, lightPos4);


	GLfloat mat_diffuse[] = {0.6, 0.6, 0.6, 1.0};
	GLfloat mat_specular[] = {0.8, 0.6, 0.6, 1.0};
	GLfloat mat_shininess[] = {100.0};

	glMaterialfv(GL_FRONT, GL_DIFFUSE, mat_diffuse);
	glMaterialfv(GL_FRONT, GL_SPECULAR, mat_specular);
	glMaterialfv(GL_FRONT, GL_SHININESS, mat_shininess);

	
}



/************************************************************************************/



/********************************************************************************/




 



void drawRectangle(point a, point b, point c, point d, int id){
	if(id == 0){
		glBegin(GL_QUADS);
				glVertex3f(a.x, a.y, a.z);
				glVertex3f(b.x, b.y, b.z);
				glVertex3f(c.x, c.y, c.z);
				glVertex3f(d.x, d.y, d.z);
			glEnd();
	}

	else if(id == 1){
		glEnable(GL_TEXTURE_2D);
		glBindTexture(GL_TEXTURE_2D,wallimage);

		
		float pix = 1;
		glBegin(GL_QUADS);{
			//glNormal3f(0,0,1);
			glColor3f(1.0f,1.0f,1.0f); 
			glTexCoord2f(0,0);
			glVertex3f(a.x, a.y, a.z);
			glTexCoord2f(0,pix);
			glVertex3f(b.x, b.y, b.z);
			glTexCoord2f(pix,pix);
			glVertex3f(c.x, c.y, c.z);
			glTexCoord2f(pix,0);
			glVertex3f(d.x, d.y, d.z);
		}glEnd();

		glDisable(GL_TEXTURE_2D);
	}
}

void drawTriangle(point a, point b, point c){
	glBegin(GL_POLYGON);
			glVertex3f(a.x, a.y, a.z);
			glVertex3f(b.x, b.y, b.z);
			glVertex3f(c.x, c.y, c.z);
		//	glVertex3f(d.x, d.y, d.z);
		glEnd();
}




void baseofMain(){
	glEnable(GL_TEXTURE_2D);
	glBindTexture(GL_TEXTURE_2D,baseimg);

	//front
	float pix = 1;

	point a(300, 0 ,0);
	point b(800, 0 ,0);
	point c(775, 25 ,250);
	point d(325, 25 ,250);
	
	dofaruknormal(a,b,c);
	glBegin(GL_QUADS);{
		//glNormal3f(0,0,1);
		glColor3f(1.0f,1.0f,1.0f); 
		glTexCoord2f(0,0);
		glVertex3f(300, 0 ,0);
		glTexCoord2f(0,pix);
		glVertex3f(800,0,0);
		glTexCoord2f(pix,pix);
		glVertex3f(775, 25 ,250);
		glTexCoord2f(pix,0);
		glVertex3f(325, 25 ,250);
	}glEnd();
	
	


	
	
	glColor3f(BASE_COLOR);
	//drawRectangle(a,b,c,d);


	//left
	a.reInit(300, 0 ,0);
	b.reInit(300, 400 ,0);
	c.reInit(325, 375 ,250);
	d.reInit(325, 25 ,250);

	dofaruknormal(a,b,c);
	glBegin(GL_QUADS);{
		//glNormal3f(0,0,1);
		glColor3f(1.0f,1.0f,1.0f); 
		glTexCoord2f(0,0);
		glVertex3f(300, 0 ,0);
		glTexCoord2f(0,pix);
		glVertex3f(300,400,0);
		glTexCoord2f(pix,pix);
		glVertex3f(325, 375 ,250);
		glTexCoord2f(pix,0);
		glVertex3f(325, 25 ,250);
	}glEnd();

	
	//drawRectangle(a,b,c,d);

	//back


	a.reInit(300, 400 ,0);
	b.reInit(800, 400 ,0);
	c.reInit(775, 375 ,250);
	d.reInit(325, 375 ,250);
	dofaruknormal(a,b,c);
	glBegin(GL_QUADS);{
		//glNormal3f(0,0,1);
		glColor3f(1.0f,1.0f,1.0f); 
		glTexCoord2f(0,0);
		glVertex3f(300, 400 ,0);
		glTexCoord2f(0,pix);
		glVertex3f(800,400,0);
		glTexCoord2f(pix,pix);
		glVertex3f(775, 375 ,250);
		glTexCoord2f(pix,0);
		glVertex3f(325, 375 ,250);
	}glEnd();

	
	//drawRectangle(a,b,c,d);

		//right

	a.reInit(800, 0 ,0);
	b.reInit(800, 400 ,0);
	c.reInit(775, 375 ,250);
	d.reInit(775, 25 ,250);

	dofaruknormal(a,b,c);
	glBegin(GL_QUADS);{
		//glNormal3f(0,0,1);
		glColor3f(1.0f,1.0f,1.0f); 
		glTexCoord2f(0,0);
		glVertex3f(800, 0 ,0);
		glTexCoord2f(0,pix);
		glVertex3f(800,400,0);
		glTexCoord2f(pix,pix);
		glVertex3f(775, 375 ,250);
		glTexCoord2f(pix,0);
		glVertex3f(775, 25 ,250);
	}glEnd();

	
	//drawRectangle(a,b,c,d);

	//floor
	a.reInit(325, 375 ,250);
	b.reInit(775, 375 ,250);
	c.reInit(775, 25 ,250);
	d.reInit(325, 25 ,250);
	
	//glColor3f(WALL_COLOR);
	dofaruknormal(a,b,c);  drawRectangle(a,b,c,d,0);

	glDisable(GL_TEXTURE_2D);
	
}


/**********************************start of first floor ****************/

void mainFirstFront(int y){


	//lower
	//int y = 25;
	int id = 1;
	point a(325, y , 250);
	point b(775, y ,250);
	point c(775, y ,275);
	point d(325, y ,275);
	
	glColor3f(1,1,1);
	dofaruknormal(a,b,c);  drawRectangle(a,b,c,d,1);


	//upper
	a.reInit(325, y , 315);
	b.reInit(775, y ,315);;
	c.reInit(775, y ,325);
	d.reInit(325, y ,325);
	drawRectangle(a,b,c,d, 1);


	//window
	a.reInit(325, y , 275);
	b.reInit(385, y ,275);;
	c.reInit(385, y ,315);
	d.reInit(325, y ,315);
	drawRectangle(a,b,c,d, 1);
	
	a.increment(30, 0 ,0);
	d.increment(30, 0 ,0);
	for(int i=0; i<11; i++){
		if(i%2 == 0){
			glColor3f(.3,.3,.3);
			a.increment( 30, 0 ,0);
			b.increment( 30, 0 ,0);
			c.increment( 30, 0 ,0);
			d.increment( 30, 0 ,0);
			dofaruknormal(a,b,c);  drawRectangle(a,b,c,d,0);
		}		//glColor3f(.3,.3,.3);
		else{ 	glColor3f(1,1,1);

		a.increment( 30, 0 ,0);
		b.increment( 30, 0 ,0);
		c.increment( 30, 0 ,0);
		d.increment( 30, 0 ,0);
		dofaruknormal(a,b,c);  drawRectangle(a,b,c,d,1);

		}
	}
	glColor3f(1,1,1);
	a.reInit(715, y , 275);
	b.reInit(775, y ,275);;
	c.reInit(775, y ,315);
	d.reInit(715, y ,315);
	dofaruknormal(a,b,c);  drawRectangle(a,b,c,d,1);

}

void mainFirstSide(int x){

	//lower
	//int x = 0;
	point a(x, 25 ,250);
	point b(x, 375 ,250);
	point c(x, 375 ,275);
	point d(x, 25 ,275);
	
	glColor3f(1,1,1);
	dofaruknormal(a,b,c);
	dofaruknormal(a,b,c);  drawRectangle(a,b,c,d,1);


	//upper
	a.reInit(x, 25 ,315);
	b.reInit(x, 375 ,315);
	c.reInit(x, 375 ,325);
	d.reInit(x, 25 ,325);
	dofaruknormal(a,b,c);  drawRectangle(a,b,c,d,1);


	//window wall
	a.reInit(x, 25 ,275);
	b.reInit(x, 75 ,275);
	c.reInit(x, 75 ,315);
	d.reInit(x, 25 ,315);
	glColor3f(WALL_COLOR);
	dofaruknormal(a,b,c);  drawRectangle(a,b,c,d,1);
	//small window left
	a.reInit(x, 75 ,275);
	b.reInit(x, 105 ,275);
	c.reInit(x, 105 ,315);
	d.reInit(x, 75 ,315);
	glColor3f(0,0,0);
	dofaruknormal(a,b,c);  drawRectangle(a,b,c,d,0);

		//window wall
	a.reInit(x, 105 ,275);
	b.reInit(x, 130 ,275);
	c.reInit(x, 130 ,315);
	d.reInit(x, 105 ,315);
	glColor3f(WALL_COLOR);
	dofaruknormal(a,b,c);  drawRectangle(a,b,c,d,1);

	//big window
	a.reInit(x, 130 ,275);
	b.reInit(x, 270 ,275);
	c.reInit(x, 270 ,315);
	d.reInit(x, 130 ,315);
	glColor3f(0,0,0);
	dofaruknormal(a,b,c);  drawRectangle(a,b,c,d,0);

	//window wall
	a.reInit(x, 270 ,275);
	b.reInit(x, 295 ,275);
	c.reInit(x, 295 ,315);
	d.reInit(x, 270 ,315);
	glColor3f(WALL_COLOR);
	dofaruknormal(a,b,c);  drawRectangle(a,b,c,d,1);

	//small window left
	a.reInit(x, 295 ,275);
	b.reInit(x, 325 ,275);
	c.reInit(x, 325 ,315);
	d.reInit(x, 295 ,315);
	glColor3f(0,0,0);
	dofaruknormal(a,b,c);  drawRectangle(a,b,c,d,0);

		//window wall
	a.reInit(x, 330 ,275);
	b.reInit(x, 375 ,275);
	c.reInit(x, 375 ,315);
	d.reInit(x, 330 ,315);
	glColor3f(WALL_COLOR);
	dofaruknormal(a,b,c);  drawRectangle(a,b,c,d,1);

}

void mainFirstShade(){

	//left
	
	point a(350, 25 ,325);
	point b(350, 375 ,325);
	point c(300, 395 ,315);
	point d(300, 0 ,315);
	
	glColor3f(SHADE_COLOR);
	dofaruknormal(a,b,c);  drawRectangle(a,b,c,d,0);


	//FRONT
	a.reInit(350, 25 ,325);
	b.reInit(750, 25 ,325);
	c.reInit(780, 0 ,315);
	d.reInit(300, 0 ,315);
	dofaruknormal(a,b,c);  drawRectangle(a,b,c,d,0);

	//RIGHT
	a.reInit(750, 25 ,325);
	b.reInit(750, 350 ,325);
	c.reInit(780, 370 ,315);
	d.reInit(780, 0 ,315);
	dofaruknormal(a,b,c);  drawRectangle(a,b,c,d,0);


		


	//front
/*	glPushMatrix();{	
		glColor3d(SHADE_COLOR);
		glTranslatef(550,25,325);
		glRotatef(20,1,0,0);
		glScalef(50,7,1);
		
		//glutSolidCube(10);
	}glPopMatrix();

	//back
	glPushMatrix();{	
		glColor3d(SHADE_COLOR);
		glTranslatef(550,390,325);
		glRotatef(-20,1,0,0);
		glScalef(50,7,1);
		
		glutSolidCube(10);
	}glPopMatrix();

	
	//left
	glPushMatrix();{	
		glColor3d(SHADE_COLOR);
		glTranslatef(325,200,325);
		glRotatef(-20,0,1,0);
		//glPushMatrix();{
			glRotatef(-90,0,0,1);
			glScalef(40,7,1);
			
		//	glutSolidCube(10);
		//}glPopMatrix();
	}glPopMatrix();

	//right

	glPushMatrix();{	
		glColor3d(SHADE_COLOR);
		glTranslatef(775,200,325);
		glRotatef(20,0,1,0);
		//glPushMatrix();{
			glRotatef(-90,0,0,1);
			glScalef(40,7,1);
			
		//	glutSolidCube(10);
		//}glPopMatrix();
	}glPopMatrix();*/


	int i;
	//cylynder
	glColor3d(CYL_COLOR);	
	GLUquadricObj *quadratic;
	quadratic=gluNewQuadric();
	for( i=0;i<33;i++){
		glPushMatrix();{
			glTranslatef(310 + i*15,46,330);
			glRotatef(100,1,0,0);

			gluCylinder(quadratic,5,5,70,20,20);
		}glPopMatrix();
	}

	for( i=0;i<28;i++){
		glPushMatrix();{
			glTranslatef(350 + i*15,325,340);
			glRotatef(-100,1,0,0);
						gluCylinder(quadratic,5,5,70,20,20);
		}glPopMatrix();
	}

	for( i=0;i<22;i++){
		glPushMatrix();{
			glTranslatef(280 ,46 + i*15,305);
			glRotatef(70,0,1,0);

			gluCylinder(quadratic,5,5,70,20,20);
		}glPopMatrix();
	}

		for( i=0;i<22;i++){
		glPushMatrix();{
			glTranslatef(820 ,46 + i*15,310);
			glRotatef(-70,0,1,0);

			gluCylinder(quadratic,5,5,70,20,20);
		}glPopMatrix();
	}
}


/**********************************end of first floor ****************/


/**********************************start of second floor ****************/


void mainSecondFront(int y){
	//lower
	//int y = 25;
	point a(350, y , 335);
	point b(750, y ,335);
	point c(750, y ,365);
	point d(350, y ,365);
	
	glColor3f(1,1,1);
	dofaruknormal(a,b,c);  drawRectangle(a,b,c,d,1);


	//upper
	a.reInit(350, y , 395+10);
	b.reInit(750, y ,395+10);;
	c.reInit(750, y ,435+10);
	d.reInit(350, y ,435+10);
	dofaruknormal(a,b,c);  drawRectangle(a,b,c,d,1);


	//window
	a.reInit(350, y , 365);
	b.reInit(385, y ,355+10);;
	c.reInit(385, y ,395+10);
	d.reInit(350, y ,395+10);
	dofaruknormal(a,b,c);  drawRectangle(a,b,c,d,1);
	
	a.increment(9, 0 ,0);
	d.increment(9, 0 ,0);
	for(int i=0; i<14; i++){
		if(i%2 == 0){		glColor3f(.3,.3,.3);
			
		a.increment( 26, 0 ,0);
		b.increment( 26, 0 ,0);
		c.increment( 26, 0 ,0);
		d.increment( 26, 0 ,0);
		dofaruknormal(a,b,c);  drawRectangle(a,b,c,d,0);
		}
		else{ 	glColor3f(1,1,1);
		if(i ==5|| i==7 ) glColor3f(.3,.3,.3);


		a.increment( 26, 0 ,0);
		b.increment( 26, 0 ,0);
		c.increment( 26, 0 ,0);
		d.increment( 26, 0 ,0);
		dofaruknormal(a,b,c);  drawRectangle(a,b,c,d,1);

		}
	}

}


void mainSecondSide(int x){
	point a(x, 50 ,335);
	point b(x, 350 ,335);
	point c(x, 350 ,365);
	point d(x, 50 ,365);
	
	glColor3f(1,1,1);
	dofaruknormal(a,b,c);  drawRectangle(a,b,c,d,1);


	a.reInit(x, 50 , 395+10);
	b.reInit(x, 350 ,395+10);;
	c.reInit(x, 350 ,435+10);
	d.reInit(x, 50 ,435+10);
	dofaruknormal(a,b,c);  drawRectangle(a,b,c,d,1);


	//window
	a.reInit(x, 50 , 365);
	b.reInit(x, 83 ,355+10);;
	c.reInit(x, 83 ,395+10);
	d.reInit(x, 50 ,395+10);
	dofaruknormal(a,b,c);  drawRectangle(a,b,c,d,1);
	

	for(int i=0; i<8; i++){
		if(i%2 == 0){		glColor3f(.3,.3,.3);
			a.increment( 0, 33 ,0);
		b.increment( 0, 33 ,0);
		c.increment( 0, 33 ,0);
		d.increment( 0, 33 ,0);
		dofaruknormal(a,b,c);  drawRectangle(a,b,c,d,1);
		}
		else{ 	glColor3f(1,1,1);
		

		a.increment( 0, 33 ,0);
		b.increment( 0, 33 ,0);
		c.increment( 0, 33 ,0);
		d.increment( 0, 33 ,0);
		dofaruknormal(a,b,c);  drawRectangle(a,b,c,d,1);
		}
	}
	

}


void mainSecondShade(){

	glPushMatrix();{	
		glColor3d(SHADE_COLOR);
		glTranslatef(550,25,435);
		glRotatef(20,1,0,0);
		glScalef(45,5,1);
		
		glutSolidCube(10);
	}glPopMatrix();

	//back
	glPushMatrix();{	
		glColor3d(SHADE_COLOR);
		glTranslatef(550,370,435);
		glRotatef(-20,1,0,0);
		glScalef(45,5,1);
		
		glutSolidCube(10);
	}glPopMatrix();

	
	//left
	glPushMatrix();{	
		glColor3d(SHADE_COLOR);
		glTranslatef(325,200,435);
		glRotatef(-20,0,1,0);
		//glPushMatrix();{
			glRotatef(-90,0,0,1);
			glScalef(35,5,1);
			
			glutSolidCube(10);
		//}glPopMatrix();
	}glPopMatrix();

	//right

	glPushMatrix();{	
		glColor3d(SHADE_COLOR);
		glTranslatef(775,200,435);
		glRotatef(20,0,1,0);
		//glPushMatrix();{
			glRotatef(-90,0,0,1);
			glScalef(35,5,1);
			
			glutSolidCube(10);
		//}glPopMatrix();
	}glPopMatrix();


	int i;
	//cylynder
	glColor3d(CYL_COLOR);	
	GLUquadricObj *quadratic;
	quadratic=gluNewQuadric();
	for( i=0;i<30;i++){
		glPushMatrix();{
			glTranslatef(320 + i*15,40,455);
			glRotatef(120,1,0,0);

			gluCylinder(quadratic,5,5,50,20,20);
		}glPopMatrix();
	}

	for( i=0;i<30;i++){
		glPushMatrix();{
			glTranslatef(320 + i*15,350,455);
			glRotatef(-120,1,0,0);

			gluCylinder(quadratic,5,5,50,20,20);
		}glPopMatrix();
	}

	for( i=0;i<25;i++){
		glPushMatrix();{
			glTranslatef(290 ,25 + i*15,430);
			glRotatef(70,0,1,0);

			gluCylinder(quadratic,5,5,50,20,20);
		}glPopMatrix();
	}

	for( i=0;i<25;i++){
		glPushMatrix();{
			glTranslatef(800 ,25 + i*15,430);
			glRotatef(-70,0,1,0);

			gluCylinder(quadratic,5,5,50,20,20);
		}glPopMatrix();
	}

}



void mainSecondRoof(){
	point a(350, 50 ,440);
	point b(750, 50 ,440);
	point c(750, 200 ,600);
	point d(350, 200 ,600);
	
	glColor3f(.5,.5,.5);
	dofaruknormal(a,b,c);  drawRectangle(a,b,c,d,0);


	a.reInit(350, 350 , 440);
	b.reInit(750, 350 ,440);;
	c.reInit(750, 200 ,600);
	d.reInit(350, 200 ,600);
	dofaruknormal(a,b,c);  drawRectangle(a,b,c,d,0);
	int i;
	//cylynder
	glColor3d(CYL_COLOR);	
	GLUquadricObj *quadratic;
	quadratic=gluNewQuadric();
	for( i=0;i<27;i++){
		glPushMatrix();{
			glTranslatef(350 + i*15,195,600);
			glRotatef(133.15,1,0,0);

			gluCylinder(quadratic,5,5,213,20,20);
		}glPopMatrix();
	}

	for(i=0;i<27;i++){
		glPushMatrix();{
			glTranslatef(350 + i*15,195,605);
			glRotatef(-133.15,1,0,0);

			gluCylinder(quadratic,5,5,213,20,20);
		}glPopMatrix();
	}

	//horizontal pillar
	glPushMatrix();{	
		glColor3f(PILLAR_COLOR);
		glTranslatef(550,200,600);
	//	glRotatef(20,1,0,0);
		glScalef(42,2,2.5);
		
		glutSolidCube(10);
	}glPopMatrix();


	glColor3f(1,1,1);
	a.reInit(350, 50 , 445);
	b.reInit(350, 350 ,445);;
	c.reInit(350, 200 ,600);

	dofaruknormal(a,b,c);  drawTriangle(a,b,c);

	glColor3f(1,1,1);
	a.reInit(750, 50 , 445);
	b.reInit(750, 350 ,445);;
	c.reInit(750, 200 ,600);

	dofaruknormal(a,b,c);  drawTriangle(a,b,c);



}


/**********************************end of second floor ****************/

/**********************************start of third floor ****************/


void mainThirdFront(int y){
	//lower
	//int y = 25;
	point a(375, y , 460+20);
	point b(725, y ,460+20);
	point c(725, y ,480+20);
	point d(375, y ,480+20);
	
	glColor3f(1,1,1);
	dofaruknormal(a,b,c);  drawRectangle(a,b,c,d,1);


	//upper
	a.reInit(375, y , 540);
	b.reInit(725, y ,540);;
	c.reInit(725, y ,560);
	d.reInit(375, y ,560);
	dofaruknormal(a,b,c);  drawRectangle(a,b,c,d,1);


	//window
	a.reInit(375, y , 500);
	b.reInit(415, y ,500);;
	c.reInit(415, y ,540);
	d.reInit(375, y ,540);
	dofaruknormal(a,b,c);  drawRectangle(a,b,c,d,1);
	
	a.increment(10, 0 ,0);
	d.increment(10, 0 ,0);
	for(int i=0; i<9; i++){
		if(i%2 == 0){		glColor3f(.3,.3,.3);
			a.increment( 30, 0 ,0);
		b.increment( 30, 0 ,0);
		c.increment( 30, 0 ,0);
		d.increment( 30, 0 ,0);
		dofaruknormal(a,b,c);  drawRectangle(a,b,c,d,0);
		
		}
		else {	glColor3f(1,1,1);

		a.increment( 30, 0 ,0);
		b.increment( 30, 0 ,0);
		c.increment( 30, 0 ,0);
		d.increment( 30, 0 ,0);
		dofaruknormal(a,b,c);  drawRectangle(a,b,c,d,1);
		}
	}
	
	glColor3f(1,1,1);
	a.reInit(685, y , 500);
	b.reInit(725, y ,500);;
	c.reInit(725, y ,540);
	d.reInit(685, y ,540);
	dofaruknormal(a,b,c);  drawRectangle(a,b,c,d,1);

}


void mainThirdSide(int x){
	point a(x, 75 , 460+20);
	point b(x, 75 ,560);
	point c(x,325 ,560);
	point d(x, 325,480);
	
	glColor3f(1,1,1);
	dofaruknormal(a,b,c);  drawRectangle(a,b,c,d,1);
}




void mainThirdRoof(){
	point a(375, 75 ,560);
	point b(450, 75 ,620);
	point c(450, 325 ,620);
	point d(375, 325 ,560);
	
	glColor3f(.5,.5,.5);
	dofaruknormal(a,b,c);  drawRectangle(a,b,c,d,0);


	a.reInit(525, 75 , 560);
	b.reInit(525, 325 ,560);;
	c.reInit(450, 325 ,620);
	d.reInit(450, 75 , 620);
	dofaruknormal(a,b,c);  drawRectangle(a,b,c,d,0);


	a.reInit(575, 75 , 560);
	b.reInit(575, 325 ,560);;
	c.reInit(650, 325 ,620);
	d.reInit(650, 75 , 620);
	dofaruknormal(a,b,c);  drawRectangle(a,b,c,d,0);


	a.reInit(725, 75 , 560);
	b.reInit(725, 325 ,560);;
	c.reInit(650, 325 ,620);
	d.reInit(650, 75 , 620);
	dofaruknormal(a,b,c);  drawRectangle(a,b,c,d,0);
	int i;
	//cylynder
	glColor3d(CYL_COLOR);	
	GLUquadricObj *quadratic;
	quadratic=gluNewQuadric();
	for( i=0;i<17;i++){
		glPushMatrix();{
			glTranslatef(380 ,75 + i*15,560);
			glRotatef(50,0,1,0);

			gluCylinder(quadratic,5,5,96,20,20);
		}glPopMatrix();
	}


	for( i=0;i<17;i++){
		glPushMatrix();{
			glTranslatef(580 ,75 + i*15,560);
			glRotatef(50,0,1,0);

			gluCylinder(quadratic,5,5,96,20,20);
		}glPopMatrix();
	}

	for( i=0;i<17;i++){
		glPushMatrix();{
			glTranslatef(530 ,75 + i*15,560);
			glRotatef(-50,0,1,0);

			gluCylinder(quadratic,5,5,96,20,20);
		}glPopMatrix();
	}

	for( i=0;i<17;i++){
		glPushMatrix();{
			glTranslatef(730 ,75 + i*15,560);
			glRotatef(-50,0,1,0);

			gluCylinder(quadratic,5,5,96,20,20);
		}glPopMatrix();
	}



	//horizontal pillar
	glPushMatrix();{	
		glColor3f(PILLAR_COLOR);
		glTranslatef(450,200,620);
		glRotatef(90,0,0,1);
		glScalef(25,1,2);
		
		glutSolidCube(10);
	}glPopMatrix();


	glPushMatrix();{	
	//	glColor3d(1,1,1);
		glTranslatef(650,200,620);
		glRotatef(90,0,0,1);
		glScalef(25,1,2);
		
		glutSolidCube(10);
	}glPopMatrix();



	//triangle

	glColor3f(1,1,1);
	a.reInit(450, 75 , 620);
	b.reInit(525, 75 ,560);;
	c.reInit(375, 75 ,560);

	dofaruknormal(a,b,c);  drawTriangle(a,b,c);

	glColor3f(1,1,1);
	a.reInit(450, 325 , 620);
	b.reInit(525, 325 ,560);;
	c.reInit(375, 325 ,560);

	dofaruknormal(a,b,c);  drawTriangle(a,b,c);

	glColor3f(1,1,1);
	a.reInit(650, 75 , 620);
	b.reInit(725, 75 ,560);;
	c.reInit(575, 75 ,560);

	dofaruknormal(a,b,c);  drawTriangle(a,b,c);


	glColor3f(1,1,1);
	a.reInit(650, 325 , 620);
	b.reInit(725, 325 ,560);;
	c.reInit(575, 325 ,560);

	dofaruknormal(a,b,c);  drawTriangle(a,b,c);



}




void mainThirdShade(){
	//front
	glPushMatrix();{	
		glColor3d(SHADE_COLOR);
		glTranslatef(550,75,560);
		glRotatef(20,1,0,0);
		glScalef(40,5,1);
		
		glutSolidCube(10);
	}glPopMatrix();

	//back
	glPushMatrix();{	
		glColor3d(SHADE_COLOR);
		glTranslatef(550,325,560);
		glRotatef(-20,1,0,0);
		glScalef(40,7,1);
		
		glutSolidCube(10);
	}glPopMatrix();

	
	//left
	glPushMatrix();{	
		glColor3d(SHADE_COLOR);
		glTranslatef(375,200,560);
		glRotatef(-20,0,1,0);
		//glPushMatrix();{
			glRotatef(-90,0,0,1);
			glScalef(30,5,1);
			
			glutSolidCube(10);
		//}glPopMatrix();
	}glPopMatrix();

	//right

	glPushMatrix();{	
		glColor3d(SHADE_COLOR);
		glTranslatef(725,200,560);
		glRotatef(20,0,1,0);
		//glPushMatrix();{
			glRotatef(-90,0,0,1);
			glScalef(30,5,1);
			
			glutSolidCube(10);
		//}glPopMatrix();
	}glPopMatrix();


	int i;
	//cylynder
	glColor3d(CYL_COLOR);	
	GLUquadricObj *quadratic;
	quadratic=gluNewQuadric();
	for( i=0;i<25;i++){
		glPushMatrix();{
			glTranslatef(375 + i*15,80,575);
			glRotatef(120,1,0,0);

			gluCylinder(quadratic,5,5,40,20,20);
		}glPopMatrix();
	}

	for( i=0;i<25;i++){
		glPushMatrix();{
			glTranslatef(375 + i*15,330,575);
			glRotatef(-120,1,0,0);

			gluCylinder(quadratic,5,5,40,20,20);
		}glPopMatrix();
	}

	for( i=0;i<20;i++){
		glPushMatrix();{
			glTranslatef(340 ,55 + i*15,560);
			glRotatef(70,0,1,0);

			gluCylinder(quadratic,5,5,40,20,20);
		}glPopMatrix();
	}

		for( i=0;i<20;i++){
		glPushMatrix();{
			glTranslatef(755 ,55 + i*15,560);
			glRotatef(-70,0,1,0);

			gluCylinder(quadratic,5,5,40,20,20);
		}glPopMatrix();
	}
}



/**********************************end of third floor ****************/






/**********************************start of fourth floor ****************/

void mainFourthFront(int y){
	//lower
	//int y = 25;
	point a(400, y , 570);
	point b(700, y ,570);
	point c(700, y ,590);
	point d(400, y ,590);
	
	glColor3f(1,1,1);
	dofaruknormal(a,b,c);  drawRectangle(a,b,c,d,1);


	//upper
	a.reInit(400, y , 630);
	b.reInit(700, y ,630);;
	c.reInit(700, y ,650);
	d.reInit(400, y ,650);
	dofaruknormal(a,b,c);  drawRectangle(a,b,c,d,1);


	//window
	a.reInit(400, y , 590);
	b.reInit(470, y ,590);;
	c.reInit(470, y ,630);
	d.reInit(400, y ,630);
	dofaruknormal(a,b,c);  drawRectangle(a,b,c,d,1);
	
	glColor3f(.3,.3,.3);
	a.reInit(470, y , 590);
	b.reInit(510, y ,590);;
	c.reInit(510, y ,630);
	d.reInit(470, y ,630);
	dofaruknormal(a,b,c);  drawRectangle(a,b,c,d,0);


	glColor3f(1,1,1);
	a.reInit(510, y , 590);
	b.reInit(590, y ,590);;
	c.reInit(590, y ,630);
	d.reInit(510, y ,630);
	dofaruknormal(a,b,c);  drawRectangle(a,b,c,d,1);



	glColor3f(.3,.3,.3);
	a.reInit(590, y , 590);
	b.reInit(630, y ,590);;
	c.reInit(630, y ,630);
	d.reInit(590, y ,630);
	dofaruknormal(a,b,c);  drawRectangle(a,b,c,d,0);


	glColor3f(1,1,1);
	a.reInit(630, y , 590);
	b.reInit(700, y ,590);;
	c.reInit(700, y ,630);
	d.reInit(630, y ,630);
	dofaruknormal(a,b,c);  drawRectangle(a,b,c,d,1);


}


void mainFourthSide(int x){
	point a(x, 100 , 570);
	point b(x,300 ,570);
	point c(x, 300 ,590);
	point d(x, 100 ,590);
	
	glColor3f(1,1,1);
	dofaruknormal(a,b,c);  drawRectangle(a,b,c,d,1);


	//upper
	a.reInit(x, 100 , 630);
	b.reInit(x, 300 ,630);;
	c.reInit(x,300 ,650);
	d.reInit(x, 100 ,650);
	dofaruknormal(a,b,c);  drawRectangle(a,b,c,d,1);


	//window
	a.reInit(x, 100 , 590);
	b.reInit(x, 140 ,590);;
	c.reInit(x, 140 ,630);
	d.reInit(x, 100 ,630);
	dofaruknormal(a,b,c);  drawRectangle(a,b,c,d,1);
	
	glColor3f(.3,.3,.3);
	a.reInit(x, 140 , 590);
	b.reInit(x, 180 ,590);;
	c.reInit(x, 180 ,630);
	d.reInit(x, 140 ,630);
	dofaruknormal(a,b,c);  drawRectangle(a,b,c,d,0);


	glColor3f(1,1,1);
	a.reInit(x, 180 , 590);
	b.reInit(x, 220 ,590);;
	c.reInit(x, 220 ,630);
	d.reInit(x, 180 ,630);
	dofaruknormal(a,b,c);  drawRectangle(a,b,c,d,1);



	glColor3f(.3,.3,.3);
	a.reInit(x, 220 , 590);
	b.reInit(x, 260 ,590);;
	c.reInit(x, 260 ,630);
	d.reInit(x, 220 ,630);
	dofaruknormal(a,b,c);  drawRectangle(a,b,c,d,0);


	glColor3f(1,1,1);
	a.reInit(x, 260 , 590);
	b.reInit(x, 300 ,590);;
	c.reInit(x, 300 ,630);
	d.reInit(x, 260 ,630);
	dofaruknormal(a,b,c);  drawRectangle(a,b,c,d,1);

}

void mainFourthRoof(){
	//front
	point a(380,80 ,645);
	point b(720, 80 ,645);
	point c(675, 135 ,670);
	point d(425, 135 ,670);
	
	glColor3f(.5,.5,.5);
	dofaruknormal(a,b,c);  drawRectangle(a,b,c,d,0);

//back
	a.reInit(380, 280+50 , 635);
	b.reInit(720, 280+50 ,635);;
	c.reInit(675, 225+50 ,665);
	d.reInit(425, 225+50 , 665);
	dofaruknormal(a,b,c);  drawRectangle(a,b,c,d,0);
//left

	a.reInit(425, 135 , 670);
	b.reInit(425, 275 ,665);;
	c.reInit(380, 330 ,635);
	d.reInit(380, 80 , 645);
	dofaruknormal(a,b,c);  drawRectangle(a,b,c,d,0);

//right
	a.reInit(675, 135 , 670);
	b.reInit(675, 275 ,665);;
	c.reInit(720, 330 ,635);
	d.reInit(720, 80 , 645);
	dofaruknormal(a,b,c);  drawRectangle(a,b,c,d,0);
	int i;
	//cylynder
	glColor3d(CYL_COLOR);	
	GLUquadricObj *quadratic;
	quadratic=gluNewQuadric();
	for( i=0;i<22;i++){
		glPushMatrix();{
			glTranslatef(380 + i*15,75 ,645);
			glRotatef(-60,1,0,0);

			gluCylinder(quadratic,5,5,60,20,20);
		}glPopMatrix();
	}


	for( i=0;i<22;i++){
		glPushMatrix();{
			glTranslatef(380+ i*15 ,330 ,640);
			glRotatef(60,1,0,0);

			gluCylinder(quadratic,5,5,60,20,20);
		}glPopMatrix();
	}

	for( i=0;i<15;i++){
		glPushMatrix();{
			glTranslatef(380 ,90 + i*15,645);
			glRotatef(55.7,0,1,0);

			gluCylinder(quadratic,5,5,50,20,20);
		}glPopMatrix();
	}

	for( i=0;i<15;i++){
		glPushMatrix();{
			glTranslatef(720 ,90 + i*15,645);
			glRotatef(-55,0,1,0);

			gluCylinder(quadratic,5,5,50,20,20);
		}glPopMatrix();
	}


	//another tricone

	//left
	glColor3f(.5,.5,.5);
	a.reInit(400, 100 , 650);
	b.reInit(400, 300 ,650);;
	c.reInit(550, 300 ,725);
	d.reInit(550, 100 , 725);
	dofaruknormal(a,b,c);  drawRectangle(a,b,c,d,0);

//right
	a.reInit(700, 100 , 650);
	b.reInit(700, 300 ,650);;
	c.reInit(550, 300 ,725);
	d.reInit(550, 100 , 725);
	dofaruknormal(a,b,c);  drawRectangle(a,b,c,d,0);

		//triangle

	glColor3f(1,1,1);
	a.reInit(400, 100 , 650);
	b.reInit(700, 100 ,650);;
	c.reInit(550, 100 ,725);

	dofaruknormal(a,b,c);  drawTriangle(a,b,c);

	glColor3f(1,1,1);
	a.reInit(400, 300 , 650);
	b.reInit(700, 300 ,650);;
	c.reInit(550, 300 ,725);

	dofaruknormal(a,b,c);  drawTriangle(a,b,c);


		//cylynder
	glColor3f(CYL_COLOR);
	for( i=0;i<14;i++){
		glPushMatrix();{
			glTranslatef(400 ,100 + i*15,660);
			glRotatef(65,0,1,0);

			gluCylinder(quadratic,5,5,168,20,20);
		}glPopMatrix();
	}

	glColor3f(CYL_COLOR);
	for( i=0;i<14;i++){
		glPushMatrix();{
			glTranslatef(700 ,100 + i*15,660);
			glRotatef(-65,0,1,0);

			gluCylinder(quadratic,5,5,168,20,20);
		}glPopMatrix();
	}







	//horizontal pillar
	glPushMatrix();{	
		glColor3f(PILLAR_COLOR);
		glTranslatef(550,200,730);
		glRotatef(90,0,0,1);
		glScalef(20,1,2);
		
		glutSolidCube(10);
	}glPopMatrix();







}

/**********************************end of fourth floor ****************/


/**********************************start of fifth floor ****************/


void mainFifthFront(int y){

	point a(425, y ,670);
	point b(675, y ,670);
	point c(675, y ,740);
	point d(425, y ,740);
	
	glColor3f(1,1,1);
	dofaruknormal(a,b,c);  drawRectangle(a,b,c,d,1);


	//upper
	a.reInit(425, y ,770);
	b.reInit(675, y ,770);
	c.reInit(675, y ,800);
	d.reInit(425, y ,800);
	dofaruknormal(a,b,c);  drawRectangle(a,b,c,d,1);


	//window
	a.reInit(425, y ,740);
	b.reInit(460, y ,740);
	c.reInit(460, y ,770);
	d.reInit(425, y ,770);
	dofaruknormal(a,b,c);  drawRectangle(a,b,c,d,1);
	
	a.increment(15, 0 ,0);
	d.increment(15, 0 ,0);
	for(int i=0; i<9; i++){
		if(i%2 == 0){		glColor3f(.3,.3,.3);
			a.increment(20, 0 ,0);
		b.increment(20, 0 ,0);
		c.increment(20, 0 ,0);
		d.increment(20, 0 ,0);
		dofaruknormal(a,b,c);  drawRectangle(a,b,c,d,0);
		}
		else{ 	glColor3f(1,1,1);

		a.increment(20, 0 ,0);
		b.increment(20, 0 ,0);
		c.increment(20, 0 ,0);
		d.increment(20, 0 ,0);
		dofaruknormal(a,b,c);  drawRectangle(a,b,c,d,1);
		}
	}
	glColor3f(1,1,1);
	a.increment(20, 0 ,0);
	b.increment(35, 0 ,0);
	c.increment(35, 0 ,0);
	d.increment(20, 0 ,0);
	dofaruknormal(a,b,c);  drawRectangle(a,b,c,d,1);

}

void mainFifthSide(int x){

	point a(x, 125 ,670);
	point b(x, 275 ,670);
	point c(x, 275 ,740);
	point d(x, 125 ,740);
	
	glColor3f(1,1,1);
	dofaruknormal(a,b,c);  drawRectangle(a,b,c,d,1);


	a.reInit(x, 125 ,770);
	b.reInit(x, 275 ,770);
	c.reInit(x, 275 ,800);
	d.reInit(x, 125 ,800);
	dofaruknormal(a,b,c);  drawRectangle(a,b,c,d,1);


	//window
	a.reInit(x, 125 ,740);
	b.reInit(x, 146.43 ,740);
	c.reInit(x, 146.43 ,770);
	d.reInit(x, 125 ,770);
	dofaruknormal(a,b,c);  drawRectangle(a,b,c,d,1);
	

	for(int i=0; i<6; i++){
		if(i%2 == 0){		glColor3f(.3,.3,.3);
			a.increment( 0,21.43 ,0);
		b.increment( 0,21.43 ,0);
		c.increment( 0,21.43 ,0);
		d.increment(0,21.43 ,0);
		dofaruknormal(a,b,c);  drawRectangle(a,b,c,d,0);
		}
		else{ 	glColor3f(1,1,1);

		a.increment( 0,21.43 ,0);
		b.increment( 0,21.43 ,0);
		c.increment( 0,21.43 ,0);
		d.increment(0,21.43 ,0);
		dofaruknormal(a,b,c);  drawRectangle(a,b,c,d,1);
		}
	}

/*	a.reInit(0, y ,100);
	b.reInit(70, y ,150);
	c.reInit(140, y ,100);
	glColor3f(1,1,1);
	dofaruknormal(a,b,c);  drawTriangle(a,b,c);*/

}


void mainFifthShade(){
		//front
	glPushMatrix();{	
		glColor3d(SHADE_COLOR);
		glTranslatef(550,120,800);
		glRotatef(20,1,0,0);
		glScalef(32,5,1);
		
		glutSolidCube(10);
	}glPopMatrix();

	//back
	glPushMatrix();{	
		glColor3d(SHADE_COLOR);
		glTranslatef(550,270,800);
		glRotatef(-20,1,0,0);
		glScalef(32,5,1);
		
		glutSolidCube(10);
	}glPopMatrix();

	
	//left
	glPushMatrix();{	
		glColor3d(SHADE_COLOR);
		glTranslatef(420,180,800);
		glRotatef(-20,0,1,0);
		//glPushMatrix();{
			glRotatef(-90,0,0,1);
			glScalef(18,5,1);
			
			glutSolidCube(10);
		//}glPopMatrix();
	}glPopMatrix();

	//right

	glPushMatrix();{	
		glColor3d(SHADE_COLOR);
		glTranslatef(690,190,800);
		glRotatef(20,0,1,0);
		//glPushMatrix();{
			glRotatef(-90,0,0,1);
			glScalef(17,5,1);
			
			glutSolidCube(10);
		//}glPopMatrix();
	}glPopMatrix();


	int i;
	//cylynder
	glColor3d(CYL_COLOR);	
	GLUquadricObj *quadratic;
	quadratic=gluNewQuadric();
	for( i=0;i<20;i++){
		glPushMatrix();{
			glTranslatef(400 + i*15,140,820);
			glRotatef(105,1,0,0);

			gluCylinder(quadratic,5,5,60,20,20);
		}glPopMatrix();
	}

	for( i=0;i<20;i++){
		glPushMatrix();{
			glTranslatef(400 + i*15,240,815);
			glRotatef(-105,1,0,0);

			gluCylinder(quadratic,5,5,60,20,20);
		}glPopMatrix();
	}

	for( i=0;i<12;i++){
		glPushMatrix();{
			glTranslatef(380 ,100 + i*15,800);
			glRotatef(70,0,1,0);

			gluCylinder(quadratic,5,5,50,20,20);
		}glPopMatrix();
	}


		for( i=0;i<12;i++){
		glPushMatrix();{
			glTranslatef(715 ,100 + i*15,805);
			glRotatef(-70,0,1,0);

			gluCylinder(quadratic,5,5,50,20,20);
		}glPopMatrix();
	}

}


void mainFifthRoof(){
	point a(425, 125 ,800);
	point b(675, 125 ,800);
	point c(675, 200 ,875);
	point d(425, 200 ,875);
	
	glColor3f(.5,.5,.5);
	dofaruknormal(a,b,c);  drawRectangle(a,b,c,d,0);

	int i;
	a.reInit(425, 275 , 800);
	b.reInit(675, 275 ,800);;
	c.reInit(675, 200 ,875);
	d.reInit(425, 200 ,875);
	dofaruknormal(a,b,c);  drawRectangle(a,b,c,d,0);

	//cylynder
	glColor3d(CYL_COLOR);	
	GLUquadricObj *quadratic;
	quadratic=gluNewQuadric();
	for( i=0;i<17;i++){
		glPushMatrix();{
			glTranslatef(425 + i*15,120,800);
			glRotatef(-45,1,0,0);

			gluCylinder(quadratic,5,5,110,20,20);
		}glPopMatrix();
	}

		for( i=0;i<17;i++){
		glPushMatrix();{
			glTranslatef(425 + i*15,280,800);
			glRotatef(45,1,0,0);

			gluCylinder(quadratic,5,5,110,20,20);
		}glPopMatrix();
	}



	//horizontal pillar
	glPushMatrix();{	
		glColor3f(PILLAR_COLOR);
		glTranslatef(550,200,880);
	//	glRotatef(20,1,0,0);
		glScalef(25,1.5,1.5);
		
		glutSolidCube(10);
	}glPopMatrix();


	glColor3f(1,1,1);
	a.reInit(425, 125 , 800);
	b.reInit(425, 200 ,875);;
	c.reInit(425, 275 ,800);

	dofaruknormal(a,b,c);  drawTriangle(a,b,c);

	glColor3f(1,1,1);
	a.reInit(675, 125 , 800);
	b.reInit(675, 200 ,875);;
	c.reInit(675, 275 ,800);

	dofaruknormal(a,b,c);  drawTriangle(a,b,c);
	
}

/**********************************end of fifth floor ****************/



void mainFirstFloor(){
	mainFirstFront(25);
	mainFirstFront(375);
	mainFirstSide(325);
	mainFirstSide(775);
	mainFirstShade();

}


void mainSecondFloor(){
	mainSecondFront(50);
	mainSecondFront(350);
	mainSecondSide(350);
	mainSecondSide(750);
	mainSecondShade();
	mainSecondRoof();

}


void mainThirdFloor(){
	mainThirdFront(75);
	mainThirdFront(325);
	mainThirdSide(375);
	mainThirdSide(725);
	mainThirdShade();
	mainThirdRoof();

}

void mainFourthFloor(){
	mainFourthFront(100);
	mainFourthFront(300);
	mainFourthSide(400);
	mainFourthSide(700);
//	mainFourthShade();
	mainFourthRoof();

}

void mainFifthFloor(){
	mainFifthFront(125);
	mainFifthFront(275);
	mainFifthSide(425);
	mainFifthSide(675);
	mainFifthShade();
	mainFifthRoof();

}


void mainBuilding(){
	baseofMain();
	mainFirstFloor();
	mainSecondFloor();
	mainThirdFloor();
	mainFourthFloor();
	mainFifthFloor();
}

/****************************end of main building *****************************/

void baseofLBuilding(){
	//L er choto part
	glEnable(GL_TEXTURE_2D);
	glBindTexture(GL_TEXTURE_2D,baseimg);
	//glColor3f(0.2, 1, 0.2);
	float pix = 1;
	
	point a(750, 400 ,0);
	point b(750, 700 ,0);
	point c(750, 700 ,210);
	point d(750, 400 ,210);
	
	dofaruknormal(a,b,c);
	
	glBegin(GL_QUADS);{
		//glNormal3f(0,0,1);
		glColor3f(1,1,1);
		glTexCoord2f(0,0);
		glVertex3f(750, 400 ,0);
		glTexCoord2f(0,pix);
		glVertex3f(750, 700 ,0);
		glTexCoord2f(pix,pix);
		glVertex3f(750, 700 ,210);
		glTexCoord2f(pix,0);
		glVertex3f(750, 400 ,210);
	}glEnd();

		

	
	
	glColor3f(BASE_COLOR);
	//drawRectangle(a,b,c,d);


	//L er long part
	
	
	a.reInit(750, 700 ,0);
	b.reInit(200, 700 ,0);
	c.reInit(200, 700 ,210);
	d.reInit(750, 700 ,210);
	dofaruknormal(a,b,c);
	glBegin(GL_QUADS);{
		//glNormal3f(0,0,1);
		glColor3f(1,1,1);
		glTexCoord2f(0,0);
		glVertex3f(750, 700 ,0);
		glTexCoord2f(0,pix);
		glVertex3f(200, 700 ,0);
		glTexCoord2f(pix,pix);
		glVertex3f(200, 700 ,210);
		glTexCoord2f(pix,0);
		glVertex3f(750, 700 ,210);
	}glEnd();

	
	//drawRectangle(a,b,c,d);

	//L er long er lej
	
	
	a.reInit(200, 700 ,0);
	b.reInit(200, 600 ,0);
	c.reInit(200, 600 ,210);
	d.reInit(200, 700 ,210);
	dofaruknormal(a,b,c);
	glBegin(GL_QUADS);{
		//glNormal3f(0,0,1);
		glColor3f(1,1,1);
		glTexCoord2f(0,0);
		glVertex3f(200, 700 ,0);
		glTexCoord2f(0,pix);
		glVertex3f(200, 600 ,0);
		glTexCoord2f(pix,pix);
		glVertex3f(200, 600 ,210);
		glTexCoord2f(pix,0);
		glVertex3f(200, 700 ,210);
	}glEnd();



	
	//drawRectangle(a,b,c,d);

		
	glDisable(GL_TEXTURE_2D);
	
}



void LFirstFloor(){
	//bottom
	//L er choto part
	point a(750, 400 ,210);
	point b(750, 700 ,210);
	point c(750, 700 ,240);
	point d(750, 400 ,240);
	
	glColor3f(1,1,1);
	dofaruknormal(a,b,c);  drawRectangle(a,b,c,d,1);


	//L er long part
	a.reInit(750, 700 ,210);
	b.reInit(200, 700 ,210);
	c.reInit(200, 700 ,240);
	d.reInit(750, 700 ,240);
	dofaruknormal(a,b,c);  drawRectangle(a,b,c,d,1);

	//L er long er lej
	a.reInit(200, 700 ,210);
	b.reInit(200, 600 ,210);
	c.reInit(200, 600 ,240);
	d.reInit(200, 700 ,240);
	dofaruknormal(a,b,c);  drawRectangle(a,b,c,d,1);

	//top
	//L er choto part
	a.reInit(750, 400 ,280);
	b.reInit(750, 700 ,280);
	c.reInit(750, 700 ,310);
	d.reInit(750, 400 ,310);
	
	glColor3f(1,1,1);
	dofaruknormal(a,b,c);  drawRectangle(a,b,c,d,1);


	//L er long part
	a.reInit(750, 700 ,280);
	b.reInit(200, 700 ,280);
	c.reInit(200, 700 ,310);
	d.reInit(750, 700 ,310);
	dofaruknormal(a,b,c);  drawRectangle(a,b,c,d,1);

	//L er long er lej
	a.reInit(200, 700 ,280);
	b.reInit(200, 600 ,280);
	c.reInit(200, 600 ,310);
	d.reInit(200, 700 ,310);
	dofaruknormal(a,b,c);  drawRectangle(a,b,c,d,1);

	int i;
	//window
	a.reInit(750, 400 , 240);
	b.reInit(750, 435 ,240);;
	c.reInit(750, 435 ,280);
	d.reInit(750, 400 ,280);
	dofaruknormal(a,b,c);  drawRectangle(a,b,c,d,1);
	
	a.increment(0, 10 ,0);
	d.increment(0, 10 ,0);
	for( i=0; i<9; i++){
		if(i%2 == 0){		glColor3f(.3,.3,.3);
		a.increment( 0, 25 ,0);
		b.increment( 0, 25 ,0);
		c.increment( 0, 25 ,0);
		d.increment( 0, 25 ,0);
		dofaruknormal(a,b,c);  drawRectangle(a,b,c,d,0);
		
		}
		else{ 	glColor3f(1,1,1);
		

		a.increment( 0, 25 ,0);
		b.increment( 0, 25 ,0);
		c.increment( 0, 25 ,0);
		d.increment( 0, 25 ,0);
		dofaruknormal(a,b,c);  drawRectangle(a,b,c,d,1);

		}
	}
	
	glColor3f(1,1,1);
	a.increment( 0, 25 ,0);
	b.increment( 0, 40 ,0);
	c.increment( 0, 40 ,0);
	d.increment( 0, 25 ,0);
	dofaruknormal(a,b,c);  drawRectangle(a,b,c,d,1);


	//L er long part

	a.reInit(750, 700 , 240);
	b.reInit(700, 700 ,240);;
	c.reInit(700, 700 ,280);
	d.reInit(750, 700 ,280);
	dofaruknormal(a,b,c);  drawRectangle(a,b,c,d,1);
	
	a.increment(-20, 0 ,0);
	d.increment(-20, 0 ,0);
	for( i=0; i<15; i++){
		if(i%2 == 0 || i==7 || i==9 ){		glColor3f(.3,.3,.3);
			a.increment( -30, 0 ,0);
			b.increment( -30, 0 ,0);
			c.increment( -30, 0 ,0);
			d.increment( -30, 0 ,0);
			dofaruknormal(a,b,c);  drawRectangle(a,b,c,d,0);
		}
		else{ 	glColor3f(1,1,1);
		//if(i==7 || i==9)  glColor3f(.3,.3,.3);
		

		a.increment( -30, 0 ,0);
		b.increment( -30, 0 ,0);
		c.increment( -30, 0 ,0);
		d.increment( -30, 0 ,0);
		dofaruknormal(a,b,c);  drawRectangle(a,b,c,d,1);
		}
	}
	
	glColor3f(1,1,1);
	a.increment( -30, 0 ,0);
	b.increment( -50, 0 ,0);
	c.increment( -50, 0 ,0);
	d.increment( -30, 0 ,0);
	dofaruknormal(a,b,c);  drawRectangle(a,b,c,d,1);

	//L er large er lej
	a.reInit(200, 700 ,240);
	b.reInit(200, 600 ,240);
	c.reInit(200, 600 ,280);
	d.reInit(200, 700 ,280);
	dofaruknormal(a,b,c);  drawRectangle(a,b,c,d,1);
	

}

void LFirstShade(){
	point a(750, 400 ,310);
	point b(750, 700 ,310);
	point c(800, 725 ,290);
	point d(800, 375 ,290);
	
	glColor3f(.5,.5,.5);
	dofaruknormal(a,b,c);  drawRectangle(a,b,c,d,0);

	
	 a.reInit(750, 700 ,310);
	 b.reInit(800, 725 ,290);
	 c.reInit(175, 725 ,290);
	 d.reInit(200, 700 ,310);
	
	glColor3f(.5,.5,.5);
	dofaruknormal(a,b,c);  drawRectangle(a,b,c,d,0);


	
	 a.reInit(175, 725 ,290);
	 b.reInit(200, 700 ,310);
	  c.reInit(200, 600 ,310);
	 d.reInit(175, 600 ,290);
	
	glColor3f(.5,.5,.5);
	dofaruknormal(a,b,c);  drawRectangle(a,b,c,d,0);



	//cylinder
	int i;
	glColor3d(CYL_COLOR);	
	GLUquadricObj *quadratic;
	quadratic=gluNewQuadric();
	for( i=0;i<23;i++){
		glPushMatrix();{
			glTranslatef(800 ,400+ i*15,300);
			glRotatef(-75,0,1,0);

			gluCylinder(quadratic,5,5,55,20,20);
		}glPopMatrix();
	}

		for( i=0;i<37;i++){
		glPushMatrix();{
			glTranslatef(200 + i*15,740,300);
			glRotatef(70,1,0,0);

			gluCylinder(quadratic,5,5,50,20,20);
		}glPopMatrix();
	}

		for( i=0;i<10;i++){
		glPushMatrix();{
			glTranslatef(170 ,600+ i*15,290);
			glRotatef(60,0,1,0);

			gluCylinder(quadratic,5,5,45,20,20);
		}glPopMatrix();
	}


}

void LSecondFloor(){
		//bottom
	//L er choto part
	point a(750, 400 ,310);
	point b(750, 700 ,310);
	point c(750, 700 ,340);
	point d(750, 400 ,340);
	
	glColor3f(1,1,1);
	dofaruknormal(a,b,c);  drawRectangle(a,b,c,d,1);


	//L er long part
	a.reInit(750, 700 ,310);
	b.reInit(200, 700 ,310);
	c.reInit(200, 700 ,340);
	d.reInit(750, 700 ,340);
	dofaruknormal(a,b,c);  drawRectangle(a,b,c,d,1);

	//L er long er lej
	a.reInit(200, 700 ,310);
	b.reInit(200, 600 ,310);
	c.reInit(200, 600 ,340);
	d.reInit(200, 700 ,340);
	dofaruknormal(a,b,c);  drawRectangle(a,b,c,d,1);

	//top
	//L er choto part
	a.reInit(750, 400 ,380);
	b.reInit(750, 700 ,380);
	c.reInit(750, 700 ,410);
	d.reInit(750, 400 ,410);
	
	glColor3f(1,1,1);
	dofaruknormal(a,b,c);  drawRectangle(a,b,c,d,1);


	//L er long part
	a.reInit(750, 700 ,380);
	b.reInit(200, 700 ,380);
	c.reInit(200, 700 ,410);
	d.reInit(750, 700 ,410);
	dofaruknormal(a,b,c);  drawRectangle(a,b,c,d,1);

	//L er long er lej
	a.reInit(200, 700 ,380);
	b.reInit(200, 600 ,380);
	c.reInit(200, 600 ,410);
	d.reInit(200, 700 ,410);
	dofaruknormal(a,b,c);  drawRectangle(a,b,c,d,1);


	//window
	a.reInit(750, 400 , 340);
	b.reInit(750, 435 ,340);;
	c.reInit(750, 435 ,380);
	d.reInit(750, 400 ,380);
	dofaruknormal(a,b,c);  drawRectangle(a,b,c,d,1);
	int i;
	a.increment(0, 10 ,0);
	d.increment(0, 10 ,0);
	for( i=0; i<9; i++){
		if(i%2 == 0){		glColor3f(.3,.3,.3);
			a.increment( 0, 25 ,0);
		b.increment( 0, 25 ,0);
		c.increment( 0, 25 ,0);
		d.increment( 0, 25 ,0);
		dofaruknormal(a,b,c);  drawRectangle(a,b,c,d,0);
		}
		else {	glColor3f(1,1,1);
		

		a.increment( 0, 25 ,0);
		b.increment( 0, 25 ,0);
		c.increment( 0, 25 ,0);
		d.increment( 0, 25 ,0);
		dofaruknormal(a,b,c);  drawRectangle(a,b,c,d,1);

		}
	}
	
	glColor3f(1,1,1);
	a.increment( 0, 25 ,0);
	b.increment( 0, 40 ,0);
	c.increment( 0, 40 ,0);
	d.increment( 0, 25 ,0);
	dofaruknormal(a,b,c);  drawRectangle(a,b,c,d,1);


	//L er long part

	a.reInit(750, 700 , 340);
	b.reInit(700, 700 ,340);;
	c.reInit(700, 700 ,380);
	d.reInit(750, 700 ,380);
	dofaruknormal(a,b,c);  drawRectangle(a,b,c,d,1);
	
	a.increment(-20, 0 ,0);
	d.increment(-20, 0 ,0);
	for( i=0; i<15; i++){
		if(i%2 == 0 || i==7 || i==9){		glColor3f(.3,.3,.3);
		a.increment( -30, 0 ,0);
		b.increment( -30, 0 ,0);
		c.increment( -30, 0 ,0);
		d.increment( -30, 0 ,0);
		dofaruknormal(a,b,c);  drawRectangle(a,b,c,d,0);
		}
		else{ 	glColor3f(1,1,1);
		//if(i==7 || i==9)  glColor3f(.3,.3,.3);
		

		a.increment( -30, 0 ,0);
		b.increment( -30, 0 ,0);
		c.increment( -30, 0 ,0);
		d.increment( -30, 0 ,0);
		dofaruknormal(a,b,c);  drawRectangle(a,b,c,d,1);
		}
	}
	
	glColor3f(1,1,1);
	a.increment( -30, 0 ,0);
	b.increment( -50, 0 ,0);
	c.increment( -50, 0 ,0);
	d.increment( -30, 0 ,0);
	dofaruknormal(a,b,c);  drawRectangle(a,b,c,d,1);

	//L er large er lej
	a.reInit(200, 700 ,340);
	b.reInit(200, 600 ,340);
	c.reInit(200, 600 ,380);
	d.reInit(200, 700 ,380);
	dofaruknormal(a,b,c);  drawRectangle(a,b,c,d,1);
	

}

void LSecondShade(){
	point a(750, 400 ,410);
	point b(750, 700 ,410);
	point c(800, 725 ,390);
	point d(800, 375 ,390);
	
	glColor3f(.5,.5,.5);
	dofaruknormal(a,b,c);  drawRectangle(a,b,c,d,0);

	
	 a.reInit(750, 700 ,410);
	 b.reInit(800, 725 ,390);
	 c.reInit(175, 725 ,390);
	 d.reInit(200, 700 ,410);
	
	glColor3f(.5,.5,.5);
	dofaruknormal(a,b,c);  drawRectangle(a,b,c,d,0);


	
	 a.reInit(175, 725 ,390);
	 b.reInit(200, 700 ,410);
	  c.reInit(200, 600 ,410);
	 d.reInit(175, 600 ,390);
	
	glColor3f(.5,.5,.5);
	dofaruknormal(a,b,c);  drawRectangle(a,b,c,d,0);



	//cylinder
	int i;
	glColor3d(CYL_COLOR);	
	GLUquadricObj *quadratic;
	quadratic=gluNewQuadric();
	for( i=0;i<23;i++){
		glPushMatrix();{
			glTranslatef(800 ,400+ i*15,400);
			glRotatef(-75,0,1,0);

			gluCylinder(quadratic,5,5,55,20,20);
		}glPopMatrix();
	}

		for( i=0;i<37;i++){
		glPushMatrix();{
			glTranslatef(200 + i*15,740,400);
			glRotatef(70,1,0,0);

			gluCylinder(quadratic,5,5,50,20,20);
		}glPopMatrix();
	}

		for( i=0;i<10;i++){
		glPushMatrix();{
			glTranslatef(170 ,600+ i*15,390);
			glRotatef(60,0,1,0);

			gluCylinder(quadratic,5,5,45,20,20);
		}glPopMatrix();
	}


}

void LSecondRoof(){
	point a(750, 400 ,410);
	point b(750, 700 ,410);
	point c(675, 700 ,460);
	point d(675, 400 ,460);
	
	glColor3f(.5,.5,.5);
	dofaruknormal(a,b,c);  drawRectangle(a,b,c,d,0);

	
	 a.reInit(600, 400 ,410);
	 b.reInit(600, 700 ,410);
	 c.reInit(675, 700 ,460);
	d.reInit(675, 400 ,460);
	
	glColor3f(.5,.5,.5);
	dofaruknormal(a,b,c);  drawRectangle(a,b,c,d,0);


	
	 a.reInit(675, 700 ,460);
	 b.reInit(600, 700 ,410);
	  c.reInit(750, 700 ,410);

	glColor3f(1,1,1);
	dofaruknormal(a,b,c);  drawTriangle(a,b,c);

	 a.reInit(675, 400 ,460);
	 b.reInit(600, 400 ,410);
	  c.reInit(750, 400 ,410);

	glColor3f(1,1,1);
	dofaruknormal(a,b,c);  drawTriangle(a,b,c);

	///L er large part

	 a.reInit(675, 500 ,410);
	 b.reInit(200, 500 ,410);
	 c.reInit(200, 600 ,460);
	d.reInit(675, 600 ,460);
	
	glColor3f(.5,.5,.5);
	dofaruknormal(a,b,c);  drawRectangle(a,b,c,d,0);

	 a.reInit(675, 700 ,410);
	 b.reInit(200, 700 ,410);
	 c.reInit(200, 600 ,460);
	d.reInit(675, 600 ,460);
	
	glColor3f(.5,.5,.5);
	dofaruknormal(a,b,c);  drawRectangle(a,b,c,d,0);
	
	a.reInit(200, 600 ,460);
	 b.reInit(200, 700 ,410);
	  c.reInit(200, 500 ,410);

	glColor3f(1,1,1);
	dofaruknormal(a,b,c);  drawTriangle(a,b,c);

	//cylinder
	int i;
	glColor3d(CYL_COLOR);	
	GLUquadricObj *quadratic;
	quadratic=gluNewQuadric();
	for( i=0;i<20;i++){
		glPushMatrix();{
			glTranslatef(750 ,400+ i*15,410);
			glRotatef(-55,0,1,0);

			gluCylinder(quadratic,5,5,90,20,20);
		}glPopMatrix();
	}
	for( i=0;i<20;i++){
		glPushMatrix();{
			glTranslatef(600 ,400+ i*15,410);
			glRotatef(55,0,1,0);

			gluCylinder(quadratic,5,5,90,20,20);
		}glPopMatrix();
	}

		for( i=0;i<37;i++){
		glPushMatrix();{
			glTranslatef(200 + i*15,705,410);
			glRotatef(60,1,0,0);

			gluCylinder(quadratic,5,5,112,20,20);
		}glPopMatrix();
	}

		for( i=0;i<37;i++){
		glPushMatrix();{
			glTranslatef(200 + i*15,505,410);
			glRotatef(-60,1,0,0);

			gluCylinder(quadratic,5,5,112,20,20);
		}glPopMatrix();
	}

		
	//horizontal pillar
	glPushMatrix();{	
		glColor3f(PILLAR_COLOR);
		glTranslatef(437,600,465);
	//	glRotatef(90,0,0,1);
		glScalef(47.5,1,2);
		
		glutSolidCube(10);
	}glPopMatrix();

	glPushMatrix();{	
		//glColor3d(1,1,1);
		glTranslatef(675,550,465);
		glRotatef(90,0,0,1);
		glScalef(30.5,1,2);
		
		glutSolidCube(10);
	}glPopMatrix();



}


void LBuilding(){
	baseofLBuilding();
	LFirstFloor();
	LFirstShade();
	LSecondFloor();
	LSecondShade();
	LSecondRoof();

}

/*** end of L building *****************/

/***start of Rest building ************/

void BaseofRest(){
	//1
	glEnable(GL_TEXTURE_2D);
	glBindTexture(GL_TEXTURE_2D,baseimg);
	//glColor3f(0.2, 1, 0.2);
	float pix = 1;
	point a(200, 620 ,0);
	point b(-20, 620 ,0);
	point c(0, 600 ,210);
	point d(200, 600 ,210);
	
	dofaruknormal(a,b,c);

	glBegin(GL_QUADS);{
		//glNormal3f(0,0,1);
		glColor3f(1,1,1);
		glTexCoord2f(0,0);
		glVertex3f(200, 620 ,0);
		glTexCoord2f(0,pix);
		glVertex3f(-20, 620 ,0);
		glTexCoord2f(pix,pix);
		glVertex3f(0, 600 ,210);
		glTexCoord2f(pix,0);
		glVertex3f(200, 600 ,210);
	}glEnd();

	
	
	glColor3f(BASE_COLOR);
	//drawRectangle(a,b,c,d);


	//2
	a.reInit(-20, 620 ,0);
	b.reInit(-20, 380 ,0);
	c.reInit(0, 400 ,210);
	d.reInit(0, 600 ,210);
	dofaruknormal(a,b,c);
	glBegin(GL_QUADS);{
		//glNormal3f(0,0,1);
		glColor3f(1,1,1);
		glTexCoord2f(0,0);
		glVertex3f(-20, 620 ,0);
		glTexCoord2f(0,pix);
		glVertex3f(-20, 380 ,0);
		glTexCoord2f(pix,pix);
		glVertex3f(0, 400 ,210);
		glTexCoord2f(pix,0);
		glVertex3f(0, 600 ,210);
	}glEnd();

	
	//drawRectangle(a,b,c,d);

	//3
	a.reInit(-20, 380 ,0);
	b.reInit(50, 380 ,0);
	c.reInit(50, 400 ,210);
	d.reInit(0, 400 ,210);
	dofaruknormal(a,b,c);
	glBegin(GL_QUADS);{
		//glNormal3f(0,0,1);
		glColor3f(1,1,1);
		glTexCoord2f(0,0);
		glVertex3f(-20, 380 ,0);
		glTexCoord2f(0,pix);
		glVertex3f(50, 380 ,0);
		glTexCoord2f(pix,pix);
		glVertex3f(50, 400 ,210);
		glTexCoord2f(pix,0);
		glVertex3f(0, 400 ,210);
	}glEnd();


	
	//drawRectangle(a,b,c,d);

	//4
	a.reInit(50, 380 ,0);
	b.reInit(50, 400 ,210);
	c.reInit(50, 100 ,210);
	d.reInit(30, 80 ,0);
	dofaruknormal(a,b,c);
	glBegin(GL_QUADS);{
		//glNormal3f(0,0,1);
		glColor3f(1,1,1);
		glTexCoord2f(0,0);
		glVertex3f(50, 380 ,0);
		glTexCoord2f(0,pix);
		glVertex3f(50, 400 ,210);
		glTexCoord2f(pix,pix);
		glVertex3f(50, 100 ,210);
		glTexCoord2f(pix,0);
		glVertex3f(30, 80 ,0);
	}glEnd();

	
	//drawRectangle(a,b,c,d);

	//5
	a.reInit(30, 80 ,0);
	b.reInit(50, 100 ,210);
	c.reInit(250, 100 ,210);
	d.reInit(270, 80 ,0);
	dofaruknormal(a,b,c);
	glBegin(GL_QUADS);{
		//glNormal3f(0,0,1);
		glColor3f(1,1,1);
		glTexCoord2f(0,0);
		glVertex3f(30, 80 ,0);
		glTexCoord2f(0,pix);
		glVertex3f(50, 100 ,210);
		glTexCoord2f(pix,pix);
		glVertex3f(250, 100 ,210);
		glTexCoord2f(pix,0);
		glVertex3f(270, 80 ,0);
	}glEnd();

	
	//drawRectangle(a,b,c,d);

	//6
	a.reInit(250, 100 ,210);
	b.reInit(270, 80 ,0);
	c.reInit(270, 400 ,0);
	d.reInit(250, 380 ,210);
	dofaruknormal(a,b,c);
	glBegin(GL_QUADS);{
		//glNormal3f(0,0,1);
		glColor3f(1,1,1);
		glTexCoord2f(0,0);
		glVertex3f(250, 100 ,210);
		glTexCoord2f(0,pix);
		glVertex3f(270, 80 ,0);
		glTexCoord2f(pix,pix);
		glVertex3f(270, 400 ,0);
		glTexCoord2f(pix,0);
		glVertex3f(250, 380 ,210);
	}glEnd();


	
	//drawRectangle(a,b,c,d);

		
	glDisable(GL_TEXTURE_2D);
	
}


void restFirstFloor(){
	//lower
	
	//1
	point a(200, 600 ,210);
	point b(0, 600 ,210);
	point c(0, 600 ,240);
	point d(200, 600 ,240);
	
	glColor3f(1,1,1);
	dofaruknormal(a,b,c);  drawRectangle(a,b,c,d,1);


	//2
	a.reInit(0, 600 ,210);
	b.reInit(0, 400 ,210);
	c.reInit(0, 400 ,240);
	d.reInit(0, 600 ,240);
	dofaruknormal(a,b,c);  drawRectangle(a,b,c,d,1);

	//3
	a.reInit(0, 400 ,210);
	b.reInit(50, 400 ,210);
	c.reInit(50, 400 ,240);
	d.reInit(0, 400 ,240);
	dofaruknormal(a,b,c);  drawRectangle(a,b,c,d,1);

	//4
	a.reInit(50, 400 ,210);
	b.reInit(50, 400 ,240);
	c.reInit(50, 100 ,240);
	d.reInit(50, 100 ,210);
	dofaruknormal(a,b,c);  drawRectangle(a,b,c,d,1);

	//5
	a.reInit(50, 100 ,210);
	b.reInit(50, 100 ,240);
	c.reInit(250, 100 ,240);
	d.reInit(250, 100 ,210);
	dofaruknormal(a,b,c);  drawRectangle(a,b,c,d,1);




	//upper
	
	//1
	 a.reInit(200, 600 ,280);
	 b.reInit(0, 600 ,280);
	 c.reInit(0, 600 ,310);
	 d.reInit(200, 600 ,310);
	
	glColor3f(1,1,1);
	dofaruknormal(a,b,c);  drawRectangle(a,b,c,d,1);


	//2
	a.reInit(0, 600 ,280);
	b.reInit(0, 400 ,280);
	c.reInit(0, 400 ,310);
	d.reInit(0, 600 ,310);
	dofaruknormal(a,b,c);  drawRectangle(a,b,c,d,1);

	//3
	a.reInit(0, 400 ,280);
	b.reInit(50, 400 ,280);
	c.reInit(50, 400 ,310);
	d.reInit(0, 400 ,310);
	dofaruknormal(a,b,c);  drawRectangle(a,b,c,d,1);

		a.reInit(0, 400 ,240);
	b.reInit(50, 400 ,240);
	c.reInit(50, 400 ,280);
	d.reInit(0, 400 ,280);
	dofaruknormal(a,b,c);  drawRectangle(a,b,c,d,1);

	//4
	a.reInit(50, 400 ,280);
	b.reInit(50, 400 ,310);
	c.reInit(50, 100 ,310);
	d.reInit(50, 100 ,280);
	dofaruknormal(a,b,c);  drawRectangle(a,b,c,d,1);

	//5
	a.reInit(50, 100 ,280);
	b.reInit(50, 100 ,310);
	c.reInit(250, 100 ,310);
	d.reInit(250, 100 ,280);
	dofaruknormal(a,b,c);  drawRectangle(a,b,c,d,1);


	//window

	//1

	a.reInit(200, 600 , 240);
	b.reInit(165, 600 ,240);;
	c.reInit(165, 600 ,280);
	d.reInit(200, 600 ,280);
	dofaruknormal(a,b,c);  drawRectangle(a,b,c,d,1);
	
	int i;
	a.increment(-10, 0 ,0);
	d.increment(-10, 0 ,0);
	for( i=0; i<5; i++){
		if(i%2 == 0){		glColor3f(.3,.3,.3);
		
			a.increment( -25, 0 ,0);
			b.increment(  -25,0 ,0);
			c.increment(  -25,0 ,0);
			d.increment(  -25,0 ,0);
			dofaruknormal(a,b,c);  drawRectangle(a,b,c,d,0);
		}
		else{ 	glColor3f(1,1,1);
		

		a.increment( -25, 0 ,0);
		b.increment(  -25,0 ,0);
		c.increment(  -25,0 ,0);
		d.increment(  -25,0 ,0);
		dofaruknormal(a,b,c);  drawRectangle(a,b,c,d,1);
		}

	}
	
	glColor3f(1,1,1);
	a.increment( -25, 0 ,0);
	b.increment( -40, 0 ,0);
	c.increment( -40, 0 ,0);
	d.increment( -25, 0 ,0);
	dofaruknormal(a,b,c);  drawRectangle(a,b,c,d,1);



	//2

	a.reInit(0, 600 , 240);
	b.reInit(0, 565 ,240);;
	c.reInit(0, 565 ,280);
	d.reInit(0, 600 ,280);
	dofaruknormal(a,b,c);  drawRectangle(a,b,c,d,1);
	
	a.increment(0, -10 ,0);
	d.increment(0, -10 ,0);
	for( i=0; i<5; i++){
		if(i%2 == 0){		glColor3f(.3,.3,.3);
		
		a.increment( 0,-25, 0 );
		b.increment(  0,-25,0 );
		c.increment( 0, -25,0 );
		d.increment( 0, -25,0 );
		dofaruknormal(a,b,c);  drawRectangle(a,b,c,d,0);
		
		}else{ 	glColor3f(1,1,1);
		

		a.increment( 0,-25, 0 );
		b.increment(  0,-25,0 );
		c.increment( 0, -25,0 );
		d.increment( 0, -25,0 );
		dofaruknormal(a,b,c);  drawRectangle(a,b,c,d,1);
		}
	}
	
	glColor3f(1,1,1);
	a.increment(0, -25, 0 );
	b.increment(0, -40, 0 );
	c.increment(0, -40, 0 );
	d.increment(0,-25, 0 );
	dofaruknormal(a,b,c);  drawRectangle(a,b,c,d,1);


		//4

	a.reInit(50, 400 , 240);
	b.reInit(50, 365 ,240);;
	c.reInit(50, 365 ,280);
	d.reInit(50, 400 ,280);
	dofaruknormal(a,b,c);  drawRectangle(a,b,c,d,1);
	
	a.increment(0, -10 ,0);
	d.increment(0, -10 ,0);
	for( i=0; i<9; i++){
		if(i%2 == 0){		glColor3f(.3,.3,.3);
			a.increment( 0,-25, 0 );
			b.increment(  0,-25,0 );
			c.increment( 0, -25,0 );
			d.increment( 0, -25,0 );
			dofaruknormal(a,b,c);  drawRectangle(a,b,c,d,0);

		}
		else{ 	glColor3f(1,1,1);
		

		a.increment( 0,-25, 0 );
		b.increment(  0,-25,0 );
		c.increment( 0, -25,0 );
		d.increment( 0, -25,0 );
		dofaruknormal(a,b,c);  drawRectangle(a,b,c,d,1);
		}
	}
	
	glColor3f(1,1,1);
	a.increment(0, -25, 0 );
	b.increment(0, -40, 0 );
	c.increment(0, -40, 0 );
	d.increment(0,-25, 0 );
	dofaruknormal(a,b,c);  drawRectangle(a,b,c,d,1);


		//5

	a.reInit(50, 100 , 240);
	b.reInit(85, 100 ,240);;
	c.reInit(85, 100 ,280);
	d.reInit(50, 100 ,280);
	dofaruknormal(a,b,c);  drawRectangle(a,b,c,d,1);
	
	a.increment(10, 0 ,0);
	d.increment(10, 0 ,0);
	for( i=0; i<5; i++){
		if(i%2 == 0){		glColor3f(.3,.3,.3);
		a.increment( 25, 0 ,0);
		b.increment(  25,0 ,0);
		c.increment(  25,0 ,0);
		d.increment(  25,0 ,0);
		dofaruknormal(a,b,c);  drawRectangle(a,b,c,d,0);
		}
		else{ 	glColor3f(1,1,1);
		

		a.increment( 25, 0 ,0);
		b.increment(  25,0 ,0);
		c.increment(  25,0 ,0);
		d.increment(  25,0 ,0);
		dofaruknormal(a,b,c);  drawRectangle(a,b,c,d,1);

		}
	}
	
	glColor3f(1,1,1);
	a.increment( 25, 0 ,0);
	b.increment( 40, 0 ,0);
	c.increment( 40, 0 ,0);
	d.increment( 25, 0 ,0);
	dofaruknormal(a,b,c);  drawRectangle(a,b,c,d,1);

}

void restFirstShade(){
		//1
	point a(200, 600 ,310);
	point b(200, 625 ,290);
	point c(-25, 625 ,290);
	point d(0, 600 ,310);
	
	glColor3f(.5,.5,.5);
	dofaruknormal(a,b,c);  drawRectangle(a,b,c,d,0);


	//2
	a.reInit(0, 600 ,310);
	b.reInit(-25, 625 ,290);
	c.reInit(-25, 375 ,290);
	d.reInit(0,400, 310);
	dofaruknormal(a,b,c);  drawRectangle(a,b,c,d,0);

	//3
	a.reInit(0,400, 310);
	b.reInit(-25, 375 ,290);
	c.reInit(50, 375 ,290);
	d.reInit(50, 400 ,310);
	dofaruknormal(a,b,c);  drawRectangle(a,b,c,d,0);

	//4
	a.reInit(50, 400 ,310);
	b.reInit(25, 375 ,290);
	c.reInit(25, 75 ,290);
	d.reInit(50, 100 ,310);
	dofaruknormal(a,b,c);  drawRectangle(a,b,c,d,0);

	//5
	a.reInit(50, 100 ,310);
	b.reInit(25, 75 ,290);
	c.reInit(275, 75 ,290);
	d.reInit(250,100 , 310);
	dofaruknormal(a,b,c);  drawRectangle(a,b,c,d,0);


	int i;
	//cylinder

	glColor3d(CYL_COLOR);	
	GLUquadricObj *quadratic;
	quadratic=gluNewQuadric();
	//1
	for(i=0;i<15;i++){
		glPushMatrix();{
			glTranslatef(-10+ i*15 ,625,300);
			glRotatef(70,1,0,0);

			gluCylinder(quadratic,5,5,35,20,20);
		}glPopMatrix();
	}

	//2
	for( i=0;i<15;i++){
		glPushMatrix();{
			glTranslatef(-30 ,400+ i*15,300);
			glRotatef(70,0,1,0);

			gluCylinder(quadratic,5,5,35,20,20);
		}glPopMatrix();
	}
	//3
	for( i=0;i<5;i++){
		glPushMatrix();{
			glTranslatef(-10+ i*15 ,370,300);
			glRotatef(-70,1,0,0);

			gluCylinder(quadratic,5,5,35,20,20);
		}glPopMatrix();
	}


	//4
	
	for( i=0;i<20;i++){
		glPushMatrix();{
			glTranslatef(15 ,100+ i*15,305);
			glRotatef(70,0,1,0);

			gluCylinder(quadratic,5,5,35,20,20);
		}glPopMatrix();
	}


	//5
	for( i=0;i<16;i++){
		glPushMatrix();{
			glTranslatef(30+ i*15 ,70,300);
			glRotatef(-70,1,0,0);

			gluCylinder(quadratic,5,5,35,20,20);
		}glPopMatrix();
	}


  }







void restSecondFloor(){
	//lower
	
	//1
	point a(200, 600 ,310);
	point b(0, 600 ,310);
	point c(0, 600 ,340);
	point d(200, 600 ,340);
	
	glColor3f(1,1,1);
	dofaruknormal(a,b,c);  drawRectangle(a,b,c,d,1);


	//2
	a.reInit(0, 600 ,310);
	b.reInit(0, 400 ,310);
	c.reInit(0, 400 ,340);
	d.reInit(0, 600 ,340);
	dofaruknormal(a,b,c);  drawRectangle(a,b,c,d,1);

	//3
	a.reInit(0, 400 ,310);
	b.reInit(50, 400 ,310);
	c.reInit(50, 400 ,340);
	d.reInit(0, 400 ,340);
	dofaruknormal(a,b,c);  drawRectangle(a,b,c,d,1);

	//4
	a.reInit(50, 400 ,310);
	b.reInit(50, 400 ,340);
	c.reInit(50, 100 ,340);
	d.reInit(50, 100 ,310);
	dofaruknormal(a,b,c);  drawRectangle(a,b,c,d,1);

	//5
	a.reInit(50, 100 ,310);
	b.reInit(50, 100 ,340);
	c.reInit(250, 100 ,340);
	d.reInit(250, 100 ,310);
	dofaruknormal(a,b,c);  drawRectangle(a,b,c,d,1);




	//upper
	
	//1
	 a.reInit(200, 600 ,380);
	 b.reInit(0, 600 ,380);
	 c.reInit(0, 600 ,410);
	 d.reInit(200, 600 ,410);
	
	glColor3f(1,1,1);
	dofaruknormal(a,b,c);  drawRectangle(a,b,c,d,1);


	//2
	a.reInit(0, 600 ,380);
	b.reInit(0, 400 ,380);
	c.reInit(0, 400 ,410);
	d.reInit(0, 600 ,410);
	dofaruknormal(a,b,c);  drawRectangle(a,b,c,d,1);

	//3
	a.reInit(0, 400 ,380);
	b.reInit(50, 400 ,380);
	c.reInit(50, 400 ,410);
	d.reInit(0, 400 ,410);
	dofaruknormal(a,b,c);  drawRectangle(a,b,c,d,1);

		a.reInit(0, 400 ,340);
	b.reInit(50, 400 ,340);
	c.reInit(50, 400 ,380);
	d.reInit(0, 400 ,380);
	dofaruknormal(a,b,c);  drawRectangle(a,b,c,d,1);

	//4
	a.reInit(50, 400 ,380);
	b.reInit(50, 400 ,410);
	c.reInit(50, 100 ,410);
	d.reInit(50, 100 ,380);
	dofaruknormal(a,b,c);  drawRectangle(a,b,c,d,1);

	//5
	a.reInit(50, 100 ,380);
	b.reInit(50, 100 ,410);
	c.reInit(250, 100 ,410);
	d.reInit(250, 100 ,380);
	dofaruknormal(a,b,c);  drawRectangle(a,b,c,d,1);

	int i;
	//window

	//1

	a.reInit(200, 600 , 340);
	b.reInit(165, 600 ,340);;
	c.reInit(165, 600 ,380);
	d.reInit(200, 600 ,380);
	dofaruknormal(a,b,c);  drawRectangle(a,b,c,d,1);
	
	a.increment(-10, 0 ,0);
	d.increment(-10, 0 ,0);
	for( i=0; i<5; i++){
		if(i%2 == 0){		glColor3f(.3,.3,.3);
		a.increment( -25, 0 ,0);
		b.increment(  -25,0 ,0);
		c.increment(  -25,0 ,0);
		d.increment(  -25,0 ,0);
		dofaruknormal(a,b,c);  drawRectangle(a,b,c,d,0);
		}
		else {	glColor3f(1,1,1);
		

		a.increment( -25, 0 ,0);
		b.increment(  -25,0 ,0);
		c.increment(  -25,0 ,0);
		d.increment(  -25,0 ,0);
		dofaruknormal(a,b,c);  drawRectangle(a,b,c,d,1);
		}
	}
	
	glColor3f(1,1,1);
	a.increment( -25, 0 ,0);
	b.increment( -40, 0 ,0);
	c.increment( -40, 0 ,0);
	d.increment( -25, 0 ,0);
	dofaruknormal(a,b,c);  drawRectangle(a,b,c,d,1);



	//2

	a.reInit(0, 600 , 340);
	b.reInit(0, 565 ,340);;
	c.reInit(0, 565 ,380);
	d.reInit(0, 600 ,380);
	dofaruknormal(a,b,c);  drawRectangle(a,b,c,d,1);
	
	a.increment(0, -10 ,0);
	d.increment(0, -10 ,0);
	for( i=0; i<5; i++){
		if(i%2 == 0){		glColor3f(.3,.3,.3);
			a.increment( 0,-25, 0 );
		b.increment(  0,-25,0 );
		c.increment( 0, -25,0 );
		d.increment( 0, -25,0 );
		dofaruknormal(a,b,c);  drawRectangle(a,b,c,d,0);
		}
		else{ 	glColor3f(1,1,1);
		

		a.increment( 0,-25, 0 );
		b.increment(  0,-25,0 );
		c.increment( 0, -25,0 );
		d.increment( 0, -25,0 );
		dofaruknormal(a,b,c);  drawRectangle(a,b,c,d,1);

		}
	}
	
	glColor3f(1,1,1);
	a.increment(0, -25, 0 );
	b.increment(0, -40, 0 );
	c.increment(0, -40, 0 );
	d.increment(0,-25, 0 );
	dofaruknormal(a,b,c);  drawRectangle(a,b,c,d,1);


		//4

	a.reInit(50, 400 , 340);
	b.reInit(50, 365 ,340);;
	c.reInit(50, 365 ,380);
	d.reInit(50, 400 ,380);
	dofaruknormal(a,b,c);  drawRectangle(a,b,c,d,1);
	
	a.increment(0, -10 ,0);
	d.increment(0, -10 ,0);
	for( i=0; i<9; i++){
		if(i%2 == 0){		glColor3f(.3,.3,.3);
		a.increment( 0,-25, 0 );
		b.increment(  0,-25,0 );
		c.increment( 0, -25,0 );
		d.increment( 0, -25,0 );
		dofaruknormal(a,b,c);  drawRectangle(a,b,c,d,0);

		}
		else {	glColor3f(1,1,1);
		

		a.increment( 0,-25, 0 );
		b.increment(  0,-25,0 );
		c.increment( 0, -25,0 );
		d.increment( 0, -25,0 );
		dofaruknormal(a,b,c);  drawRectangle(a,b,c,d,1);
		}
	}
	
	glColor3f(1,1,1);
	a.increment(0, -25, 0 );
	b.increment(0, -40, 0 );
	c.increment(0, -40, 0 );
	d.increment(0,-25, 0 );
	dofaruknormal(a,b,c);  drawRectangle(a,b,c,d,1);


		//5

	a.reInit(50, 100 , 340);
	b.reInit(85, 100 ,340);;
	c.reInit(85, 100 ,380);
	d.reInit(50, 100 ,380);
	dofaruknormal(a,b,c);  drawRectangle(a,b,c,d,1);
	
	a.increment(10, 0 ,0);
	d.increment(10, 0 ,0);
	for( i=0; i<5; i++){
		if(i%2 == 0){		glColor3f(.3,.3,.3);
		a.increment( 25, 0 ,0);
		b.increment(  25,0 ,0);
		c.increment(  25,0 ,0);
		d.increment(  25,0 ,0);
		dofaruknormal(a,b,c);  drawRectangle(a,b,c,d,0);
		}
		else{ 	glColor3f(1,1,1);
		

		a.increment( 25, 0 ,0);
		b.increment(  25,0 ,0);
		c.increment(  25,0 ,0);
		d.increment(  25,0 ,0);
		dofaruknormal(a,b,c);  drawRectangle(a,b,c,d,1);

		}
	}
	
	glColor3f(1,1,1);
	a.increment( 25, 0 ,0);
	b.increment( 40, 0 ,0);
	c.increment( 40, 0 ,0);
	d.increment( 25, 0 ,0);
	dofaruknormal(a,b,c);  drawRectangle(a,b,c,d,1);

}

void restSecondShade(){
		//1
	point a(200, 600 ,410);
	point b(200, 625 ,390);
	point c(-25, 625 ,390);
	point d(0, 600 ,410);
	
	glColor3f(.5,.5,.5);
	dofaruknormal(a,b,c);  drawRectangle(a,b,c,d,0);


	//2
	a.reInit(0, 600 ,410);
	b.reInit(-25, 625 ,390);
	c.reInit(-25, 375 ,390);
	d.reInit(0,400, 410);
	dofaruknormal(a,b,c);  drawRectangle(a,b,c,d,0);

	//3
	a.reInit(0,400, 410);
	b.reInit(-25, 375 ,390);
	c.reInit(50, 375 ,390);
	d.reInit(50, 400 ,410);
	dofaruknormal(a,b,c);  drawRectangle(a,b,c,d,0);

	//4
	a.reInit(50, 400 ,410);
	b.reInit(25, 375 ,390);
	c.reInit(25, 75 ,390);
	d.reInit(50, 100 ,410);
	dofaruknormal(a,b,c);  drawRectangle(a,b,c,d,0);

	//5
	a.reInit(50, 100 ,410);
	b.reInit(25, 75 ,390);
	c.reInit(275, 75 ,390);
	d.reInit(250,100 , 410);
	dofaruknormal(a,b,c);  drawRectangle(a,b,c,d,0);



	//cylinder
	int i;
	glColor3d(CYL_COLOR);	
	GLUquadricObj *quadratic;
	quadratic=gluNewQuadric();
	//1
	for(i=0;i<15;i++){
		glPushMatrix();{
			glTranslatef(-10+ i*15 ,625,400);
			glRotatef(70,1,0,0);

			gluCylinder(quadratic,5,5,35,20,20);
		}glPopMatrix();
	}

	//2
	for( i=0;i<15;i++){
		glPushMatrix();{
			glTranslatef(-30 ,400+ i*15,400);
			glRotatef(70,0,1,0);

			gluCylinder(quadratic,5,5,35,20,20);
		}glPopMatrix();
	}
	//3
	for( i=0;i<5;i++){
		glPushMatrix();{
			glTranslatef(-10+ i*15 ,370,400);
			glRotatef(-70,1,0,0);

			gluCylinder(quadratic,5,5,35,20,20);
		}glPopMatrix();
	}


	//4
	
	for( i=0;i<20;i++){
		glPushMatrix();{
			glTranslatef(15 ,100+ i*15,405);
			glRotatef(70,0,1,0);

			gluCylinder(quadratic,5,5,35,20,20);
		}glPopMatrix();
	}


	//5
	for( i=0;i<16;i++){
		glPushMatrix();{
			glTranslatef(30+ i*15 ,70,400);
			glRotatef(-70,1,0,0);

			gluCylinder(quadratic,5,5,35,20,20);
		}glPopMatrix();
	}


  }


 void restSecondRoof(){
 
	 //1
	 point a(200, 600 ,410);
	point b(0, 600 ,410);
	point c(0, 500 ,460);
	point d(200, 500 ,460);
	
	glColor3f(.5,.5,.5);
	dofaruknormal(a,b,c);  drawRectangle(a,b,c,d,0);

	
	 a.reInit(0, 500 ,460);
	 b.reInit(200, 500 ,460);
	 c.reInit(200, 400 ,410);
	d.reInit(0, 400 ,410);
	
	glColor3f(.5,.5,.5);
	dofaruknormal(a,b,c);  drawRectangle(a,b,c,d,0);


	
	 a.reInit(0, 600 ,410);
	 b.reInit(0, 400 ,410);
	 c.reInit(0, 500 ,460);

	glColor3f(1,1,1);
	dofaruknormal(a,b,c);  drawTriangle(a,b,c);

	  a.reInit(200, 600 ,410);
	 b.reInit(200, 400 ,410);
	 c.reInit(200, 500 ,460);

	glColor3f(1,1,1);
	dofaruknormal(a,b,c);  drawTriangle(a,b,c);

	///2

	 a.reInit(50, 400 ,410);
	 b.reInit(50, 100 ,410);
	 c.reInit(150, 100 ,460);
	d.reInit(150, 400 ,460);
	
	glColor3f(.5,.5,.5);
	dofaruknormal(a,b,c);  drawRectangle(a,b,c,d,0);

	 a.reInit(250, 100 ,410);
	 b.reInit(250, 400 ,410);
	 c.reInit(150, 400 ,460);
	d.reInit(150, 100 ,460);
	
	glColor3f(.5,.5,.5);
	dofaruknormal(a,b,c);  drawRectangle(a,b,c,d,0);
	
	a.reInit(150, 100 ,460);
	 b.reInit(250, 100 ,410);
	  c.reInit(50, 100 ,410);

	glColor3f(1,1,1);
	dofaruknormal(a,b,c);  drawTriangle(a,b,c);

		a.reInit(150, 400 ,460);
	 b.reInit(250, 400 ,410);
	  c.reInit(50, 400 ,410);

	glColor3f(1,1,1);
	dofaruknormal(a,b,c);  drawTriangle(a,b,c);

	//cylinder
	int i;
	glColor3d(CYL_COLOR);	
	GLUquadricObj *quadratic;
	quadratic=gluNewQuadric();
	for( i=0;i<20;i++){
		glPushMatrix();{
			glTranslatef(250 ,100+ i*15,410);
			glRotatef(-65,0,1,0);

			gluCylinder(quadratic,5,5,110,20,20);
		}glPopMatrix();
	}
	for( i=0;i<20;i++){
		glPushMatrix();{
			glTranslatef(50 ,100+ i*15,410);
			glRotatef(65,0,1,0);

			gluCylinder(quadratic,5,5,110,20,20);
		}glPopMatrix();
	}

		for( i=0;i<14;i++){
		glPushMatrix();{
			glTranslatef(0 + i*15,600,410);
			glRotatef(60,1,0,0);

			gluCylinder(quadratic,5,5,105,20,20);
		}glPopMatrix();
	}

		for( i=0;i<14;i++){
		glPushMatrix();{
			glTranslatef(0 + i*15,400,410);
			glRotatef(-60,1,0,0);

			gluCylinder(quadratic,5,5,105,20,20);
		}glPopMatrix();
	}

		
	//horizontal pillar
	glPushMatrix();{	
		glColor3f(PILLAR_COLOR);
		glTranslatef(100,500,465);
	//	glRotatef(90,0,0,1);
		glScalef(20.5,1,2);
		
		glutSolidCube(10);
	}glPopMatrix();

	glPushMatrix();{	
		//glColor3d(1,1,1);
		glTranslatef(150,250,465);
		glRotatef(90,0,0,1);
		glScalef(30.5,1,2);
		
		glutSolidCube(10);
	}glPopMatrix();

  
 }


void restBuilding(){
	BaseofRest();
	restFirstFloor();
	restFirstShade();
	restSecondFloor();
	restSecondShade();
	restSecondRoof();


}

void baseofBaranda(){

	glEnable(GL_TEXTURE_2D);
	glBindTexture(GL_TEXTURE_2D,baseimg);
	//glColor3f(0.2, 1, 0.2);
	float pix = 1;
	
	point a(225, 100 ,0);
	point b(225, 50 ,0);
	point c(225, 50 ,150);
	point d(225, 100 ,150);
	
	dofaruknormal(a,b,c);
	
	glBegin(GL_QUADS);{
		//glNormal3f(0,0,1);
		glColor3f(1,1,1);
		glTexCoord2f(0,0);
		glVertex3f(225, 100 ,0);
		glTexCoord2f(0,pix);
		glVertex3f(225, 50 ,0);
		glTexCoord2f(pix,pix);
		glVertex3f(225, 50 ,150);
		glTexCoord2f(pix,0);
		glVertex3f(225, 100 ,150);
	}glEnd();

	glColor3f(1,1,1);
	
	
	
	//drawRectangle(a,b,c,d);
	a.reInit(225, 50 ,0);
	 b.reInit(225, 50 ,150);
	 c.reInit(130, 50 ,150);
	d.reInit(130, 50 ,0);
	dofaruknormal(a,b,c);
	glBegin(GL_QUADS);{
		//glNormal3f(0,0,1);
		glColor3f(1,1,1);
		glTexCoord2f(0,0);
		glVertex3f(225, 50 ,0);
		glTexCoord2f(0,pix);
		glVertex3f(225, 50 ,150);
		glTexCoord2f(pix,pix);
		glVertex3f(130, 50 ,150);
		glTexCoord2f(pix,0);
		glVertex3f(130, 50 ,0);
	}glEnd();

	
	
	//glColor3f(.5,.5,.5);
	//drawRectangle(a,b,c,d);
	a.reInit(120, 50 ,0);
	 b.reInit(130, 50 ,125);
	 c.reInit(130, -60 ,125);
	d.reInit(120, -60 ,0);
	dofaruknormal(a,b,c);
	glBegin(GL_QUADS);{
		//glNormal3f(0,0,1);
		glColor3f(1,1,1);
		glTexCoord2f(0,0);
		glVertex3f(120, 50 ,0);
		glTexCoord2f(0,pix);
		glVertex3f(130, 50 ,125);
		glTexCoord2f(pix,pix);
		glVertex3f(130, -60 ,125);
		glTexCoord2f(pix,0);
		glVertex3f(120, -60 ,0);
	}glEnd();

	
	
	glColor3f(BASE_COLOR);
	//drawRectangle(a,b,c,d);
	
	a.reInit(120, -60 ,0);
	 b.reInit(130, -60 ,125);
	 c.reInit(310, -60 ,125);
	d.reInit(330, -60 ,0);
	dofaruknormal(a,b,c);
	glBegin(GL_QUADS);{
		//glNormal3f(0,0,1);
		glColor3f(1,1,1);
		glTexCoord2f(0,0);
		glVertex3f(120, -60 ,0);
		glTexCoord2f(0,pix);
		glVertex3f(130, -60 ,125);
		glTexCoord2f(pix,pix);
		glVertex3f(310, -60 ,125);
		glTexCoord2f(pix,0);
		glVertex3f(330, -60 ,0);
	}glEnd();


	
	
	glColor3f(BASE_COLOR);
	//drawRectangle(a,b,c,d);
	
	a.reInit(330, -60 ,0);
	 b.reInit(310, -60 ,125);
	 c.reInit(310, 20 ,125);
	d.reInit(330, 00 ,0);
	dofaruknormal(a,b,c);
	glBegin(GL_QUADS);{
		//glNormal3f(0,0,1);
		glColor3f(1,1,1);
		glTexCoord2f(0,0);
		glVertex3f(330, -60 ,0);
		glTexCoord2f(0,pix);
		glVertex3f(310, -60 ,125);
		glTexCoord2f(pix,pix);
		glVertex3f(310, 20 ,125);
		glTexCoord2f(pix,0);
		glVertex3f(330, 0 ,0);
	}glEnd();

	
	
	glColor3f(BASE_COLOR);
	//drawRectangle(a,b,c,d);

	
	glDisable(GL_TEXTURE_2D);

	/***end of base ****/
	
	//WHITE TOP

	a.reInit(130, 50 ,125);
	 b.reInit(130, 50 ,150);
	 c.reInit(130, -60 ,150);
	d.reInit(130, -60 ,125);
	
	glColor3f(1,1,1);
	dofaruknormal(a,b,c);  drawRectangle(a,b,c,d,1);

	a.reInit(130, -60 ,125);
	 b.reInit(130, -60 ,150);
	 c.reInit(310, -60 ,150);
	d.reInit(310, -60 ,125);
	
//	glColor3f(BASE_COLOR);
	dofaruknormal(a,b,c);  drawRectangle(a,b,c,d,1);

	a.reInit(310, -60 ,125);
	 b.reInit(310, -60 ,150);
	 c.reInit(310, 00 ,150);
	d.reInit(310, 00 ,125);
	
	//glColor3f(BASE_COLOR);
	dofaruknormal(a,b,c);  drawRectangle(a,b,c,d,1);

}


void cubeofBaranda(){
	glPushMatrix();{	
		glColor3f(0,0,0);
		glTranslatef(215,-60,150);
	//	glRotatef(20,1,0,0);
		glScalef(19,2,1);
		
		glutSolidCube(10);
	}glPopMatrix();

	glPushMatrix();{	
		glColor3f(0,0,0);
		glTranslatef(177,50,150);
	//	glRotatef(20,1,0,0);
		glScalef(10.5,2,1);
		
		glutSolidCube(10);
	}glPopMatrix();



	glPushMatrix();{	
		glColor3f(0,0,0);
		glTranslatef(130,0,150);
		glRotatef(90,0,0,1);
		glScalef(11,2,1);
		
		glutSolidCube(10);
	}glPopMatrix();

	glPushMatrix();{	
		glColor3f(0,0,0);
		glTranslatef(320,-30,150);
		glRotatef(90,0,0,1);
		glScalef(7.5,2,1);
		
		glutSolidCube(10);
	}glPopMatrix();

	glPushMatrix();{	
		glColor3f(0,0,0);
		glTranslatef(225,75,150);
		glRotatef(90,0,0,1);
		glScalef(6,2,1);
		
		glutSolidCube(10);
	}glPopMatrix();


}


void baranda(){
	baseofBaranda();
	cubeofBaranda();

}



/****************************START OF SASHI *********************************************/



void sashiLeft(){
		glBegin(GL_POLYGON);
				glVertex3f(.61,-.36,0);
				glVertex3f(1.8,-.36,0);
				glVertex3f(2.02,-.26,.39);
				glVertex3f(2.02, -.26,1.42);


				glVertex3f(1.35, -.26, 1.1);
				glVertex3f(.91, -.26, .42);
				glVertex3f(.52, -.21, .36);
				//glVertex3f(.45,-.36,.11);
				glVertex3f(.61,-.36,0);
			glEnd();


			glBegin(GL_POLYGON);
				glVertex3f(2.02, -.26,1.42);

				glVertex3f(1.66, -.26,2);
				glVertex3f(1.24, -.26,1.78);
				glVertex3f(1.35, -.26, 1.1);
				glVertex3f(2.02, -.26,1.42);

			glEnd();

			glBegin(GL_POLYGON);
				
				glVertex3f(1.66, -.26,2);

				glVertex3f(1.59,-.24,2.17);

				glVertex3f(1.59,-.16,2.63);
				glVertex3f(1.7,-.16,2.79);
				glVertex3f(1.43,-.16,2.79);

				glVertex3f(1.21,-.16,2.65);
				glVertex3f(1.09, -.16,2.65);


				glVertex3f(1.09, -.16,2.34);
				glVertex3f(.72, -.16,2.34);

				glVertex3f(.44,-.16,2);
				glVertex3f(.44,-.16,1.9);
				glVertex3f(.66, -.21, 2.03);

				glVertex3f(.90,-.21,2.03);	

				glVertex3f(1.24, -.26,1.78);
				glVertex3f(1.66, -.26,2);
				

			glEnd();

}


void sashiRight(){
	glBegin(GL_POLYGON);
				glVertex3f(1.8, .13, 0);
				glVertex3f(.61,.13,0);
				glVertex3f(.45,-.02,.22);
				glVertex3f(.52, -.02,.36);
				glVertex3f(.91,.03,.42);
				glVertex3f(1.35,.03,1.1);
				glVertex3f(2.02, .03, 1.42);
				glVertex3f(2.02, .03, .39);
				glVertex3f(1.8, .13, 0);
	glEnd();

	glBegin(GL_POLYGON);
				
				glVertex3f(1.35,.03,1.1);
				
				glVertex3f(1.24,-.26,1.78);
				glVertex3f(1.66,.03,2);
				glVertex3f(2.02, .03, 1.42);
				glVertex3f(1.35,.03,1.1);
	glEnd();


	glBegin(GL_POLYGON);
				
				
				
				glVertex3f(1.24,-.26,1.78);
				glVertex3f(.9,-.07,2.03);
				glVertex3f(.66,-.07,2.03);
				//glVertex3f(.66, -.02,2.1);
				glVertex3f(.44, -.07, 1.9);
				glVertex3f(.44,-.07,2);
				glVertex3f(.72, -.07,2.34);
				glVertex3f(1.09,-.07,2.34);
				glVertex3f(1.09, -.07, 2.65);
				glVertex3f(1.21, -.07,2.65);
				glVertex3f(1.43, -.07, 2.79);
				glVertex3f(1.5, -.07, 2.79);
				glVertex3f(1.59,-.07,2.63);
				glVertex3f(1.59, .01, 2.17);
				glVertex3f(1.66,.03,2);
				glVertex3f(1.24,-.26,1.78);
				
	glEnd();

}


void innerBack(){
	glBegin(GL_POLYGON);
				/*glVertex3f(1.8, .13,0);
				glVertex3f(2.02, .03,.39);
				glVertex3f(2.02,.03,1.42);
				glVertex3f(1.66,.03,2);
				glVertex3f(1.59,-.07,2.63);
				glVertex3f(1.5, -.07, 2.79);

				glVertex3f(1.5, -.16, 2.79);
				glVertex3f(1.59,-.16,2.63);
				glVertex3f(1.66,-.26,2);
				glVertex3f(2.02,-.26,1.42);
				//glVertex3f(2.02,.26,.39);
				glVertex3f(1.8,-.36,.39);
				//glVertex3f(2.02,-.26,1.42);
				glVertex3f(1.8, .13,0);*/

				glVertex3f(1.8, .13,0);
				glVertex3f(2.02, .03,.39);
				glVertex3f(2.02,.03,1.42);
				glVertex3f(1.66,.03,2);
				glVertex3f(1.59,-.07,2.63);
				glVertex3f(1.5, -.07, 2.79);

				glVertex3f(1.5, -.16, 2.79);
				glVertex3f(1.59,-.16,2.63);
				glVertex3f(1.66,-.26,2);
				glVertex3f(2.02,-.26,1.42);
				glVertex3f(2.02,-.26,.39);
				glVertex3f(1.8,-.36,0);
				//glVertex3f(2.02,-.26,.39);
				glVertex3f(1.8, .13,0);
	glEnd();
}

void innerFront(){
	glBegin(GL_POLYGON);
				/*glVertex3f(1.5,-.07,2.79);
				glVertex3f(1.43,-.07,2.79);
				glVertex3f(1.09,-.07,2.65);
				glVertex3f(1.09, -.07,2.34);
				glVertex3f(.72,-.07,2.34);
				glVertex3f(.44,-.07,2);
				glVertex3f(.44,-.07,1.9);
				glVertex3f(.9,-.02,2.03);
				glVertex3f(1.24, .03,1.78);
				glVertex3f(1.35, .03,1.1);*/
				
				glVertex3f(.91,.03,.42);
				glVertex3f(.52, -.02,.36);
				glVertex3f(.45,.13,.11);
				glVertex3f(.61,.13,0);
				glVertex3f(.61,-.36,0);
				glVertex3f(.45,-.36,.11);
				glVertex3f(.52,-.21,.36);
				glVertex3f(.91,-.26,.42);
				glVertex3f(.91,.03,.42);

				/*glVertex3f(1.35,-.26,1.1);
				glVertex3f(1.24,-.26,1.78);
				glVertex3f(.9,-.21,2.03);
				glVertex3f(.44,-.16,1.9);
				glVertex3f(.44,-.16,2);
				glVertex3f(.72,-.16,2.34);
				glVertex3f(1.09,-.16,2.34);
				glVertex3f(1.09,-.16,2.65);
				glVertex3f(1.43,-.16,2.79);
				glVertex3f(1.5,-.16,2.79);
				glVertex3f(1.5,-.07,2.79);*/
	glEnd();


	glBegin(GL_POLYGON);
				glVertex3f(1.5,-.07,2.79);
				glVertex3f(1.43,-.07,2.79);
				glVertex3f(1.09,-.07,2.65);
				glVertex3f(1.09, -.07,2.34);
				/*glVertex3f(.72,-.07,2.34);
				glVertex3f(.44,-.07,2);
				glVertex3f(.44,-.07,1.9);
				glVertex3f(.9,-.02,2.03);
				glVertex3f(1.24, .03,1.78);
				glVertex3f(1.35, .03,1.1);*/
				
				

				/*glVertex3f(1.35,-.26,1.1);
				glVertex3f(1.24,-.26,1.78);
				glVertex3f(.9,-.21,2.03);
				glVertex3f(.44,-.16,1.9);
				glVertex3f(.44,-.16,2);
				glVertex3f(.72,-.16,2.34);*/
				glVertex3f(1.09,-.16,2.34);
				glVertex3f(1.09,-.16,2.65);
				glVertex3f(1.43,-.16,2.79);
				glVertex3f(1.5,-.16,2.79);
				glVertex3f(1.5,-.07,2.79);
	glEnd();
}

void upperWing(float y){
	//float y = .21;
	glBegin(GL_POLYGON);
				//glColor3f(.7,.7,.7);
				glVertex3f(1.8,y,1.4);
				glVertex3f(1.55,y,1.4);
				glVertex3f(1.55,y,1.65);
				glVertex3f(1.3,y,2);
				glVertex3f(1.45,y,2);
				glVertex3f(1.65,y,1.85);
				glVertex3f(1.75,y,2);
				glVertex3f(2,y,2);
				glVertex3f(1.8,y,1.4);
	glEnd();
}

void lowerWing(float y){
	//float y =.21;
	glBegin(GL_POLYGON);
				//glColor3f(1,1,1);

				glVertex3f(2.05,y,1);
				glVertex3f(1.9,y,1.15);
				glVertex3f(1.6,y,1.1);
				glVertex3f(1.6,y,1.2);
				glVertex3f(1.9,y,1.4);
				glVertex3f(1.9,y,1.6);
				glVertex3f(2,y,1.6);
				glVertex3f(2.05,y,1.4);
				glVertex3f(1.8,y,1.3);
				glVertex3f(2.05,y,1.2);
				glVertex3f(2.2,y,1.15);
				glVertex3f(2.05,y,1);
	glEnd();
}

void sashiLeftWings(){
	upperWing(.21);
	lowerWing(.21);
}

void sashiRightWings(){
	upperWing(-.4);
	lowerWing(-.4);
}

void sashiWings(){
	sashiLeftWings();
	sashiRightWings();
}

void drawSashi(){
	
	glPushMatrix();{
		glColor3f(SASHICOLOR);
		sashiLeft();	
		sashiRight();
		//glColor3f(.3,	.3,	.3);
		glPushMatrix();{
			//glScalef(1,.8,1);
			glTranslatef(0,.02,0);
			innerBack();	
			innerFront();
		}glPopMatrix();
		
		glColor3f(SASHIWINGSCOLOR);
		sashiWings();

		}glPopMatrix();

}



void mySashi(){
	//1
		glPushMatrix();{
			glTranslatef(630, 200,885);
			//glRotatef(-70,1,0,0);

			glScalef(20,20,20);
			drawSashi();
		}glPopMatrix();

	//2	
		glPushMatrix();{
			glTranslatef(470, 200,885);
			glRotatef(180,0,0,1);

			glScalef(20,20,20);
			drawSashi();
		}glPopMatrix();

	//3 Pair
		glPushMatrix();{
			glTranslatef(550, 115,730);
			glRotatef(270,0,0,1);

			glScalef(15,15,15);
			drawSashi();
		}glPopMatrix();

		//3
		glPushMatrix();{
			glTranslatef(550, 275,730);
			glRotatef(90,0,0,1);

			glScalef(15,15,15);
			drawSashi();
		}glPopMatrix();




	// Third floor er dual shasi
		glPushMatrix();{
			glTranslatef(450, 80,620);
			glRotatef(270,0,0,1);

			glScalef(12,12,10);
			drawSashi();
		}glPopMatrix();

			glPushMatrix();{
			glTranslatef(650, 80,620);
			glRotatef(270,0,0,1);

			glScalef(12,12,10);
			drawSashi();
		}glPopMatrix();

	// Third floor er dual shasi  Back
		glPushMatrix();{
			glTranslatef(450, 320,620);
			glRotatef(90,0,0,1);

			glScalef(12,12,10);
			drawSashi();
		}glPopMatrix();

			glPushMatrix();{
			glTranslatef(650, 320,620);
			glRotatef(90,0,0,1);

			glScalef(12,12,10);
			drawSashi();
		}glPopMatrix();


			//6

			glPushMatrix();{
			glTranslatef(365, 195,610);
			glRotatef(180,0,0,1);

			glScalef(15,15,15);
			drawSashi();
		}glPopMatrix();

		//7
			glPushMatrix();{
			glTranslatef(735, 195,610);
			//glRotatef(180,0,0,1);

			glScalef(15,15,15);
			drawSashi();
		}glPopMatrix();


			//side bulilding sushi

			glPushMatrix();{
		    glTranslatef(150, 120,475);
			glRotatef(270,0,0,1);

			glScalef(15,15,15);
			drawSashi();
		}glPopMatrix();

			glPushMatrix();{
		    glTranslatef(150, 380,475);
			glRotatef(90,0,0,1);

			glScalef(15,15,15);
			drawSashi();
		}glPopMatrix();

			//side bulilding sushi another pair

			glPushMatrix();{
		    glTranslatef(15, 500,470);
		    glRotatef(180,0,0,1);

			glScalef(15,15,15);
			drawSashi();
		}glPopMatrix();

			glPushMatrix();{
		    glTranslatef(185, 500,470);
			//glRotatef(180,0,0,1);

			glScalef(15,15,15);
			drawSashi();
		}glPopMatrix();
	
}

void ground(){

	glColor3f(0.3,0.3,0.3);

	glEnable(GL_TEXTURE_2D);
	glBindTexture(GL_TEXTURE_2D,grassimg);

	//front
	float pix = 1;
	int sz = 3000;
	glBegin(GL_QUADS);{
		glNormal3f(0,0,1);
		glColor3f(1.0f,1.0f,1.0f); 
		glTexCoord2f(0,0);
		glVertex3f(-sz,sz,0);
		glTexCoord2f(0,pix);
		glVertex3f(-sz,-sz,0);
		glTexCoord2f(pix,pix);
		glVertex3f(sz,-sz,0);
		glTexCoord2f(pix,0);
		glVertex3f(sz,sz,0);
	}glEnd();
	
	glDisable(GL_TEXTURE_2D);

	glColor3f(0.3,0.3,0.3);


	glBegin(GL_POLYGON);
				//glVertex3f(60,100,10);
				glVertex3f(250,100,10);
				glVertex3f(250,50,10);
				glVertex3f(130,50,10);
				glVertex3f(130,-60,10);
				glVertex3f(330,-60,10);
				glVertex3f(330,0,10);
				glVertex3f(800,00,10);
				glVertex3f(800,400,10);
				glVertex3f(750,400,10);
				glVertex3f(750,700,10);
				glVertex3f(200,700,10);
				glVertex3f(200,600,10);
				/*glVertex3f(0,600,10);
				glVertex3f(0,400,10);
				glVertex3f(60,400,10);
				glVertex3f(60,100,10);*/

				glVertex3f(250,100,10);
				
	glEnd();


	/*glBegin(GL_QUADS);{
		glVertex3f(210,680,10);	// intentionally extended to -150 to 150, no big dealnnnnnnnn
		glVertex3f(600,680,10);
		glVertex3f(600,200,10);
		glVertex3f(210,200,10);
	}glEnd();*/


}



/************************END OF SASHI ******************************************************/

void display(){

	//clear the display
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
	glClearColor(BLACK, 0);	//color black
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

	

	/********************
	/ set-up camera here
	********************/
	/********************
	/ set-up camera here
	********************/
	//load the correct matrix -- MODEL-VIEW matrix
	glMatrixMode(GL_MODELVIEW);

	//initialize the matrix
	glLoadIdentity();

	//now give three info
	//1. where is the camera (viewer)?
	//2. where is the camera is looking?
	//3. Which direction is the camera's UP direction?

	//instead of CONSTANT information, we will define a circular path.
	//	gluLookAt(-30,-30,50,	0,0,0,	0,0,1);

	if(!bird){
		V to=loc+dir;
		gluLookAt(loc.x, loc.y, loc.z,		to.x,to.y,to.z,		perp.x,perp.y,perp.z);
		//printf("%lf  %lf  %lf", loc.x, loc.y, loc.z  );
	}else gluLookAt(cameraRadius*cos(cameraAngle), cameraRadius*sin(cameraAngle), cameraHeight,   0,0,0,   0,0,1);
	//NOTE: the camera still CONSTANTLY looks at the center
	// cameraAngle is in RADIAN, since you are using inside COS and SIN

	//again select MODEL-VIEW
	glMatrixMode(GL_MODELVIEW);


	/****************************
	/ Add your objects from here
	****************************/
	//add objects
	//rotate this rectangle around the Z axis

			//the effect of rotation is not there now.



	// draw the two AXES
	glColor3f(1, 1, 1);	//100% white
	/*glBegin(GL_LINES);{
		//Y axis
		glColor3f(0, 1, 0);
		glVertex3f(0, 0, 0);	// intentionally extended to -150 to 150, no big deal
		glVertex3f(0,  700, 0);

		//X axis
		glColor3f(1, 0, 0);
		glVertex3f(0, 0, 0);
		glVertex3f( 700, 0, 0);

			glColor3f(0, 0, 1);
		glVertex3f(0, 0, 0);
		glVertex3f(0, 0, 700);
	}glEnd();*/

	ground();

	
	mainBuilding();

	LBuilding();

	restBuilding();

	baranda();

	mySashi();

	glFlush();


	//ADD this line in the end --- if you use double buffer (i.e. GL_DOUBLE)
	glutSwapBuffers();
}


void animate(){
	//codes for any changes in Camera

	if(bird) cameraAngle += cameraAngleDelta;
	
	//codes for any changes in Models

	//MISSING SOMETHING? -- YES: add the following
	glutPostRedisplay();	//this will call the display AGAIN
}

void keyboardListener(unsigned char key, int x,int y){
	switch(key){
	case '0':	//reverse the rotation of camera
		cameraAngleDelta = -cameraAngleDelta;
		break;

	case '+':case '=':	//increase rotation of camera by 10%
		cameraAngleDelta *= 1.1;
		break;

	case '-':case '_':	//decrease rotation
		cameraAngleDelta /= 1.1;
		break;

	case 'k':case 'K':		//down arrow key
		cameraRadius += 50;
		break;
	case 'i':case 'I':		// up arrow key
		if(cameraRadius > 10)
			cameraRadius -= 50;
		break;

	case 'l':case 'L':
		cameraHeight += 50;
		break;
	case 'j':case 'J':
		cameraHeight -= 50;
		break;

	case '.':case '>':
		ang_speed+=.1;
		break;

	case ',':case '<':
		ang_speed-=.1;
		break;

	case ']':case '}':
		speed+=5;
		break;

	case '[':case '{':
		speed-=5;
		break;

	case 'w':case 'W':
		loc=loc+speed*dir;
		break;

	case 'a':case 'A':
		loc=loc+speed*((perp*dir).unit());
		break;

	case 's':case 'S':
		loc=loc-speed*dir;
		break;

	case 'd':case 'D':
		loc=loc+speed*((dir*perp).unit());
		break;

	case 'q':case 'Q':
		perp=perp.rot(dir,ang_speed);
		break;

	case 'e':case 'E':
		perp=perp.rot(-dir,ang_speed);
		break;

	case 'r':case 'R':
		dir=_D,perp=_P;
		break;

	case 'b':case 'B':
		bird^=1;
		break;

	case 'n':{
			GLfloat lightColorN2[] = {223/700.0,150/700.0,56/700.0,1.0}; //Color (0.5, 0.2, 0.2)
			GLfloat lightColorN1[] = {150/512.0,223/512.0,56/512.0,1.0}; //Color (0.5, 0.2, 0.2)
			GLfloat lightColorN3[] = {56/512.0,150/512.0,223/512.0,1.0}; //Color (0.5, 0.2, 0.2)
			GLfloat lightColorSpecN2[] = {223/256.0,150/256.0,56/256.0,1.0}; //Color (0.5, 0.2, 0.2)
			GLfloat lightColorSpecN1[] = {150/228.0,223/228.0,56/228.0,1.0}; //Color (0.5, 0.2, 0.2)
			GLfloat lightColorSpecN3[] = {56/228.0,150/228.0,223/228.0,1.0}; //Color (0.5, 0.2, 0.2)
			double directedFact = 0.6;
			double directedFactSpec = 0.5;
							
			GLfloat lightColor[] = {directedFact, directedFact, directedFact, 1.0f}; //Color (0.5, 0.2, 0.2)
			GLfloat lightColorSpec[] = {directedFactSpec, directedFactSpec, directedFactSpec, 1.0f}; //Color (0.5, 0.2, 0.2)


			//GLfloat nightColor[]={198.0/4096,123.0*2/8192,26.0*2/8192,1.0};
			GLfloat nightColor[]={0.2, 0.2, 0.2, 1.0};
			//GLfloat ambientColor[] = {0.1, 0.1,0.1, 1.0f};
			GLfloat ambientColor[] = {0.6, 0.6, 0.6, 1.0};
			if(nightMode)
			{
				/*glLightfv(GL_LIGHT0, GL_DIFFUSE, lightColorN1);
				glLightfv(GL_LIGHT0, GL_SPECULAR, lightColorSpecN1);
				glLightfv(GL_LIGHT1, GL_DIFFUSE, lightColorN2);
				glLightfv(GL_LIGHT1, GL_SPECULAR, lightColorSpecN2);
				glLightfv(GL_LIGHT2, GL_DIFFUSE, lightColorN2);
				glLightfv(GL_LIGHT2, GL_SPECULAR, lightColorSpecN2);
				glLightfv(GL_LIGHT3, GL_DIFFUSE, lightColorN3);
				glLightfv(GL_LIGHT3, GL_SPECULAR, lightColorSpecN3);
				glLightfv(GL_LIGHT4, GL_DIFFUSE, lightColorN3);
				glLightfv(GL_LIGHT4, GL_SPECULAR, lightColorSpecN3);
				glLightfv(GL_LIGHT5, GL_DIFFUSE, lightColorN1);
				glLightfv(GL_LIGHT5, GL_SPECULAR, lightColorSpecN1);
				glLightModelfv(GL_LIGHT_MODEL_AMBIENT, nightColor);*/
				initlights();
			}
			else{
				glLightfv(GL_LIGHT0, GL_DIFFUSE, lightColor);
				glLightfv(GL_LIGHT0, GL_SPECULAR, lightColorSpec);
				glLightfv(GL_LIGHT1, GL_DIFFUSE, lightColor);
				glLightfv(GL_LIGHT1, GL_SPECULAR, lightColorSpec);
				glLightfv(GL_LIGHT2, GL_DIFFUSE, lightColor);
				glLightfv(GL_LIGHT2, GL_SPECULAR, lightColorSpec);
				glLightfv(GL_LIGHT3, GL_DIFFUSE, lightColor);
				glLightfv(GL_LIGHT3, GL_SPECULAR, lightColorSpec);
				glLightfv(GL_LIGHT4, GL_DIFFUSE, lightColor);
				glLightfv(GL_LIGHT4, GL_SPECULAR, lightColorSpec);
				glLightfv(GL_LIGHT5, GL_DIFFUSE, lightColor);
				glLightfv(GL_LIGHT5, GL_SPECULAR, lightColorSpec);
								
								
				glLightModelfv(GL_LIGHT_MODEL_AMBIENT, ambientColor);
			}
			nightMode=!nightMode;
			break;
			}


		 case '1':
				if(l_0){
					if(twoLightOnly) { if(lightOn<0) break; }
					lightOn--;
					glDisable(GL_LIGHT0);
					glDisable(GL_LIGHT5);
					l_0=!l_0;
				}
				else{
					if(twoLightOnly) { if(lightOn>=0) break; }
					lightOn++;
					glEnable(GL_LIGHT0);
					glEnable(GL_LIGHT5);
					l_0=!l_0;
				}
				break;
			case '2':
				if(l_1){
					if(twoLightOnly) { if(lightOn<0) break; }
					lightOn--;
					glDisable(GL_LIGHT1);
					glDisable(GL_LIGHT2);
					l_1=!l_1;
				}
				else{
					if(twoLightOnly) { if(lightOn>=0) break; }
					lightOn++;
					glEnable(GL_LIGHT1);
					glEnable(GL_LIGHT2);
					l_1=!l_1;
				}
				break;
			case '3':

				if(l_2){
					if(twoLightOnly) { if(lightOn<0) break; }
					lightOn--;
					glDisable(GL_LIGHT3);
					glDisable(GL_LIGHT4);
					l_2=!l_2;
				}
				else{
					if(twoLightOnly) { if(lightOn>=0) break; }
					lightOn++;
					glEnable(GL_LIGHT3);
					glEnable(GL_LIGHT4);
					l_2=!l_2;
				}
				break;


	/*case '8':	
			dl+=.25;
			//printf("ar: %f ag: %f ab: %f dr: %f dg: %f db: %f sr: %f sg: %f sb: %f\n", &ar,&ag,&ab,&dr,&dg,&db,&sr,&sg,&sb);
			if(dl<10.0)
			{
				ar+=0.05;
				ag+=0.035;
				ab+=.0485;
				dr=0.0;
				dg=0.0;
				db=0.0;
				sr=1.0;
				sg=1.0;
				sb=0.0;
				lighting();
			}
			if(dl>=10.0 &&dl<20.0)
			{
				ar-=0.05;
				ag-=0.035;
				ab-=.0485;
				dr=0.0;
				dg=0.0;
				db=0.0;
				sr=1.0;
				sg=1.0;
				sb+=0.05;
				lighting();

			}
			if(dl==24.0)
			{ 
				dl=0;
				ar=0.0;
				ag+=0.015;
				ab+=.00015;
				dr=0.0;
				dg=0.0;
				db=0.0;
				sr=1.0;
				sg=1.0;
				sb=0.0;
				lighting();
			}
		break;

	
	case '2':	
			dl-=.25;
			//printf("ar: %f ag: %f ab: %f dr: %f dg: %f db: %f sr: %f sg: %f sb: %f\n", &ar,&ag,&ab,&dr,&dg,&db,&sr,&sg,&sb);
			if(dl<10.0)
			{
				ar-=0.05;
				ag-=0.035;
				ab-=.0485;
				dr=0.0;
				dg=0.0;
				db=0.0;
				sr=1.0;
				sg=1.0;
				sb=0.0;
				lighting();
			}
			if(dl>=10.0 &&dl<20.0)
			{
				ar+=0.05;
				ag+=0.035;
				ab+=.0485;
				dr=0.0;
				dg=0.0;
				db=0.0;
				sr=1.0;
				sg=0.0;
				sb-=0.05;
				lighting();

			}
			if(dl==24.0)
			{ 
				dl=0;
				ar=0.0;
				ag-=0.015;
				ab-=.00015;
				dr=0.0;
				dg=0.0;
				db=0.0;
				sr=1.0;
				sg=1.0;
				sb=0.0;
				lighting();
			}
		break;*/


	case 27:	//ESCAPE KEY -- simply exit
		exit(0);
		break;

	default:
		break;
	}
}

void specialKeyListener(int key, int x,int y){
	V p;

	switch(key){
	case GLUT_KEY_DOWN:
		p=(dir*perp).unit();
		dir=dir.rot(p,ang_speed);
		perp=perp.rot(p,ang_speed);
		break;
	case GLUT_KEY_UP:
		p=(perp*dir).unit();
		dir=dir.rot(p,ang_speed);
		perp=perp.rot(p,ang_speed);
		break;

	case GLUT_KEY_LEFT:
		dir=dir.rot(-perp,ang_speed);
		break;
	case GLUT_KEY_RIGHT:
		dir=dir.rot(perp,ang_speed);
		break;

	case GLUT_KEY_PAGE_UP:
		loc=loc+speed*perp;
		break;
	case GLUT_KEY_PAGE_DOWN:
		loc=loc-speed*perp;
		break;

	case GLUT_KEY_INSERT:
		break;

	case GLUT_KEY_HOME:
		cameraAngle = 0;	//// init the cameraAngle
		cameraAngleDelta = 0.03;
		cameraHeight = 300;
		cameraRadius = 2100;

		loc=_L,dir=_D,perp=_P;
		break;
	case GLUT_KEY_END:
		break;

	default:
		break;
	}
}



void init(){

	dl=5;
	ar=0.0;
	ag=0.0;
	ab=0.0;
	dr=0.0;
	dg=0.0;
	db=0.0;
	sr=0.0;
	sg=0.0;
	sb=0.0;

	loadImage();
	initlights();
	//lighting();


	//codes for initialization
	
	//clear the screen
	/*glClearColor(BLACK, 0);
	cameraAngle = 0;
*/
	/************************
	/ set-up projection here
	************************/
	//load the PROJECTION matrix
	
	//give PERSPECTIVE parameters



	/********************************/

	//codes for initialization
	cameraAngle = 0;	//// init the cameraAngle
	cameraAngleDelta = 0.03;
	cameraHeight = 300;
	cameraRadius = 2100;

	loc=_L,dir=_D,perp=_P;

	//clear the screen
	glClearColor(BLACK, 0);

	/************************
	/ set-up projection here
	************************/
	//load the PROJECTION matrix
	glMatrixMode(GL_PROJECTION);
	
	//initialize the matrix
	glLoadIdentity();

	//give PERSPECTIVE parameters
	gluPerspective(70,	1,	0.1,	10000.0);


	/*GLfloat mat_specular[] = { 0.0, 0.0, 0.0, 1.0 };
    GLfloat mat_shininess[] = { 50.0 };
    GLfloat light_position[] = { 1.0, 1.0, 1.0,0.0 };

    glMaterialfv(GL_FRONT, GL_SPECULAR, mat_specular);
    glMaterialfv(GL_FRONT, GL_SHININESS, mat_shininess);
    glLightfv(GL_LIGHT0, GL_POSITION, light_position);
	
	glColorMaterial(GL_FRONT_AND_BACK, GL_AMBIENT_AND_DIFFUSE);
	glEnable(GL_COLOR_MATERIAL);
    glEnable(GL_LIGHTING);
    glEnable(GL_LIGHT0);
    glDepthFunc(GL_LEQUAL);*/

	
}

int main(int argc, char **argv){
	glutInit(&argc,argv);
	glutInitWindowSize(500, 500);
	glutInitWindowPosition(0, 0);
	glutInitDisplayMode(GLUT_DEPTH | GLUT_DOUBLE | GLUT_RGB);	//Depth, Double buffer, RGB color

	glutCreateWindow("My OpenGL Program");

	init();

	glEnable(GL_DEPTH_TEST);	//enable Depth Testing

	glutDisplayFunc(display);	//display callback function
	glutIdleFunc(animate);		//what you want to do in the idle time (when no drawing is occuring)

	//ADD keyboard listeners:
	glutKeyboardFunc(keyboardListener);
	glutSpecialFunc(specialKeyListener);

//	glViewport(0, 0, 640, 480);
   // cam.setShape(65.0f, 64.0f/48.0f, 10.0f, 4000.0f);
   // cam.set(eye1, look, up); // make the initial camera

	glutMainLoop();		//The main loop of OpenGL

	return 0;
}
