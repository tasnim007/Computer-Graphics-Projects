#include<stdio.h>
#include<stdlib.h>
#include<math.h>
#include <windows.h>		
#include<GL/glut.h>

#define BLACK 0, 0, 0
#define EPSILON 1.0e-8
#define BALLRADIUS 40
#define HOUSEHEIGHT 500

template <class T> inline T sqr(const T &x){
	return x*x;
}



/******************************** Class TVECTOR***************************************************/


class myVector {
	public:
		enum TStatus { INVALID, DEFAULT, UNIT };

	private:
		double _x, _y, _z;
		TStatus _Status;

	public:
		myVector(){ _x =0.0; _y=0.0; _z = 0.0; _Status=INVALID; }

		myVector(double x, double y, double z) { _x = x; _y = y; _z = z; _Status =DEFAULT; }

		double X() const { return _x; }

		double Y() const { return _y; }

		double Z() const { return _z; }

		int isUnit() const { return _Status==UNIT; }

		int isDefault() const { return _Status==DEFAULT; }

		int isValid() const { return _Status!=INVALID; }

		myVector &unit() {
			if (isDefault()) {
				double rep = mag();
				if (rep < EPSILON) {
					_x = 0.0;
					_y = 0.0;
					_z = 0.0;
				} else {
					double temp = 1.0 / rep;
					_x *= temp;
					_y *= temp;
					_z *= temp;
				}
				_Status = UNIT;
			}
			return *this;
		}

		static myVector &unit(const myVector &v, myVector &result) { result = v; return result.unit(); }

		static myVector unit(const myVector &v) { return myVector(v).unit(); }

		double mag() const { return (isValid() ? (isUnit() ? 1.0 : sqrt(sqr(X()) + sqr(Y()) + sqr(Z()))) : 0.0); }

		double dot(const myVector &v) const { return ((isValid() && v.isValid()) ? (X()*v.X() + Y()*v.Y() + Z()*v.Z()) : 0.0); }

		static double dot(const myVector &v1, const myVector &v2) { return v1.dot(v2); }

		double dist(const myVector &v) const { return (*this-v).mag(); }

		static myVector &add(const myVector &v1, const myVector &v2, myVector &result) {
			if (v1.isValid() && v2.isValid()) {
				result._x = v1._x + v2._x;
				result._y = v1._y + v2._y;
				result._z = v1._z + v2._z;
				result._Status = DEFAULT;
			} else
				result = myVector();
			return result;
		}
		
		static myVector &subtract(const myVector &v1, const myVector &v2, myVector &result){
			if (v1.isValid() && v2.isValid()) {
				result._x = v1._x - v2._x;
				result._y = v1._y - v2._y;
				result._z = v1._z - v2._z;
				result._Status = DEFAULT;
			} else
				result = myVector();
			return result;
		}

		static myVector &cross(const myVector &v1, const myVector &v2, myVector &result) {
			if (v1.isValid() && v2.isValid()) {
				result._x = v1._y * v2._z - v1._z * v2._y;
				result._y = v1._z * v2._x - v1._x * v2._z;
				result._z = v1._x * v2._y - v1._y * v2._x;
				result._Status = DEFAULT;
			} else
				result = myVector();
			return result;
		}

		static myVector &invert(const myVector &v1, myVector &result) {
			if (v1.isValid()) {
				result._x = -v1._x;
				result._y = -v1._y;
				result._z = -v1._z;
				result._Status = v1._Status;
			} else
				result = myVector();
			return result;
		}

		static myVector &multiply(const myVector &v1, const double &scale, myVector &result) {
			if (v1.isValid()) {
				result._x = v1._x * scale;
				result._y = v1._y * scale;
				result._z = v1._z * scale;
				result._Status = DEFAULT;
			} else
				result = myVector();
			return result;
		}

		// Vector arithmetic, addition, subtraction and vector product

		myVector operator-() const { return invert(*this, myVector()); }

		myVector &operator+=(const myVector &v) { return add(*this, v, *this); }

		myVector &operator-=(const myVector &v) { return subtract(*this, v, *this); }

		myVector &operator*=(const myVector &v) { myVector tv(*this); return cross(tv, v, *this); }

		myVector &operator*=(const double &scale) { return multiply(*this, scale, *this); }

		myVector operator+(const myVector &v) const { myVector tv; return add(*this, v, tv); }

		myVector operator-(const myVector &v) const { myVector tv; return subtract(*this, v, tv); }

		myVector operator*(const myVector &v) const { myVector tv; return cross(*this, v, tv); }

		myVector operator*(const double &scale) const { myVector tv; return multiply(*this, scale, tv); }

};


/******************************** Class TRAY ***************************************************/

class myRay{
	private:
		myVector _P; // Any point on the line
		myVector _V; // Direction of the line

	public:
		myRay() {}
		
		myRay(const myVector &point1, const myVector &point2): _P(point1) {
			_V = (point2.isUnit() ? point2 : myVector::unit(point2-point1));
		}

		myVector P() const { return _P; }

		myVector V() const { return _V; }

		int isValid() const { return V().isUnit() && P().isValid(); }

		double dist(const myVector &point) const {
			if (isValid() && point.isValid()) {
				myVector tv, point2;
				double lambda = myVector::dot(_V, point-_P);
				myVector::add(_P, myVector::multiply(_V, lambda, tv), point2);
				return point.dist(point2);
			}
			return 0.0;
		}

};


/*********************************************************************************************/


double cameraAngle;			
double cameraAngleDelta;

double cameraHeight;	
double cameraRadius;


myVector veloc(0.5,-0.1,0.5);              //initial velocity of balls
myVector accel(0,-.05,0);                 //acceleration

myVector ArrayVel[10];                     //holds velocity of balls
myVector ArrayPos[10];                     //position of balls
myVector OldPos[10];                       //old position of balls
int NrOfBalls;                            //sets the number of balls
double Time=1.0; 
double prevTime;                         //timestep of simulation

struct Plane{
	        myVector _Position;
			myVector _Normal;
};
                                          
struct Cylinder{                          
	   myVector _Position;
       myVector _Axis;
       double _Radius;
};


Plane pl1,pl2,pl3,pl4,pl5,pl6;             //6 planes 
Cylinder cyl1,cyl2,cyl3;                  //3 cylinders 
GLUquadricObj *cylinder_obj;   


/************************* Textures **************************************************/

int num_texture  = -1;
GLuint wallimg, baseimg, grassimg, cylimg,ballimg;
float ar,ag,ab,dr,dg,db,sr,sg,sb;
float dl;



int LoadBitmapImage(char *filename){
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

void loadImage(){
	baseimg = LoadBitmapImage("image/base.bmp");
	wallimg = LoadBitmapImage("image/wall.bmp");
	cylimg = LoadBitmapImage("image/cyl.bmp");
	ballimg = LoadBitmapImage("image/ball.bmp");
	//printf("Load successful");
}



/********************************* Draw the Display ***************************************************/

int DrawGLScene(){								
	int i;
	//glEnable(GL_TEXTURE_2D);
	//glBindTexture(GL_TEXTURE_2D,ballimg);
	for (i=0;i<NrOfBalls;i++){
		switch(i){
			case 1: glColor3f(1.0f,1.0f,1.0f);
					   break;
			case 2: glColor3f(1.0f,1.0f,0.0f);
					   break;
			case 3: glColor3f(0.0f,1.0f,1.0f);
					   break;
			case 4: glColor3f(0.0f,1.0f,0.0f);
					   break;
			case 5: glColor3f(0.0f,0.0f,1.0f);
					   break;
			case 6: glColor3f(0.65f,0.2f,0.3f);
					   break;
			case 7: glColor3f(1.0f,0.0f,1.0f);
					   break;
			case 8: glColor3f(0.0f,0.7f,0.4f);
					   break;
			case 9: glColor3f(0.4f,0.2f,0.0f);
					   break;
			default: glColor3f(1.0f,0,0);
		}
		glPushMatrix();
		glTranslated(ArrayPos[i].X(),ArrayPos[i].Y(),ArrayPos[i].Z());
		glutSolidSphere(BALLRADIUS,20,20);
		glPopMatrix();
	}
		

	glEnable(GL_TEXTURE_2D);
	glBindTexture(GL_TEXTURE_2D,wallimg);
	glColor3f(1, 1, 1);

	glBegin(GL_QUADS);
		glTexCoord2f(1.0f, 0.0f); glVertex3f(320,HOUSEHEIGHT,320);
		glTexCoord2f(1.0f, 1.0f); glVertex3f(320,-320,320);
		glTexCoord2f(0.0f, 1.0f); glVertex3f(-320,-320,320);
		glTexCoord2f(0.0f, 0.0f); glVertex3f(-320,HOUSEHEIGHT,320);
			
		glTexCoord2f(1.0f, 0.0f); glVertex3f(-320,HOUSEHEIGHT,-320);
		glTexCoord2f(1.0f, 1.0f); glVertex3f(-320,-320,-320);
		glTexCoord2f(0.0f, 1.0f); glVertex3f(320,-320,-320);
		glTexCoord2f(0.0f, 0.0f); glVertex3f(320,HOUSEHEIGHT,-320);
		
		glTexCoord2f(1.0f, 0.0f); glVertex3f(320,HOUSEHEIGHT,-320);
		glTexCoord2f(1.0f, 1.0f); glVertex3f(320,-320,-320);
		glTexCoord2f(0.0f, 1.0f); glVertex3f(320,-320,320);
		glTexCoord2f(0.0f, 0.0f); glVertex3f(320,HOUSEHEIGHT,320);
		
		glTexCoord2f(1.0f, 0.0f); glVertex3f(-320,HOUSEHEIGHT,320);
		glTexCoord2f(1.0f, 1.0f); glVertex3f(-320,-320,320);
		glTexCoord2f(0.0f, 1.0f); glVertex3f(-320,-320,-320);
		glTexCoord2f(0.0f, 0.0f); glVertex3f(-320,HOUSEHEIGHT,-320);
	glEnd();

	glBindTexture(GL_TEXTURE_2D, baseimg); 
    glBegin(GL_QUADS);
		glTexCoord2f(1.0f, 0.0f); glVertex3f(-320,-320,320);
		glTexCoord2f(1.0f, 1.0f); glVertex3f(320,-320,320);
		glTexCoord2f(0.0f, 1.0f); glVertex3f(320,-320,-320);
		glTexCoord2f(0.0f, 0.0f); glVertex3f(-320,-320,-320);

	/*	glTexCoord2f(1.0f, 0.0f); glVertex3f(-320,HOUSEHEIGHT,320);
		glTexCoord2f(1.0f, 1.0f); glVertex3f(320,HOUSEHEIGHT,320);
		glTexCoord2f(0.0f, 1.0f); glVertex3f(320,HOUSEHEIGHT,-320);
		glTexCoord2f(0.0f, 0.0f); glVertex3f(-320,HOUSEHEIGHT,-320);*/
	glEnd();


	glBindTexture(GL_TEXTURE_2D, cylimg);  
	
	glColor3f(0.5,0.5,0.5);
    
	glPushMatrix();
		glRotatef(90, 1,0,0);
		glTranslatef(0,0,-500);
		gluCylinder(cylinder_obj, 60, 60, HOUSEHEIGHT+300, 20, 2);
	glPopMatrix();

  	glPushMatrix();
		glTranslatef(200,-300,-320);
		gluCylinder(cylinder_obj, 60, 60, 650, 20, 2);
	glPopMatrix();

	glPushMatrix();
		glTranslatef(-200,0,0);
		glRotatef(135, 1,0,0);
		glTranslatef(0,0,-450);
		gluCylinder(cylinder_obj, 30, 30, 880, 20, 2);
	glPopMatrix();

	glPushMatrix();{
		glColor3f(1,1,1);
		glTranslatef(0,HOUSEHEIGHT-2,0);
		glRotatef(90,1,0,0);
		gluDisk(cylinder_obj, .001, 60, 20, 20);
	}glPopMatrix();

	glDisable(GL_TEXTURE_2D);

	

	glPushMatrix();{
		glEnable(GL_BLEND);
		glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
		glColor4f(.2,.2,.8, .3);
		//glColor4f(.7,	.7,	.7, .7);
		//glColor3f(0,0,0.4);

		glBegin(GL_QUADS);
		
			 glVertex3f(-320,HOUSEHEIGHT,320);
			 glVertex3f(320,HOUSEHEIGHT,320);
			 glVertex3f(320,HOUSEHEIGHT,-320);
			 glVertex3f(-320,HOUSEHEIGHT,-320);
		glEnd();
		

	}glPopMatrix();
	
	return TRUE;										
}





void display(){

	//clear the display
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
	glClearColor(BLACK, 0);	//color black
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

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

//	gluLookAt(300, 150, 50,		0,0,0,		0,0,1);

	gluLookAt(cameraRadius*cos(cameraAngle), cameraHeight,cameraRadius*sin(cameraAngle),		0,0,0,		0,1,0);
	
	//NOTE: the camera still CONSTANTLY looks at the center
	
	
	//again select MODEL-VIEW
	glMatrixMode(GL_MODELVIEW);


	/****************************
	/ Add your objects from here
	****************************/
	

	//some gridlines along the field
/*	int i;

	glColor3f(0.3, 0.3, 0.3);	//grey
	glBegin(GL_LINES);{
		for(i=-10;i<=10;i++){

			if(i==0)
				continue;	//SKIP the MAIN axes

			//lines parallel to Y-axis
			glVertex3f(i*10, -100, 0);
			glVertex3f(i*10,  100, 0);

			//lines parallel to X-axis
			glVertex3f(-100, i*10, 0);
			glVertex3f( 100, i*10, 0);
		}
	}glEnd();

	// draw the two AXES
	glColor3f(1, 1, 1);	//100% white
	glBegin(GL_LINES);{
		//Y axis
		glColor3f(0, 1, 0);
		glVertex3f(0, 0, 0);	// intentionally extended to -150 to 150, no big deal
		glVertex3f(0,  150, 0);

		//X axis
		glColor3f(1, 0, 0);
		glVertex3f(0, 0, 0);
		glVertex3f( 150, 0, 0);
	}glEnd();

*/


	DrawGLScene();



	//ADD this line in the end --- if you use double buffer (i.e. GL_DOUBLE)
	glutSwapBuffers();
}


/******************************* Ball collision *********************************************/
/**	The parameters:
	Time2
	
	The values returned through the parameters are:
	point-> position of first one of colliding ball
	TimePoint-> colliding time of balls
	BallNr1-> first one of colliding balls
	BallNr2-> second one of colliding balls

**/

int Ball_BallCollisionDetection(myVector& point, double& TimePoint, double Time2, int& BallNr1, int& BallNr2)
{
	myVector RelativeV;
	myRay rays;
	double MyTime=0.0, Add=Time2/150.0, Timedummy=10000;
	myVector posi;
	
	//Test all balls against eachother in 150 small steps
	for (int i=0;i<NrOfBalls-1;i++)
	{
	 for (int j=i+1;j<NrOfBalls;j++)
	 {	
		    RelativeV=ArrayVel[i]-ArrayVel[j];
			rays=myRay(OldPos[i],myVector::unit(RelativeV));	//rays-> direction to RelativeV
			MyTime=0.0;

			if ( (rays.dist(OldPos[j])) > 2*BALLRADIUS) continue; 

			while (MyTime<Time2)	//until the current timestep
			{
			   MyTime+=Add;			//add slice of time in every loop
			   posi=OldPos[i]+RelativeV*MyTime;	//position of ith ball
			   if (posi.dist(OldPos[j])<= 2*BALLRADIUS) {
										   point=posi;
										   if (Timedummy>(MyTime-Add)) Timedummy=MyTime-Add;	//time of collision
										   BallNr1=i;
										   BallNr2=j;
										   break;
										}
			
			}
	 }

	}

	if (Timedummy!=10000) {		//if collide with atleast one ball
			TimePoint=Timedummy;	
	        return 1;
	}

	return 0;
}



/*********************************Plane collision check *************************************/

/** The parameters:
	the plane, 
	the Position of ball,
	direction of the ball (unit vector of velocity),
	
	The values returned through the parameters are:
		a double (lamda) where the collision distance is stored
		and the returned normal at the collision point
**/
int Ball_PlaneCollisionDetection(const Plane& plane,const myVector& position,const myVector& direction, double& lamda, myVector& pNormal)
{

    double DotProduct=direction.dot(plane._Normal);
	double t;

    if ((DotProduct<EPSILON)&&(DotProduct>-EPSILON)) //ball direction is paralle to plane
		return 0;

    t=(plane._Normal.dot(plane._Position-position))/DotProduct; //Distance To Collision Point (parametric eqn's t)

    if (t<-EPSILON) 	// Test If Collision is Behind Start (case when t<0  in parametric eqn)
		return 0;

    pNormal=plane._Normal;
	lamda=t;
    return 1;

}


/******************************** Cylinder collision ***************************************************/
/** The parameters:
	cylinder, 
	the Position of ball,
	direction of the ball (unit vector of velocity),
	
	The values returned through the parameters are:
		a double (lamda) where the collision distance is stored,
		the normal at the intersection point,
		and the intersection point itself.
**/
int Ball_CylinderCollisionDetection(const Cylinder& cylinder,const myVector& position,const myVector& direction, double& lamda, myVector& pNormal,myVector& newposition)
{
	myVector RC;
	double d;
	double t,s;
	myVector n,D,O;
	double ln;
	double in,out;
	

	myVector::subtract(position,cylinder._Position,RC);	//RC -> vector representing from ball pos to cyl pos 
	myVector::cross(direction,cylinder._Axis,n);			//n-> normal to the plain denoted by ball's velocity dir and cyl axis

    ln=n.mag();

	if ( (ln<EPSILON)&&(ln>-EPSILON) ) return 0;	//if direction of ball and cyl axis is parallel -> don't collide

	n.unit();

	d= fabs( RC.dot(n) );	//projection of RC onto n	

    if (d<=cylinder._Radius)
	{
		myVector::cross(RC,cylinder._Axis,O);
		t= - O.dot(n)/ln;	//projection of O along n
		myVector::cross(n,cylinder._Axis,O);
		O.unit();
		s= fabs( sqrt(cylinder._Radius*cylinder._Radius - d*d) / direction.dot(O) );

		in=t-s;
		out=t+s;

		if (in<-EPSILON){
			if (out<-EPSILON) return 0;
			else lamda=out;
		}
		else
        if (out<-EPSILON) {
			      lamda=in;
		}
		else
		if (in<out) lamda=in;
		else lamda=out;

    	newposition=position+direction*lamda;
		myVector HB=newposition-cylinder._Position;
		pNormal=HB - cylinder._Axis*(HB.dot(cylinder._Axis));
		pNormal.unit();

		return 1;
	}
    
	return 0;
}


/*******************************collision's code *****************************************************/


void physicsSimulation()
{
  double rt,rt2,rt4,lamda;
  myVector norm,uveloc;
  myVector normal,point,time;
  double RestTime,BallTime;
  myVector Pos2;
  int BallNr=0,dummy=0,BallColNr1,BallColNr2;
  myVector Nc;

 
	RestTime=Time;	//each loop time(time step)
	  

	//Compute velocity for next timestep 
	for (int j=0;j<NrOfBalls;j++)
	  ArrayVel[j]+=accel*RestTime;	//v = u+gt

	//While timestep not over
	while (RestTime>EPSILON)
	{
	   lamda=10000;   //lamda-> denotes collision time for detecting first collision
	
	   //For all the balls find closest intersection between balls and planes/cylinders
   	   for (int i=0;i<NrOfBalls;i++)
	   {
		      //compute new position and distance
			  OldPos[i]=ArrayPos[i];
			  myVector::unit(ArrayVel[i],uveloc);
			  ArrayPos[i]=ArrayPos[i]+ArrayVel[i]*RestTime;	
			  rt2=OldPos[i].dist(ArrayPos[i]);		//rt2= DST -> distance between old pos and current pos of ball

			  //Test if collision occured between ball and all 5 planes
			  if (Ball_PlaneCollisionDetection(pl1,OldPos[i],uveloc,rt,norm))
			  {  
				  //Find intersection time
				  rt4=rt*RestTime/rt2;		//time when the collision takes place, Tc= Dsc*T / Dst  (distance between start - collision point Dsc)

				  //if smaller than the one already stored replace and in timestep
				  if (rt4<=lamda)
				  { 
				    if (rt4<=RestTime+EPSILON)	//check whether collision occured in current timestep
						 if (! ((rt<=EPSILON)&&(uveloc.dot(norm)>EPSILON)) )	// if collision takes place behind the old pos or ball dir and plane dir are parallel
						  {
							normal=norm;
							point=OldPos[i]+uveloc*rt;
							lamda=rt4;
							BallNr=i;
						  }
				  }
			  }
			  
			  if (Ball_PlaneCollisionDetection(pl2,OldPos[i],uveloc,rt,norm))
			  {
				   rt4=rt*RestTime/rt2;

				  if (rt4<=lamda)
				  { 
				    if (rt4<=RestTime+EPSILON)
						if (! ((rt<=EPSILON)&&(uveloc.dot(norm)>EPSILON)) )
						 {
							normal=norm;
							point=OldPos[i]+uveloc*rt;
							lamda=rt4;
							BallNr=i;
							//dummy=1;
						 }
				  }
				 
			  }

			  if (Ball_PlaneCollisionDetection(pl3,OldPos[i],uveloc,rt,norm))
			  {
			      rt4=rt*RestTime/rt2;

				  if (rt4<=lamda)
				  { 
				    if (rt4<=RestTime+EPSILON)
						if (! ((rt<=EPSILON)&&(uveloc.dot(norm)>EPSILON)) )
						 {
							normal=norm;
							point=OldPos[i]+uveloc*rt;
							lamda=rt4;
							BallNr=i;
						 }
				  }
			  }

			  if (Ball_PlaneCollisionDetection(pl4,OldPos[i],uveloc,rt,norm))
			  {
				  rt4=rt*RestTime/rt2;

				  if (rt4<=lamda)
				  { 
				    if (rt4<=RestTime+EPSILON)
						if (! ((rt<=EPSILON)&&(uveloc.dot(norm)>EPSILON)) )
						 {
							normal=norm;
							point=OldPos[i]+uveloc*rt;
							lamda=rt4;
							BallNr=i;
						 }
				  }
			  }

			  if (Ball_PlaneCollisionDetection(pl5,OldPos[i],uveloc,rt,norm))
			  {
				  rt4=rt*RestTime/rt2;

				  if (rt4<=lamda)
				  { 
				    if (rt4<=RestTime+EPSILON)
						if (! ((rt<=EPSILON)&&(uveloc.dot(norm)>EPSILON)) )
						 {
							normal=norm;
							point=OldPos[i]+uveloc*rt;
							lamda=rt4;
							BallNr=i;
						 }
				  }
			  }


			 if (Ball_PlaneCollisionDetection(pl6,OldPos[i],uveloc,rt,norm))
			  {
				  rt4=rt*RestTime/rt2;

				  if (rt4<=lamda)
				  { 
				    if (rt4<=RestTime+EPSILON)
						if (! ((rt<=EPSILON)&&(uveloc.dot(norm)>EPSILON)) )
						 {
							normal=norm;
							point=OldPos[i]+uveloc*rt;
							lamda=rt4;
							BallNr=i;
						 }
				  }
			  }


              //intersection with the 3 cylinders
			  if (Ball_CylinderCollisionDetection(cyl1,OldPos[i],uveloc,rt,norm,Nc))
			  {
				  rt4=rt*RestTime/rt2;

				  if (rt4<=lamda)
				  { 
				    if (rt4<=RestTime+EPSILON)
						if (! ((rt<=EPSILON)&&(uveloc.dot(norm)>EPSILON)) )
						 {
							normal=norm;
							point=Nc;
							lamda=rt4;
							BallNr=i;
						 }
				  }
				 
			  }
			  if (Ball_CylinderCollisionDetection(cyl2,OldPos[i],uveloc,rt,norm,Nc))
			  {
				  rt4=rt*RestTime/rt2;

				  if (rt4<=lamda)
				  { 
				    if (rt4<=RestTime+EPSILON)
						if (! ((rt<=EPSILON)&&(uveloc.dot(norm)>EPSILON)) )
						 {
							normal=norm;
							point=Nc;
							lamda=rt4;
							BallNr=i;
						 }
				  }
				 
			  }
			  if (Ball_CylinderCollisionDetection(cyl3,OldPos[i],uveloc,rt,norm,Nc))
			  {
				  rt4=rt*RestTime/rt2;

				  if (rt4<=lamda)
				  { 
				    if (rt4<=RestTime+EPSILON)
						if (! ((rt<=EPSILON)&&(uveloc.dot(norm)>EPSILON)) )
						 {
							normal=norm;
							point=Nc;
							lamda=rt4;
							BallNr=i;
						 }
				  }
				 
			  }
	   }

	   //After all balls were tested with planes/cylinders test for collision between them and replace if collision time smaller
	   
	   if (Ball_BallCollisionDetection(Pos2,BallTime,RestTime,BallColNr1,BallColNr2))
		  {
			  
			  if ( (lamda==10000) || (lamda>BallTime) )	//if ball doesn't collide with cyl or plane OR ball collision occurs before other collisions
			  {
				//Find the resultant velocity using conservation of momentum theory

				  RestTime=RestTime-BallTime;

				  myVector pb1,pb2,xaxis,U1x,U1y,U2x,U2y,V1x,V1y,V2x,V2y;
				  double a,b;

				  pb1=OldPos[BallColNr1]+ArrayVel[BallColNr1]*BallTime;	//Position Of Ball1( s=vt )
				  pb2=OldPos[BallColNr2]+ArrayVel[BallColNr2]*BallTime;	//Position Of Ball1
				  xaxis=(pb2-pb1).unit();				//axis vector which joins the 2 centers of the spheres for ball 1

				  a=xaxis.dot(ArrayVel[BallColNr1]);	//projection of old velocity along the '2 centere's ' axis
				  U1x=xaxis*a;							//projected vector of old velocity along the '2 centere's ' axis
				  U1y=ArrayVel[BallColNr1]-U1x;			//projected vector along perp to the prev axis

				  xaxis=(pb1-pb2).unit();				//axis vector which joins the 2 centers of the spheres for ball 2(opposite dir of preve)
				  b=xaxis.dot(ArrayVel[BallColNr2]);
      			  U2x=xaxis*b;
				  U2y=ArrayVel[BallColNr2]-U2x;
					
				//conservation of momentum theory: V1x=(m1U1x + m2U2x - m2(U1x-U2x)) / (m1+m2)
				//we considered m1=m2=1
				
				  V1x=(U1x+U2x-(U1x-U2x))*0.5;
				  V2x=(U1x+U2x-(U2x-U1x))*0.5;
				  V1y=U1y;
				  V2y=U2y;

				  for (j=0;j<NrOfBalls;j++)
				  ArrayPos[j]=OldPos[j]+ArrayVel[j]*BallTime;

				  ArrayVel[BallColNr1]=V1x+V1y;
				  ArrayVel[BallColNr2]=V2x+V2y;

				  continue;
			  }
		  }
		  

		   
		if (lamda!=10000)	//collision with cyl or plane occured before ball collision
		{		 
				  RestTime-=lamda;

				  for (j=0;j<NrOfBalls;j++)
				  ArrayPos[j]=OldPos[j]+ArrayVel[j]*lamda;

				  rt2=ArrayVel[BallNr].mag();
				  ArrayVel[BallNr].unit();
				  
				  // Compute Reflection (Collision Response): R= 2*(-I dot N)*N + I
				  ArrayVel[BallNr]=myVector::unit( (normal*(2*normal.dot(-ArrayVel[BallNr]))) + ArrayVel[BallNr] );
				  ArrayVel[BallNr]=ArrayVel[BallNr]*rt2;
					
				 // continue;
				  
		}
		else RestTime=0;

	}

}






/*******************************************************************************************************/


void animate(){

	physicsSimulation();
	//printf("Radius = %lf Height = %lf  cameraAngle= %lf\n",cameraRadius, cameraHeight,cameraAngle);

	//codes for any changes in Camera
	
//	cameraAngle += 0.002;	// camera will rotate at 0.002 radians per frame.
	
	//codes for any changes in Models

	//MISSING SOMETHING? -- YES: add the following
	glutPostRedisplay();	//this will call the display AGAIN
}





void keyboardListener(unsigned char key, int x,int y){
	switch(key){

		case '1':	//move car forward
			//for rotating the wheel in its axis
			break;

		case '2':	//move car backward
			
			break;

		case '3':	//rotate car along x axis 
			
			break;

		case '4':	//rotate car along x axis
		
			break;
		
		case '5':	//rotate car along y axis
			
			break;

		case '6':	//rotate car along y axis
		
			break;

		case '7':	//rotate car along z axis
			
			break;

		case '8':	//rotate car along z axis
		
			break;

		case '9':	//increase speed
		
			break;

		case '0':	//decrease speed
		
			break;

		case '+':	//increase timestep
			if(Time> EPSILON)
			Time += .1;
			break;

		case '-':	//decrease timestep
			if(Time> EPSILON)
			Time -= .1;
			break;

		case 27:	//ESCAPE KEY -- simply exit
			exit(0);
			break;

		case 'p':
			if(Time> EPSILON)
				prevTime = Time;
			Time = 0;
			break;

		case 's':
			if(Time< EPSILON)
				Time = prevTime;
			break;

		case 'l':
			
			break;


		default:
			break;
	}
}



void specialKeyListener(int key, int x,int y){
	switch(key){
		case GLUT_KEY_DOWN:	//moving camera down
			cameraHeight -= 3;
			break;

		case GLUT_KEY_UP:	//moving camera up
			cameraHeight += 3;
			break;

		case GLUT_KEY_RIGHT://moving camera to right
			cameraAngle += cameraAngleDelta;
			break;

		case GLUT_KEY_LEFT://moving camera to left
			cameraAngle -= cameraAngleDelta;
			break;

		case GLUT_KEY_PAGE_UP:
			cameraRadius -= 10;
			break;

		case GLUT_KEY_PAGE_DOWN:
			cameraRadius += 10;
			break;

		case GLUT_KEY_INSERT:
			break;

		case GLUT_KEY_HOME:
			break;
		case GLUT_KEY_END:
			break;

		default:
			break;
	}
}



/***************************Init Variables of display **********************************************************/
void InitVars()
{
	 //create palnes
	pl1._Position=myVector(0,-(320-BALLRADIUS),0);
	pl1._Normal=myVector(0,1,0);
	pl2._Position=myVector((320-BALLRADIUS),0,0);
	pl2._Normal=myVector(-1,0,0);
	pl3._Position=myVector(-(320-BALLRADIUS),0,0);
	pl3._Normal=myVector(1,0,0);
	pl4._Position=myVector(0,0,(320-BALLRADIUS));
	pl4._Normal=myVector(0,0,-1);
	pl5._Position=myVector(0,0,-(320-BALLRADIUS));
	pl5._Normal=myVector(0,0,1);
	pl6._Position=myVector(0,(HOUSEHEIGHT-BALLRADIUS),0);
	pl6._Normal=myVector(0,-1,0);


	//create cylinders
	cyl1._Position=myVector(0,0,0);
	cyl1._Axis=myVector(0,1,0);
	cyl1._Radius=60+BALLRADIUS;				//calculation is done before ball penetrates the cyl i.e enhancing cyl radius for better result

	cyl2._Position=myVector(200,-300,0);
	cyl2._Axis=myVector(0,0,1);
	cyl2._Radius=60+BALLRADIUS;

	cyl3._Position=myVector(-200,0,0);
	cyl3._Axis=myVector(0,1,1);
    cyl3._Axis.unit();
	cyl3._Radius=30+BALLRADIUS;
	
	//create quadratic object to render cylinders
	cylinder_obj= gluNewQuadric();
	gluQuadricTexture(cylinder_obj, GL_TRUE);

    //Set initial positions and velocities of balls
	NrOfBalls=10;
	ArrayVel[0]=veloc;
	ArrayPos[0]=myVector(199,180,10);
	ArrayVel[1]=veloc;
	ArrayPos[1]=myVector(0,150,100);
	ArrayVel[2]=veloc;
	ArrayPos[2]=myVector(-100,180,-100);
	for (int i=3; i<10; i++)
	{
         ArrayVel[i]=veloc;
	     ArrayPos[i]=myVector(-500+i*75, 300, -500+i*50);
	}

}


void init(){

	cameraAngle = 2.34;	
	cameraAngleDelta = 0.02;
	cameraHeight = 498;
	cameraRadius = -220;

	

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
	//field of view in the Y (vertically)
	//aspect ratio that determines the field of view in the X direction (horizontally)
	//near distance
	//far distance


	InitVars();
	loadImage();

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

	//ADD mouse listeners:
//	glutMouseFunc(mouseListener);


	//**Instructions ***//

	printf("Camera control: \n");
	printf("  --left,right arrow ==> moving around the structure \n");
	printf("  --up,down arrow ==> Up and Down the camera \n");
	printf("  --page up and page down for zooming(by changing camera radius)	\n");
	printf("  \n\n");
	

	glutMainLoop();		//The main loop of OpenGL


	return 0;
}