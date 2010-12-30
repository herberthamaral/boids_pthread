/******************************************************************/
/* TRABALHO PRATIO II - COMPUTACAO GRAFICA DCC/UFMG - 2009.2      */
/* ALUNO: ANTONIO WILSON VIEIRA                                   */
/*                                                                */
/* INSTRUCOES:                                                    */
/* Cameras com teclas (1/2/3)                                     */
/* Direcao com SETAS (Left/Right/Up/Down)                         */
/* Velocidade com teclas (PgUP/PgDown)                            */
/* Congela tela com tecla (D)                                     */
/* Muda quantidade de boids com (+/-)                             */
/******************************************************************/;

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <time.h>
#include <list>
#include <GL/glut.h>
#include "vector.h"
#include <pthread.h>
using namespace std;

void Display(void); // Funcao de desenho
void CreateEnvironment(void); // Define parametros do ambiente grafico
void MakeGeometry(void); // Desenha os objetos geometricos da cena
void MakeLighting(void); // Define parametros de iluminacao
void MakeCamera(int,int);
void HandleKeyboard(unsigned char key,int x, int y);
void HandleSpecialKeyboard(int key,int x, int y);
void HandleMouse(int,int,int,int);
void HandleVisibility(int vis);
void HandleIdle(void);
void MoveAll();

GLfloat globalambient[] = {0.3,0.3,0.3,1.0};

/* The specifications for 3 light sources */
GLfloat pos0[] = {0.0,0.0,0.9,0.0};      /* w = 0 == infinite distance */
GLfloat dif0[] = {0.3,0.3,0.8,1.0};
GLfloat pos1[] = {5.0,-5.0,0.0,0.0};   /* Light from below */
GLfloat dif1[] = {0.4,0.4,0.4,1.0};    /* Fainter */

#define PI 3.141592653589793238462643
float alpha=45;
float beta =90;
float ratio=0.4;
double phi = 60; // Angulo de elevacao da camera 
double theta = 0;
int  path= 0;
int cam=2;
float specialtorus=0;

float torus[4][8]={{10,20,0,80,30,0,1,1},{30,35,-150,0,65,0,0,1},{10,30,150,0,55,1,0,0.5},{10,30,-50,100,40,0.5,0.3,1}}; // raio menor, raio maior, x, y, z
float ball [4][7]={{25,200,130,25,1,0,1},{30,-160,160,30,0.4,0.6,0.4},{10,-250,-60,10,0.1,0.5,0},{5,12,-12,5,0,1,1}};  // raio, x, y, z;
float cone [4][8]={{20,100,0,0,0,1,1,0},{15,100,-160,-110,0,1,0,0},{25,100,-250,-20,0,0.3,1,0.5},{5,20,0,0,0,1,1,0.5}};   // raio, altura, x, y, z
int nt=4;
int nb=3;
int nc=3;
int doidle=1;
int zaxis=0;

vector mean;
float atx,aty,atz;
float atx2,aty2,atz2;

pthread_mutex_t MUTEX;

typedef struct {int id; vector pos; vector speed; int status;pthread_t thread;  } Boid;
typedef list<Boid>::iterator It;
list<Boid> BOIDS;      

Boid* boid, *goal, *boid1, *boid2;

void *MoveOne(void *b);

int time1=0;
int time2=0;
int size=40;
float vs1[3]={0,0,0};
float vs2[3]={0,0,0};
float vs3[3]={0,0,0};
float vs4[3]={0,0,0};

void DrawShadow(float x, float y, float z, float turn, int id , int status)
{
    if (id==0) return;
    glTranslatef(x,y,0);
    glScalef(2,2,2);
    glRotatef(turn,0,0,1);
    glColor3f(0.2,0.2,0.2); // Sombra
    glBegin(GL_POLYGON);
    glVertex3f(0.4,0,0.1);
    glVertex3f(0.6,0,0.1);
    glVertex3f(0.5,0.15,0.1);
    glEnd();
    glBegin(GL_POLYGON);
    glVertex3f(0.5,0.15,0.1);
    glVertex3f(0.4,0.6,0.1);
    glVertex3f(0.6,0.6,0.1);
    glEnd();
    glBegin(GL_POLYGON);
    glVertex3f(0.5,0.15,0.1);
    glVertex3f(0.5,0.6,0.1);
    glVertex3f(0.6,0.6,0.1);
    glEnd();
    glBegin(GL_POLYGON);
    glVertex3f(0.5,0.15,0.1);
    glVertex3f(0.4,0.6,0.1);
    glVertex3f(0.5,0.6,0.1);
    glEnd();
    glBegin(GL_POLYGON);
    glVertex3f(0.5,0.8,0.1);
    glVertex3f(0.4,0.6,0.1);
    glVertex3f(0.6,0.6,0.1);
    glEnd();
    glBegin(GL_POLYGON);
    glVertex3f(0.5,0.6,0.1);
    glVertex3f(0.25,0.5,0.1);
    glVertex3f(0.3,0.4,0.1);
    glVertex3f(0.48,0.3,0.1);
    glEnd();
    glBegin(GL_POLYGON);
    glVertex3f(0.4,0.6,0.1);
    glVertex3f(0.25,0.5,0.1);
    glVertex3f(0.3,0.4,0.1);
    glVertex3f(0.48,0.3,0.1);
    glEnd();
    glBegin(GL_POLYGON);
    glVertex3f(0.25,0.5,0.1);
    glVertex3f(0.3,0.4,0.1);
    glVertex3f(0.1,0.3,0.1);
    glEnd();
    glBegin(GL_POLYGON);
    glVertex3f(0.5,0.6,0.1);
    glVertex3f(0.75,0.5,0.1);
    glVertex3f(0.7,0.4,0.1);
    glVertex3f(0.52,0.3,0.1);
    glEnd();
    glBegin(GL_POLYGON);
    glVertex3f(0.6,0.6,0.1);
    glVertex3f(0.75,0.5,0.1);
    glVertex3f(0.7,0.4,0.1);
    glVertex3f(0.52,0.3,0.1);
    glEnd();
    glBegin(GL_POLYGON);
    glVertex3f(0.75,0.5,0.1);
    glVertex3f(0.7,0.4,0.1);
    glVertex3f(0.9,0.3,0.1);
    glEnd();
    glRotatef(-turn,0,0,1);
    glScalef(0.5,0.5,0.5);
    glTranslatef(-x,-y,0);
}

void DrawBird(float x, float y, float z, float turn, int id , int status)
{
    float dz=0.2*cos(status*PI/10);
    float dz1=0;//0.05+0.05*sin(status*PI/10);
    float red=float(id)/float(size);
    glColor3f(1,1,0);
    glTranslatef(x,y,z);
    glScalef(2,2,2);
    glRotatef(turn,0,0,1);
    glColor3f(0,1-red,red*red); // cauda
    if (id==0) glColor3f(1,1,1);
    glBegin(GL_POLYGON);
    glVertex3f(0.4,0,dz1-0.2);
    glVertex3f(0.6,0,dz1-0.2);
    glVertex3f(0.5,0.15,0-0.2);
    glEnd();
    glColor3f(0,1-red,red*red); // corpo
    if (id==0) glColor3f(1,1,1);
    glBegin(GL_POLYGON);
    glVertex3f(0.5,0.15,0-0.2);
    glVertex3f(0.4,0.6,0);
    glVertex3f(0.6,0.6,0);
    glEnd();
    glBegin(GL_POLYGON);
    glVertex3f(0.5,0.15,0-0.2);
    glVertex3f(0.5,0.6,-0.1);
    glVertex3f(0.6,0.6,0);
    glEnd();
    glBegin(GL_POLYGON);
    glVertex3f(0.5,0.15,0-0.2);
    glVertex3f(0.4,0.6,0);
    glVertex3f(0.5,0.6,-0.1);
    glEnd();
    glColor3f(0.8,red,0); // Bico
    if (id==0) glColor3f(1,1,1);
    glBegin(GL_POLYGON);
    glVertex3f(0.5,0.8,0);
    glVertex3f(0.4,0.6,0);
    glVertex3f(0.6,0.6,0);
    glEnd();
    glColor3f(1,1,red); // antiasa
    if (id==0) glColor3f(1,1,1);
    glBegin(GL_POLYGON);
    glVertex3f(0.5,0.6,-0.1);
    glVertex3f(0.25,0.5,dz);
    glVertex3f(0.3,0.4,dz);
    glVertex3f(0.48,0.3,0-0.1);
    glEnd();
    glBegin(GL_POLYGON);
    glVertex3f(0.4,0.6,0);
    glVertex3f(0.25,0.5,dz);
    glVertex3f(0.3,0.4,dz);
    glVertex3f(0.48,0.3,0-0.1);
    glEnd();
    glColor3f(0,1-red,red*red); // ponta da asa
    if (id==0) glColor3f(1,1,1);
    glBegin(GL_POLYGON);
    glVertex3f(0.25,0.5,dz);
    glVertex3f(0.3,0.4,dz);
    glVertex3f(0.1,0.3,1.5*dz);
    glEnd();
    glColor3f(1,1,red); // antiasa
    if (id==0) glColor3f(1,1,1);
    glBegin(GL_POLYGON);
    glVertex3f(0.5,0.6,-0.1);
    glVertex3f(0.75,0.5,dz);
    glVertex3f(0.7,0.4,dz);
    glVertex3f(0.52,0.3,0-0.1);
    glEnd();
    glBegin(GL_POLYGON);
    glVertex3f(0.6,0.6,0);
    glVertex3f(0.75,0.5,dz);
    glVertex3f(0.7,0.4,dz);
    glVertex3f(0.52,0.3,0-0.1);
    glEnd();
    glColor3f(0,1-red,red*red); // ponta da asa
    if (id==0) glColor3f(1,1,1);
    glBegin(GL_POLYGON);
    glVertex3f(0.75,0.5,dz);
    glVertex3f(0.7,0.4,dz);
    glVertex3f(0.9,0.3,1.5*dz);
    glEnd();
    glRotatef(-turn,0,0,1);
    glScalef(0.5,0.5,0.5);
    glTranslatef(-x,-y,-z);
}

void CreateEnvironment(void)
{
    glEnable(GL_DEPTH_TEST);

    glLineWidth(1.0);
    glPointSize(1.0);
    glPolygonMode(GL_FRONT_AND_BACK,GL_FILL);
    glFrontFace(GL_CW);
    glDisable(GL_CULL_FACE);
    glClearColor(0.0,0.0,0.0,0.0);         /* Background colour */
    glEnable(GL_COLOR_MATERIAL);
}

void Display(void)
{
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

    glPushMatrix();
    MakeCamera(0,0);
    MakeLighting();
    MakeGeometry();
    glPopMatrix();
    /* glFlush(); This isn't necessary for double buffers */
    glutSwapBuffers();
}

void MakeGeometry(void)
{
    int i,j,k;
    glPushMatrix();

    glColor3f(0.6,0.6,0.6);
    glBegin(GL_LINES);
    for (i=0;i<100;i++) {   
        glVertex3f(1000-20*i,-1000,0.1);
        glVertex3f(1000-20*i, 1000,0.1);
        glVertex3f(-1000,1000-20*i,0.1);
        glVertex3f( 1000,1000-20*i,0.1);
    }
    glEnd();

    glColor3f(0.3,0.5,0.3);
    glBegin(GL_POLYGON);
    glVertex3f(-1000,-1000,0);
    glVertex3f(-1000, 1000,0);
    glVertex3f( 1000, 1000,0); 
    glVertex3f( 1000,-1000,0); 
    glEnd();

    for (i=0; i<nb; i++) {
        glColor3f(ball[i][4],ball[i][5],ball[i][6]);
        glTranslatef(ball[i][1],ball[i][2],ball[i][3]);
        glutSolidSphere(ball[i][0],20,20);
        glTranslatef(-ball[i][1],-ball[i][2],-ball[i][3]);
    }

    for (i=0; i<nc; i++) {
        glColor3f(cone[i][5],cone[i][6],cone[i][7]);
        glTranslatef(cone[i][2],cone[i][3],cone[i][4]);
        glutSolidCone(cone[i][0],cone[i][1],10,10);	
        glTranslatef(-cone[i][2],-cone[i][3],-cone[i][4]);
    }

    for (i=0; i<nt; i++) {
        glColor3f(torus[i][5],torus[i][6],torus[i][7]);
        glTranslatef(torus[i][2],torus[i][3],torus[i][4]);
        glRotatef(90,0,1,0);
        glutSolidTorus(torus[i][0],torus[i][1],20,20);
        glRotatef(-90,0,1,0);
        glTranslatef(-torus[i][2],-torus[i][3],-torus[i][4]);
    }

    It itb = BOIDS.begin();
    float turn=alpha-90;
    while (itb!=BOIDS.end())
    {
        boid=&(*itb);

        float norma=sqrt(boid->speed.x*boid->speed.x+boid->speed.y*boid->speed.y);
        float turn1 = acos(boid->speed.x/norma)*180/PI;
        float turn2 = asin(boid->speed.y/norma)*180/PI;
        turn = turn1;
        if (turn2<0) turn=-turn1;
        turn = turn-90;       
        glDisable(GL_LIGHTING);
        if (boid->id < size) DrawBird(boid->pos.x,boid->pos.y,boid->pos.z,turn,boid->id,boid->status);
        if (boid->id < size) DrawShadow(boid->pos.x,boid->pos.y,boid->pos.z,turn,boid->id,boid->status);
        glEnable(GL_LIGHTING);
        itb++;
    }
    glPopMatrix();
}

/*
   Set up the lighing environment
   */
void MakeLighting(void)
{
    /* Set ambient globally, default ambient for light sources is 0 */
    glLightModelfv(GL_LIGHT_MODEL_AMBIENT,globalambient);

    glLightfv(GL_LIGHT0,GL_POSITION,pos0);
    glLightfv(GL_LIGHT0,GL_DIFFUSE,dif0);

    glLightfv(GL_LIGHT1,GL_POSITION,pos1);
    glLightfv(GL_LIGHT1,GL_DIFFUSE,dif1);

    glEnable(GL_LIGHT0);
    glEnable(GL_LIGHT1);
    glEnable(GL_LIGHTING);
}

/*
   Set up the camera
   Optionally creating a small viewport about 
   the mouse click point for object selection
   */
void MakeCamera(int x,int y)
{
    GLint viewport[4];

    /* Camera setup */
    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();


    gluPerspective(30.0,1.5,5.0,1000.0);

    glMatrixMode(GL_MODELVIEW);
    glLoadIdentity();

    if (cam==1) gluLookAt(0,0,110+zaxis,goal->pos.x,goal->pos.y,goal->pos.z,0.0,0.0,1.0); 
    if (cam==2) gluLookAt(atx ,aty ,atz +zaxis,goal->pos.x,goal->pos.y,goal->pos.z,0.0,0.0,1.0); 
    if (cam==3) gluLookAt(atx2,aty2,atz2+10+zaxis,goal->pos.x,goal->pos.y,goal->pos.z,0.0,0.0,1.0); 
}

/*
   Deal with plain key strokes
   */
void HandleKeyboard(unsigned char key,int x, int y)
{

    if (key=='1') { cam=1; }
    if (key=='2') { cam=2; }
    if (key=='3') { cam=3; }
    if (key=='d' || key=='D') doidle=(doidle+1)%2;
    if (key=='z') zaxis++;
    if (key=='Z') zaxis--;
    if (key=='n' || key=='+')
    {
        //colocar inicialização da thread aqui.
        if (BOIDS.size()>size) {size++; return;}
        boid = new Boid;
        boid->id=size;
        boid->status=rand()%360;
        boid->speed = NullVector(); 
        boid->pos.x   = goal->pos.x+rand()%10;
        boid->pos.y   = goal->pos.y+rand()%10;
        boid->pos.z   = goal->pos.z+rand()%10;
        boid->thread  = NULL;
        BOIDS.push_back(*boid);
        size++;      
    }
    if (key=='N' || key =='-') { if (size>2) size--;}
}

/*
   Deal with special key strokes
   */
void HandleSpecialKeyboard(int key,int x, int y)
{
    switch (key) {

        /*
           case GLUT_KEY_UP:    phi -= 2;  break;
           case GLUT_KEY_DOWN:  phi += 2;  break;
           case GLUT_KEY_LEFT:  theta -=2; break;
           case GLUT_KEY_RIGHT: theta +=2; break;
           */

        case GLUT_KEY_PAGE_UP:    if (ratio<1) ratio+=0.1; break;
        case GLUT_KEY_PAGE_DOWN:  if (ratio>0.1) ratio-=0.1; break;	
        case GLUT_KEY_UP:    if (beta> 2) beta -=2;  break;
        case GLUT_KEY_DOWN:  if (beta<180) beta +=2;  break;	
        case GLUT_KEY_LEFT:  alpha+=2; break;
        case GLUT_KEY_RIGHT: alpha-=2; break;
    }
    goal->speed.x=ratio*cos(PI*alpha/180);
    goal->speed.y=ratio*sin(PI*alpha/180);
    goal->speed.z=ratio*cos(beta*PI/180);
}

/*
   Handle mouse events
   */
void HandleMouse(int button,int state,int x,int y)
{
    int i,maxselect = 100,nhits = 0;
    GLuint selectlist[100];

    if (state == GLUT_DOWN) {
        glSelectBuffer(maxselect,selectlist);
        glRenderMode(GL_SELECT);
        glInitNames();
        glPushName(-1);

        glPushMatrix();
        MakeCamera(x,y);
        MakeGeometry();
        glPopMatrix();
        nhits = glRenderMode(GL_RENDER);

        if (button == GLUT_LEFT_BUTTON) {

        } else if (button == GLUT_MIDDLE_BUTTON) {

        } /* Right button events are passed to menu handlers */
    }
}

/*
   How to handle visibility
   */
void HandleVisibility(int visible)
{
    if (visible == GLUT_VISIBLE)
        glutIdleFunc(HandleIdle);
    else
        glutIdleFunc(NULL);
}

/*
   What to do on an idle event
   */
void HandleIdle(void)
{
    if (doidle==0) return;
    if (clock()-time2>4000)
    {
        specialtorus+=1;
        torus[2][0]=15+10*cos(specialtorus*PI/180);
        torus[2][4]=torus[2][0]+torus[2][1];
        if (beta>91) beta-=0.1;
        if (beta<89) beta+=0.1;
        goal->pos = Add(goal->pos, goal->speed);
        if (goal->pos.x<-900 || goal->pos.x> 900) { alpha=180-alpha; goal->pos.x=895*goal->pos.x/900; }
        if (goal->pos.y<-900 || goal->pos.y> 900) { alpha=360-alpha; goal->pos.y=895*goal->pos.y/900; }
        if (goal->pos.z<  10)                    { beta =90 ; goal->pos.z=10; }
        if (goal->pos.z>100)                    { beta =95 ; goal->pos.z=100; }
        float norma=Norm(goal->speed);
        atx=goal->pos.x-80*goal->speed.x/norma;
        aty=goal->pos.y-80*goal->speed.y/norma;
        atz=goal->pos.z-40*goal->speed.z/norma+5; if (atz<1) atz=5;
        atx2=goal->pos.x-60*goal->speed.y/norma;
        aty2=goal->pos.y+60*goal->speed.x/norma;
        atz2=atz;       

        time2=clock();
        It itb = BOIDS.begin();
        
        //bater as asas
        while (itb!=BOIDS.end())
        {
            boid=&(*itb);
            boid->status=boid->status+1;
            itb++;
        }
        goal->speed.x=ratio*cos(PI*alpha/180);
        goal->speed.y=ratio*sin(PI*alpha/180);
        goal->speed.z=ratio*cos(beta*PI/180);
        MoveAll();
        time1=clock();
    }
    glutPostRedisplay();
}

vector Obstaculo(Boid* bj)
{
    vector v=NullVector();
    vector c=NullVector(); 
    float cz;
    int i;
    
    for (i=0; i<nc; i++) {
        c.x=bj->pos.x-cone[i][2];
        c.y=bj->pos.y-cone[i][3];
        c.z=0; cz=bj->pos.z;//-cone[i][4];
        float R=cone[i][0];
        float H=cone[i][1];
        if (c.z<H && sqrt(c.x*c.x+c.y*c.y)<(R*(H-cz)/H+4))
        { 
            v = Add(bj->speed, c);
        }
    }

    for (i=0; i<nb; i++) {
        vector xball = NullVector(); xball.x=ball[i][1]; xball.y=ball[i][2]; xball.z=ball[i][3];
        xball=Diff(bj->pos,xball);
        float d=Norm(xball)-ball[i][0];
        if (d<ball[i][0]/4)
        {
            v = Add(bj->speed,Mult(xball,ball[i][0]/(d*d)));
        }
    }

    float x,y,z;
    for (i=0; i<nt; i++) {
        x=bj->pos.x; y=bj->pos.y; z=bj->pos.z;;
        float a=torus[i][0]; float c=torus[i][1]; float cx=torus[i][2]; float cy=torus[i][3]; float cz=torus[i][4];
        float tmp1=sqrt(pow(z-cz,2)+pow(y-cy,2));
        float tmp2=sqrt(pow(-cy+y,2)+pow(-cz+z,2));
        float d=pow(c-tmp1,2)+pow(x-cx,2);
        if ( d<a*a+80)
        {
            v.x = (  2*(-cx+x));
            v.y = (-(2*(-cy+y)*(c-tmp1))/tmp2);
            v.z = (-(2*(-cz+z)*(c-tmp1))/tmp2)+1;
            v=Add(bj->speed,v);
        }
    }
    return v;
}

vector Coesao(Boid* bj)
{
    vector tmp;
    tmp = Diff(goal->pos,bj->pos);

    if (bj->pos.x<-900) tmp.x= 10;
    if (bj->pos.x> 900) tmp.x=-10;
    if (bj->pos.y<-900) tmp.y= 10;
    if (bj->pos.y> 900) tmp.y=-10;
    if (bj->pos.z<  5) tmp.z= 100;
    if (bj->pos.z> 100) tmp.z=-10;

    if (bj->id==0) {bj->pos=goal->pos; bj->speed=goal->speed;}

    return tmp;
}

vector Separacao(Boid* bj) // Cada boid mantem uma distancia minima dos vizinhos
{
    vector c = NullVector();
    int q=1;
    It itb = BOIDS.begin();
    while (itb!=BOIDS.end())
    {
        boid2=&(*itb);
        float d=1+Norm(Diff(bj->pos, boid2->pos));
        if (boid2->id != bj->id && d < 20)
            c = Add(c,Mult(Diff(bj->pos, boid2->pos),1/(2*d*d)));

        itb++;
    }
    return c;
}

vector Alinhamento(Boid* bj) // Cada boid tende a voar na mesma direcao e velocidade dos vizinhos
{
    vector mspeed = NullVector();
    It itb = BOIDS.begin();
    while (itb!=BOIDS.end())
    {
        boid1=&(*itb);
        float d=1+Norm(Diff(bj->pos, boid2->pos));
        if (bj->id != boid1->id && d < 20) mspeed=Add(mspeed,Mult(boid1->speed,1/(d*d)));
        itb++;
    }
    mspeed = Mult(mspeed,(float(1)/float(size-1)));
    return  Diff(mspeed,bj->speed);
}

void MoveAll()
{
    It itb = BOIDS.begin();
    pthread_t thread[1000];
    pthread_attr_t attr;
    
    pthread_attr_init(&attr);
    pthread_attr_setdetachstate(&attr,PTHREAD_CREATE_JOINABLE);

    int i = 0;
    while (itb!=BOIDS.end())
    {
        boid=&(*itb);
        
        pthread_create(&thread[i],&attr,MoveOne,(void *)boid); 
        itb++;
        i++;
    }

    itb = BOIDS.begin();
    i = 0;
    //pthread_attr_destroy(&attr);
    while(itb!=BOIDS.end())
    {
       //printf("Tentando joinar a thread %d... ",i);
       pthread_join(thread[i],NULL);
       //printf("Thread %d joinada com sucesso!\n",i);
       i++;
       itb++;
    }
}

void *MoveOne (void *b)
{
    Boid*  boid =(Boid *)b;
    vector v1,v2,v3,v4,vnew;
    v1=Mult( Coesao(boid), 0.0005);
    v2=Mult( Separacao(boid), 0.01);
    v3=Mult( Alinhamento(boid) , 0.01);
    v4=Mult( Obstaculo(boid), 0.01);
    if (Norm(v4)>0) v1=Mult(v1,0);

    vector tmp=NullVector();

    tmp=Add(boid->speed,v1);
    tmp=Add(tmp,v2);
    tmp=Add(tmp,v3);
    tmp=Add(tmp,v4);

    float norm=sqrt(tmp.x*tmp.x + tmp.y*tmp.y + tmp.z*tmp.z);
    if (norm>1)   tmp = Mult(tmp,1/norm);
    if (norm<0.1) tmp = Mult(tmp,0.2/(norm+0.01));
    boid->speed=tmp;
    boid->pos = Add(boid->pos,boid->speed);
}

int main(int argc,char **argv)
{
    int i,j;

    for (i=0; i<size; i++)
    {
        boid = new Boid;
        boid->id=i;
        boid->status=rand()%360;
        boid->speed = NullVector();
        boid->pos.x   = (float)(rand()%50)/2-450;
        boid->pos.y   = (float)(rand()%50)/2-450;
        boid->pos.z   = 20+rand()%7/3;
        BOIDS.push_back(*boid);
    }

    goal = new Boid;
    goal->id=0;
    goal->speed.x =   0.1;
    goal->speed.y =   0.1;
    goal->speed.z =   0.0;
    goal->pos.x   =  -400;
    goal->pos.y   =  -400;
    goal->pos.z   =  30;
    float norma=Norm(goal->speed);
    atx=goal->pos.x-60*goal->speed.x/norma;
    aty=goal->pos.y-60*goal->speed.y/norma;
    atz=goal->pos.z-40*goal->speed.z/norma+15; if (atz<5) atz=5;
    atx2=goal->pos.x-60*goal->speed.y/norma;
    aty2=goal->pos.y+60*goal->speed.x/norma;
    atz2=atz;       

    printf("\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\nINSTRUCOES:\n\n");
    printf("Velocidade com SETAS (Left/Right/Up/Down) Congela com tecla (D) Quantidade (+/-)\n");

    /* Set things up and go */
    glutInit(&argc,argv);
    glutInitDisplayMode(GLUT_DOUBLE | GLUT_RGB | GLUT_DEPTH);
    glutInitWindowPosition( 50, 50 );
    glutInitWindowSize( 1200, 800 );
    glutCreateWindow("TP II - CG - Instrucoes: Cameras teclas (1/2/3) Direcao (Left/Right/Up/Down) Velocidade (pgUP/pgDown) Congela c/tecla (D) Quantidade (+/-)");
    glutDisplayFunc(Display);
    glutVisibilityFunc(HandleVisibility);
    glutKeyboardFunc(HandleKeyboard);
    glutSpecialFunc(HandleSpecialKeyboard);
    glutMouseFunc(HandleMouse);
    CreateEnvironment();

    glutMainLoop();
    return(0);
}

