typedef struct {float x,y,z;} vector;

vector Add(vector u, vector v)
{
   vector w;;
   w.x=u.x+v.x;
   w.y=u.y+v.y;
   w.z=u.z+v.z;
   return w;
}

vector Diff(vector u, vector v)
{
   vector w;
   w.x=u.x-v.x;
   w.y=u.y-v.y;
   w.z=u.z-v.z;
   return w;
}

vector Mult(vector u, float a)
{
   vector w;
   w.x=u.x*a;
   w.y=u.y*a;
   w.z=u.z*a;
   return w;
}

float Dot(vector u, vector v)
{
   float a=u.x*v.x + u.y*v.y + u.z*v.z;
   return a;
}

float Norm(vector u)
{
   return sqrt(Dot(u,u));
}

void Print(vector u)
{
   printf("[%f, %f, %f]\n",u.x,u.y,u.z);

}

float Dist(vector u, vector v)
{
   vector w=Diff(u,v);
   return Norm(w);
}

vector NullVector()
{
   vector w;
   w.x=0;
   w.y=0;
   w.z=0;
   return w;
}
