#include <math.h>

typedef struct vector_coord {
  double x;
  double y;
  double z;
} vector;

typedef struct polar_coord {
  double az;
  double ze;
} polar;

// Converts a spherical unit vector to a rectangular unit vector
vector sph2rect (polar p)
{
  vector v;
  v.x = sin(p.az)*sin(p.ze);
  v.y = cos(p.az)*sin(p.ze);
  v.z = cos(p.ze);
  return v;
}

// Converts a rectangular unit vector to a spherical unit vector
polar rect2sph (vector v)
{
  polar p;
  p.az = atan2(v.x , v.y);
  p.ze = atan2(sqrt(v.x*v.x + v.y*v.y) , v.z);
  return p;
}


// Used for translating the sun's azimuth and zenith angles
// Gets values that are with respect to the normal of the panel
polar translate(polar p_i, double d_a, double d_z)
{
  p_i.az = p_i.az + d_a; // azimuth translation
  
  // convert polar_initial coordinates to rectangular
  vector v_i = sph2rect (p_i);

  // Use Rodrigues' Formula to rotate the plane of the panel
  vector v_f;
  double factor = v_i.y - v_i.y*cos(d_z) + cos(d_z);
  v_f.x = v_i.x*(factor) + v_i.z*sin(d_z);
  v_f.y = v_i.y*(factor); 
  v_f.z = v_i.z*(factor) - v_i.x*sin(d_z);

  polar p_f = rect2sph(v_f);
  return p_f;
}

