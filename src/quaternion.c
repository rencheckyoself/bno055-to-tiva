#include <stdlib.h>
#include <stdio.h>
#include <math.h>

#include "utils/uartstdio.h"

#include "bno055.h"
#include "bmx_quaternion.h"

Quaternion bnoquat_to_float(struct bno055_quaternion_t *q)
{
  Quaternion qf;

  qf.x = (float) q->x;
  qf.y = (float) q->y;
  qf.z = (float) q->z;
  qf.w = (float) q->w;
  return qf;
}

float magnitude(Quaternion *q)
{


    return sqrtf(q->w*q->w + q->x*q->x + q->y*q->y + q->z*q->z);
}

void normalize(Quaternion *q)
{
  float mag = magnitude(q);
  scale_divide(q, mag);
}

void scale_divide(Quaternion *q, float val)
{
  q->w /= val;
  q->x /= val;
  q->y /= val;
  q->z /= val;
}

struct bno055_euler_float_t toEuler(Quaternion *q)
{
  struct bno055_euler_float_t ea;

  float sqw = q->w * q->w;
  float sqx = q->x * q->x;
  float sqy = q->y * q->y;
  float sqz = q->z * q->z;

  ea.p = atan2f(2.0 * (q->x * q->y + q->z * q->w), (sqx - sqy - sqz + sqw));
  ea.r = asinf(-2.0 * (q->x * q->z - q->y * q->w) / (sqx + sqy + sqz + sqw));
  ea.h = atan2f(2.0 * (q->y * q->z + q->x * q->w), (-sqx - sqy + sqz + sqw));

  return ea;
}
