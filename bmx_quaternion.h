/// \file quaternion.h
/// \brief A file to perform quaternion math. It is meant to be used with the
/// BNO055 IMU driver.

#ifndef BMX_QUATERNION_HG
#define BMX_QUATERNION_HG

/// \def QUATERNION_SCALING
/// \brief scale raw quaternion output by 2^14 per documentation
#define QUATERNION_SCALING 16384

#include "bno055.h"

typedef struct Quaternion
{
/// \brief Convert Bosch quaternion to float
/// \param *q: pointer to a bosch quaternion struct
/// \returns a Quaternion with float type members

    float x;
    float y;
    float z;
    float w;
} Quaternion;

Quaternion bnoquat_to_float(struct bno055_quaternion_t *q);

/// \brief Calculate the magnitude of a quaternion.
/// \param *q: a pointer to a quaternion
/// \return the magnitude
float magnitude(Quaternion *q);

/// \brief Normalize a quaternion.
/// \param *q: a pointer to a quaternion
void normalize(Quaternion *q);

/// \brief Scale down a quaternion through division.
/// \param *q: a pointer to a quaternion
/// \param val: the number to divde by
void scale_divide(Quaternion *q, float val);

/// \brief Calculate the corresponding euler angles of a quaternion.
/// \param *q: a pointer to a quaternion
/// \return ea: a struct containing euler angles
struct bno055_euler_float_t toEuler(Quaternion *q);

#endif
