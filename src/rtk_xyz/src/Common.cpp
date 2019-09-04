#include "Common.h"

void gps2xy(positionConf &xy_p, const positionConf &real_p)
{
  double RE0 = R0 / (sqrt(1 - e * e * sin(L0 * M_PI / 180) * sin(L0 * M_PI / 180)));
  double x0  = (RE0 + hb) * cos(L0 * M_PI / 180) * cos(lamda0 * M_PI / 180);
  double y0  = (RE0 + hb) * cos(L0 * M_PI / 180) * sin(lamda0 * M_PI / 180);
  double z0  = ((1 - e * e) * RE0 + hb) * sin(L0 * M_PI / 180);

  double L     = real_p.lat;
  double lamda = real_p.lon;
  double h     = real_p.height;
  double RE    = R0 / (sqrt(1 - e * e * sin(L * M_PI / 180) * sin(L * M_PI / 180)));
  double dx    = (RE + h) * cos(L * M_PI / 180) * cos(lamda * M_PI / 180) - x0;
  double dy    = (RE + h) * cos(L * M_PI / 180) * sin(lamda * M_PI / 180) - y0;
  double dz    = ((1 - e * e) * RE + h) * sin(L * M_PI / 180) - z0;

  double dn = -sin(L * M_PI / 180) * cos(lamda * M_PI / 180)*dx - sin(L * M_PI / 180) * sin(lamda * M_PI / 180) * dy +
              cos(L * M_PI / 180) * dz;
  double de = -sin(lamda * M_PI / 180) * dx + cos(lamda * M_PI / 180) * dy;
  double dd = -cos(L * M_PI / 180) * cos(lamda * M_PI / 180) * dx - cos(L * M_PI / 180) * sin(lamda * M_PI / 180) * dy -
              sin(L * M_PI / 180) * dz;

  xy_p.x          = de;
  xy_p.y          = dn;
  xy_p.z          = -dd;
  xy_p.lon        = real_p.lon;
  xy_p.lat        = real_p.lat;
  xy_p.height     = real_p.height;
  xy_p.velocity   = real_p.velocity;
  xy_p.velocity_x = real_p.velocity_x;
  xy_p.velocity_y = real_p.velocity_y;
  xy_p.velocity_z = real_p.velocity_z;
  xy_p.heading    = real_p.heading;
  xy_p.pitch      = real_p.pitch;
  xy_p.roll       = real_p.roll;
}


