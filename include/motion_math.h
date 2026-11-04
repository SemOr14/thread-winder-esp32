#pragma once
#include <math.h>
inline double windingLength(double turns,double core,double thread,double pitch,double width,double factor) {
 return factor * 3.141592653589793 * (core*turns + thread*pitch/width*turns*turns)/1000.;
}
inline double windingTurns(double meters,double core,double thread,double pitch,double width,double factor) {
 const double b=thread*pitch/width, c=meters*1000./(factor*3.141592653589793);
 return 2*c/(core+sqrt(core*core+4*b*c));
}
inline double nextRate(double speed,double target,double acceleration,double remaining,double dt) {
 const double limit=sqrt(fmax(0.,2*acceleration*fmax(0.,remaining-2.)));
 const double desired=fmin(target,limit);
 return desired>speed ? fmin(desired,speed+acceleration*dt) : fmax(desired,speed-acceleration*dt);
}
