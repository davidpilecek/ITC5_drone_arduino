
double Ts = 0.01;

double e  = 0.0;
double u  = 0.0;
double r0 = 3;
double h0 = Ts;
double v1 = 0.0;
double v2 = 0.0;
double v2_prev = 0.0;
double v1_prev = 0.0;

double fhan(double e, double v2, double r0, double h0){

d = r0*h0*h-;
a0 = h0*v2;
y = e + a0;

a1 = sqrt(d*(d + 8*abs(y)));
a2 = a0 + sign(y)*(a1 - d)/2;

sy = (sign(y + d) - sign(y - d))/2;
a = (a0 + y - a2)*sy + a2;

sa = (sign(a + d) - sign(a - d))/2;

u = -r0*(a/d - sign(a))*sa - r0*sign(a);
}

void setup() {
  // put your setup code here, to run once:

}

void loop() {

  u = fhan(v1_prev - v, v2, r0, h0);

  v2 = Ts*u + v2_prev;
  v1 = Ts*v2_prev + v1_prev;

  v1_prev = v1;
  v2_prev = v2;



}
