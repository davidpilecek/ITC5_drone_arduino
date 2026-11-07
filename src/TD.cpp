#include <cmath>

namespace itc {

struct TDOutput {
    double ref;
    double dref;
};

class TrackingDifferentiator {
public:
    // Ts, r0, and h0 must be positive.
    TrackingDifferentiator(double Ts, double r0, double h0)
        : Ts_(Ts), r0_(r0), h0_(h0) {}

    // Call once every Ts seconds.
    TDOutput update(double input) {
        const double u = fhan(v1_ - input, v2_, r0_, h0_);

        // Both updates use the previous states.
        const double v1_next = v1_ + Ts_ * v2_;
        const double v2_next = v2_ + Ts_ * u;

        v1_ = v1_next;
        v2_ = v2_next;

        return {v1_, v2_};
    }

    void reset(double initialRef = 0.0, double initialDref = 0.0) {
        v1_ = initialRef;
        v2_ = initialDref;
    }

private:
    double Ts_;
    double r0_;
    double h0_;

    double v1_ = 0.0;
    double v2_ = 0.0;

    static double sign(double x) {
        return (x > 0.0) - (x < 0.0);
    }

    static double fhan(double e, double v2, double r0, double h0) {
        const double d  = r0 * h0 * h0;
        const double a0 = h0 * v2;
        const double y  = e + a0;

        const double a1 = std::sqrt(d * (d + 8.0 * std::abs(y)));
        const double a2 = a0 + sign(y) * (a1 - d) / 2.0;

        const double sy = (sign(y + d) - sign(y - d)) / 2.0;
        const double a  = (a0 + y - a2) * sy + a2;
        const double sa = (sign(a + d) - sign(a - d)) / 2.0;

        return -r0 * (a / d - sign(a)) * sa - r0 * sign(a);
    }
};

}