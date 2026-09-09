/**
 * quaternion_test -- what does IMUCalibrator::computeAvgStaticPose() actually compute?
 *
 * Deliberately SimTK-only, no ROS, no OpenSim, no model. The question is purely about
 * SimTK::Quaternion's inherited operators, and keeping it dependency-free means it builds
 * in seconds and can be run anywhere, including on the pi to check for version skew.
 *
 * Answers three things:
 *   1. what `~q` and `~q * q` resolve to for a SimTK::Quaternion
 *   2. whether `Quaternion * Quaternion` is a Hamilton product or something else
 *   3. what the averaging loop in IMUCalibrator.h:290 therefore returns
 */

#include <SimTKcommon.h>
#include <cstdio>
#include <vector>

using SimTK::Quaternion;
using SimTK::Rotation;
using SimTK::Vec3;
using SimTK::Vec4;
using SimTK::Real;

static void show(const char* label, const Quaternion& q) {
    std::printf("  %-34s [w=% .6f  x=% .6f  y=% .6f  z=% .6f]  |q|=%.6f\n",
                label, q[0], q[1], q[2], q[3], q.norm());
}

/** textbook Hamilton product, for comparison */
static Quaternion hamilton(const Quaternion& a, const Quaternion& b) {
    return Quaternion(a[0]*b[0] - a[1]*b[1] - a[2]*b[2] - a[3]*b[3],
                      a[0]*b[1] + a[1]*b[0] + a[2]*b[3] - a[3]*b[2],
                      a[0]*b[2] - a[1]*b[3] + a[2]*b[0] + a[3]*b[1],
                      a[0]*b[3] + a[1]*b[2] - a[2]*b[1] + a[3]*b[0]);
}

static Quaternion fromAngleAxis(Real deg, const Vec3& axis) {
    Vec3 u = axis / axis.norm();
    Real a = deg * SimTK::Pi / 180.0;
    return Quaternion(std::cos(a/2), u[0]*std::sin(a/2),
                      u[1]*std::sin(a/2), u[2]*std::sin(a/2));
}

int main() {
    std::printf("(build against whatever simbody this machine has -- run it on the pi too)\n\n");

    const Quaternion q = fromAngleAxis(30.0, Vec3(0.3, -0.7, 0.65));

    std::printf("== 1. what is `~q * q` ==\n");
    show("q", q);
    // ~q on a Vec4 is the TRANSPOSE (a Row4), so ~q * q is Row4 * Vec4 = a dot product.
    Real dot = ~q * q;
    std::printf("  ~q * q                             = % .12f   (a scalar, not a quaternion)\n", dot);
    std::printf("  q.conjugate() would give           ");
    Quaternion qc(q[0], -q[1], -q[2], -q[3]);
    show("", qc);
    std::printf("  hamilton(conj(q), q)               ");
    show("", hamilton(qc, q));

    std::printf("\n== 2. what is `Quaternion * Quaternion` ==\n");
    const Quaternion p = fromAngleAxis(50.0, Vec3(1, 0.2, -0.4));
    show("p", p);
    show("hamilton(q, p)  [correct]", hamilton(q, p));
    show("Rotation(q)*Rotation(p) as quat",
         (Rotation(q) * Rotation(p)).convertRotationToQuaternion());

    std::printf("\n== 3. the actual loop from IMUCalibrator.h:290 ==\n");
    // ten samples of a sensor that is NOT perfectly still: small jitter about a mean
    const int n = 10, m = 1;
    std::vector<Quaternion> samples;
    for (int i = 0; i < n; ++i)
        samples.push_back(fromAngleAxis(30.0 + 2.0 * i, Vec3(0.3, -0.7, 0.65)));

    std::printf("  first sample q0:\n");
    show("q0", samples[0]);
    std::printf("  last sample  q9 (18 deg away):\n");
    show("q9", samples[n-1]);

    std::vector<Quaternion> avgQuaternionErrors(m, Quaternion());
    std::vector<Quaternion> avgQuaternions(avgQuaternionErrors);

    for (int j = 0; j < m; ++j)
        for (int i = 0; i < n; ++i) {
            const Quaternion& s = samples[i];
            // THE LINE, verbatim in spirit. `~s * s` is Row4*Vec4 -> a scalar, so this is
            // a Quaternion scaled by a double, giving a Vec4. The explicit Quaternion(Vec4)
            // is what the original's implicit conversion does (and it NORMALISES).
            avgQuaternionErrors[j] = Quaternion(Vec4(avgQuaternionErrors[j] * (~s * s)));
        }
    show("accumulated error", avgQuaternionErrors[0]);

    for (auto& e : avgQuaternionErrors)
        e.setQuaternionFromAngleAxis(
            e.convertQuaternionToAngleAxis().scalarDivide(double(n)));
    show("after /n", avgQuaternionErrors[0]);

    for (int j = 0; j < m; ++j)
        // original writes `qe * q0` (Quaternion*Quaternion). In SimTK that is Vec4*Vec4,
        // which is deliberately non-conformant and yields void -- see notes. Using the
        // Hamilton product here, i.e. the most charitable reading of the intent.
        avgQuaternions[j] = hamilton(avgQuaternionErrors[j], samples[0]);
    show("RETURNED as the average", avgQuaternions[0]);
    show("...vs q0", samples[0]);

    std::printf("\n== 4. what the fix (~q0 * q) would have done ==\n");
    Quaternion q0c(samples[0][0], -samples[0][1], -samples[0][2], -samples[0][3]);
    Vec4 acc(0);
    for (int i = 0; i < n; ++i) {
        Quaternion e = hamilton(q0c, samples[i]);
        acc += e.convertQuaternionToAngleAxis();
    }
    Quaternion mean;
    mean.setQuaternionFromAngleAxis(Vec4(acc / double(n)));
    show("mean error from q0", mean);
    show("q0 * mean error [the average]", hamilton(samples[0], mean));
    std::printf("  (expected: ~9 deg past q0, i.e. the middle of the 0..18 deg spread)\n");

    return 0;
}
