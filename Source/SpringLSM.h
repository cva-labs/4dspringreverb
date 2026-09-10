//==============================================================================
//  4D Spring Reverb — 4D Lattice Spring Model (header-only DSP engine)
//  Copyright (C) 2026 CVA Labs — https://github.com/cva-labs/4dspringreverb
//
//  This program is free software: you can redistribute it and/or modify it
//  under the terms of the GNU Affero General Public License as published by
//  the Free Software Foundation, either version 3 of the License, or
//  (at your option) any later version.
//
//  This program is distributed in the hope that it will be useful, but
//  WITHOUT ANY WARRANTY; without even the implied warranty of
//  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
//  GNU Affero General Public License for more details.
//
//  You should have received a copy of the GNU Affero General Public License
//  along with this program. If not, see <https://www.gnu.org/licenses/>.
//==============================================================================

#pragma once

/*
    SpringLSM.h — Spring-reverb engine built on a 4D Lattice Spring Model (LSM).
    Header-only, no external dependencies (JUCE-free).

    MODEL
    =====
    The spring's rest shape is a helix embedded in 4-dimensional space: three
    "spatial" axes (x = along the spring, y/z = transverse) plus a fourth axis
    w ("winding phase"):

        C(t) = ( L*t,  r*cos(θ),  r*sin(θ),  rW*sin(2θ) ),   θ = 2π*nCoils*t

    Rings of 4 point masses (2x2 cross-section in y/z) are laid along that
    curve and neighbouring nodes are joined by Hookean springs acting along
    their FIXED rest direction (small-strain lattice spring model). Every node
    carries a 4-component displacement (x,y,z,w). Because the rest directions
    have components on all four axes (and explicit "winding" springs tie
    neighbouring rings along w), longitudinal, transverse and winding motion
    continuously exchange energy — the source of the dispersive, metallic
    "boing" of a real spring tank.

    INPUT / OUTPUT TRANSDUCERS
    ==========================
    An extra point mass (the input transducer) is coupled to the first ring by
    four stiff springs; the input signal is a force on that mass through a
    tanh() soft clipper, with an optional injection along the w axis.
    Two pickups read ring-averaged VELOCITIES near the far end (left channel)
    and at a movable position along the spring (right channel); the w velocity
    is mixed in with opposite polarity per channel (stereo decorrelation,
    "4D chirp"). Each pickup runs through a one-pole low-pass and a DC blocker.

    NUMERICS
    ========
    Node state stores DISPLACEMENT from the rest configuration (not absolute
    position): spring extensions are then small differences of small numbers —
    no catastrophic cancellation, no round-off noise floor pumping the modes.

    Symplectic (semi-implicit) Euler at the host sample rate. Axial stiffness
    is budgeted so that dt * ω_max ≈ 1.15 (stability limit 2). Losses:
      * per-spring viscous damping (material hysteresis, HF rolloff),
      * per-node decay g = exp(-dt/τ) with τ chosen for -60 dB at `decay` s,
      * weak 1.5 Hz anchors to the rest positions (kills DC drift),
      * the drive mass has its own fast damping (transducer load).
    A per-sample isfinite() guard resets the engine if it is ever driven
    unstable.
*/

#include <algorithm>
#include <atomic>
#include <cmath>
#include <cstdint>
#include <vector>

namespace spring4d
{

class SpringLSM
{
public:
    static constexpr int kMaxSegments  = 96;
    static constexpr int kNodesPerRing = 4;                                  // 2x2 cross-section (y,z)
    static constexpr int kMaxNodes     = kMaxSegments * kNodesPerRing + 1;   // + input transducer mass
    static constexpr int kMaxSprings   = 18 * kMaxSegments + 16;

    struct Parameters
    {
        float drive      = 0.50f;   // 0..1  input gain + soft clip
        float tension    = 0.50f;   // 0..1  axial stiffness
        float decay      = 2.00f;   // s     -60 dB decay time
        float helix4d    = 0.50f;   // 0..1  4th-dimension (winding) coupling
        float pickupB    = 0.35f;   // 0..1  position of the second pickup
        float brightness = 0.60f;   // 0..1  pickup low-pass cutoff
        int   segments   = 48;      // 24..kMaxSegments rings along the spring
        int   debugMask  = 0x3F;    // bit per spring class enabled (debug/bisection)
    };

    SpringLSM() = default;

    /** (Re)builds the lattice for a given sample rate and number of segments.
        Allocates once for kMaxSegments; later rebuilds are allocation-free. */
    void prepare (double sampleRate, int segments)
    {
        sampleRate_ = sampleRate;
        dt_         = (float) (1.0 / sampleRate);
        allocate();
        buildTopology (segments);
        updateCoefficients();
        reset();
    }

    void setParameters (const Parameters& p) noexcept
    {
        params_ = p;
        coefficientsDirty_ = true;
    }

    void reset() noexcept;

    /** Renders the wet signal only. In/out: non-aliased host sample rate. */
    void processBlock (const float* input, float* outL, float* outR, int numSamples) noexcept;

    int getSegments() const noexcept { return nSeg; }

    // ---- 4D helix geometry (public: shared with the GUI visualiser) --------
    static constexpr int   kCoils   = 10;      // turns of the 4D helix
    static constexpr float kLength  = 1.0f;    // length along x
    static constexpr float kRadius  = 0.25f;   // helix radius (y/z)
    static constexpr float kRadiusW = 0.14f;   // winding amplitude (w)
    static constexpr float kWireQ   = 0.045f;  // cross-section size
    static constexpr float kPi      = 3.14159265358979f;

    /*  Snapshot of the lattice for the GUI visualiser. The audio thread
        publishes frames; the GUI thread copies the latest published one
        (triple-buffered, lock-free). */
    struct VisualFrame
    {
        int   numNodes = 0;
        int   numRings = 0;
        int   ringA = 0, ringB = 0;
        float disp[4 * kMaxNodes];   // displacement from rest, component-major
        float kin[kMaxNodes];        // |velocity| per node
        float inputPeak = 0.0f;      // peak |input| since last frame
        float wetPeak  = 0.0f;       // peak |wet| since last frame
    };

    bool copyVisualFrame (VisualFrame& out) const noexcept;

    struct Diagnostics
    {
        float velX = 0, velY = 0, velZ = 0, velW = 0;      // max |v| per 4D component
        float dispX = 0, dispY = 0, dispZ = 0, dispW = 0;  // max |x - rest| per component
        int   maxVelNode = -1;
        float gNode = 0, dt = 0, kAx = 0;                  // runtime coefficients (debug)
    };

    Diagnostics diagnose() const noexcept;

private:
    static constexpr float mDrive_  = 4.0f;    // input transducer mass

    void publishVisualFrame (float inPeak, float wetPeak) noexcept;

    void allocate();
    void buildTopology (int segments);
    void updateCoefficients() noexcept;
    inline void processSample (float input, float& outL, float& outR) noexcept;

    Parameters params_ {};
    double     sampleRate_ = 48000.0;
    float      dt_         = 1.0f / 48000.0f;

    int  nSeg = 0, nNodes = 0, lastNode = 0, nSprings = 0, ringB_ = 24;
    bool coefficientsDirty_ = true;

    // component-major node arrays: [component][node]  (component = x,y,z,w)
    std::vector<float> pos_, vel_, force_, rest_;
    std::vector<int>   si_, sj_;
    std::vector<float> sdx_, sdy_, sdz_, sdw_, srest_, k_, c_;
    std::vector<std::uint8_t> scls_;

    // derived coefficients
    float gNode_ = 1.0f, gDrive_ = 1.0f;
    float lpfCoef_ = 0.5f, dcR_ = 0.996f, hpInR_ = 0.9974f;
    float kAnchor_ = 88.8f;                    // ~1.5 Hz anchor to rest
    float makeup_  = 60.0f;                    // wet output normalisation

    // filter state
    float lpfA_ = 0.0f, lpfB_ = 0.0f;
    float dcA_ = 0.0f, dcB_ = 0.0f, dcPrevA_ = 0.0f, dcPrevB_ = 0.0f;
    float hpIn_ = 0.0f, inPrev_ = 0.0f;        // input DC blocker state

    // GUI visual snapshot (triple-buffered; audio thread publishes, GUI copies)
    VisualFrame vis_[3];
    std::atomic<int> visPublished_ { -1 };
    int visWrite_ = 0, visCountdown_ = 0;
};

inline void SpringLSM::allocate()
{
    pos_  .resize (4 * kMaxNodes);
    vel_  .resize (4 * kMaxNodes);
    force_.resize (4 * kMaxNodes);
    rest_ .resize (4 * kMaxNodes);

    si_  .resize (kMaxSprings);
    sj_  .resize (kMaxSprings);
    sdx_ .resize (kMaxSprings);
    sdy_ .resize (kMaxSprings);
    sdz_ .resize (kMaxSprings);
    sdw_ .resize (kMaxSprings);
    srest_.resize (kMaxSprings);
    k_   .resize (kMaxSprings);
    c_   .resize (kMaxSprings);
    scls_.resize (kMaxSprings);
}

/** Lays out the 4D helix, the rings of nodes and all spring classes.
    Spring classes: 0 axial, 1 ring side, 2 ring diagonal, 3 winding (w),
    4 shear, 5 drive transducer. */
inline void SpringLSM::buildTopology (int segments)
{
    nSeg     = std::clamp (segments, 12, kMaxSegments);
    nNodes   = nSeg * kNodesPerRing + 1;
    lastNode = nNodes - 1;

    float* rx = rest_.data();
    float* ry = rx + kMaxNodes;
    float* rz = ry + kMaxNodes;
    float* rw = rz + kMaxNodes;
    const float q2 = 0.5f * kWireQ;

    for (int x = 0; x < nSeg; ++x)
    {
        const float t  = (nSeg > 1) ? (float) x / (float) (nSeg - 1) : 0.0f;
        const float th = (float) (2.0 * kPi * kCoils) * t;
        const float cx = kLength  * t;
        const float cy = kRadius  * std::cos (th);
        const float cz = kRadius  * std::sin (th);
        const float cw = kRadiusW * std::sin (2.0f * th);

        for (int n = 0; n < kNodesPerRing; ++n)
        {
            const int i = x * kNodesPerRing + n;
            rx[i] = cx;
            ry[i] = cy + (((n & 1) != 0) ? q2 : -q2);
            rz[i] = cz + (((n & 2) != 0) ? q2 : -q2);
            rw[i] = cw;
        }
    }

    // input transducer mass rest position (just before the first ring)
    rx[lastNode] = -0.10f;
    ry[lastNode] = kRadius;
    rz[lastNode] = 0.0f;
    rw[lastNode] = 0.0f;

    nSprings = 0;
    auto addSpring = [&] (int i, int j, int cls)
    {
        const float dx = rx[j] - rx[i];
        const float dy = ry[j] - ry[i];
        const float dz = rz[j] - rz[i];
        const float dw = rw[j] - rw[i];
        float len = std::sqrt (dx * dx + dy * dy + dz * dz + dw * dw);
        if (len < 1.0e-9f) len = 1.0e-9f;

        si_[nSprings]   = i;
        sj_[nSprings]   = j;
        sdx_[nSprings]  = dx / len;
        sdy_[nSprings]  = dy / len;
        sdz_[nSprings]  = dz / len;
        sdw_[nSprings]  = dw / len;
        srest_[nSprings]  = len;
        scls_[nSprings]   = (std::uint8_t) cls;
        ++nSprings;
    };

    // 0: axial springs along the 4D helix
    for (int x = 0; x + 1 < nSeg; ++x)
        for (int n = 0; n < kNodesPerRing; ++n)
            addSpring (x * kNodesPerRing + n, (x + 1) * kNodesPerRing + n, 0);

    // 1: ring sides + 2: ring diagonals (stiff wire cross-section)
    for (int x = 0; x < nSeg; ++x)
    {
        const int b = x * kNodesPerRing;
        addSpring (b + 0, b + 1, 1);
        addSpring (b + 2, b + 3, 1);
        addSpring (b + 0, b + 2, 1);
        addSpring (b + 1, b + 3, 1);
        addSpring (b + 0, b + 3, 2);
        addSpring (b + 1, b + 2, 2);
    }

    // 4: shear springs across rings (y/z flip)
    for (int x = 0; x + 1 < nSeg; ++x)
    {
        const int b = x * kNodesPerRing;
        const int c = b + kNodesPerRing;
        addSpring (b + 0, c + 1, 4);
        addSpring (b + 1, c + 0, 4);
        addSpring (b + 2, c + 3, 4);
        addSpring (b + 3, c + 2, 4);
    }

    // 3: winding springs across rings (along the 4th dimension w)
    for (int x = 0; x + 1 < nSeg; ++x)
    {
        const int b = x * kNodesPerRing;
        const int c = b + kNodesPerRing;
        addSpring (b + 0, c + 0, 3);
        addSpring (b + 1, c + 1, 3);
        addSpring (b + 2, c + 2, 3);
        addSpring (b + 3, c + 3, 3);
    }

    // 5: input transducer mass -> first ring
    for (int n = 0; n < kNodesPerRing; ++n)
        addSpring (lastNode, n, 5);
}

inline void SpringLSM::updateCoefficients() noexcept
{
    const Parameters& p = params_;

    // Stability-budgeted axial stiffness. Worst ("fully frustrated") lattice
    // mode sees TWICE the per-node stiffness sum (Gershgorin bound):
    //   omega_max^2 ~= 2 * 3.7 * kAx  ->  require dt * omega_max <= 1.25
    // (well under the symplectic-Euler limit of 2, even at max Tension).
    const float wDt  = 1.25f / dt_;
    const float kAxB = wDt * wDt / 7.4f;

    const float ts     = 0.40f + 0.95f * std::clamp (p.tension, 0.0f, 1.0f);
    const float kAx    = kAxB * ts;
    const float kRing  = 0.30f * kAx;
    const float kDiag  = 0.21f * kAx;
    const float kShear = 0.22f * kAx;
    const float kWind  = kAx * (0.04f + 0.18f * std::clamp (p.helix4d, 0.0f, 1.0f));
    const float w900   = 2.0f * kPi * 900.0f;
    const float kDrive = 0.25f * mDrive_ * w900 * w900;    // ~900 Hz transducer resonance
    const float zeta   = 0.0003f;                           // material hysteresis

    for (int s = 0; s < nSprings; ++s)
    {
        float k = kAx;
        switch (scls_[s])
        {
            case 1:  k = kRing;  break;
            case 2:  k = kDiag;  break;
            case 3:  k = kWind;  break;
            case 4:  k = kShear; break;
            case 5:  k = kDrive; break;
            default: break;
        }
        k_[s] = k;
        if (scls_[s] < 6 && (params_.debugMask & (1 << scls_[s])) == 0)
            k_[s] = 0.0f;                        // debug bisection: class disabled
        c_[s] = (2.0f * zeta) * std::sqrt (k_[s] > 0.0f ? k_[s] : 1.0e-9f);
    }

    // per-node decay: amplitude reaches -60 dB after `decay` seconds
    const float tau = std::max (p.decay, 0.05f) / 6.9078f;
    gNode_  = std::exp (-dt_ / tau);
    gDrive_ = std::exp (-dt_ / 0.0012f);

    // pickup low-pass + DC blocker coefficients
    const float fc = 700.0f * std::pow (12000.0f / 700.0f, std::clamp (p.brightness, 0.0f, 1.0f));
    lpfCoef_ = 1.0f - std::exp (-2.0f * kPi * fc / (float) sampleRate_);
    dcR_     = std::min (1.0f - 2.0f * kPi * 24.0f / (float) sampleRate_, 0.9995f);
    hpInR_   = std::min (1.0f - 2.0f * kPi * 20.0f / (float) sampleRate_, 0.9995f);

    // second pickup position along the spring
    ringB_ = std::clamp (
        (int) std::lround ((0.45f + 0.50f * std::clamp (p.pickupB, 0.0f, 1.0f)) * (float) (nSeg - 1)),
        1, nSeg - 2);

    coefficientsDirty_ = false;
}

inline void SpringLSM::reset() noexcept
{
    // pos_ stores displacement from rest: the rest state is all-zero
    std::fill (pos_.begin(),   pos_.end(),   0.0f);
    std::fill (vel_.begin(),   vel_.end(),   0.0f);
    std::fill (force_.begin(), force_.end(), 0.0f);

    lpfA_ = lpfB_ = dcA_ = dcB_ = dcPrevA_ = dcPrevB_ = 0.0f;
    hpIn_ = inPrev_ = 0.0f;
}

inline SpringLSM::Diagnostics SpringLSM::diagnose() const noexcept
{
    Diagnostics d;
    auto scan = [&] (const float* v, const float* p, const float* r, float& mv, float& md)
    {
        for (int i = 0; i < nNodes; ++i)
        {
            const float av = std::abs (v[i]);
            if (av > mv) { mv = av; d.maxVelNode = i; }
            md = std::max (md, std::abs (p[i]));   // p is displacement from rest
        }
    };
    scan (vel_.data (),                 pos_.data (),                 rest_.data (),                 d.velX, d.dispX);
    scan (vel_.data () + kMaxNodes,     pos_.data () + kMaxNodes,     rest_.data () + kMaxNodes,     d.velY, d.dispY);
    scan (vel_.data () + 2 * kMaxNodes, pos_.data () + 2 * kMaxNodes, rest_.data () + 2 * kMaxNodes, d.velZ, d.dispZ);
    scan (vel_.data () + 3 * kMaxNodes, pos_.data () + 3 * kMaxNodes, rest_.data () + 3 * kMaxNodes, d.velW, d.dispW);

    d.gNode = gNode_;
    d.dt    = dt_;
    d.kAx   = (nSprings > 0) ? k_[0] : 0.0f;
    return d;
}

inline bool SpringLSM::copyVisualFrame (VisualFrame& out) const noexcept
{
    const int idx = visPublished_.load (std::memory_order_acquire);
    if (idx < 0)
        return false;
    out = vis_[idx];
    return true;
}

inline void SpringLSM::publishVisualFrame (float inPeak, float wetPeak) noexcept
{
    VisualFrame& f = vis_[visWrite_];

    f.numNodes = nNodes;
    f.numRings = nSeg;
    f.ringA    = nSeg - 2;
    f.ringB    = ringB_;

    for (int c = 0; c < 4; ++c)
        std::copy_n (pos_.data() + c * kMaxNodes, (size_t) nNodes,
                     f.disp + c * kMaxNodes);

    for (int i = 0; i < nNodes; ++i)
    {
        const float vx = vel_[i];
        const float vy = vel_[kMaxNodes + i];
        const float vz = vel_[2 * kMaxNodes + i];
        const float vw = vel_[3 * kMaxNodes + i];
        f.kin[i] = std::sqrt (vx * vx + vy * vy + vz * vz + vw * vw);
    }

    f.inputPeak = inPeak;
    f.wetPeak   = wetPeak;

    visPublished_.store (visWrite_, std::memory_order_release);
    visWrite_ = (visWrite_ + 1) % 3;
}

/** Per-sample 4D LSM step: spring forces -> anchors -> drive -> integrate ->
    pickups (LPF + DC block). */
inline void SpringLSM::processSample (float input, float& outL, float& outR) noexcept
{
    float* px = pos_.data();
    float* py = px + kMaxNodes;
    float* pz = py + kMaxNodes;
    float* pw = pz + kMaxNodes;
    float* vx = vel_.data();
    float* vy = vx + kMaxNodes;
    float* vz = vy + kMaxNodes;
    float* vw = vz + kMaxNodes;
    float* fx = force_.data();
    float* fy = fx + kMaxNodes;
    float* fz = fy + kMaxNodes;
    float* fw = fz + kMaxNodes;
    // 1) clear forces
    for (int i = 0; i < nNodes; ++i)
    {
        fx[i] = 0.0f; fy[i] = 0.0f; fz[i] = 0.0f; fw[i] = 0.0f;
    }

    // 2) spring forces:  f = k*e + c*(dv.n),  F_i = +f*n,  F_j = -f*n
    {
        const int*         si  = si_.data();
        const int*         sj  = sj_.data();
        const float* dxs = sdx_.data();
        const float* dys = sdy_.data();
        const float* dzs = sdz_.data();
        const float* dws = sdw_.data();
        const float* kk  = k_.data();
        const float* cc  = c_.data();

        for (int s = 0; s < nSprings; ++s)
        {
            const int i = si[s];
            const int j = sj[s];

            const float dx = px[j] - px[i];
            const float dy = py[j] - py[i];
            const float dz = pz[j] - pz[i];
            const float dw = pw[j] - pw[i];
            // pos_ stores DISPLACEMENTS from rest, so the extension is just
            // dot(u_j - u_i, dir):  dot(dir, r_j - r_i) == rest, exactly.
            const float e  = dx * dxs[s] + dy * dys[s] + dz * dzs[s] + dw * dws[s];

            const float rvx = vx[j] - vx[i];
            const float rvy = vy[j] - vy[i];
            const float rvz = vz[j] - vz[i];
            const float rvw = vw[j] - vw[i];

            const float f = kk[s] * e
                          + cc[s] * (rvx * dxs[s] + rvy * dys[s] + rvz * dzs[s] + rvw * dws[s]);

            const float fxv = f * dxs[s];
            const float fyv = f * dys[s];
            const float fzv = f * dzs[s];
            const float fwv = f * dws[s];

            fx[i] += fxv; fy[i] += fyv; fz[i] += fzv; fw[i] += fwv;
            fx[j] -= fxv; fy[j] -= fyv; fz[j] -= fzv; fw[j] -= fwv;
        }
    }

    // 3) weak anchors to the rest configuration (prevent DC drift, bound the
    //    model). pos_ stores displacement from rest -> anchor force is -k*u.
    for (int i = 0; i < nNodes; ++i)
    {
        fx[i] -= kAnchor_ * px[i];
        fy[i] -= kAnchor_ * py[i];
        fz[i] -= kAnchor_ * pz[i];
        fw[i] -= kAnchor_ * pw[i];
    }

    // 4) input transducer: force on the drive mass (+ 4D injection on w).
    //    The force scale must be commensurate with the lattice stiffness
    //    (~4e8 per unit displacement) for a healthy working displacement.
    {
        // AC-couple the input: a DC component would displace the free chain
        const float hp = (input - inPrev_) + hpInR_ * hpIn_;
        inPrev_ = input;
        hpIn_   = hp;

        const float driveF = 4.0e6f * std::clamp (params_.drive, 0.0f, 2.0f);
        const float fi = std::tanh (1.7f * hp) * driveF;
        fx[lastNode] += fi;
        fw[lastNode] += fi * (0.35f * std::clamp (params_.helix4d, 0.0f, 1.0f));
    }

    // 5) symplectic (semi-implicit) Euler integration
    {
        const int nReg = nNodes - 1;
        for (int i = 0; i < nReg; ++i)
        {
            const float nvx = (vx[i] + dt_ * fx[i]) * gNode_;
            const float nvy = (vy[i] + dt_ * fy[i]) * gNode_;
            const float nvz = (vz[i] + dt_ * fz[i]) * gNode_;
            const float nvw = (vw[i] + dt_ * fw[i]) * gNode_;
            vx[i] = nvx; vy[i] = nvy; vz[i] = nvz; vw[i] = nvw;
            px[i] += dt_ * nvx; py[i] += dt_ * nvy;
            pz[i] += dt_ * nvz; pw[i] += dt_ * nvw;
        }

        const int i = lastNode;               // heavier, separately damped mass
        const float invM = dt_ / mDrive_;
        const float nvx = (vx[i] + invM * fx[i]) * gDrive_;
        const float nvy = (vy[i] + invM * fy[i]) * gDrive_;
        const float nvz = (vz[i] + invM * fz[i]) * gDrive_;
        const float nvw = (vw[i] + invM * fw[i]) * gDrive_;
        vx[i] = nvx; vy[i] = nvy; vz[i] = nvz; vw[i] = nvw;
        px[i] += dt_ * nvx; py[i] += dt_ * nvy;
        pz[i] += dt_ * nvz; pw[i] += dt_ * nvw;
    }

    // 6) pickups: ring-averaged velocities, w mixed with opposite polarity
    {
        const int bA = (nSeg - 2) * kNodesPerRing;
        const int bB = ringB_ * kNodesPerRing;
        float ax = 0.0f, ay = 0.0f, az = 0.0f, aw = 0.0f;
        float bx = 0.0f, by = 0.0f, bz = 0.0f, bw = 0.0f;
        for (int n = 0; n < kNodesPerRing; ++n)
        {
            ax += vx[bA + n]; ay += vy[bA + n]; az += vz[bA + n]; aw += vw[bA + n];
            bx += vx[bB + n]; by += vy[bB + n]; bz += vz[bB + n]; bw += vw[bB + n];
        }

        const float sA = 0.25f * (ax + 0.35f * (ay + az) + 0.35f * aw);
        const float sB = 0.25f * (bx + 0.35f * (by - bz) - 0.35f * bw);

        lpfA_ += lpfCoef_ * (sA - lpfA_);
        lpfB_ += lpfCoef_ * (sB - lpfB_);

        dcA_ = (lpfA_ - dcPrevA_) + dcR_ * dcA_;
        dcB_ = (lpfB_ - dcPrevB_) + dcR_ * dcB_;
        dcPrevA_ = lpfA_;
        dcPrevB_ = lpfB_;

        outL = dcA_ * makeup_;
        outR = dcB_ * makeup_;
    }
}

inline void SpringLSM::processBlock (const float* input, float* outL, float* outR,
                                     int numSamples) noexcept
{
    if (coefficientsDirty_)
        updateCoefficients();

    float inPeak = 0.0f, wetPeak = 0.0f;
    for (int n = 0; n < numSamples; ++n)
    {
        processSample (input[n], outL[n], outR[n]);

        inPeak  = std::max (inPeak,  std::abs (input[n]));
        wetPeak = std::max (wetPeak, std::max (std::abs (outL[n]), std::abs (outR[n])));

        if (! (std::isfinite (outL[n]) && std::isfinite (outR[n])))
        {
            reset();                              // safety net
            outL[n] = 0.0f;
            outR[n] = 0.0f;
        }
    }

    if ((visCountdown_ -= numSamples) <= 0)
    {
        publishVisualFrame (inPeak, wetPeak);     // feed the GUI visualiser
        visCountdown_ = 512;
    }
}

} // namespace spring4d
