/* -*- Mode: C++; tab-width: 4; indent-tabs-mode: nil; c-basic-offset: 4 -*-    */
/* ex: set filetype=cpp softtabstop=4 shiftwidth=4 tabstop=4 cindent expandtab: */

/*
  (C) Copyright 2026 Johns Hopkins University (JHU), All Rights Reserved.

  --- begin cisst license - do not edit ---

  This software is provided "as is" under an open source license, with
  no warranty.  The complete license can be found in license.txt and
  http://www.cisst.org/cisst/license.txt.

  --- end cisst license ---
*/

#ifndef _mtsDynamicsCompensator_h
#define _mtsDynamicsCompensator_h

#include <deque>
#include <memory>
#include <string>
#include <vector>

#include <json/json.h>

#include <cisstVector/vctDynamicVectorTypes.h>
#include <cisstVector/vctFixedSizeVectorTypes.h>

// Always include last
#include <sawIntuitiveResearchKitBilateralTeleopExport.h>

namespace Ort {
    struct Env;
    struct Session;
}

// Wraps a learned inverse-dynamics identification model (Yilmaz et al. 2024,
// "Sensorless Transparency Optimized Haptic Teleoperation on the da Vinci
// Research Kit", Sec. III-C): an LSTM over (position, velocity) for one
// arm's first 3 (positioning-axis) joints, predicting the joint effort that
// arm's own dynamics (inertia, gravity, friction, cable coupling) would
// produce in free motion -- i.e. tau_dyn ~= tau_m when tau_ext = 0. The
// difference measured_effort - tau_dyn is an estimate of the *external*
// (contact) joint torque, with the arm's own dynamics subtracted out.
//
// Model layout matches dynamics_estimation/train.py's output: a folder
// containing config.json (feature_cols, seq_len, normalization_mean/std,
// output_mean/std) and <basename(model_dir)>.onnx -- same convention
// mtsContactDetector uses, plus the two output_* fields since this model's
// output (torque) needs un-normalizing, unlike the contact classifier's
// already-in-[0,1] sigmoid output.
//
// Only position is passed in; velocity is computed internally via a
// central-ish finite difference plus a single-pole low-pass filter (the
// bag used for training has no real joint-velocity signal from dVRK's ROS
// bridge, so training and inference compute velocity the same way -- see
// velocity_filter_alpha in config.json).
class CISST_EXPORT mtsDynamicsCompensator
{
public:
    mtsDynamicsCompensator();
    ~mtsDynamicsCompensator();

    // jsonConfig fields:
    //   model_dir  (required) folder containing config.json and
    //              <basename(model_dir)>.onnx, produced by
    //              dynamics_estimation/train.py. May be relative to base_dir.
    void Configure(const Json::Value & jsonConfig, const std::string & base_dir);

    // joint_position: first 3 (positioning-axis) joint positions, in the
    // same units/order the model was trained on. dt: seconds since the
    // previous call (the teleop task's period). predicted_torque is set to
    // the model's estimate of this arm's own-dynamics joint effort for
    // those 3 joints. Returns false during the seq_len-1 cycle warmup after
    // Configure() (predicted_torque left at zero), true once inference is
    // running.
    bool Update(const vct3 & joint_position, double dt, vct3 & predicted_torque);

    // clears the finite-difference velocity state and rolling seq_len
    // window, forcing a fresh warmup on the next Update() call -- use this
    // whenever Update() stops being called for a stretch of real time
    // (e.g. MTM released out of contact), so it doesn't compute a
    // spurious large velocity from a stale previous position once calls
    // resume. Does not require re-Configure()/reload the model.
    void Reset();

protected:
    std::unique_ptr<Ort::Env> m_env;
    std::unique_ptr<Ort::Session> m_session;

    vctDoubleVec m_normalization_mean; // size 6: pos(3), vel(3)
    vctDoubleVec m_normalization_std;
    vct3 m_output_mean;
    vct3 m_output_std;
    int m_seq_len = 1;
    double m_velocity_filter_alpha = 0.2;

    bool m_have_previous_position = false;
    vct3 m_previous_position;
    vct3 m_filtered_velocity;

    // rolling window of the last m_seq_len normalized [pos(3), vel(3)]
    // feature vectors, oldest first -- same convention as
    // mtsContactDetector::m_feature_history
    std::deque<std::vector<float>> m_feature_history;
};

#endif // _mtsDynamicsCompensator_h
