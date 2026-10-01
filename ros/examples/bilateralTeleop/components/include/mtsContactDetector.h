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

#ifndef _mtsContactDetector_h
#define _mtsContactDetector_h

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

// Wraps an ONNX contact-classification model, loaded the same way as the
// Python reference implementation (contact_detection/bilateral_teleop/
// bilateral_teleop_classic.py): point Configure() at a model directory
// containing config.json (feature_cols, input_dim, seq_len,
// normalization_mean/std) and <basename(model_dir)>.onnx -- the layout
// contact_detection/onnx/model_to_onnx.py already produces. Each entry in
// feature_cols (e.g. "VELOCITY_FEEDBACK_1", "MTM_TORQUE_FEEDBACK_3",
// "FORCE_X", "MTM_FORCE_Y") is resolved against whichever live signal it
// names, the same mapping bilateral_teleop_classic.py's
// resolve_feature_vector() uses. Swapping models -- different joint
// subsets, with/without force features, PSM-only vs PSM+MTM -- is purely a
// model_dir change, no code change.
class CISST_EXPORT mtsContactDetector
{
public:
    mtsContactDetector();
    ~mtsContactDetector();

    // live signals available for feature resolution; whichever ones the
    // configured model's feature_cols doesn't reference are simply never
    // read, so callers don't need to populate fields an unused model won't
    // ask for (e.g. MTM signals for a PSM-only model)
    struct Signals {
        vctDoubleVec psm_position;
        vctDoubleVec psm_velocity;
        vctDoubleVec psm_effort;
        vctDoubleVec mtm_position;
        vctDoubleVec mtm_velocity;
        vctDoubleVec mtm_effort;
        vct3 psm_force; // body-frame Cartesian force, linear only (X, Y, Z)
        vct3 mtm_force;
    };

    // jsonConfig fields:
    //   model_dir              (required) folder containing config.json and
    //                          <basename(model_dir)>.onnx, same layout
    //                          model_to_onnx.py produces and
    //                          bilateral_teleop_classic.py loads. May be
    //                          relative to base_dir.
    //   probability_threshold  (optional, default 0.5)
    //   debounce_samples_in    (optional, default 20) consecutive in-contact
    //                          predictions required before reporting contact
    //   debounce_samples_out   (optional, default 5) consecutive no-contact
    //                          predictions required before reporting no
    //                          contact
    void Configure(const Json::Value & jsonConfig, const std::string & base_dir);

    // probability is the raw model output for this cycle. Returns the
    // current debounced in-contact state (unchanged unless a debounce
    // threshold was just crossed).
    bool Update(const Signals & signals, double & probability);

    // where a single feature_cols entry pulls its value from -- public so
    // the free functions in mtsContactDetector.cpp that parse feature_cols
    // names into these can build them
    enum class Source {
        PSM_POSITION, PSM_VELOCITY, PSM_EFFORT,
        MTM_POSITION, MTM_VELOCITY, MTM_EFFORT,
        PSM_FORCE, MTM_FORCE
    };
    struct FeatureSource {
        Source source;
        size_t index;
    };

protected:
    // resolved once at Configure time (not per-cycle, unlike the Python
    // reference's regex match every call -- functionally identical, just
    // precomputed), in feature_cols order
    std::vector<FeatureSource> m_feature_sources;

    std::unique_ptr<Ort::Env> m_env;
    std::unique_ptr<Ort::Session> m_session;

    vctDoubleVec m_normalization_mean;
    vctDoubleVec m_normalization_std;
    int m_seq_len = 1;

    // rolling window of the last m_seq_len normalized feature vectors,
    // oldest first -- Update() pushes this cycle's vector to the back and
    // pops from the front once it exceeds m_seq_len, so iterating front to
    // back always yields chronological order for the model input. Empty
    // (and inference skipped) until it fills up for the first time after
    // Configure(), which takes m_seq_len - 1 cycles.
    std::deque<std::vector<float>> m_feature_history;

    double m_probability_threshold = 0.5;
    int m_debounce_samples_in = 20;
    int m_debounce_samples_out = 5;

    bool m_in_contact = false;
    int m_pending_count = 0;
    // held over during the seq_len-1 cycle warmup, when Update() has
    // nothing new to report yet -- see the size check at the top of Update()
    double m_last_probability = 0.0;
};

#endif // _mtsContactDetector_h
