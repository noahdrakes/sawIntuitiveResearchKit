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

#include "mtsContactDetector.h"

#include <algorithm>
#include <array>
#include <cctype>
#include <fstream>
#include <sstream>
#include <stdexcept>

#include <onnxruntime_cxx_api.h>

namespace {
    std::string ResolvePath(const std::string & path, const std::string & base_dir)
    {
        if (path.empty() || path[0] == '/' || base_dir.empty()) {
            return path;
        }
        return base_dir + "/" + path;
    }

    // matches bilateral_teleop_classic.py's _JOINT_FEATURE_PATTERN:
    // r'^(?:(MTM|PSM)_)?([A-Z]+_FEEDBACK)_(\d+)$'
    // returns true and fills arm/base/index (1-based, as in the name) on match
    bool ParseJointFeatureName(const std::string & name, std::string & arm,
                                std::string & base, int & index)
    {
        std::string rest = name;
        arm = "PSM";
        if (rest.compare(0, 4, "MTM_") == 0) {
            arm = "MTM";
            rest = rest.substr(4);
        } else if (rest.compare(0, 4, "PSM_") == 0) {
            rest = rest.substr(4);
        }

        const std::string suffix = "_FEEDBACK_";
        const auto suffix_pos = rest.find(suffix);
        if (suffix_pos == std::string::npos) {
            return false;
        }
        base = rest.substr(0, suffix_pos) + "_FEEDBACK";
        const std::string index_str = rest.substr(suffix_pos + suffix.size());
        if (index_str.empty()
            || !std::all_of(index_str.begin(), index_str.end(),
                             [](unsigned char c) { return std::isdigit(c); })) {
            return false;
        }
        index = std::stoi(index_str);
        return true;
    }

    // matches bilateral_teleop_classic.py's _FORCE_FEATURE_SOURCES table
    bool ParseForceFeatureName(const std::string & name, mtsContactDetector::Source & source, size_t & index)
    {
        static const std::pair<const char *, std::pair<mtsContactDetector::Source, size_t>> table[] = {
            {"FORCE_X",     {mtsContactDetector::Source::PSM_FORCE, 0}},
            {"FORCE_Y",     {mtsContactDetector::Source::PSM_FORCE, 1}},
            {"FORCE_Z",     {mtsContactDetector::Source::PSM_FORCE, 2}},
            {"MTM_FORCE_X", {mtsContactDetector::Source::MTM_FORCE, 0}},
            {"MTM_FORCE_Y", {mtsContactDetector::Source::MTM_FORCE, 1}},
            {"MTM_FORCE_Z", {mtsContactDetector::Source::MTM_FORCE, 2}},
        };
        for (const auto & entry : table) {
            if (name == entry.first) {
                source = entry.second.first;
                index = entry.second.second;
                return true;
            }
        }
        return false;
    }

    mtsContactDetector::FeatureSource ResolveFeatureName(const std::string & name, const std::string & config_path)
    {
        mtsContactDetector::Source source;
        size_t index;
        if (ParseForceFeatureName(name, source, index)) {
            return {source, index};
        }

        std::string arm, base;
        int one_based_index;
        if (ParseJointFeatureName(name, arm, base, one_based_index)) {
            if (arm == "PSM" && base == "POSITION_FEEDBACK") { return {mtsContactDetector::Source::PSM_POSITION, static_cast<size_t>(one_based_index - 1)}; }
            if (arm == "PSM" && base == "VELOCITY_FEEDBACK") { return {mtsContactDetector::Source::PSM_VELOCITY, static_cast<size_t>(one_based_index - 1)}; }
            if (arm == "PSM" && base == "TORQUE_FEEDBACK")   { return {mtsContactDetector::Source::PSM_EFFORT,   static_cast<size_t>(one_based_index - 1)}; }
            if (arm == "MTM" && base == "POSITION_FEEDBACK") { return {mtsContactDetector::Source::MTM_POSITION, static_cast<size_t>(one_based_index - 1)}; }
            if (arm == "MTM" && base == "VELOCITY_FEEDBACK") { return {mtsContactDetector::Source::MTM_VELOCITY, static_cast<size_t>(one_based_index - 1)}; }
            if (arm == "MTM" && base == "TORQUE_FEEDBACK")   { return {mtsContactDetector::Source::MTM_EFFORT,   static_cast<size_t>(one_based_index - 1)}; }
        }

        throw std::runtime_error("mtsContactDetector::Configure: " + config_path
                                  + ": unrecognized feature name \"" + name + "\"");
    }

    void ReadNormalizationVector(const Json::Value & jsonArray, vctDoubleVec & vec,
                                  size_t expected_size, const std::string & field_name,
                                  const std::string & config_path)
    {
        if (!jsonArray.isArray() || jsonArray.size() != expected_size) {
            std::ostringstream message;
            message << "mtsContactDetector::Configure: " << config_path << ": expected \"" << field_name
                    << "\" to be a " << expected_size << "-element array";
            throw std::runtime_error(message.str());
        }
        vec.SetSize(expected_size);
        for (Json::ArrayIndex i = 0; i < jsonArray.size(); i++) {
            vec.at(i) = jsonArray[i].asDouble();
        }
    }
}

mtsContactDetector::mtsContactDetector()
{
}

// defined here (not in the header) because Ort::Env/Ort::Session are only
// forward-declared in the header; std::unique_ptr's deleter needs the
// complete type, which onnxruntime_cxx_api.h provides in this file
mtsContactDetector::~mtsContactDetector()
{
}

void mtsContactDetector::Configure(const Json::Value & jsonConfig, const std::string & base_dir)
{
    Json::Value value;

    value = jsonConfig["model_dir"];
    if (value.empty()) {
        throw std::runtime_error("mtsContactDetector::Configure: \"model_dir\" is required");
    }
    std::string model_dir = ResolvePath(value.asString(), base_dir);
    while (!model_dir.empty() && model_dir.back() == '/') {
        model_dir.pop_back();
    }

    value = jsonConfig["probability_threshold"];
    if (!value.empty()) {
        m_probability_threshold = value.asDouble();
    }
    value = jsonConfig["debounce_samples_in"];
    if (!value.empty()) {
        m_debounce_samples_in = value.asInt();
    }
    value = jsonConfig["debounce_samples_out"];
    if (!value.empty()) {
        m_debounce_samples_out = value.asInt();
    }

    // config.json / <basename>.onnx layout produced by
    // contact_detection/onnx/model_to_onnx.py, loaded the same way
    // bilateral_teleop_classic.py does
    const std::string config_json_path = model_dir + "/config.json";
    std::ifstream config_stream(config_json_path);
    if (!config_stream.is_open()) {
        throw std::runtime_error("mtsContactDetector::Configure: unable to open \"" + config_json_path + "\"");
    }
    Json::Value model_config;
    config_stream >> model_config;

    const Json::Value & feature_cols = model_config["feature_cols"];
    if (!feature_cols.isArray() || feature_cols.empty()) {
        throw std::runtime_error("mtsContactDetector::Configure: " + config_json_path
                                  + " is missing a non-empty \"feature_cols\"");
    }
    const size_t input_dim = feature_cols.size();
    value = model_config["input_dim"];
    if (!value.empty() && static_cast<size_t>(value.asInt()) != input_dim) {
        throw std::runtime_error("mtsContactDetector::Configure: " + config_json_path
                                  + ": \"input_dim\" doesn't match \"feature_cols\" length");
    }

    m_feature_sources.clear();
    m_feature_sources.reserve(input_dim);
    for (Json::ArrayIndex i = 0; i < feature_cols.size(); i++) {
        m_feature_sources.push_back(ResolveFeatureName(feature_cols[i].asString(), config_json_path));
    }

    value = model_config["seq_len"];
    m_seq_len = value.empty() ? 1 : value.asInt();
    if (m_seq_len < 1) {
        throw std::runtime_error("mtsContactDetector::Configure: " + config_json_path
                                  + ": \"seq_len\" must be a positive integer");
    }
    m_feature_history.clear();

    ReadNormalizationVector(model_config["normalization_mean"], m_normalization_mean, input_dim, "normalization_mean", config_json_path);
    ReadNormalizationVector(model_config["normalization_std"], m_normalization_std, input_dim, "normalization_std", config_json_path);

    const auto slash = model_dir.find_last_of('/');
    const std::string basename = (slash == std::string::npos) ? model_dir : model_dir.substr(slash + 1);
    const std::string onnx_model_path = model_dir + "/" + basename + ".onnx";

    m_env = std::make_unique<Ort::Env>(ORT_LOGGING_LEVEL_WARNING, "mtsContactDetector");

    Ort::SessionOptions session_options;
    session_options.SetIntraOpNumThreads(1);
    m_session = std::make_unique<Ort::Session>(*m_env, onnx_model_path.c_str(), session_options);
}

bool mtsContactDetector::Update(const Signals & signals, double & probability)
{
    if (!m_session) {
        throw std::runtime_error("mtsContactDetector::Update: called before Configure");
    }

    std::vector<float> features(m_feature_sources.size());
    for (size_t i = 0; i < m_feature_sources.size(); i++) {
        const FeatureSource & fs = m_feature_sources[i];
        double raw_value = 0.0;
        switch (fs.source) {
        case Source::PSM_POSITION: raw_value = signals.psm_position.at(fs.index); break;
        case Source::PSM_VELOCITY: raw_value = signals.psm_velocity.at(fs.index); break;
        case Source::PSM_EFFORT:   raw_value = signals.psm_effort.at(fs.index);   break;
        case Source::MTM_POSITION: raw_value = signals.mtm_position.at(fs.index); break;
        case Source::MTM_VELOCITY: raw_value = signals.mtm_velocity.at(fs.index); break;
        case Source::MTM_EFFORT:   raw_value = signals.mtm_effort.at(fs.index);   break;
        case Source::PSM_FORCE:    raw_value = signals.psm_force[fs.index];       break;
        case Source::MTM_FORCE:    raw_value = signals.mtm_force[fs.index];       break;
        }
        features[i] = static_cast<float>((raw_value - m_normalization_mean.at(i)) / m_normalization_std.at(i));
    }

    // push this cycle's vector, then trim from the front -- m_feature_history
    // is always in chronological order (oldest first) once full, since we
    // only ever push to the back and pop from the front
    m_feature_history.push_back(features);
    while (m_feature_history.size() > static_cast<size_t>(m_seq_len)) {
        m_feature_history.pop_front();
    }

    if (m_feature_history.size() < static_cast<size_t>(m_seq_len)) {
        // still filling the window for the first time since Configure() --
        // nothing to infer yet, report the unchanged debounced state
        probability = m_last_probability;
        return m_in_contact;
    }

    std::vector<float> window;
    window.reserve(static_cast<size_t>(m_seq_len) * features.size());
    for (const auto & step : m_feature_history) {
        window.insert(window.end(), step.begin(), step.end());
    }

    Ort::AllocatorWithDefaultOptions allocator;
    Ort::AllocatedStringPtr input_name = m_session->GetInputNameAllocated(0, allocator);
    Ort::AllocatedStringPtr output_name = m_session->GetOutputNameAllocated(0, allocator);
    const char * input_names[] = {input_name.get()};
    const char * output_names[] = {output_name.get()};

    // shape (batch=1, seq_len=m_seq_len, features=input_dim)
    const std::array<int64_t, 3> input_shape = {1, m_seq_len, static_cast<int64_t>(features.size())};
    Ort::MemoryInfo memory_info = Ort::MemoryInfo::CreateCpu(OrtArenaAllocator, OrtMemTypeDefault);
    Ort::Value input_tensor = Ort::Value::CreateTensor<float>(
        memory_info, window.data(), window.size(), input_shape.data(), input_shape.size());

    auto output_tensors = m_session->Run(Ort::RunOptions{nullptr}, input_names, &input_tensor, 1,
                                          output_names, 1);

    // model's final layer is a sigmoid, so the raw output is already the
    // contact probability (see contact_prob = float(outputs[0][0][0][0])
    // in bilateral_teleop_classic.py's realtimehandler2)
    probability = static_cast<double>(output_tensors.front().GetTensorData<float>()[0]);
    m_last_probability = probability;
    const bool contact_predicted = probability >= m_probability_threshold;

    if (contact_predicted != m_in_contact) {
        m_pending_count++;
    } else {
        m_pending_count = 0;
    }

    const int required_samples = contact_predicted ? m_debounce_samples_in : m_debounce_samples_out;
    if (m_pending_count >= required_samples) {
        m_pending_count = 0;
        m_in_contact = contact_predicted;
    }

    return m_in_contact;
}
