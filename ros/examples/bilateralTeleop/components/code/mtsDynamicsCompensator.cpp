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

#include "mtsDynamicsCompensator.h"

#include <array>
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

    void ReadVector(const Json::Value & jsonArray, vctDoubleVec & vec,
                     size_t expected_size, const std::string & field_name,
                     const std::string & config_path)
    {
        if (!jsonArray.isArray() || jsonArray.size() != expected_size) {
            std::ostringstream message;
            message << "mtsDynamicsCompensator::Configure: " << config_path << ": expected \"" << field_name
                    << "\" to be a " << expected_size << "-element array";
            throw std::runtime_error(message.str());
        }
        vec.SetSize(expected_size);
        for (Json::ArrayIndex i = 0; i < jsonArray.size(); i++) {
            vec.at(i) = jsonArray[i].asDouble();
        }
    }

    void ReadVector3(const Json::Value & jsonArray, vct3 & vec,
                      const std::string & field_name, const std::string & config_path)
    {
        if (!jsonArray.isArray() || jsonArray.size() != 3) {
            throw std::runtime_error("mtsDynamicsCompensator::Configure: " + config_path
                                      + ": expected \"" + field_name + "\" to be a 3-element array");
        }
        for (Json::ArrayIndex i = 0; i < 3; i++) {
            vec[i] = jsonArray[i].asDouble();
        }
    }
}

mtsDynamicsCompensator::mtsDynamicsCompensator()
{
}

// defined here (not in the header) because Ort::Env/Ort::Session are only
// forward-declared in the header; std::unique_ptr's deleter needs the
// complete type, which onnxruntime_cxx_api.h provides in this file
mtsDynamicsCompensator::~mtsDynamicsCompensator()
{
}

void mtsDynamicsCompensator::Configure(const Json::Value & jsonConfig, const std::string & base_dir)
{
    Json::Value value = jsonConfig["model_dir"];
    if (value.empty()) {
        throw std::runtime_error("mtsDynamicsCompensator::Configure: \"model_dir\" is required");
    }
    std::string model_dir = ResolvePath(value.asString(), base_dir);
    while (!model_dir.empty() && model_dir.back() == '/') {
        model_dir.pop_back();
    }

    const std::string config_json_path = model_dir + "/config.json";
    std::ifstream config_stream(config_json_path);
    if (!config_stream.is_open()) {
        throw std::runtime_error("mtsDynamicsCompensator::Configure: unable to open \"" + config_json_path + "\"");
    }
    Json::Value model_config;
    config_stream >> model_config;

    value = model_config["input_dim"];
    const size_t input_dim = value.empty() ? 6 : static_cast<size_t>(value.asInt());
    if (input_dim != 6) {
        throw std::runtime_error("mtsDynamicsCompensator::Configure: " + config_json_path
                                  + ": expected \"input_dim\" == 6 (position+velocity, 3 joints)");
    }

    value = model_config["seq_len"];
    m_seq_len = value.empty() ? 1 : value.asInt();
    if (m_seq_len < 1) {
        throw std::runtime_error("mtsDynamicsCompensator::Configure: " + config_json_path
                                  + ": \"seq_len\" must be a positive integer");
    }
    m_feature_history.clear();

    value = model_config["velocity_filter_alpha"];
    if (!value.empty()) {
        m_velocity_filter_alpha = value.asDouble();
    }

    ReadVector(model_config["normalization_mean"], m_normalization_mean, input_dim, "normalization_mean", config_json_path);
    ReadVector(model_config["normalization_std"], m_normalization_std, input_dim, "normalization_std", config_json_path);
    ReadVector3(model_config["output_mean"], m_output_mean, "output_mean", config_json_path);
    ReadVector3(model_config["output_std"], m_output_std, "output_std", config_json_path);

    m_have_previous_position = false;
    m_filtered_velocity.SetAll(0.0);

    const auto slash = model_dir.find_last_of('/');
    const std::string basename = (slash == std::string::npos) ? model_dir : model_dir.substr(slash + 1);
    const std::string onnx_model_path = model_dir + "/" + basename + ".onnx";

    m_env = std::make_unique<Ort::Env>(ORT_LOGGING_LEVEL_WARNING, "mtsDynamicsCompensator");

    Ort::SessionOptions session_options;
    session_options.SetIntraOpNumThreads(1);
    m_session = std::make_unique<Ort::Session>(*m_env, onnx_model_path.c_str(), session_options);
}

void mtsDynamicsCompensator::Reset()
{
    m_have_previous_position = false;
    m_filtered_velocity.SetAll(0.0);
    m_feature_history.clear();
}

bool mtsDynamicsCompensator::Update(const vct3 & joint_position, double dt, vct3 & predicted_torque)
{
    if (!m_session) {
        throw std::runtime_error("mtsDynamicsCompensator::Update: called before Configure");
    }

    predicted_torque.SetAll(0.0);

    // finite-difference velocity + single-pole low-pass, matching
    // dynamics_estimation/extract_bag.py's velocity_from_position() so
    // training and inference compute this signal the same way
    vct3 raw_velocity(0.0);
    if (m_have_previous_position && dt > 0.0) {
        raw_velocity = (joint_position - m_previous_position) / dt;
    }
    m_previous_position = joint_position;
    m_have_previous_position = true;

    m_filtered_velocity = m_velocity_filter_alpha * raw_velocity
        + (1.0 - m_velocity_filter_alpha) * m_filtered_velocity;

    std::vector<float> features(6);
    for (size_t i = 0; i < 3; i++) {
        features[i] = static_cast<float>((joint_position[i] - m_normalization_mean.at(i)) / m_normalization_std.at(i));
        features[3 + i] = static_cast<float>((m_filtered_velocity[i] - m_normalization_mean.at(3 + i)) / m_normalization_std.at(3 + i));
    }

    m_feature_history.push_back(features);
    while (m_feature_history.size() > static_cast<size_t>(m_seq_len)) {
        m_feature_history.pop_front();
    }

    if (m_feature_history.size() < static_cast<size_t>(m_seq_len)) {
        return false; // still filling the window since Configure()
    }

    std::vector<float> window;
    window.reserve(static_cast<size_t>(m_seq_len) * 6);
    for (const auto & step : m_feature_history) {
        window.insert(window.end(), step.begin(), step.end());
    }

    Ort::AllocatorWithDefaultOptions allocator;
    Ort::AllocatedStringPtr input_name = m_session->GetInputNameAllocated(0, allocator);
    Ort::AllocatedStringPtr output_name = m_session->GetOutputNameAllocated(0, allocator);
    const char * input_names[] = {input_name.get()};
    const char * output_names[] = {output_name.get()};

    const std::array<int64_t, 3> input_shape = {1, m_seq_len, 6};
    Ort::MemoryInfo memory_info = Ort::MemoryInfo::CreateCpu(OrtArenaAllocator, OrtMemTypeDefault);
    Ort::Value input_tensor = Ort::Value::CreateTensor<float>(
        memory_info, window.data(), window.size(), input_shape.data(), input_shape.size());

    auto output_tensors = m_session->Run(Ort::RunOptions{nullptr}, input_names, &input_tensor, 1,
                                          output_names, 1);
    const float * output_data = output_tensors.front().GetTensorData<float>();

    for (size_t i = 0; i < 3; i++) {
        predicted_torque[i] = static_cast<double>(output_data[i]) * m_output_std[i] + m_output_mean[i];
    }

    return true;
}
