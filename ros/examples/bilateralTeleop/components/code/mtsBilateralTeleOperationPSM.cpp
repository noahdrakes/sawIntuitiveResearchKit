/* -*- Mode: C++; tab-width: 4; indent-tabs-mode: nil; c-basic-offset: 4 -*-    */
/* ex: set filetype=cpp softtabstop=4 shiftwidth=4 tabstop=4 cindent expandtab: */

/*
  Author(s):  Brendan Burkhart
  Created on: 2025-01-23

  (C) Copyright 2013-2026 Johns Hopkins University (JHU), All Rights Reserved.

  --- begin cisst license - do not edit ---

  This software is provided "as is" under an open source license, with
  no warranty.  The complete license can be found in license.txt and
  http://www.cisst.org/cisst/license.txt.

  --- end cisst license ---
*/

// header
#include "mtsBilateralTeleOperationPSM.h"

// cisst includes
#include <cisstMultiTask/mtsManagerLocal.h>

#include <algorithm>
#include <iostream>
#include <vector>

CMN_IMPLEMENT_SERVICES_DERIVED_ONEARG(mtsBilateralTeleOperationPSM,
                                      mtsTeleOperationPSM,
                                      mtsTaskPeriodicConstructorArg);

void mtsBilateralTeleOperationPSM::ForceSource::Configure(mtsBilateralTeleOperationPSM* teleop, const Json::Value & jsonConfig) {
    this->teleop = teleop;

    Json::Value value;
    value = jsonConfig["component"];
    if (!value.empty()) {
        component_name = value.asString();
    }

    value = jsonConfig["interface"];
    if (!value.empty()) {
        provided_interface_name = value.asString();
    }

    value = jsonConfig["function"];
    if (!value.empty()) {
        function_name = value.asString();
    }

    value = jsonConfig["body_cf_orientation_absolute"];
    if (!value.empty()) {
        body_cf_orientation_absolute = value.asBool();
    }

    std::string required_interface_name = component_name + "_" + provided_interface_name + "_force_source";
    mtsInterfaceRequired* interface = teleop->AddInterfaceRequired(required_interface_name);
    if (interface) {
        interface->AddFunction(function_name, measured_cf);

        if (body_cf_orientation_absolute) {
            // derive the sibling setter from the function name, e.g.
            // "body/measured_cf" -> "body/set_cf_orientation_absolute"
            // (see mtsIntuitiveResearchKitArm::body_set_cf_orientation_absolute)
            std::string setter_name = "set_cf_orientation_absolute";
            auto slash = function_name.rfind('/');
            if (slash != std::string::npos) {
                setter_name = function_name.substr(0, slash) + "/" + setter_name;
            }
            interface->AddFunction(setter_name, set_cf_orientation_absolute, MTS_OPTIONAL);
        }
    }
}

void mtsBilateralTeleOperationPSM::ForceSource::sample()
{
    if (body_cf_orientation_absolute && !m_orientation_absolute_applied
        && set_cf_orientation_absolute.IsValid()) {
        set_cf_orientation_absolute(true);
        m_orientation_absolute_applied = true;
    }
    measured_cf(m_measured_cf);
}

void mtsBilateralTeleOperationPSM::Arm::populateInterface(mtsInterfaceRequired* interface)
{
    interface->AddFunction("servo_cs", servo_cs, MTS_OPTIONAL);
    interface->AddFunction("measured_cs", measured_cs, MTS_OPTIONAL);
}

prmStateCartesian mtsBilateralTeleOperationPSM::Arm::computeGoal(Arm* target, double scale,
                                                                  double target_force_scale, double current_force_scale)
{
    prmStateCartesian goal;
    prmStateCartesian target_state = target->state();

    vct3 target_translation = target_state.Position().Translation() - target->ClutchOrigin().Translation();
    vct3 goal_translation = scale * target_translation + ClutchOrigin().Translation();

    auto align = vctMatRot3(target->ClutchOrigin().Rotation().TransposeRef() * ClutchOrigin().Rotation());

    if (target_state.PositionIsValid()) {
        goal.Position().Translation() = goal_translation;
        goal.Position().Rotation() = target_state.Position().Rotation() * align;
    }
    goal.PositionIsValid() = target_state.PositionIsValid();

    if (target_state.VelocityIsValid()) {
        goal.Velocity().Ref<3>(0) = scale * target_state.Velocity().Ref<3>(0);
        goal.Velocity().Ref<3>(3) = target_state.Velocity().Ref<3>(3);
    }
    goal.VelocityIsValid() = target_state.VelocityIsValid();

    prmStateCartesian current_state = state();
    if (target_state.ForceIsValid() && current_state.ForceIsValid()) {
        goal.Force() = -(target_force_scale * target_state.Force()) - (current_force_scale * current_state.Force());
    }
    goal.ForceIsValid() = target_state.ForceIsValid() && current_state.ForceIsValid();

    return goal;
}

prmStateCartesian mtsBilateralTeleOperationPSM::Arm::state()
{
    prmStateCartesian measured_state;
    measured_cs(measured_state);

    if (force_source) {
        force_source->sample();
        measured_state.Force() = force_source->m_measured_cf.Force();
        measured_state.ForceIsValid() = force_source->m_measured_cf.Valid();
    }

    return measured_state;
}

void mtsBilateralTeleOperationPSM::Arm::servo(prmStateCartesian goal)
{
    servo_cs(goal);
}

vctFrm4x4& mtsBilateralTeleOperationPSM::ArmMTM::ClutchOrigin() { return teleop->mMTM.m_pose_initial; }

prmStateCartesian mtsBilateralTeleOperationPSM::ArmMTM::state()
{
    if (measured_cs.IsValid()) {
        return Arm::state();
    }

    // measured_cs not available, fall back to measured_cp/measured_cv
    prmStateCartesian state;
    state.Position() = teleop->mMTM.m_measured_cp.Position();
    state.PositionIsValid() = teleop->mMTM.m_measured_cp.Valid();

    if (teleop->m_config.use_MTM_linear_velocity || teleop->m_config.use_MTM_angular_velocity) {
        auto mtm_velocity = teleop->mMTM.m_measured_cv;
        state.Velocity().Ref<3>(0) = mtm_velocity.VelocityLinear();
        state.Velocity().Ref<3>(3) = mtm_velocity.VelocityAngular();
        state.VelocityIsValid() = mtm_velocity.Valid();
    } else {
        state.VelocityIsValid() = false;
    }

    if (force_source) {
        force_source->sample();
        state.Force() = force_source->m_measured_cf.Force();
        state.ForceIsValid() = force_source->m_measured_cf.Valid();
    } else {
        state.ForceIsValid() = false;
    }

    return state;
}

void mtsBilateralTeleOperationPSM::ArmMTM::servo(prmStateCartesian goal)
{
    // Use servo_cs if available, otherwise fall back to servo_cp
    if (servo_cs.IsValid()) {
        Arm::servo(goal);
    } else {
        prmPositionCartesianSet& servo = teleop->mArmMTM.m_servo_cp;
        servo.Goal() = goal.Position();

        if (goal.VelocityIsValid()) {
            servo.Velocity() = goal.Velocity().Ref<3>(0);
            servo.VelocityAngular() = goal.Velocity().Ref<3>(3);
        } else {
            servo.Velocity().Assign(vct3(0));
            servo.VelocityAngular().Assign(vct3(0));
        }

        teleop->mArmMTM.servo_cp(servo);
    }
}

vctFrm4x4& mtsBilateralTeleOperationPSM::ArmPSM::ClutchOrigin() { return teleop->mPSM.m_pose_initial; };

prmStateCartesian mtsBilateralTeleOperationPSM::ArmPSM::state()
{
    if (measured_cs.IsValid()) {
        return Arm::state();
    }

    // measured_cs not available, fall back to measured_cp/measured_cv

    prmStateCartesian state;
    teleop->mArmPSM.measured_cp(teleop->mArmPSM.m_measured_cp);
    state.Position() = teleop->mArmPSM.m_measured_cp.Position();
    state.PositionIsValid() = teleop->mArmPSM.m_measured_cp.Valid();

    teleop->mArmPSM.measured_cv(teleop->mArmPSM.m_measured_cv);
    auto psm_velocity = teleop->mArmPSM.m_measured_cv;
    state.Velocity().Ref<3>(0) = psm_velocity.VelocityLinear();
    state.Velocity().Ref<3>(3) = psm_velocity.VelocityAngular();
    state.VelocityIsValid() = psm_velocity.Valid();

    if (force_source) {
        force_source->sample();
        state.Force() = force_source->m_measured_cf.Force();
        state.ForceIsValid() = force_source->m_measured_cf.Valid();
    } else {
        state.ForceIsValid() = false;
    }

    return state;
}

void mtsBilateralTeleOperationPSM::ArmPSM::servo(prmStateCartesian goal)
{
    // Use servo_cs if available, otherwise fall back to servo_cp
    if (servo_cs.IsValid()) {
        Arm::servo(goal);
    } else {
        prmPositionCartesianSet& servo = teleop->mPSM.m_servo_cp;
        servo.Goal() = goal.Position();

        if (goal.VelocityIsValid()) {
            servo.Velocity() = goal.Velocity().Ref<3>(0);
            servo.VelocityAngular() = goal.Velocity().Ref<3>(3);
        } else {
            servo.Velocity().Assign(vct3(0));
            servo.VelocityAngular().Assign(vct3(0));
        }

        teleop->mPSM.servo_cp(servo);
    }
}

mtsBilateralTeleOperationPSM::mtsBilateralTeleOperationPSM(const std::string & componentName,
                                                       const double periodInSeconds) :
    mtsTeleOperationPSM(componentName, periodInSeconds), mArmMTM(this), mArmPSM(this) { Init(); }

mtsBilateralTeleOperationPSM::mtsBilateralTeleOperationPSM(const mtsTaskPeriodicConstructorArg & arg) :
    mtsTeleOperationPSM(arg), mArmMTM(this), mArmPSM(this) { Init(); }

void mtsBilateralTeleOperationPSM::Init() {
    mtsInterfaceRequired* interface;

    interface = GetInterfaceRequired("MTM");
    if (interface) {
        interface->AddFunction("servo_cp", mArmMTM.servo_cp);
        interface->AddFunction("measured_js", mArmMTM.measured_js, MTS_OPTIONAL);
        mArmMTM.populateInterface(interface);
    }

    interface = GetInterfaceRequired("PSM");
    if (interface) {
        interface->AddFunction("measured_cp", mArmPSM.measured_cp);
        interface->AddFunction("measured_js", mArmPSM.measured_js, MTS_OPTIONAL);
        mArmPSM.populateInterface(interface);
    }

    // see m_psm_pid_configuration/m_mtm_pid_configuration
    interface = AddInterfaceRequired("PSM_PID");
    if (interface) {
        interface->AddFunction("configuration", m_psm_pid_configuration, MTS_OPTIONAL);
        interface->AddFunction("configure", m_psm_pid_configure, MTS_OPTIONAL);
    }
    interface = AddInterfaceRequired("MTM_PID");
    if (interface) {
        interface->AddFunction("configuration", m_mtm_pid_configuration, MTS_OPTIONAL);
        interface->AddFunction("configure", m_mtm_pid_configure, MTS_OPTIONAL);
    }

    mConfigurationStateTable->AddData(m_teleop_mode, "teleop_mode");

    mtsInterfaceProvided* setting_interface = GetInterfaceProvided("Setting");
    if (setting_interface) {
        setting_interface->AddCommandWrite(&mtsBilateralTeleOperationPSM::set_teleop_mode, this,
                                        "set_teleop_mode", m_teleop_mode);
        setting_interface->AddCommandReadState(*(mConfigurationStateTable),
                                        m_teleop_mode, "teleop_mode");
        setting_interface->AddCommandWrite(&mtsBilateralTeleOperationPSM::set_psm_disturbance_observer,
                                        this, "set_psm_disturbance_observer", false);
        setting_interface->AddCommandWrite(&mtsBilateralTeleOperationPSM::set_mtm_disturbance_observer,
                                        this, "set_mtm_disturbance_observer", false);
    }
}

void mtsBilateralTeleOperationPSM::Configure(const Json::Value & jsonConfig)
{
    mtsTeleOperationPSM::Configure(jsonConfig);
    Json::Value jsonValue;

    jsonValue = jsonConfig["psm_force_source"];
    if (!jsonValue.empty()) {
        auto source = std::make_unique<ForceSource>();
        source->Configure(this, jsonValue);
        mArmPSM.add_force_source(std::move(source));
    }

    jsonValue = jsonConfig["mtm_force_source"];
    if (!jsonValue.empty()) {
        auto source = std::make_unique<ForceSource>();
        source->Configure(this, jsonValue);
        mArmMTM.add_force_source(std::move(source));
    }

    jsonValue = jsonConfig["mtm_torque_gain"];
    if (!jsonValue.empty()) {
        m_mtm_torque_gain = jsonValue.asDouble();
    }

    jsonValue = jsonConfig["mtm_force_gain"];
    if (!jsonValue.empty()) {
        m_mtm_force_gain = jsonValue.asDouble();
    }

    jsonValue = jsonConfig["psm_force_scale"];
    if (!jsonValue.empty()) {
        m_psm_force_scale = jsonValue.asDouble();
    }

    jsonValue = jsonConfig["contact_detection"];
    if (!jsonValue.empty()) {
        m_contact_detector = std::make_unique<mtsContactDetector>();
        m_contact_detector->Configure(jsonValue, BILATERAL_TELEOP_SHARE_DIR);
        // no contact info yet -- start soft rather than assuming contact
        m_in_contact = false;
    }

    jsonValue = jsonConfig["teleop_mode"];
    if (!jsonValue.empty()) {
        set_teleop_mode(jsonValue.asString());
    } else {
        // set_teleop_mode() below prints this same summary as a side
        // effect, but if "teleop_mode" wasn't in the JSON at all, it never
        // gets called -- print it here instead so startup always shows
        // what actually loaded, regardless of whether teleop_mode was set
        print_configuration_summary();
    }
}

namespace {
    // prints "ON/off/MIXED (name=val, ...)" for the first 3 joints. Two
    // overloads: one formats an already-fetched mtsPIDConfiguration
    // directly, the other reads live from PID first -- kept separate
    // because a config this process just wrote and a config freshly
    // re-read from PID are not interchangeable (see set_disturbance_observer).
    void print_disturbance_observer_status(std::ostream & out, const mtsPIDConfiguration & config)
    {
        if (config.empty()) {
            out << "connected, but PID returned no joints\n";
            return;
        }
        const size_t n = std::min<size_t>(3, config.size());
        bool any_on = false;
        bool any_off = false;
        for (size_t i = 0; i < n; i++) {
            (config.at(i).use_disturbance_observer ? any_on : any_off) = true;
        }
        out << (any_on && any_off ? "MIXED" : (any_on ? "ON" : "off")) << "  (";
        for (size_t i = 0; i < n; i++) {
            const auto & axis = config.at(i);
            out << axis.name << "=" << (axis.use_disturbance_observer ? "on" : "off");
            if (i + 1 < n) { out << ", "; }
        }
        out << ")\n";
    }

    // reads live from PID first -- for general status display, not right
    // after a write (see set_disturbance_observer)
    void print_disturbance_observer_status(std::ostream & out, const mtsFunctionRead & pid_configuration)
    {
        if (!pid_configuration.IsValid()) {
            out << "not connected (see manager JSON: needs a connection to "
                   "<arm>_PID's \"Controller\" interface)\n";
            return;
        }
        mtsPIDConfiguration config;
        pid_configuration(config);
        print_disturbance_observer_status(out, config);
    }
}

void mtsBilateralTeleOperationPSM::print_configuration_summary(void) const
{
    std::cout << "======================================================\n"
              << "mtsBilateralTeleOperationPSM [" << this->GetName() << "] configuration:\n"
              << "  teleop_mode:          \"" << m_teleop_mode << "\"\n"
              << "  contact_detection:    "
              << (m_contact_detector ? "CONFIGURED" : "not configured") << "\n"
              << "  disturbance_observer (live, read from PID -- not the JSON file):\n"
              << "    PSM: ";
    print_disturbance_observer_status(std::cout, m_psm_pid_configuration);
    std::cout << "    MTM: ";
    print_disturbance_observer_status(std::cout, m_mtm_pid_configuration);
    std::cout << "======================================================" << std::endl;
}

void mtsBilateralTeleOperationPSM::set_disturbance_observer(const bool & enable, const mtsFunctionRead & get_config,
                                                              const mtsFunctionWrite & set_config,
                                                              const std::string & arm_label)
{
    if (!get_config.IsValid() || !set_config.IsValid()) {
        if (mInterface) {
            mInterface->SendError(this->GetName() + ": set_" + arm_label
                                   + "_disturbance_observer: PID_config connection not wired "
                                   + "(see manager JSON) -- can't read or write live PID configuration");
        }
        return;
    }
    mtsPIDConfiguration config;
    get_config(config);
    for (auto & axis : config) {
        axis.use_disturbance_observer = enable;
    }
    set_config(config);

    // print from the local config we just sent, not a fresh read from PID:
    // "configure" is a queued write, only applied on PID's own task's next
    // cycle, so reading it back immediately here would race that and could
    // print the stale, pre-write value (this is what previously made it
    // look like a command had no effect until a second, redundant call).
    std::cout << arm_label << " disturbance_observer -> ";
    print_disturbance_observer_status(std::cout, config);
}

void mtsBilateralTeleOperationPSM::set_psm_disturbance_observer(const bool & enable)
{
    set_disturbance_observer(enable, m_psm_pid_configuration, m_psm_pid_configure, "psm");
}

void mtsBilateralTeleOperationPSM::set_mtm_disturbance_observer(const bool & enable)
{
    set_disturbance_observer(enable, m_mtm_pid_configuration, m_mtm_pid_configure, "mtm");
}

void mtsBilateralTeleOperationPSM::set_teleop_mode(const std::string & mode)
{
    static const std::vector<std::string> valid_modes = {
        "bilateral", "contact", "unilateral"
    };
    if (std::find(valid_modes.begin(), valid_modes.end(), mode) == valid_modes.end()) {
        if (mInterface) {
            mInterface->SendError(this->GetName() + ": set_teleop_mode: unknown mode \"" + mode + "\"");
        }
        return;
    }
    mConfigurationStateTable->Start();
    m_teleop_mode = mode;
    mConfigurationStateTable->Advance();

    // see print_configuration_summary()
    print_configuration_summary();
}

void mtsBilateralTeleOperationPSM::RunCartesianTeleop()
{
    if (m_clutched) {
        return;
    }

    const bool is_contact_mode = (m_teleop_mode == "contact");

    if (is_contact_mode && m_contact_detector && mArmPSM.measured_js.IsValid()) {
        mtsContactDetector::Signals signals;

        mArmPSM.measured_js(mArmPSM.m_measured_js);
        signals.psm_position = mArmPSM.m_measured_js.Position();
        signals.psm_velocity = mArmPSM.m_measured_js.Velocity();
        signals.psm_effort = mArmPSM.m_measured_js.Effort();

        if (mArmMTM.measured_js.IsValid()) {
            mArmMTM.measured_js(mArmMTM.m_measured_js);
            signals.mtm_position = mArmMTM.m_measured_js.Position();
            signals.mtm_velocity = mArmMTM.m_measured_js.Velocity();
            signals.mtm_effort = mArmMTM.m_measured_js.Effort();
        }

        // body-frame Cartesian force, linear only -- same signals already
        // used for the bilateral force goals below
        signals.psm_force = mArmPSM.state().Force().Ref<3>(0);
        signals.mtm_force = mArmMTM.state().Force().Ref<3>(0);

        double probability;
        m_in_contact = m_contact_detector->Update(signals, probability);
    }

    // "unilateral": MTM is never coupled at all, position or force -- same
    // out-of-contact treatment as "contact", just unconditional (see below).
    // "bilateral": always fully coupled, ignore contact entirely.
    // "contact": gated by the detector (updated above).
    const bool out_of_contact = (m_teleop_mode == "unilateral")
        || (is_contact_mode && !m_in_contact);

    if (!out_of_contact && m_mtm_was_released) {
        // re-engaging after the MTM was released: release_mtm() puts it in
        // true EFFORT_MODE, completely free to be moved, so it may have
        // drifted far from wherever ClutchOrigin()/m_pose_initial was last
        // set. Without this, the first engaged cycle's computeGoal() would
        // compute a position target from that stale anchor and servo_cs's
        // stiff PID would snap straight to it -- that's the large,
        // direction-depends-on-how-far-it-drifted force/"magnetic" feeling
        // right at the moment coupling engages. UpdateInitialState()
        // resyncs both arms' anchors to their current positions, same as
        // the base class does when resuming tracking after a clutch, so
        // this cycle's goal starts at zero position error instead.
        UpdateInitialState();
    }
    if (out_of_contact && !m_mtm_was_released) {
        // both "contact"'s out-of-contact path and "unilateral" fall back to
        // the base class's own RunCartesianTeleop() below, which never
        // touches MTM's commanded force itself -- it just replays
        // m_following_mtm_body_servo_cf (see its own body_servo_cf call),
        // whatever that was last set to. Without this one-time release
        // right at the transition, MTM would keep applying whatever
        // nonzero wrench the previous (coupled) branch last commanded,
        // forever.
        release_mtm();
    }
    m_mtm_was_released = out_of_contact;

    if (out_of_contact) {
        // fall back to the base class's own control law entirely -- see
        // m_teleop_mode's header comment for why "unilateral" was previously
        // kept on its own separate release_mtm()-based path instead of
        // sharing this one, and why that reasoning no longer holds
        mtsTeleOperationPSM::RunCartesianTeleop();
        return;
    }

    auto psm_goal = mArmPSM.computeGoal(&mArmMTM, m_config.scale, 1.0, m_psm_force_scale);
    mArmPSM.servo(psm_goal);

    auto mtm_goal = mArmMTM.computeGoal(&mArmPSM, 1.0 / m_config.scale, m_psm_force_scale, 1.0);

    // scale MTM torque goal to reduce oscillations
    mtm_goal.Force().Ref<3>(3) = m_mtm_torque_gain * mtm_goal.Force().Ref<3>(3);
    mtm_goal.Force().Ref<3>(0) = m_mtm_force_gain * mtm_goal.Force().Ref<3>(0);

    mArmMTM.servo(mtm_goal);
}

void mtsBilateralTeleOperationPSM::command_mtm_wrench(const vct6 & force)
{
    if (!m_config.MTM_is_haptic) {
        return;
    }
    if (mMTM.use_gravity_compensation.IsValid()) {
        mMTM.use_gravity_compensation(true);
    }
    if (mMTM.body_servo_cf.IsValid()) {
        prmForceCartesianSet wrench;
        wrench.Force() = force;
        mMTM.body_servo_cf(wrench);
        m_following_mtm_body_servo_cf = wrench;
    }
    if (m_config.rotation_locked && mMTM.lock_orientation.IsValid()) {
        mMTM.lock_orientation(mMTM.m_measured_cp.Position().Rotation());
    } else if (mMTM.unlock_orientation.IsValid()) {
        mMTM.unlock_orientation();
    }
}

void mtsBilateralTeleOperationPSM::release_mtm()
{
    command_mtm_wrench(vct6(0.0));
}
