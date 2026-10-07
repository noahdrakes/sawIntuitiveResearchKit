/* -*- Mode: C++; tab-width: 4; indent-tabs-mode: nil; c-basic-offset: 4 -*-    */
/* ex: set filetype=cpp softtabstop=4 shiftwidth=4 tabstop=4 cindent expandtab: */

/*
  Author(s):  Brendan Burkhart
  Created on: 2025-01-23

  (C) Copyright 2025 Johns Hopkins University (JHU), All Rights Reserved.

--- begin cisst license - do not edit ---

This software is provided "as is" under an open source license, with
no warranty.  The complete license can be found in license.txt and
http://www.cisst.org/cisst/license.txt.

--- end cisst license ---
*/

#ifndef _mtsBilateralTeleOperationPSM_h
#define _mtsBilateralTeleOperationPSM_h

#include <sawIntuitiveResearchKit/mtsTeleOperationPSM.h>

#include <cisstMultiTask/mtsInterfaceRequired.h>
#include <cisstMultiTask/mtsInterfaceProvided.h>

#include <cisstParameterTypes/prmForceCartesianGet.h>
#include <cisstParameterTypes/prmStateCartesian.h>
#include <cisstParameterTypes/prmStateJoint.h>

#include <memory>

#include <sawControllers/mtsPIDConfiguration.h>

#include "mtsContactDetector.h"

// Always include last
#include <sawIntuitiveResearchKitBilateralTeleopExport.h>

class CISST_EXPORT mtsBilateralTeleOperationPSM: public mtsTeleOperationPSM
{
    CMN_DECLARE_SERVICES(CMN_DYNAMIC_CREATION_ONEARG, CMN_LOG_ALLOW_DEFAULT);

public:
    mtsBilateralTeleOperationPSM(const std::string & componentName, const double periodInSeconds);
    mtsBilateralTeleOperationPSM(const mtsTaskPeriodicConstructorArg & arg);
    ~mtsBilateralTeleOperationPSM() {}

    void Configure(const Json::Value & jsonConfig) override;

    // switches m_teleop_mode (see its own comment below for the valid
    // values and what each does), and re-prints print_configuration_summary()
    // on every switch, not just at startup -- so it's always obvious in the
    // console which mode just took effect and whether the contact
    // detection model it depends on is actually loaded.
    void set_teleop_mode(const std::string & mode);

protected:
    class ForceSource {
    public:
        void Configure(mtsBilateralTeleOperationPSM* teleop, const Json::Value & jsonConfig);

        // samples measured_cf, applying set_cf_orientation_absolute (if
        // configured) the first time it becomes valid
        void sample();

        mtsFunctionRead measured_cf;
        prmForceCartesianGet m_measured_cf;

        // optional: when the source function is a "body/..." wrench, this
        // requests the arm re-express it in a fixed orientation instead of
        // the (rotating) tool-tip frame -- see set_cf_orientation_absolute
        // below and mtsIntuitiveResearchKitArm::body_set_cf_orientation_absolute
        bool body_cf_orientation_absolute = false;
        mtsFunctionWrite set_cf_orientation_absolute;
        bool m_orientation_absolute_applied = false;

        mtsBilateralTeleOperationPSM* teleop;
        std::string component_name;
        std::string provided_interface_name;
        std::string function_name;
    };

    class Arm {
    public:
        Arm(mtsBilateralTeleOperationPSM* teleop) : teleop(teleop) {}
        virtual ~Arm() {};

        virtual void populateInterface(mtsInterfaceRequired* interface);
        virtual void add_force_source(std::unique_ptr<ForceSource> source) { force_source = std::move(source); }

        // target_force_scale/current_force_scale multiply target's/this
        // arm's own force before they're combined into goal.Force(); used
        // to scale the PSM's own force contribution (beta) independently
        // of the MTM<->PSM position scale (alpha, i.e. "scale" above)
        virtual prmStateCartesian computeGoal(Arm* target, double scale,
                                              double target_force_scale = 1.0,
                                              double current_force_scale = 1.0);

        virtual vctFrm4x4& ClutchOrigin() = 0;

        virtual prmStateCartesian state();
        virtual void servo(prmStateCartesian goal);

        mtsFunctionRead measured_js;
        prmStateJoint m_measured_js;

    protected:
        mtsBilateralTeleOperationPSM* teleop;
        std::unique_ptr<ForceSource> force_source;

        mtsFunctionWrite servo_cs;
        mtsFunctionRead measured_cs;
    };

    class ArmMTM : public Arm {
    public:
        ArmMTM(mtsBilateralTeleOperationPSM* teleop) : Arm(teleop) {}
        ~ArmMTM() {}

        vctFrm4x4& ClutchOrigin() override;

        prmStateCartesian state() override;
        void servo(prmStateCartesian goal) override;

        mtsFunctionWrite servo_cp;
        prmPositionCartesianSet m_servo_cp;
    };

    class ArmPSM : public Arm {
    public:
        ArmPSM(mtsBilateralTeleOperationPSM* teleop) : Arm(teleop) {}
        ~ArmPSM() {}

        vctFrm4x4& ClutchOrigin() override;

        prmStateCartesian state() override;
        void servo(prmStateCartesian goal) override;

        mtsFunctionRead  measured_cp;
        mtsFunctionRead  measured_cv;
        prmPositionCartesianGet m_measured_cp;
        prmVelocityCartesianGet m_measured_cv;
    };

    ArmMTM mArmMTM;
    ArmPSM mArmPSM;

    double m_mtm_torque_gain = 0.2;
    double m_mtm_force_gain = 1;

    // beta: scales the PSM's own force in the combined F_mtm + beta*F_psm
    // term used for both arms' force goals (see RunCartesianTeleop)
    double m_psm_force_scale = 1.0;

    // contact detection: gates MTM force/position feedback on/off based on
    // an ONNX contact classifier. No base-class mode switching involved --
    // PSM is always driven by the bilateral control law; out of contact,
    // the MTM is simply released (see release_mtm) instead of being servoed.
    std::unique_ptr<mtsContactDetector> m_contact_detector;
    bool m_in_contact = true;
    // true whenever the MTM was released (out of contact) last cycle, so
    // RunCartesianTeleop can detect the re-engage edge and resync the
    // tracking anchor -- see the UpdateInitialState() call there
    bool m_mtm_was_released = false;

    // which of three ways the MTM's coupling is gated, runtime-settable via
    // set_teleop_mode for A/B testing (e.g. a user study) without editing
    // config or rebuilding:
    //   "bilateral"   -- always fully coupled, contact detection ignored,
    //                    the detector isn't even run
    //   "contact"     -- gated by m_in_contact from the ONNX contact
    //                    detector (the default); out of contact, falls
    //                    back entirely to the base class's own
    //                    RunCartesianTeleop() ("vanilla" teleop), after a
    //                    one-time release_mtm() at the exact transition
    //                    edge (see the out_of_contact checks in
    //                    RunCartesianTeleop() -- the base class replays
    //                    whatever wrench release_mtm() last set via
    //                    m_following_mtm_body_servo_cf, so without that
    //                    one-time zero it would keep applying a stale
    //                    nonzero wrench forever). This exact fallback was at
    //                    one point suspected of causing a torque
    //                    discontinuity (blamed on PSM's control law
    //                    switching between servo_cs and the base class's
    //                    servo_cp) and briefly dropped in favor of
    //                    release_mtm()-based zeroing -- reinstated after
    //                    finding the actual cause elsewhere: mtsPID's
    //                    disturbance observer only zeroes its OUTPUT when
    //                    use_disturbance_observer is off, but never resets
    //                    its internal disturbance_state -- so state left
    //                    over from whenever DO was last on gets reused,
    //                    stale, the next time DO turns back on, producing a
    //                    discontinuity that has nothing to do with which
    //                    PSM control law is active. "unilateral" (below)
    //                    now shares this exact fallback for that reason.
    //   "unilateral"  -- MTM is never coupled at all (position or force).
    //                    Same out-of-contact treatment as "contact" above --
    //                    one-time release_mtm() at the transition, then
    //                    the base class's own RunCartesianTeleop() the rest
    //                    of the time, unconditionally (out_of_contact is
    //                    always true for this mode). Previously kept on a
    //                    separate, always-on release_mtm() path instead of
    //                    sharing "contact"'s fallback, out of the same
    //                    (mistaken -- see "contact" above) discontinuity
    //                    concern; unified once the real cause (DO state,
    //                    not the fallback itself) was identified.
    std::string m_teleop_mode = "contact";

    // reads the *live* PID configuration (not the JSON file on disk, which
    // can be edited/reloaded independently and drift out of sync with
    // what's actually running) -- connects directly to each arm's own PID
    // component's "Controller" interface (component name "<arm>_PID", e.g.
    // "PSM2_PID"), separate from and not wired automatically the way the
    // "PSM"/"MTM" Arm interfaces above are, so it needs an explicit
    // connection in the manager JSON (manager-MTMR-PSM2-bilateral-ros.json).
    // See print_configuration_summary(). MTS_OPTIONAL: printed as "not
    // connected" rather than failing if the manager config doesn't wire
    // these up (e.g. an older manager JSON that predates this).
    mtsFunctionRead m_psm_pid_configuration;
    mtsFunctionRead m_mtm_pid_configuration;
    // the write side of the same "Controller" interface -- lets
    // set_psm_disturbance_observer()/set_mtm_disturbance_observer() push a
    // read-modify-write back to PID live, without needing a Python-side
    // binding for mtsPIDConfiguration (which doesn't exist in this build --
    // no sawControllersPython module). A plain bool is all Python needs to
    // pass in.
    mtsFunctionWrite m_psm_pid_configure;
    mtsFunctionWrite m_mtm_pid_configure;

    // read-modify-write use_disturbance_observer for every joint of one
    // arm's PID and push it back live via "configure" -- no restart, no
    // JSON file involved. Does nothing (SendError) if that arm's PID_config
    // connection isn't wired (see Init()/manager JSON).
    void set_psm_disturbance_observer(const bool & enable);
    void set_mtm_disturbance_observer(const bool & enable);
    void set_disturbance_observer(const bool & enable, const mtsFunctionRead & get_config,
                                   const mtsFunctionWrite & set_config, const std::string & arm_label);

    // gravity comp on, given body_servo_cf wrench, unlock (or lock, per
    // config) orientation -- used by release_mtm() (zero wrench, called
    // once on the transition out of contact)
    void command_mtm_wrench(const vct6 & force);
    void release_mtm();

    // prints current teleop_mode plus whether contact_detection actually
    // loaded a model --
    // printed once at Configure() and again on every set_teleop_mode()
    // switch, so it's always obvious in the console without querying the
    // interpreter separately. Deliberately plain std::cout, not CMN_LOG/
    // SendStatus, so it's unmissable without depending on log verbosity
    // settings or a connected interface being ready yet.
    void print_configuration_summary(void) const;

    void Init() override;

    void RunCartesianTeleop() override;
};

CMN_DECLARE_SERVICES_INSTANTIATION(mtsBilateralTeleOperationPSM);

#endif // _mtsBilateralTeleOperationPSM_h
