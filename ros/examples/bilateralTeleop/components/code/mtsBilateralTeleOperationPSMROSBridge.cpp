/* -*- Mode: C++; tab-width: 4; indent-tabs-mode: nil; c-basic-offset: 4 -*-    */
/* ex: set filetype=cpp softtabstop=4 shiftwidth=4 tabstop=4 cindent expandtab: */

/*
  (C) Copyright 2025 Johns Hopkins University (JHU), All Rights Reserved.

--- begin cisst license - do not edit ---

This software is provided "as is" under an open source license, with
no warranty.  The complete license can be found in license.txt and
http://www.cisst.org/cisst/license.txt.

--- end cisst license ---
*/

#include "mtsBilateralTeleOperationPSMROSBridge.h"

#include <cisst_ros_bridge/mtsROSBridge.h>

CMN_IMPLEMENT_SERVICES_DERIVED_ONEARG(mtsBilateralTeleOperationPSMROSBridge,
                                      mts_ros_crtk_bridge_provided,
                                      mtsTaskPeriodicConstructorArg);

mtsBilateralTeleOperationPSMROSBridge::mtsBilateralTeleOperationPSMROSBridge(
                                               const std::string & component_name,
                                               cisst_ral::node_ptr_t node_handle,
                                               const double period_in_seconds):
    mts_ros_crtk_bridge_provided(component_name, node_handle, period_in_seconds) {}

mtsBilateralTeleOperationPSMROSBridge::mtsBilateralTeleOperationPSMROSBridge(const mtsTaskPeriodicConstructorArg & arg):
    mts_ros_crtk_bridge_provided(arg) {}

void mtsBilateralTeleOperationPSMROSBridge::Configure(const std::string & teleop_name)
{
    std::string ros_namespace = teleop_name;
    cisst_ral::clean_namespace(ros_namespace);

    // "bilateral_enabled"/"set_bilateral_enabled" no longer exist on the
    // component (replaced by "teleop_mode"/"set_teleop_mode" -- see
    // mtsBilateralTeleOperationPSM's mode cleanup); bridging the old names
    // here left this whole "Setting" interface connection failing at
    // startup (cisst's BindCommands is all-or-nothing), silently taking
    // set_teleop_mode/set_scale/lock_rotation/lock_translation/state_command
    // down with it over ROS too, even though they're otherwise unaffected.
    events_bridge().AddPublisherFromCommandRead<std::string, CISST_RAL_MSG(std_msgs, String)>
        ("Setting", "teleop_mode", ros_namespace + "/teleop_mode");

    subscribers_bridge().AddSubscriberToCommandWrite<std::string, CISST_RAL_MSG(std_msgs, String)>
        ("Setting", "set_teleop_mode",
         ros_namespace + "/set_teleop_mode");

    mtsManagerLocal * manager = mtsComponentManager::GetInstance();
    manager->AddComponent(this);
    manager->Connect(teleop_name, "Setting", events_bridge().GetName(), "Setting");
    manager->Connect(teleop_name, "Setting", subscribers_bridge().GetName(), "Setting");
}
