# chargepal_monitor_gui

The chargepal_monitor_gui starts the **ChargePal Cockpit** application, that allows users to configure ChargePal environment information and monitor databases on the server.

**Section: Set Environment**

The application interacts with the `ldb.db` database, specifically updating the `env_info`,`robot_info`,`cart_info` table each time the user saves the environment settings from the cockpit. Note that previous entries in the `env_info` table are not considered when the cockpit is restarted; each session starts fresh.

The environment configuration in the ChargePal Cockpit is divided into four sections:
- Robot Details
- Cart Details
- ADS Station Details
- BCS Station Details


In each of these sections, users can add rows and populate them with relevant details. It is important to note the following associations:
- Every robot is linked to a robot station.
- Every cart is linked to a battery waiting station (BWS).

These initial values are crucial for setting up the environment. When the user clicks the "Save Environment" button, the details entered in the cockpit are recorded into the `env_info`, `robot_info`, and `cart_info` tables within the `ldb.db` database, ensuring that all relevant information is stored appropriately.


**Section: Local Database**

Every table present inside `ldb.db` can be monitored. Refresh rate is 1Hz.

**Section: Planner Database**

Every table present inside `pdb.db` can be monitored. Refresh rate is 1Hz.

## Execution
- Inside the **chargepal_monitor_gui/scripts** folder, run `gui_core.py`
- Open http://localhost:8080/ to view the **ChargePal Cockpit**

## Dependency
- The [databases](https://git.ni.dfki.de/chargepal/system-integration/server-packages/chargepal_local_server/-/tree/main/src/chargepal_local_server/db?ref_type=heads) 


