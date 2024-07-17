#!/usr/bin/env python3
import os

# Configuration and file paths
server_dir = os.path.abspath(os.path.join(os.getcwd(), "..", ".."))
envi_toml_file_path = os.path.join(server_dir, "chargepal_monitor_gui/cfg/envi.toml")
databases_path = os.path.join(server_dir, "chargepal_local_server/src/chargepal_local_server/db")
logs_path = os.path.join(server_dir, "chargepal_local_server/src/chargepal_local_server/logs")
ldb_path = os.path.join(
    databases_path,"ldb.db"
)
pdb_path = os.path.join(
    databases_path,"pdb.db"
)
dfki_logo_path = os.path.join(server_dir, "chargepal_monitor_gui/img/dfki_logo.png")
chargepal_logo_path = os.path.join(server_dir, "chargepal_monitor_gui/img/chargepal_logo.svg")