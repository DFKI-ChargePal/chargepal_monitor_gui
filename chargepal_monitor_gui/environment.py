#!/usr/bin/env python3
import config as cfg
import toml
from database import (
    clear_database,
    insert_cart_data,
    insert_env_info,
    insert_robot_data,
)

env_toml_labels = ['Robot', 'ADS_Station', 'BCS_Station', 'Cart']


# Load environment data from the TOML file
def load_environment(file_path):
    try:
        with open(file_path) as f:
            return toml.load(f)
    except FileNotFoundError:
        return {}


# Save environment data to the TOML file
def save_environment(tables):
    clear_database()
    env_data = load_environment(cfg.envi_toml_file_path)

    # Clear existing data in the TOML file
    for label in env_toml_labels:
        if label in env_data:
            for key in env_data[label]:
                env_data[label][key] = []

    # Populate the TOML data with table data
    for table in tables:
        label = table.columns[0]['label']
        if label in env_toml_labels:
            for row in table.rows:
                for key in env_data[label]:
                    if key in row and row[key] not in env_data[label][key]:
                        env_data[label][key].append(row[key])

    # Save the updated data to the TOML file
    with open(cfg.envi_toml_file_path, 'w') as f:
        toml.dump(env_data, f)

    for robot_name, robot_station in zip(
        env_data['Robot']['robot_names'], env_data['Robot']['rbs_names']
    ):
        robot_data = (
            f'{robot_name}',
            f'{robot_station}',
            0,
            'none',
            'none',
            'none',
            'none',
            'none',
            0,
            0.0,
            0,
        )
        insert_robot_data(robot_data)
    for cart_name, cart_station in zip(
        env_data['Cart']['cart_names'], env_data['Cart']['bws_names']
    ):
        cart_data = (f'{cart_name}', f'{cart_station}', 'none', 'none', 'none', 0)
        insert_cart_data(cart_data)

    for label in env_toml_labels:
        if label in env_data:
            for key, value in env_data[label].items():
                data = (f'{key}', str(value), len(value))
                insert_env_info(data)
